"""
random_move — turtle 하나를 무작위로 주행시키는 상주 노드 (백그라운드 실행)

여섯 도구 중 유일하게 계속 실행되는 프로그램이다. turtle 마다 하나씩 실행한다.
기본 동작 = **백그라운드**: 명령은 즉시 돌아오고, 주행 노드는 터미널과 분리된 별도 프로세스로 계속 실행된다.
그 프로세스는 turtle 이 kill 되면 스스로 종료한다.

[이 파일이 다루는 개념]
  ROS2   : 발행(cmd_vel) · 구독(pose) · 타이머 두 개(주행·감시) · 서비스 서버(SetBool) · 파라미터(선언·실행 중 읽기)
           노드의 자기 종료 — 자신이 제어하던 turtle 이 사라지면(kill) 스스로 끝난다
           정상 종료 순서 — 마지막 명령을 0 으로 발행 → destroy_node → shutdown
  Python : 클래스 상속(Node) · Enum 으로 상태 기계 · 콜백 메서드 · 예외로 종료 신호 받기(KeyboardInterrupt)
           subprocess 로 자기 자신을 다시 실행 · 세션 분리(start_new_session) · 로그 파일로 출력 전환

[읽기 전 알아야 할 것]
  Day 1 §5 (프로세스 관리 — ps · kill · 세션) · Day 2 §4~5 (rclpy 노드 구조) · Day 3 §4 (파라미터) · Day 3 §9 (상태 기계)

[실행]
  ros2 run my_first_pkg random_move leo                       # 백그라운드 시작 → 즉시 프롬프트로 돌아온다
  ros2 run my_first_pkg random_move leo --ros-args -p linear_max:=3.0 -p margin:=2.0
  ros2 run my_first_pkg random_move leo --foreground          # 터미널을 점유하며 실행 (로그를 직접 보며 관찰·디버깅)
  정지·재개 = stop leo / stop leo --resume   ·   종료 = kill leo (turtle 제거 → 노드 자기 종료)
  turtle 은 두고 노드만 끝내려면 = pkill -f "random_move --foreground leo"

[관찰할 것]
  · `ros2 node list` — leo_driver 가 보인다.  `rqt_graph` — 노드 ↔ /leo/cmd_vel ↔ turtlesim 연결.
  · `ps -ef | grep random_move` — 백그라운드 프로세스. 터미널을 닫아도 살아 있다(세션 분리).
  · `tail -f /tmp/leo_driver.log` — 백그라운드 노드의 로그.
  · `ros2 service list | grep driver` — /leo/driver/enable 이 이 노드가 제공하는 서비스다.
  · `ros2 param set /leo_driver linear_max 0.5` — 실행 중에 속도가 바뀐다.
  · kill leo → 이 노드가 1~2초 안에 스스로 종료한다 (`ps` 로 확인).
"""

from __future__ import annotations

import argparse
import math
import random
import subprocess
import sys
import tempfile
from enum import Enum, auto
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from turtlesim.msg import Pose

from my_first_pkg.common import (
    TurtleToolError,
    driver_node_name,
    enable_service_name,
    make_tool_node,
    resolve_name,
    user_args,
)

# turtlesim 창의 좌표 범위(양 축 동일). 소스 코드의 상수와 같다.
FIELD_MAX = 11.088889

# 감시 타이머 주기(초)와 "사라졌다" 판정까지의 연속 횟수. 0.5초 × 3 = 1.5초 후 종료.
WATCH_PERIOD_SEC = 0.5
MISSING_LIMIT = 3


class DriveState(Enum):
    """주행 상태 기계의 세 상태. Day 3 §9 미로 주행과 같은 구조(주행 → 후진 → 회전 → 주행)."""
    DRIVING = auto()            # 무작위 속도·조향으로 전진
    REVERSING = auto()          # 가장자리에 닿아 잠시 후진
    TURNING_TO_CENTER = auto()  # 화면 중앙을 향해 회전 후 주행으로 복귀


class RandomMover(Node):
    """turtle 하나를 담당하는 주행 노드. 노드 이름 = <turtle>_driver (common.driver_node_name 과 약속)."""

    def __init__(self, turtle: str):
        super().__init__(driver_node_name(turtle))
        self.turtle = turtle

        # ── 파라미터 선언 ─────────────────────────────────────────
        # 값을 코드 밖으로 빼는 이유(Day 3 §4.1): 실행할 때(-p) 또는 실행 중(param set) 바꾸기 위해서다.
        self.declare_parameter("linear_min", 1.0)     # 전진 속도 하한 (m/s 에 해당하는 turtlesim 단위)
        self.declare_parameter("linear_max", 2.0)     # 전진 속도 상한
        self.declare_parameter("angular_max", 2.5)    # 조향 각속도의 절댓값 상한 (rad/s)
        self.declare_parameter("margin", 1.5)         # 가장자리로 판정하는 여유 폭
        self.declare_parameter("period", 0.1)         # 주행 명령 발행 주기(초) — 실행 중 변경은 반영되지 않음(아래 주석)
        self.declare_parameter("hold_sec", 1.0)       # 한 번 정한 무작위 명령을 유지하는 시간(초)

        # ── 상태 ──────────────────────────────────────────────────
        self.pose: Pose | None = None          # 마지막으로 받은 위치. 첫 메시지 전에는 None.
        self.enabled = True                    # False 면 명령을 발행하지 않는다(stop 도구가 바꿈)
        self.gone = False                      # True 면 main 루프가 종료한다(turtle 소멸 감지)
        self.state = DriveState.DRIVING
        self.reverse_ticks = 0                 # REVERSING 상태 잔여 틱
        self.hold_ticks = 0                    # 현재 무작위 명령의 잔여 유지 틱
        self.current_cmd = Twist()             # 유지 중인 무작위 명령
        self.missing_count = 0                 # pose 발행자가 연속으로 관찰되지 않은 횟수

        # ── 통신 요소 ─────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, f"/{turtle}/cmd_vel", 10)
        self.pose_sub = self.create_subscription(Pose, f"/{turtle}/pose", self._on_pose, 10)
        self.enable_srv = self.create_service(SetBool, enable_service_name(turtle), self._on_enable)

        # ── 타이머 두 개 ─────────────────────────────────────────
        # period 는 타이머를 만들 때 한 번만 읽힌다. 그래서 실행 중 `param set period` 는 효과가 없다.
        # (실행 중 변경이 반영되는 값과 되지 않는 값의 차이 — Day 3 §4.5 의 논점을 여기서 직접 확인할 수 있다)
        period = self.get_parameter("period").value
        self.drive_timer = self.create_timer(period, self._drive)
        self.watch_timer = self.create_timer(WATCH_PERIOD_SEC, self._watch)

        self.get_logger().info(f"'{turtle}' 주행 시작 — 정지/재개: ros2 run my_first_pkg stop {turtle} [--resume]")

    # ──────────────────────────────────────────────────────────
    # 콜백 1 — 위치 수신
    # ──────────────────────────────────────────────────────────
    def _on_pose(self, msg: Pose) -> None:
        self.pose = msg

    # ──────────────────────────────────────────────────────────
    # 콜백 2 — 정지·재개 서비스 (stop.py 가 호출)
    # ──────────────────────────────────────────────────────────
    def _on_enable(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        """
        서비스 서버 콜백. 요청(request)을 읽고 응답(response)을 채워 돌려준다.
        서버 콜백 안에서 오래 걸리는 일을 하면 다른 콜백이 밀리므로, 플래그만 바꾸고 바로 반환한다.
        """
        self.enabled = request.data
        if not self.enabled:
            # 정지 시 0 명령을 한 번 발행한다. turtlesim 은 새 명령이 약 1초간 없으면 스스로 멈추지만,
            # 즉시·확실하게 멈추게 하는 것이 안전하다(실물 모터라면 이 한 줄이 필수다 — Day 8).
            self.cmd_pub.publish(Twist())
            self.state = DriveState.DRIVING     # 재개 시 후진·회전 잔여 상태를 이어가지 않도록 초기화
            self.hold_ticks = 0
        response.success = True
        response.message = "주행 재개" if self.enabled else "주행 정지 (turtle·노드는 유지)"
        self.get_logger().info(response.message)
        return response

    # ──────────────────────────────────────────────────────────
    # 콜백 3 — 주행 타이머
    # ──────────────────────────────────────────────────────────
    def _drive(self) -> None:
        if self.pose is None or not self.enabled:
            return                              # 위치를 아직 모르거나 정지 상태면 발행하지 않는다

        # 파라미터는 매 틱 읽는다 → `ros2 param set` 으로 바꾼 값이 다음 틱부터 반영된다.
        # (period 처럼 만들 때 한 번만 읽으면 반영되지 않는다 — 두 방식의 차이를 의도적으로 나란히 두었다)
        lin_min = self.get_parameter("linear_min").value
        lin_max = self.get_parameter("linear_max").value
        ang_max = self.get_parameter("angular_max").value
        margin = self.get_parameter("margin").value
        hold_ticks_total = max(1, int(self.get_parameter("hold_sec").value / max(self.drive_timer.timer_period_ns / 1e9, 1e-3)))

        p = self.pose
        near_edge = (p.x < margin or p.x > FIELD_MAX - margin or
                     p.y < margin or p.y > FIELD_MAX - margin)

        cmd = Twist()

        if self.state == DriveState.DRIVING:
            if near_edge:
                # 가장자리 → 후진 상태로 전환. 후진 시간 = 0.4초 분량의 틱
                self.state = DriveState.REVERSING
                self.reverse_ticks = max(1, int(0.4 / (self.drive_timer.timer_period_ns / 1e9)))
                cmd.linear.x = -1.5
            else:
                # hold_sec 마다 새 무작위 명령을 정한다. 매 틱 바꾸면 떨림만 커지고 궤적이 보이지 않는다.
                if self.hold_ticks <= 0:
                    self.current_cmd = Twist()
                    self.current_cmd.linear.x = random.uniform(lin_min, lin_max)
                    self.current_cmd.angular.z = random.uniform(-ang_max, ang_max)
                    self.hold_ticks = hold_ticks_total
                self.hold_ticks -= 1
                cmd = self.current_cmd

        elif self.state == DriveState.REVERSING:
            cmd.linear.x = -1.5
            self.reverse_ticks -= 1
            if self.reverse_ticks <= 0:
                self.state = DriveState.TURNING_TO_CENTER

        elif self.state == DriveState.TURNING_TO_CENTER:
            # 중앙(5.54, 5.54)을 향하는 각도와 현재 방향의 차이를 -π~π 로 정규화한 뒤 비례 제어로 회전
            center = FIELD_MAX / 2.0
            target = math.atan2(center - p.y, center - p.x)
            diff = math.atan2(math.sin(target - p.theta), math.cos(target - p.theta))
            if abs(diff) > 0.1:
                cmd.angular.z = max(-ang_max, min(ang_max, 2.0 * diff))
            else:
                self.state = DriveState.DRIVING
                self.hold_ticks = 0             # 주행 복귀 시 새 무작위 명령을 바로 정한다

        self.cmd_pub.publish(cmd)

    # ──────────────────────────────────────────────────────────
    # 콜백 4 — 감시 타이머 (turtle 소멸 → 자기 종료)
    # ──────────────────────────────────────────────────────────
    def _watch(self) -> None:
        """
        내가 담당하는 turtle 이 아직 있는지 0.5초마다 확인한다.

        근거 = /<turtle>/pose 토픽의 발행자 수. turtlesim_node 는 kill 하면 그 turtle 의 발행자를 없앤다.
        발견 정보는 즉시 갱신되지 않으므로 연속 MISSING_LIMIT 회(1.5초) 0 이어야 "사라졌다"로 판정한다.
        kill 도구는 이 노드를 알지 못한다 — 노드가 환경 변화를 스스로 감지해 정리하는 구조다.
        """
        if self.count_publishers(f"/{self.turtle}/pose") == 0:
            self.missing_count += 1
            if self.missing_count >= MISSING_LIMIT:
                self.get_logger().info(f"'{self.turtle}' 이(가) 사라져 주행 노드를 종료합니다.")
                self.gone = True
        else:
            self.missing_count = 0

    # ──────────────────────────────────────────────────────────
    # 종료 처리
    # ──────────────────────────────────────────────────────────
    def halt(self) -> None:
        """마지막으로 0 명령을 발행한다. 이미 통신이 닫힌 뒤라면 조용히 넘어간다."""
        try:
            if rclpy.ok():
                self.cmd_pub.publish(Twist())
        except Exception:                       # noqa: BLE001 — 종료 중의 예외는 무시한다
            pass


# ──────────────────────────────────────────────────────────────
# main
# ──────────────────────────────────────────────────────────────

def _parse(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="ros2 run my_first_pkg random_move", description="turtle 무작위 주행 노드")
    parser.add_argument("name", nargs="?", help="주행시킬 turtle 이름 (생략 시 목록 출력)")
    parser.add_argument("--foreground", action="store_true",
                        help="터미널을 점유하며 실행 (기본 = 백그라운드). 백그라운드 모드가 내부적으로 이 옵션으로 자신을 다시 실행한다")
    return parser.parse_args(argv)


def _log_path(name: str) -> Path:
    """백그라운드 노드의 출력이 저장되는 파일. /tmp/<turtle>_driver.log"""
    return Path(tempfile.gettempdir()) / f"{driver_node_name(name)}.log"


def _launch_background(name: str) -> int:
    """
    자기 자신을 `--foreground` 옵션으로 **다시 실행**하되, 터미널과 분리된 새 세션에서 실행한다.

    - `python -m my_first_pkg.random_move` : 이 모듈을 프로그램으로 실행 (`if __name__ == "__main__"` 경로)
    - sys.argv[1:] 를 그대로 넘기므로 `--ros-args -p …` 도 자식에게 전달된다
    - start_new_session=True : 자식이 새 세션의 리더가 된다 → 터미널을 닫아도(SIGHUP) 살아남는다
    - stdout/stderr → 로그 파일 : 터미널이 사라진 뒤에도 출력이 갈 곳이 있어야 한다
    - stdin → DEVNULL : 배경 프로세스가 키 입력을 기다리는 일이 없도록
    부모(이 함수)는 자식의 종료를 기다리지 않고 바로 돌아온다. 자식의 생애는 turtle 의 존재(_watch)가 결정한다.
    """
    log = _log_path(name)
    cmd = [sys.executable, "-m", "my_first_pkg.random_move", "--foreground"] + sys.argv[1:]
    with open(log, "ab") as log_file:
        proc = subprocess.Popen(
            cmd,
            stdin=subprocess.DEVNULL,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    print(f"백그라운드 시작: {driver_node_name(name)}  (PID {proc.pid})  로그 = {log}")
    print(f"→ 정지/재개:  ros2 run my_first_pkg stop {name} [--resume]")
    print(f"→ 종료:       ros2 run my_first_pkg kill {name}   (turtle 제거 → 노드 자기 종료)")
    print(f"→ 노드만 종료: pkill -f \"random_move --foreground {name}\"")
    return 0


def main() -> int:
    rclpy.init(args=sys.argv)                   # --ros-args -p … 는 여기서 처리된다
    mover: RandomMover | None = None
    try:
        args = _parse(user_args())

        # 1) 이름 확정·중복 실행 확인 — 임시 노드로 조회한 뒤 버린다.
        #    백그라운드 모드에서는 이 검사를 부모(터미널)에서 먼저 하므로 오류가 터미널에 바로 표시된다.
        probe = make_tool_node("turtle_random_move_probe")
        try:
            name = resolve_name(probe, args.name)
            if driver_node_name(name) in probe.get_node_names():
                raise TurtleToolError(
                    f"'{name}' 의 주행 노드가 이미 실행 중입니다. (`ps -ef | grep random_move` 로 확인)"
                )
        finally:
            probe.destroy_node()

        # 2-a) 백그라운드 모드(기본): 자신을 다시 실행하고 즉시 돌아온다.
        if not args.foreground:
            return _launch_background(name)

        # 2-b) 포그라운드 모드: 주행 노드 생성 후 spin. rclpy.spin() 대신 spin_once 반복을 쓰는 이유 = gone 플래그 확인.
        mover = RandomMover(name)
        while rclpy.ok() and not mover.gone:
            rclpy.spin_once(mover, timeout_sec=0.1)
        return 0

    except TurtleToolError as e:
        print(f"[오류] {e}", file=sys.stderr)
        return 1
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C. rclpy 가 먼저 컨텍스트를 닫으면 ExternalShutdownException, 아니면 KeyboardInterrupt 로 들어온다.
        print("\n종료 요청(Ctrl+C)")
        return 0
    finally:
        # 순서가 중요하다: ① 0 명령 발행(통신이 살아 있을 때) → ② 노드 파괴 → ③ rclpy 종료
        if mover is not None:
            mover.halt()
            mover.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
