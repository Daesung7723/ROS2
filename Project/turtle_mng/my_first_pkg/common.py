"""
my_first_pkg.common — 여섯 도구가 공유하는 함수·상수

[이 파일이 다루는 개념]
  ROS2   : 그래프 내성(introspection) — 실행 중인 토픽 목록에서 turtle을 찾는다
           서비스 클라이언트의 동기 호출 패턴 (wait_for_service → call_async → spin_until_future_complete)
           `ros2 run` 인자와 ROS 인자(`--ros-args …`)의 분리
  Python : 모듈 분리 · 타입 힌트 · dataclass · 예외 클래스 정의 · 정규식

[읽기 전 알아야 할 것]
  Day 2 §1~2 (서비스 요청·응답) · Day 3 §2~3 (패키지 · ros2 run)

[실행]
  이 파일은 직접 실행하지 않는다. 다른 도구가 import 해서 사용한다.

[관찰할 것]
  `ros2 topic list` 의 결과와 list_turtles() 의 결과가 같은지 비교한다.
  turtle 하나를 kill 한 직후 두 결과가 잠시 어긋나는 순간이 있는지 확인한다(발견 지연).
"""

from __future__ import annotations

import os
import re
import sys
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.client import Client
from rclpy.utilities import remove_ros_args


# ──────────────────────────────────────────────────────────────
# 상수
# ──────────────────────────────────────────────────────────────

#: 동시에 존재할 수 있는 turtle 개체 수의 상한. turtlesim_node 가 자동 생성하는 turtle1 을 포함한다.
MAX_TURTLES = 3

#: turtlesim 기본 이름(turtle1, turtle2 …)의 형태. 목록 정렬에서 번호 순서를 맞추는 데만 쓴다 — 이름 제한이 아니다.
TURTLE_NAME_PATTERN = re.compile(r"^turtle(\d+)$")

#: turtlesim 의 pose 토픽 타입. 이 타입의 토픽이 있으면 그 이름의 turtle 이 존재한다고 판정한다.
POSE_TYPE = "turtlesim/msg/Pose"

#: 그래프 정보(토픽·노드 목록)가 이 프로세스에 도착하기를 기다리는 시간(초).
#: 노드를 만든 직후에는 다른 노드의 정보가 아직 도착하지 않았을 수 있다.
DISCOVERY_SETTLE_SEC = 0.6

#: 서비스가 나타나기를 기다리는 기본 시간(초).
SERVICE_WAIT_SEC = 3.0


# ──────────────────────────────────────────────────────────────
# 예외
# ──────────────────────────────────────────────────────────────

class TurtleToolError(Exception):
    """도구가 정상 종료할 수 없을 때 발생. main() 에서 잡아 메시지만 출력하고 종료 코드 1 을 돌려준다."""


# ──────────────────────────────────────────────────────────────
# 인자 처리
# ──────────────────────────────────────────────────────────────

def user_args(argv: Optional[list[str]] = None) -> list[str]:
    """
    `ros2 run my_first_pkg kill turtle2 --ros-args -p x:=1` 처럼 실행하면
    sys.argv 에는 사용자 인자(turtle2)와 ROS 인자(--ros-args 이하)가 섞여 들어온다.
    ROS 인자는 rclpy.init() 이 처리하므로, argparse 에 넘길 사용자 인자만 골라낸다.

    remove_ros_args() 는 argv[0](프로그램 이름)을 유지한 채 ROS 인자만 제거한다.
    """
    argv = sys.argv if argv is None else argv
    return remove_ros_args(argv)[1:]


# ──────────────────────────────────────────────────────────────
# 노드 생성
# ──────────────────────────────────────────────────────────────

def make_tool_node(base_name: str) -> Node:
    """
    일회성 도구용 노드를 만든다.

    같은 도구를 두 터미널에서 동시에 실행해도 노드 이름이 겹치지 않도록
    프로세스 번호(PID)를 이름 뒤에 붙인다. ROS2 는 같은 이름의 노드가 둘 있으면 경고를 낸다.
    """
    return Node(f"{base_name}_{os.getpid()}")


def settle(node: Node, seconds: float = DISCOVERY_SETTLE_SEC) -> None:
    """
    그래프 정보가 도착할 시간을 준다.

    rclpy.spin_once() 를 짧게 반복하면 그 동안 DDS 발견(discovery) 메시지가 처리된다.
    time.sleep() 으로 기다리면 메시지가 처리되지 않으므로 spin 을 사용해야 한다.
    """
    end = node.get_clock().now().nanoseconds + int(seconds * 1e9)
    while rclpy.ok() and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.05)


# ──────────────────────────────────────────────────────────────
# turtle 목록
# ──────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class TurtleInfo:
    """turtle 하나의 관찰 결과. name 은 'turtle2' 같은 이름, has_driver 는 주행 노드 실행 여부."""
    name: str
    has_driver: bool


def _turtle_sort_key(name: str) -> tuple[int, str]:
    """turtle1, turtle2, turtle10 이 문자열 순서가 아니라 번호 순서로 정렬되게 한다."""
    m = TURTLE_NAME_PATTERN.match(name)
    return (int(m.group(1)), name) if m else (10**9, name)


def list_turtles(node: Node, wait: bool = True) -> list[str]:
    """
    현재 turtlesim 에 존재하는 turtle 이름을 돌려준다.

    판정 근거 = `/<이름>/pose` 토픽(타입 turtlesim/msg/Pose)의 존재.
    turtlesim_node 는 turtle 마다 pose 발행자를 하나씩 만들고, kill 하면 그 발행자를 없앤다.
    따라서 토픽 목록이 turtle 목록의 정본이다 — 이 도구 모음은 별도의 상태 파일을 두지 않는다.

    주의 — 발견 지연: 방금 만든 노드는 다른 노드의 토픽 정보를 아직 받지 못했을 수 있다.
    그래서 wait=True 이면 settle() 로 짧게 기다린 뒤 조회한다.
    """
    if wait:
        settle(node)
    names: list[str] = []
    for topic, types in node.get_topic_names_and_types():
        # topic 은 '/turtle1/pose' 형태. 앞의 '/' 를 떼고 '/' 로 나누면 ['turtle1', 'pose'].
        parts = topic.strip("/").split("/")
        if len(parts) == 2 and parts[1] == "pose" and POSE_TYPE in types:
            names.append(parts[0])
    return sorted(set(names), key=_turtle_sort_key)


def driver_node_name(turtle: str) -> str:
    """turtle 이름 → 그 turtle 을 주행시키는 노드의 이름. random_move.py 와 약속된 규칙이다."""
    return f"{turtle}_driver"


def enable_service_name(turtle: str) -> str:
    """turtle 이름 → 주행 노드가 제공하는 정지·재개 서비스의 이름. stop.py 와 random_move.py 가 공유한다."""
    return f"/{turtle}/driver/enable"


def list_turtles_with_driver(node: Node) -> list[TurtleInfo]:
    """turtle 목록에 '주행 노드가 실행 중인가'를 덧붙인다. 노드 목록(get_node_names)으로 판정한다."""
    turtles = list_turtles(node)                     # 안에서 settle() 수행
    running_nodes = set(node.get_node_names())
    return [TurtleInfo(t, driver_node_name(t) in running_nodes) for t in turtles]


def print_turtle_list(infos: list[TurtleInfo]) -> None:
    """목록을 표 형태로 출력한다. 모든 도구가 같은 모양으로 출력하도록 한곳에 둔다."""
    if not infos:
        print("현재 turtle 이 없습니다. (turtlesim_node 가 실행 중인지 확인)")
        return
    print(f"현재 turtle {len(infos)}개체 (상한 {MAX_TURTLES})")
    print(f"  {'이름':<10}  주행 노드")
    for info in infos:
        state = "실행 중" if info.has_driver else "없음"
        print(f"  {info.name:<10}  {state}")


# ──────────────────────────────────────────────────────────────
# 이름 확정
# ──────────────────────────────────────────────────────────────

def resolve_name(node: Node, requested: Optional[str]) -> str:
    """
    도구 인자로 받은 이름을 확정한다.

    - 이름이 주어지면 존재 여부를 확인하고 그대로 돌려준다.
    - 이름이 없거나 존재하지 않으면 현재 목록을 출력하고 TurtleToolError 를 일으킨다.
      → 사용자는 목록을 보고 이름을 붙여 다시 실행한다. (대화형 입력은 하지 않는다)
    """
    infos = list_turtles_with_driver(node)
    names = [i.name for i in infos]

    if requested is None:
        print_turtle_list(infos)
        raise TurtleToolError("대상 turtle 이름을 인자로 지정하십시오. 예: turtle2")

    if requested not in names:
        print_turtle_list(infos)
        raise TurtleToolError(f"'{requested}' 라는 turtle 이 없습니다.")

    return requested


# ──────────────────────────────────────────────────────────────
# 서비스 호출
# ──────────────────────────────────────────────────────────────

def call_service(node: Node, client: Client, request, timeout_sec: float = SERVICE_WAIT_SEC):
    """
    서비스를 동기식으로 한 번 호출하고 응답을 돌려준다.

    순서
      1. wait_for_service — 서버가 나타날 때까지 기다린다(없으면 TurtleToolError).
      2. call_async       — 요청을 보내고 Future 를 받는다. 이 시점에는 응답이 없다.
      3. spin_until_future_complete — Future 가 완료될 때까지 이 노드를 spin 한다.
         (spin 을 하지 않으면 응답 메시지가 처리되지 않아 영원히 기다린다)

    일회성 도구는 타이머·구독이 없으므로 이 방식이 가장 단순하다.
    상주 노드(random_move.py)는 이 방식을 쓰면 다른 콜백이 막히므로 다른 구조를 쓴다.
    """
    if not client.wait_for_service(timeout_sec=timeout_sec):
        raise TurtleToolError(f"서비스 '{client.srv_name}' 를 찾을 수 없습니다. (제공 노드가 실행 중인지 확인)")

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)

    if not future.done():
        raise TurtleToolError(f"서비스 '{client.srv_name}' 응답이 {timeout_sec:.0f}초 안에 오지 않았습니다.")
    if future.exception() is not None:
        raise TurtleToolError(f"서비스 '{client.srv_name}' 호출 실패: {future.exception()}")
    return future.result()


# ──────────────────────────────────────────────────────────────
# 도구 실행 골격
# ──────────────────────────────────────────────────────────────

def run_tool(body) -> int:
    """
    일회성 도구의 공통 골격.

      rclpy.init → body(argv) 실행 → rclpy.shutdown

    body 는 사용자 인자 리스트를 받아 작업을 수행하는 함수. TurtleToolError 가 나오면
    메시지만 출력하고 종료 코드 1 을 돌려준다. 성공하면 0.
    각 도구의 main() 이 `return run_tool(_body)` 한 줄로 끝나는 이유가 이 함수다.
    """
    rclpy.init(args=sys.argv)
    try:
        body(user_args())
        return 0
    except TurtleToolError as e:
        print(f"[오류] {e}", file=sys.stderr)
        return 1
    finally:
        # 예외가 나도 반드시 정리한다. try_shutdown 은 이미 종료된 상태에서도 예외를 내지 않는다.
        rclpy.try_shutdown()
