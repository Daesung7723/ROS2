"""
spawn — 새 turtle 을 생성한다 (이름 필수 · 상한 3개체)

[이 파일이 다루는 개념]
  ROS2   : 서비스 클라이언트 — turtlesim/srv/Spawn 요청·응답
           응답 검증 — 서비스가 "성공적으로 실패"할 수 있다(응답은 왔으나 내용이 실패)
           이름 규칙 — turtle 이름은 토픽·서비스 이름의 일부(/leo/cmd_vel)가 된다
  Python : argparse 필수 위치 인자 + 선택 위치 인자(nargs) · 정규식 검증 · random

[읽기 전 알아야 할 것]
  Day 2 §2.3 (CLI 로 spawn 해 본 경험) · common.py

[실행]
  ros2 run my_first_pkg spawn leo                   # 이름 leo · 위치·방향 무작위
  ros2 run my_first_pkg spawn leo 2.0 8.0 1.57      # 이름 + x y theta(라디안)
  ros2 run my_first_pkg spawn                       # 이름 없음 → 사용법 출력 후 종료

[관찰할 것]
  실행 전후로 `ros2 topic list` 를 비교한다 — /leo/cmd_vel · /leo/pose 등 새 turtle 의 토픽이 한 번에 생긴다.
  같은 이름으로 한 번 더 실행해 거절 메시지를 확인한다. 상한(3개체)에 도달한 뒤에도 확인한다.
"""

import argparse
import math
import random
import re

from turtlesim.srv import Spawn

from my_first_pkg.common import (
    MAX_TURTLES,
    TurtleToolError,
    call_service,
    list_turtles,
    make_tool_node,
    run_tool,
)

# turtlesim 창의 좌표 범위는 0 ~ 약 11.09. 가장자리에 붙어 생성되지 않도록 여유를 둔다.
FIELD_MIN, FIELD_MAX = 1.0, 10.0

# 이름 규칙. turtle 이름은 토픽·서비스 이름의 일부(/leo/cmd_vel · /leo/set_pen)가 되므로
# ROS2 이름 규칙을 따라야 한다 — 영문자로 시작, 영문·숫자·밑줄만. 공백·하이픈·한글은 토픽 이름에 쓸 수 없다.
NAME_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")


def _parse(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="ros2 run my_first_pkg spawn",
        description="새 turtle 을 생성한다. 이름은 필수, 위치를 생략하면 무작위.",
    )
    # 필수 위치 인자. 생략하면 argparse 가 사용법을 출력하고 종료 코드 2 로 끝난다.
    parser.add_argument("name", help="turtle 이름 (영문자 시작 · 영문/숫자/밑줄)")
    # nargs='*' : 0개 또는 여러 개. 0개(생략) 아니면 정확히 3개(x y theta)여야 한다 — 아래서 검사.
    parser.add_argument("pose", nargs="*", type=float, metavar="X Y THETA",
                        help="x y theta(라디안). 셋 다 쓰거나 모두 생략.")
    args = parser.parse_args(argv)

    if not NAME_PATTERN.match(args.name):
        parser.error(f"'{args.name}': 이름은 영문자로 시작하고 영문·숫자·밑줄만 쓸 수 있습니다.")
    if len(args.pose) not in (0, 3):
        parser.error("위치는 x y theta 세 값을 모두 쓰거나 모두 생략해야 합니다.")
    return args


def _body(argv: list[str]) -> None:
    args = _parse(argv)
    name = args.name

    node = make_tool_node("turtle_spawn")
    try:
        # 1) 현재 목록 확인 → 상한·중복 이름 판정
        existing = list_turtles(node)
        if len(existing) >= MAX_TURTLES:
            raise TurtleToolError(
                f"turtle 이 이미 {len(existing)}개체입니다 (상한 {MAX_TURTLES}). "
                f"현재: {', '.join(existing)} — kill 로 하나를 제거한 뒤 다시 실행하십시오."
            )
        if name in existing:
            # turtlesim 의 spawn 은 중복 이름을 조용히 실패시키므로(아래 4단계) 호출 전에 먼저 거른다.
            raise TurtleToolError(f"'{name}' 은(는) 이미 있습니다. 현재: {', '.join(existing)}")

        # 2) 요청 메시지 구성. 필드 이름은 `ros2 interface show turtlesim/srv/Spawn` 으로 확인할 수 있다.
        req = Spawn.Request()
        if args.pose:
            req.x, req.y, req.theta = args.pose
        else:
            req.x = random.uniform(FIELD_MIN, FIELD_MAX)
            req.y = random.uniform(FIELD_MIN, FIELD_MAX)
            req.theta = random.uniform(0.0, 2.0 * math.pi)
        req.name = name

        # 3) 호출. /spawn 은 turtlesim_node 가 제공하는 전역 서비스(turtle 별 서비스가 아님).
        client = node.create_client(Spawn, "/spawn")
        result = call_service(node, client, req)

        # 4) 응답 검증. spawn 은 실패해도 예외를 내지 않고 name 을 빈 문자열로 돌려준다.
        #    1단계에서 걸렀더라도, 조회와 호출 사이에 다른 터미널이 같은 이름을 만들 수 있으므로 여기서 다시 확인한다.
        if result.name != name:
            raise TurtleToolError(
                f"생성 실패 — 응답 name='{result.name}'. 같은 이름이 이미 있거나 turtlesim 이 거절했습니다."
            )

        print(f"생성: {name}  x={req.x:.2f}  y={req.y:.2f}  theta={req.theta:.2f}")
        print(f"→ 주행시키려면:  ros2 run my_first_pkg random_move {name}")
    finally:
        node.destroy_node()


def main() -> int:
    return run_tool(_body)


if __name__ == "__main__":
    raise SystemExit(main())
