"""
spawn — 새 turtle 을 생성한다 (상한 3개체 · 이름 자동 부여)

[이 파일이 다루는 개념]
  ROS2   : 서비스 클라이언트 — turtlesim/srv/Spawn 요청·응답
           응답 검증 — 서비스가 "성공적으로 실패"할 수 있다(응답은 왔으나 내용이 실패)
  Python : argparse 선택 인자(nargs) · random · f-string 서식

[읽기 전 알아야 할 것]
  Day 2 §2.3 (CLI 로 spawn 해 본 경험) · common.py

[실행]
  ros2 run my_first_pkg spawn                  # 위치·방향 무작위
  ros2 run my_first_pkg spawn 2.0 8.0 1.57     # x y theta(라디안) 지정

[관찰할 것]
  실행 전후로 `ros2 topic list` 를 비교한다 — 새 turtle 의 토픽 5개가 한 번에 생긴다.
  상한에 도달한 뒤 한 번 더 실행해 거절 메시지를 확인한다.
"""

import argparse
import math
import random

from turtlesim.srv import Spawn

from my_first_pkg.common import (
    MAX_TURTLES,
    TurtleToolError,
    call_service,
    list_turtles,
    make_tool_node,
    next_turtle_name,
    run_tool,
)

# turtlesim 창의 좌표 범위는 0 ~ 약 11.09. 가장자리에 붙어 생성되지 않도록 여유를 둔다.
FIELD_MIN, FIELD_MAX = 1.0, 10.0


def _parse(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="ros2 run my_first_pkg spawn",
        description="새 turtle 을 생성한다. 위치를 생략하면 무작위.",
    )
    # nargs='*' : 0개 또는 여러 개. 0개(생략) 아니면 정확히 3개(x y theta)여야 한다 — 아래서 검사.
    parser.add_argument("pose", nargs="*", type=float, metavar="X Y THETA",
                        help="x y theta(라디안). 셋 다 쓰거나 모두 생략.")
    args = parser.parse_args(argv)
    if len(args.pose) not in (0, 3):
        parser.error("위치는 x y theta 세 값을 모두 쓰거나 모두 생략해야 합니다.")
    return args


def _body(argv: list[str]) -> None:
    args = _parse(argv)

    node = make_tool_node("turtle_spawn")
    try:
        # 1) 현재 개체 수 확인 → 상한 판정 → 이름 결정
        existing = list_turtles(node)
        name = next_turtle_name(existing)
        if name is None:
            raise TurtleToolError(
                f"turtle 이 이미 {len(existing)}개체입니다 (상한 {MAX_TURTLES}). "
                f"현재: {', '.join(existing)} — kill 로 하나를 제거한 뒤 다시 실행하십시오."
            )

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
