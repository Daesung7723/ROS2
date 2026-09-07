"""
kill — turtle 을 제거한다. 그 turtle 의 주행 노드는 스스로 종료한다.

[이 파일이 다루는 개념]
  ROS2   : 서비스 클라이언트 — turtlesim/srv/Kill
           느슨한 결합 — 이 도구는 주행 노드를 알지 못한다. 주행 노드가 turtle 의 소멸을 감지해 스스로 끝난다.
  Python : 인자 검증을 공용 함수(resolve_name)에 위임하는 구조

[읽기 전 알아야 할 것]
  Day 2 §2.3 · common.py · (주행 노드의 자기 종료는 random_move.py 의 watchdog 참조)

[실행]
  ros2 run my_first_pkg kill leo
  ros2 run my_first_pkg kill            # 이름 없음 → 목록만 출력하고 종료

[관찰할 것]
  random_move 를 실행한 터미널을 보이게 두고 kill 을 실행한다.
  1초 안에 그 터미널의 노드가 "turtle 이 사라져 종료" 메시지를 내고 끝나는 것을 확인한다.
  `ros2 node list` 에서도 사라졌는지 본다.
"""

import argparse

from turtlesim.srv import Kill

from my_first_pkg.common import call_service, make_tool_node, resolve_name, run_tool


def _parse(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="ros2 run my_first_pkg kill", description="turtle 을 제거한다.")
    parser.add_argument("name", nargs="?", help="제거할 turtle 이름 (생략 시 목록 출력)")
    return parser.parse_args(argv)


def _body(argv: list[str]) -> None:
    args = _parse(argv)

    node = make_tool_node("turtle_kill")
    try:
        name = resolve_name(node, args.name)   # 없으면 목록 출력 후 TurtleToolError

        req = Kill.Request()
        req.name = name
        client = node.create_client(Kill, "/kill")
        call_service(node, client, req)        # Kill 의 응답은 빈 메시지 — 검증할 필드가 없다

        print(f"제거: {name}")
        print("→ 주행 노드가 실행 중이었다면 곧 스스로 종료합니다.")
        print("→ 화면의 궤적은 남습니다. 지우려면:  ros2 service call /clear std_srvs/srv/Empty")
    finally:
        node.destroy_node()


def main() -> int:
    return run_tool(_body)


if __name__ == "__main__":
    raise SystemExit(main())
