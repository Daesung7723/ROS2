"""
list — 현재 turtle 목록과 주행 노드 실행 여부를 출력한다

[이 파일이 다루는 개념]
  ROS2   : 토픽 목록·노드 목록으로 시스템 상태를 읽는다(내성). 상태 파일 없이 "지금 무엇이 있는가"를 묻는다.
  Python : 가장 작은 도구 — common 의 함수를 조립하기만 한다. 읽기 순서의 첫 파일.

[실행]
  ros2 run my_first_pkg list

[관찰할 것]
  터미널을 하나 더 열어 `ros2 topic list` · `ros2 node list` 를 실행하고,
  이 도구의 출력이 그 두 명령의 어떤 항목에서 나오는지 대응시켜 본다.
"""

from my_first_pkg.common import (
    list_turtles_with_driver,
    make_tool_node,
    print_turtle_list,
    run_tool,
)


def _body(argv: list[str]) -> None:
    # 이 도구는 인자를 받지 않는다. 무엇이 들어오든 무시하지 않고 알려 준다.
    if argv:
        print(f"[안내] list 는 인자를 사용하지 않습니다. 무시함: {argv}")

    node = make_tool_node("turtle_list")
    try:
        infos = list_turtles_with_driver(node)   # 내부에서 발견 대기 후 조회
        print_turtle_list(infos)
    finally:
        node.destroy_node()


def main() -> int:
    return run_tool(_body)


if __name__ == "__main__":
    raise SystemExit(main())
