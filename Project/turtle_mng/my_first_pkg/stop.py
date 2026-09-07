"""
stop — 주행 노드를 일시정지하거나(--resume 으로) 재개한다. turtle 과 노드는 그대로 남는다.

[이 파일이 다루는 개념]
  ROS2   : 노드 간 제어 채널 설계 — 주행 노드가 std_srvs/srv/SetBool 서비스를 열어 두고,
           이 도구는 그 서비스의 클라이언트가 된다. "명령을 보내는 쪽"과 "받는 쪽"의 약속(서비스 이름·타입)이
           common.enable_service_name() 한곳에 있다.
  Python : 불 플래그를 서비스 요청으로 번역하는 얇은 도구

[읽기 전 알아야 할 것]
  random_move.py 의 서비스 서버 부분(_on_enable)

[실행]
  ros2 run my_first_pkg stop leo            # 정지
  ros2 run my_first_pkg stop leo --resume   # 재개
  ros2 run my_first_pkg stop                    # 목록만 출력

[관찰할 것]
  정지 직후 `ros2 topic echo /leo/cmd_vel --once` 를 실행한다 — 메시지가 오지 않고 대기하는 것이 정상이다
  (주행 노드가 발행을 멈췄으므로). 재개 후 같은 명령을 실행하면 바로 한 건이 출력된다.
"""

import argparse

from std_srvs.srv import SetBool

from my_first_pkg.common import (
    TurtleToolError,
    call_service,
    enable_service_name,
    make_tool_node,
    resolve_name,
    run_tool,
)


def _parse(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="ros2 run my_first_pkg stop", description="주행 일시정지 / 재개")
    parser.add_argument("name", nargs="?", help="대상 turtle 이름 (생략 시 목록 출력)")
    parser.add_argument("--resume", action="store_true", help="정지 대신 재개")
    return parser.parse_args(argv)


def _body(argv: list[str]) -> None:
    args = _parse(argv)

    node = make_tool_node("turtle_stop")
    try:
        name = resolve_name(node, args.name)

        req = SetBool.Request()
        req.data = args.resume                 # True = 주행 허용, False = 정지

        client = node.create_client(SetBool, enable_service_name(name))
        try:
            result = call_service(node, client, req, timeout_sec=2.0)
        except TurtleToolError:
            # 서비스가 없다 = 그 turtle 의 주행 노드가 실행 중이 아니다. 원인을 사용자 언어로 다시 쓴다.
            raise TurtleToolError(
                f"'{name}' 의 주행 노드가 실행 중이 아닙니다. "
                f"먼저:  ros2 run my_first_pkg random_move {name}"
            )

        # SetBool 응답 = success(bool) + message(string). message 는 서버가 채운 설명 문장.
        verb = "재개" if args.resume else "정지"
        print(f"{verb}: {name} — {result.message}")
    finally:
        node.destroy_node()


def main() -> int:
    return run_tool(_body)


if __name__ == "__main__":
    raise SystemExit(main())
