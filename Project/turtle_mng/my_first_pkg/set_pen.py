"""
set_pen — turtle 의 펜 색·두께·on/off 를 바꾼다

[이 파일이 다루는 개념]
  ROS2   : turtle 별 네임스페이스 서비스 — /turtle2/set_pen 처럼 이름이 경로에 들어간다
           uint8 필드 — 0~255 범위를 넘기면 메시지 생성 시점에 예외가 난다
  Python : argparse 옵션 인자(--r --g --b) · store_true 플래그 · 범위 검증용 type 함수

[읽기 전 알아야 할 것]
  Day 2 §2.4 (CLI 로 set_pen 해 본 경험)

[실행]
  ros2 run my_first_pkg set_pen turtle2 --r 255 --g 0 --b 0 --width 3
  ros2 run my_first_pkg set_pen turtle2 --off        # 펜을 든다(궤적을 그리지 않음)
  ros2 run my_first_pkg set_pen                       # 목록만 출력

[관찰할 것]
  `ros2 service list | grep set_pen` — turtle 개체 수만큼 서비스가 있다.
  `ros2 interface show turtlesim/srv/SetPen` — off 가 bool 이 아니라 uint8 인 것을 확인한다.
"""

import argparse

from turtlesim.srv import SetPen

from my_first_pkg.common import call_service, make_tool_node, resolve_name, run_tool


def _uint8(text: str) -> int:
    """argparse 의 type 으로 쓰는 검증 함수. 0~255 밖이면 argparse 가 오류 메시지를 만든다."""
    value = int(text)
    if not 0 <= value <= 255:
        raise argparse.ArgumentTypeError(f"{value}: 0~255 범위여야 합니다.")
    return value


def _parse(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="ros2 run my_first_pkg set_pen", description="펜 색·두께·on/off 설정")
    parser.add_argument("name", nargs="?", help="대상 turtle 이름 (생략 시 목록 출력)")
    parser.add_argument("--r", type=_uint8, default=255, help="빨강 0~255 (기본 255)")
    parser.add_argument("--g", type=_uint8, default=255, help="초록 0~255 (기본 255)")
    parser.add_argument("--b", type=_uint8, default=255, help="파랑 0~255 (기본 255)")
    parser.add_argument("--width", type=_uint8, default=3, help="선 두께 (기본 3)")
    parser.add_argument("--off", action="store_true", help="펜을 든다 — 궤적을 그리지 않음")
    return parser.parse_args(argv)


def _body(argv: list[str]) -> None:
    args = _parse(argv)

    node = make_tool_node("turtle_set_pen")
    try:
        name = resolve_name(node, args.name)

        req = SetPen.Request()
        req.r, req.g, req.b = args.r, args.g, args.b
        req.width = args.width
        req.off = 1 if args.off else 0        # uint8 — True/False 가 아니라 1/0

        # 서비스 이름에 turtle 이름이 들어간다. turtle 마다 별개의 서버가 있다.
        client = node.create_client(SetPen, f"/{name}/set_pen")
        call_service(node, client, req)

        state = "펜 올림(그리지 않음)" if args.off else f"RGB({args.r},{args.g},{args.b}) 두께 {args.width}"
        print(f"펜 설정: {name} → {state}")
    finally:
        node.destroy_node()


def main() -> int:
    return run_tool(_body)


if __name__ == "__main__":
    raise SystemExit(main())
