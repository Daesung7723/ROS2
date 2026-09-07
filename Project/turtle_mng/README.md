# turtle_mng — turtlesim 제어 도구 모음 (my_first_pkg 에 추가)

**ROS2 · Python 학습용 독립 미니프로젝트** · 한국폴리텍대학교 하이테크과정 ROS2 (2026-2)

수업 진도와 별개로 진행하는 과제입니다. 제출은 없습니다. 코드를 읽고, 실행하고, 고쳐 보는 것이 목적이며, 파일마다 다루는 개념과 관찰 방법을 주석으로 적어 두었습니다.

이 프로젝트는 새 패키지를 만들지 않습니다. **Day 3 에서 만든 `my_first_pkg` 에 실행 파일 6개를 추가**합니다 — 기존 `circle_driver`·`pose_printer`·`square_driver` 와 나란히 동작합니다.

---

## 1. 무엇을 만드는가

`ros2 run my_first_pkg <도구>` 로 실행되는 **명령행 도구 6개**입니다. 메뉴 프로그램이 아니라, 각 도구가 한 가지 일만 하고 종료합니다(`random_move` 만 계속 실행됩니다).

| 도구 | 실행 예 | 동작 |
|------|------|------|
| `list` | `ros2 run my_first_pkg list` | 현재 turtle 목록 + 주행 노드 실행 여부 |
| `spawn` | `ros2 run my_first_pkg spawn [x y theta]` | 새 turtle 생성. **상한 3개체**(turtle1 포함) · 이름 자동 부여(turtle2·turtle3) · 위치 생략 시 무작위 |
| `kill` | `ros2 run my_first_pkg kill turtle2` | turtle 제거. 그 turtle 의 주행 노드는 **스스로 종료** |
| `random_move` | `ros2 run my_first_pkg random_move turtle2` | turtle 하나를 무작위 주행시키는 **상주 노드**(터미널 하나 = turtle 하나) |
| `set_pen` | `ros2 run my_first_pkg set_pen turtle2 --r 255 --g 0 --b 0 --width 3` | 펜 색·두께 변경. `--off` 로 펜 올림 |
| `stop` | `ros2 run my_first_pkg stop turtle2` / `--resume` | 주행 일시정지·재개. turtle 과 노드는 유지 |

이름이 필요한 도구(`kill`·`random_move`·`set_pen`·`stop`)에서 이름을 생략하면 **현재 목록을 출력하고 종료**합니다. 목록을 보고 이름을 붙여 다시 실행합니다.

---

## 2. 설치 — 기존 my_first_pkg 에 추가하기

Day 3 §3 에서 스크립트를 패키지로 옮길 때 했던 절차와 같습니다: **파일 배치 → `setup.py` 등록 → 빌드**. 여기에 **`package.xml` 의존 선언**(Day 3 §5)이 한 단계 더 붙습니다.

### 2-1. 모듈 복사

```bash
cd ~
git clone https://github.com/Daesung7723/ROS2.git        # 이미 있으면 cd ROS2 && git pull
cp ROS2/Project/turtle_mng/my_first_pkg/*.py ~/ros2_ws/src/my_first_pkg/my_first_pkg/
ls ~/ros2_ws/src/my_first_pkg/my_first_pkg/
```

`common.py` `list_turtles.py` `spawn.py` `kill.py` `random_move.py` `set_pen.py` `stop.py` 7개가 기존 파일 옆에 놓여야 합니다. `__init__.py` 는 한 줄짜리 설명 문자열만 담고 있어 기존 빈 파일을 덮어써도 무방합니다.

### 2-2. `setup.py` — 실행 파일 등록

`~/ros2_ws/src/my_first_pkg/setup.py` 의 `entry_points` → `console_scripts` 목록에 아래 6줄을 **추가**합니다(기존 줄은 그대로 둡니다).

```python
        "list = my_first_pkg.list_turtles:main",
        "spawn = my_first_pkg.spawn:main",
        "kill = my_first_pkg.kill:main",
        "random_move = my_first_pkg.random_move:main",
        "set_pen = my_first_pkg.set_pen:main",
        "stop = my_first_pkg.stop:main",
```

> 왼쪽 = `ros2 run` 에서 부르는 이름, 오른쪽 = `모듈:함수`. `list` 도구의 모듈 이름이 `list_turtles` 인 이유는 Python 내장 함수 `list` 와 모듈 이름이 겹치지 않게 하기 위해서입니다.

### 2-3. `package.xml` — 의존 선언

`~/ros2_ws/src/my_first_pkg/package.xml` 에 아래 3줄을 추가합니다(`<exec_depend>rclpy</exec_depend>` 옆). 이미 있는 줄은 중복 추가하지 않습니다.

```xml
  <exec_depend>turtlesim</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>std_srvs</exec_depend>
```

### 2-4. 빌드·확인

```bash
cd ~/ros2_ws
colcon build --packages-select my_first_pkg
source install/setup.bash
ros2 pkg executables my_first_pkg
```

마지막 명령의 출력에 기존 실행 파일과 함께 `list` `spawn` `kill` `random_move` `set_pen` `stop` 이 나열되면 완료입니다.

> 완성된 `setup.py`·`package.xml` 의 예시는 `reference/` 폴더에 있습니다. 편집 결과를 대조할 때 참고하십시오. **`reference/` 를 통째로 `src/` 에 복사하지 마십시오** — Day 3 패키지와 이름이 같아 충돌합니다.

---

## 3. 실행 시나리오

터미널 4개를 사용합니다(terminator 분할 권장 — Day 2 §0.4).

| 터미널 | 명령 | 확인할 것 |
|:--:|------|------|
| ① | `ros2 run turtlesim turtlesim_node` | turtle1 이 중앙에 생성됨 |
| ② | `ros2 run my_first_pkg list` | turtle1 · 주행 노드 없음 |
| ② | `ros2 run my_first_pkg spawn` ×2 | turtle2·turtle3 생성. **세 번째 spawn 은 거절**된다 |
| ③ | `ros2 run my_first_pkg random_move turtle2` | turtle2 가 무작위 주행. 이 터미널은 계속 점유됨 |
| ④ | `ros2 run my_first_pkg random_move turtle3` | turtle3 도 주행 |
| ② | `ros2 run my_first_pkg list` | turtle2·turtle3 = 주행 노드 실행 중 |
| ② | `ros2 run my_first_pkg set_pen turtle2 --r 255 --g 0 --b 0 --width 5` | turtle2 의 궤적이 빨간 굵은 선으로 바뀜 |
| ② | `ros2 run my_first_pkg stop turtle3` | turtle3 정지. 터미널 ④ 에 "주행 정지" 로그 |
| ② | `ros2 run my_first_pkg stop turtle3 --resume` | turtle3 재개 |
| ② | `ros2 run my_first_pkg kill turtle2` | turtle2 사라짐. **1~2초 뒤 터미널 ③ 의 노드가 스스로 종료** |
| ② | `ros2 node list` | `turtle2_driver` 가 없고 `turtle3_driver` 만 남음 |

---

## 4. 설계 — 왜 이렇게 만들었는가

### 4-1. 상태를 저장하는 프로그램이 없다

"현재 turtle 이 몇 개인가"를 기억하는 파일·데몬을 두지 않았습니다. 모든 도구는 실행될 때마다 **토픽 목록을 읽어** `/turtle2/pose` 같은 토픽이 있는지로 turtle 의 존재를 판정합니다(`common.list_turtles`). turtlesim 이 곧 정본이므로 도구와 실제 상태가 어긋날 여지가 없습니다.

대신 **발견 지연**이 생깁니다. 노드를 만든 직후에는 다른 노드의 정보가 아직 도착하지 않았을 수 있어, 조회 전에 짧게 spin 하며 기다립니다(`common.settle`). `time.sleep()` 으로는 메시지가 처리되지 않으므로 spin 을 사용해야 한다는 점이 이 함수의 요지입니다.

### 4-2. kill 은 주행 노드를 알지 못한다

`kill` 도구는 turtlesim 에 제거 요청만 보냅니다. 주행 노드(`random_move`)는 **자신이 담당하는 turtle 의 pose 발행자 수를 0.5초마다 확인**하고, 연속 3회 0 이면 사라진 것으로 판정해 스스로 종료합니다(`RandomMover._watch`).

두 프로그램이 서로를 몰라도 동작하는 이 구조가 ROS2 의 기본 설계 방식입니다. 반대로 `kill` 이 주행 노드의 프로세스를 직접 종료하도록 만들면, 두 프로그램은 항상 같은 컴퓨터·같은 사용자로 실행되어야 한다는 제약이 생깁니다.

### 4-3. stop 은 서비스를 경유한다

주행 노드는 `/turtle2/driver/enable`(`std_srvs/srv/SetBool`) 서비스를 제공하고, `stop` 도구는 그 클라이언트입니다. 서비스 이름과 노드 이름의 규칙은 `common.enable_service_name` · `common.driver_node_name` **한곳**에 있습니다 — 보내는 쪽과 받는 쪽이 같은 함수를 import 하므로 규칙이 어긋날 수 없습니다.

정지 시 주행 노드는 0 속도를 한 번 발행합니다. turtlesim 은 새 명령이 약 1초간 없으면 스스로 멈추지만, 실물 모터(Day 8)에서는 이 한 줄이 필수이므로 시뮬레이션에서도 같은 습관을 유지합니다.

### 4-4. 이름은 자동으로 정한다

turtlesim 의 `spawn` 은 이미 있는 이름을 요청하면 **응답은 정상으로 오되 `name` 이 빈 문자열**입니다. 예외가 아니라 응답 내용으로 실패를 알리는 서비스이므로 반드시 검증해야 합니다(`spawn.py` 4단계). 이름을 사용자가 입력하도록 두면 이 실패 경로가 열리므로, 도구가 비어 있는 가장 작은 번호를 골라 붙입니다.

### 4-5. 새 패키지를 만들지 않는다

도구 6개를 `my_first_pkg` 에 추가하는 이유는 두 가지입니다. ① 패키지 하나가 여러 실행 파일을 담는 것이 ROS2 의 보통 모습이며, 실행 파일마다 패키지를 만들지 않습니다. ② `setup.py` 에 줄을 추가하고 재빌드하는 과정 자체가 Day 3 §3 순환의 반복 연습입니다.

---

## 5. 코드 읽기 순서와 개념 지도

아래 순서로 읽으면 앞 파일의 개념이 다음 파일에서 재사용됩니다.

| 순서 | 파일 | ROS2 개념 | Python 개념 | 줄 수 |
|:--:|------|------|------|:--:|
| 1 | `common.py` | 그래프 내성(토픽·노드 목록) · 서비스 동기 호출 4단계 · `--ros-args` 분리 | 모듈 분리 · 타입 힌트 · `dataclass` · 사용자 정의 예외 · 정규식 | ~230 |
| 2 | `list_turtles.py` | 내성 결과의 표시 | 가장 작은 도구 — 함수 조립만 | ~40 |
| 3 | `spawn.py` | 전역 서비스(`/spawn`) · **응답 내용으로 실패를 판정** | `argparse` `nargs` · `random` | ~90 |
| 4 | `kill.py` | 느슨한 결합 — 상대 노드를 모른 채 동작 | 검증의 함수 위임 | ~60 |
| 5 | `set_pen.py` | turtle 별 네임스페이스 서비스 · `uint8` 범위 | `argparse` 옵션·플래그·`type` 검증 함수 | ~70 |
| 6 | `random_move.py` | 발행·구독·타이머 2개·**서비스 서버**·파라미터(선언·실행 중 읽기)·**자기 종료**·정상 종료 순서 | `Enum` 상태 기계 · 클래스 상속 · 콜백 · 종료 예외 처리 | ~230 |
| 7 | `stop.py` | 노드 간 제어 채널 = 서비스 | 플래그 → 요청 번역 | ~70 |
| — | `reference/setup.py` · `package.xml` | `entry_points` = `ros2 run` 이름 · `exec_depend` | 패키징 | — |

`random_move.py` 안에는 **실행 중 변경이 반영되는 파라미터**(속도·여유 폭 — 매 틱 읽음)와 **반영되지 않는 파라미터**(`period` — 타이머 생성 시 한 번 읽음)를 의도적으로 나란히 두었습니다. `ros2 param set` 으로 둘을 바꿔 보고 차이를 확인하십시오(Day 3 §4.5).

---

## 6. 관찰 포인트

| 명령 | 무엇을 보는가 |
|------|------|
| `ros2 topic list` | spawn 직후 토픽 5개(`cmd_vel`·`pose`·`color_sensor` + 서비스 내부 토픽)가 한 번에 생김 |
| `ros2 node list` | `turtle2_driver` 처럼 노드 이름이 turtle 이름을 따름. 일회성 도구는 이름 뒤에 PID 가 붙음 |
| `rqt_graph` | 주행 노드 → `/turtle2/cmd_vel` → `turtlesim` · `/turtle2/pose` → 주행 노드의 양방향 연결 |
| `ros2 service list \| grep driver` | 주행 노드가 제공하는 `enable` 서비스 |
| `ros2 param list /turtle2_driver` · `ros2 param set /turtle2_driver linear_max 0.5` | 실행 중 파라미터 변경 → 속도 즉시 반영 |
| `ros2 topic echo /turtle2/cmd_vel --once` | stop 상태에서는 대기, 재개 후에는 즉시 1건 출력 |
| `ros2 interface show turtlesim/srv/Spawn` · `SetPen` | 요청·응답 필드. `SetPen.off` 가 `uint8` 인 것 |

---

## 7. 자주 하는 실수

| 증상 | 원인 | 대응 |
|------|------|------|
| `No module named 'my_first_pkg'` 또는 `'turtle_mng'` | 모듈이 다른 패키지 폴더에 복사됨 / import 문의 패키지 이름 불일치 | 2-1 의 경로 확인 — `my_first_pkg/my_first_pkg/` 안에 있어야 함 |
| `No executable found` | `setup.py` 편집 후 재빌드하지 않음 | `colcon build --packages-select my_first_pkg` 재실행 |
| `Package 'my_first_pkg' not found` | `source install/setup.bash` 누락 | 빌드한 워크스페이스의 setup.bash 를 source |
| `list` 가 빈 목록을 출력 | turtlesim_node 미실행, 또는 `ROS_DOMAIN_ID` 가 터미널마다 다름 | ① 터미널에서 turtlesim 확인 · `echo $ROS_DOMAIN_ID` 비교 |
| `stop` 이 "주행 노드가 실행 중이 아닙니다" | 그 turtle 에 `random_move` 를 실행하지 않았음 | `random_move <이름>` 먼저 실행 |
| kill 후 주행 노드가 종료되지 않음 | 1.5초의 판정 지연 안에 확인함 | 2초 이상 기다린 뒤 `ros2 node list` |
| kill 후 궤적이 남음 | turtlesim 의 정상 동작 — 펜 자국은 turtle 과 별개 | `ros2 service call /clear std_srvs/srv/Empty` |
| spawn 이 거절됨 | 상한 3개체 도달 | `kill` 로 하나 제거 후 재실행 |

---

## 8. 연습 문제

제출하지 않습니다. 방향만 제시하고 정답은 두지 않습니다.

1. **`clear` 도구 추가** — `ros2 run my_first_pkg clear` 로 궤적을 지우는 도구. `std_srvs/srv/Empty` 를 사용합니다. `kill.py` 를 원형으로 삼으면 30줄이면 됩니다. `setup.py` 의 `entry_points` 에 등록하는 것을 잊지 마십시오.
2. **상한을 파라미터로** — `MAX_TURTLES` 가 상수라서 바꾸려면 코드를 고쳐야 합니다. `spawn` 이 `--ros-args -p max_turtles:=5` 를 받도록 바꾸십시오. (힌트: 일회성 도구의 노드에도 `declare_parameter` 를 쓸 수 있습니다.)
3. **주행 노드 일괄 기동 launch** — 현재 존재하는 모든 turtle 에 대해 `random_move` 를 한 번에 실행하는 launch 파일. launch 파일은 실행 시점에 turtle 목록을 알아야 하므로 `OpaqueFunction` 으로 목록을 조회해 `Node` 액션을 동적으로 만들어야 합니다(Day 3 §8 의 정적 launch 와 무엇이 다른지 정리해 보십시오).
4. **경계 회피 방식 교체** — 지금은 가장자리에서 후진 → 중앙 방향 회전 → 재주행입니다. 후진 없이 **벽에 평행하게 미끄러지듯 회전**하는 방식으로 바꾸고, 어느 쪽이 궤적을 더 넓게 덮는지 5분간 실행해 비교하십시오.
5. **자기 종료의 다른 근거** — `_watch` 는 pose 발행자 수를 확인합니다. pose **메시지의 마지막 수신 시각**(2초 이상 수신 없음)을 근거로 바꾸면 어떤 상황에서 판정이 달라지는지 생각해 보십시오. (turtlesim 을 통째로 종료한 경우·네트워크가 잠시 끊긴 경우)

---

## 9. 이전 판과 비교 읽기

이 저장소의 `Old_version/Codes/01_Trtl_Mng.py` + `01_turtle_driver_node.py` 는 같은 기능(생성·제거·펜·주행)을 **메뉴형 단일 매니저 + 서브프로세스** 구조로 구현한 2024~2025 판입니다. 두 판을 나란히 읽으면 구조 차이가 무엇을 바꾸는지 보입니다.

| 관점 | 이전 판 (Old_version) | 이 판 (turtle_mng) |
|------|------|------|
| 진입점 | 메뉴 루프 1개(`input()`) | 도구 6개 (`ros2 run`) |
| 상태 | 매니저의 `dict` 가 주행 프로세스 핸들을 보관 | 없음 — 매번 토픽 목록을 조회 |
| 주행 노드 기동 | 매니저가 `subprocess.Popen` | 사용자가 터미널에서 직접 실행 |
| 주행 노드 종료 | 매니저가 `terminate()` | 노드가 turtle 소멸을 감지해 자기 종료 |
| 입력과 spin 의 공존 | `MultiThreadedExecutor` + 별도 스레드 | 필요 없음 — 일회성 도구는 `spin_until_future_complete` 만 사용 |
| 패키지화 | 스크립트(파일 경로 의존) | ament_python 패키지의 실행 파일 |

이전 판이 잘못된 것은 아닙니다. 한 프로그램 안에서 모든 것을 제어해야 할 때(예: GUI 앱)는 이전 판의 구조가 맞습니다. 과제는 **어느 구조가 어떤 조건에서 유리한가**를 스스로 설명할 수 있게 되는 것입니다.

---

## 10. 파일 구조

```
Project/turtle_mng/
├─ README.md                ← 이 문서
├─ my_first_pkg/            ← ★ 이 폴더의 .py 를 여러분의 my_first_pkg/my_first_pkg/ 에 복사
│   ├─ __init__.py
│   ├─ common.py            ← 공용: 목록 조회·이름 확정·서비스 호출·도구 골격
│   ├─ list_turtles.py      ← list
│   ├─ spawn.py
│   ├─ kill.py
│   ├─ random_move.py       ← 상주 주행 노드
│   ├─ set_pen.py
│   └─ stop.py
└─ reference/               ← 완성된 패키징 파일 예시 (대조용 — src 에 복사하지 않음)
    ├─ setup.py             ← entry_points 6개가 등록된 형태
    ├─ setup.cfg
    ├─ package.xml          ← exec_depend 4개
    └─ resource/my_first_pkg
```

환경: Ubuntu 24.04 · ROS2 Jazzy · Python 3.12. 다른 배포판(Humble)에서도 API 차이 없이 동작하도록 작성했으나 검증은 Jazzy 에서만 수행했습니다.
