# Day 2 — turtlesim 종합 제어와 rclpy 첫 코드

**2026-08-31 · 한국폴리텍대학교 하이테크과정 ROS2**

이 자료는 수업에서 진행한 실습 절차와 명령어를 정리한 것입니다. 복습 기준 = 이 자료 + 수업 중 필기.

---

## 목차

1. [오늘의 목표](#1-오늘의-목표)
2. [터미널 배치와 메시지 관찰](#2-터미널-배치와-메시지-관찰)
3. [통신 메커니즘 — 토픽·서비스·액션](#3-통신-메커니즘--토픽서비스액션)
4. [서비스 CLI 실습](#4-서비스-cli-실습)
5. [액션 CLI 실습](#5-액션-cli-실습)
6. [rclpy — 코드로 진입](#6-rclpy--코드로-진입)
7. [발행·구독 스크립트](#7-발행구독-스크립트)
8. [미니프로젝트 — 도형 궤적 자동 주행](#8-미니프로젝트--도형-궤적-자동-주행)
9. [다음 시간](#9-다음-시간)

---

## 1. 오늘의 목표

| 구간 | 내용 |
|------|------|
| 오전 | **통신 메커니즘 3종 완성** — 토픽(Day 1)에 이어 서비스·액션을 개념부터 CLI(Command Line Interface) 제어까지 |
| 오후 | **첫 코드** — 패키지·빌드 없이 `.py` 파일 하나로 rclpy 노드 실행 |

Day 1과의 관계:

| Day 1 내용 | 오늘과의 관계 |
|------|------|
| 환경 구축 — 강의실 PC + WSL2 + Ubuntu 24.04 + Jazzy | 오늘의 개발 기반 |
| 노드·토픽 개념 + 서비스·액션 이름만 확인 | 오늘 서비스·액션을 **본격 학습·제어** |
| CLI 체계 `ros2 <대상> <동작>` + `topic pub`로 원 궤적 | 같은 체계로 `service`·`action` 대상 사용 |
| (도전) 정사각형 궤적 — 명령 반복 입력·오차 누적 | 미니프로젝트에서 **코드로 개선** |

**왜 코드인가** — CLI는 사람이 입력할 때만 동작합니다. 자율 동작은 사람 없이 스스로 판단하고 발행하는 코드를 필요로 합니다.

수업 시작 시 환경 점검:

```bash
ros2 doctor                              # 설치 상태 재확인
ros2 run turtlesim turtlesim_node        # turtlesim 실행 유지 (오늘 내내 사용)
```

---

## 2. 터미널 배치와 메시지 관찰

### 2.1 모니터 터미널

**오늘부터 터미널 하나를 토픽 모니터 전용으로 고정합니다.**

| # | 역할 | 실행 |
|:--:|------|------|
| 1 | 시뮬레이터 | `ros2 run turtlesim turtlesim_node` — 종료하지 않고 유지 |
| 2 | **모니터** | `ros2 topic echo /turtle1/cmd_vel` — **오늘 내내 유지** |
| 3 | 작업 | CLI 명령·서비스 호출·스크립트 실행 |
| 4 | 여분 | 두 번째 스크립트 실행·`node list` 등 조회 |

- 목적 — 지금까지는 "명령 → 화면 변화"만 확인했으나, 모니터가 있으면 **명령 → 메시지 → 화면 변화**의 중간 단계가 보입니다
- 진단 수단 — turtle이 움직이지 않을 때 **모니터에 메시지가 흐르는지**로 원인이 둘로 나뉩니다. 흐르면 수신 측 문제, 흐르지 않으면 발행 측 문제입니다. Day 8 실물 차량의 1차 점검도 같은 방식입니다

터미널 4개를 각각의 창으로 띄우면 전환이 번거로우므로 분할 도구를 사용합니다:

```bash
sudo apt install terminator      # 미설치 시
terminator                       # 실행 후 Ctrl+Shift+E(세로 분할)·Ctrl+Shift+O(가로 분할)
```

> **자주 하는 실수 —** 모든 토픽을 모니터에 걸지 않습니다. `/turtle1/pose`는 약 60Hz(Hertz — 초당 반복 횟수)로 발행되므로 `echo`를 걸면 화면이 순식간에 넘어가 판독할 수 없습니다. 주기가 빠른 토픽은 `ros2 topic hz`로 주기만 확인합니다.

### 2.2 메시지 필드 확인 — 타입 → 구조 → 값

새 메시지·서비스 타입이 처음 나올 때마다 아래 3단을 수행합니다. 이미 확인한 타입은 생략합니다.

| 단계 | 명령 | 확인하는 것 |
|:--:|------|------|
| ① 타입 | `ros2 topic list -t` · `ros2 service type <이름>` | 무슨 타입인가 |
| ② 구조 | `ros2 interface show <타입>` | 어떤 **필드**로 구성되는가 |
| ③ 값 | `ros2 topic echo <토픽>` | 지금 어떤 **값**이 흐르는가 |

- Day 1에서 `Twist`·`Pose`에 ①②를 이미 수행했습니다. 오늘은 서비스·액션 타입으로 확장합니다(서비스는 요청·응답 구조라 ③ 대신 호출 결과로 확인)
- 이후 적용 — Day 4 카메라 이미지 · Day 8 초음파 거리 · Day 3 커스텀 인터페이스

---

## 3. 통신 메커니즘 — 토픽·서비스·액션

### 3.1 세 가지 방식

| 방식 | 성격 | 예 |
|------|------|------|
| **토픽** (Day 1) | 지속 데이터 흐름 — 발행/구독 | 속도 명령·센서 값 |
| **서비스** | 1회성 요청-응답 | 지정 좌표로 즉시 이동 |
| **액션** | 장시간 작업 — 목표·피드백·결과 | 지정 각도까지 회전(진행 상황 보고) |

**서비스** — 클라이언트가 요청을 보내면 서버가 처리 후 응답을 돌려줍니다.

- 서버 1 : 클라이언트 N 구조 — 복수 클라이언트가 같은 서비스를 호출할 수 있으며, **응답은 요청한 쪽에만** 전달됩니다(토픽의 1:N 방송과 다른 점)
- 처리가 끝날 때까지 클라이언트는 대기합니다

**액션** — 완료까지 시간이 걸리는 작업을 위한 방식으로, 세 가지를 주고받습니다.

| 구성 | 내용 |
|------|------|
| 목표(goal) | 무엇을 할 것인가 |
| 피드백(feedback) | 진행 상황 — 작업 도중 계속 전달 |
| 결과(result) | 최종 산출 |

목표 상태의 흐름 — `ACCEPTED` → `EXECUTING` → `SUCCEEDED` / `CANCELED` / `ABORTED`

- **진행 중인 작업을 취소할 수 있다**는 점이 토픽·서비스에는 없는 액션의 성질입니다

### 3.2 선택 기준

| 상황 | 선택 |
|------|------|
| 값이 계속 흐름 (센서·속도 명령) | 토픽 |
| 한 번 요청하고 결과를 즉시 받음 | 서비스 |
| 완료까지 시간이 걸리고 진행 상황을 알아야 함 | 액션 |

- 파라미터는 별도 방식이 아니라 **서비스 위에 구현**된 기능입니다 (Day 3)

### 3.3 종합 비교

| 항목 | 토픽 | 서비스 | 액션 |
|------|------|------|------|
| 연속성 | 연속 | 일회성 | 복합(토픽+서비스) |
| 방향성 | 단방향 | 양방향 | 양방향 |
| 동기성 | 비동기 | 동기 | 동기 + 비동기 |
| 다자간 연결 | 1:1 · 1:N · N:1 · N:N | 1:1 (서버:클라이언트) | 1:1 (서버:클라이언트) |
| 노드 역할 | 발행자 · 구독자 | 서버 · 클라이언트 | 서버 · 클라이언트 |
| 동작 트리거 | **발행자** | **클라이언트** | **클라이언트** |
| 인터페이스 | msg | srv | action |
| CLI 명령 | `ros2 topic` | `ros2 service` | `ros2 action` |
| 사용 예 | 센서 데이터·로봇 상태·속도 명령 | LED 제어·모터 토크 On/Off·경로 계산 | 목적지 이동·물건 파지·복합 작업 |

- **동작 트리거의 차이가 설계 판단의 기준**입니다 — 토픽은 데이터를 가진 쪽이 내보내고, 서비스·액션은 필요한 쪽이 요청합니다

### 3.4 인터페이스 파일 — 구분자로 판별

| 항목 | msg | srv | action |
|------|------|------|------|
| 확장자 | `.msg` | `.srv` | `.action` |
| 구분자 `---` | 없음 | **1개** | **2개** |
| 구성 | 데이터 | 요청 / 응답 | 목표 / 결과 / 피드백 |
| 예 | `geometry_msgs/msg/Twist` | `turtlesim/srv/Spawn` | `turtlesim/action/RotateAbsolute` |

- `interface show` 출력에서 **구분자 개수만 보면 어느 종류인지 판별**할 수 있습니다
- 직접 정의하는 방법은 Day 3에서 다룹니다

---

## 4. 서비스 CLI 실습

### 4.1 서비스 조회

```bash
ros2 service list                                    # 서비스 목록 조회
ros2 service type /turtle1/teleport_absolute         # 서비스가 쓰는 타입 확인
ros2 interface show turtlesim/srv/TeleportAbsolute   # 타입의 요청/응답 구조 출력
```

- 위 두 명령이 **필드 확인 3단의 서비스 판**입니다 — ① `service type`으로 타입 → ② `interface show`로 구조
- `interface show` 출력의 `---` 구분선 — **위 = Request(보낼 것) / 아래 = Response(받을 것)**

타입을 알 때 역방향으로 서비스를 찾는 명령:

```bash
ros2 service find std_srvs/srv/Empty      # 그 타입을 쓰는 서비스 전부 나열 → /clear·/reset
ros2 service find turtlesim/srv/Kill      # → /kill
```

### 4.2 순간 이동

```bash
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 2, y: 2, theta: 1.57}"
```

- `service call` — 요청을 보내고 응답을 받는 동작 (`<서비스 이름> <타입> "<요청 데이터>"`)
- 호출 즉시 지정 좌표로 이동합니다 — 흐름이 아니라 단발 처리라는 점에서 토픽 발행과 다릅니다
- 상대 좌표판 `teleport_relative`(전진 거리·회전량)도 같은 방식으로 호출합니다

### 4.3 turtle 생성·제거

```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 8, y: 8, theta: 3.14, name: 'turtle2'}"
ros2 topic list                                      # /turtle2/... 토픽 세트 등장 확인
ros2 service call /kill turtlesim/srv/Kill "{name: 'turtle2'}"
```

- **생성된 개체마다 토픽·서비스 세트 전체가 함께 생성됩니다**(`/turtle2/cmd_vel`·`/turtle2/pose` 등)
- 다중 제어 — `ros2 topic pub --rate 1 /turtle2/cmd_vel ...`로 두 개체를 각각 주행시킬 수 있습니다

### 4.4 화면·펜 제어

```bash
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{r: 255, g: 0, b: 0, width: 5, off: 0}"
ros2 service call /clear std_srvs/srv/Empty          # 배경 궤적 지우기 (요청 데이터 없음)
ros2 service call /reset std_srvs/srv/Empty          # 초기 상태로 리셋
```

- `std_srvs/srv/Empty` — **빈 타입**: 보낼 데이터 없이 "실행하라"만 전달하는 서비스의 표준형

### 4.5 종합 연습 — 다중 turtle 제어와 메시지 관찰

개체를 늘리는 것 자체가 아니라, **개체가 늘어날 때 노드·토픽·메시지가 어떻게 되는지를 관찰**하는 것이 목적입니다.

**① 개체 3개 구성**

```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 3, y: 3, theta: 0, name: 'turtle2'}"
ros2 service call /spawn turtlesim/srv/Spawn "{x: 8, y: 3, theta: 1.57, name: 'turtle3'}"
ros2 node list                                  # 노드는 몇 개인가
ros2 topic list                                 # 토픽 세트는 몇 벌인가
```

- 확인 — **노드는 `/turtlesim` 하나뿐인데 토픽 세트는 3벌**입니다. 개체 하나당 노드 하나가 아니라 한 노드가 세 개체의 토픽을 모두 담당합니다

**② 모니터 터미널로 명령의 도착 확인**

```bash
ros2 topic echo /turtle2/cmd_vel                # 모니터 터미널에서 대상만 교체
```

- 작업 터미널에서 `/turtle3/cmd_vel`에 발행하면 **모니터에는 아무것도 흐르지 않습니다** — 토픽 이름이 다르면 별개의 통로입니다
- 모니터 대상을 `/turtle3/cmd_vel`로 바꾸면 그때 메시지가 흐릅니다

**③ 필드 확인 3단 — Pose 메시지**

```bash
ros2 topic list -t                              # ① 타입 — /turtleN/pose = turtlesim/msg/Pose
ros2 interface show turtlesim/msg/Pose          # ② 구조 — 5개 필드
ros2 topic echo /turtle2/pose                   # ③ 값 — 몇 줄 확인 후 Ctrl+C
```

| 필드 | 의미 | 비고 |
|------|------|------|
| `x` · `y` | 화면상 위치 | 화면 좌표(0~11) |
| `theta` | 바라보는 방향 | rad — 오른손 법칙 |
| `linear_velocity` | 현재 직진 속도 | `Twist`의 `linear.x`에 대응 |
| `angular_velocity` | 현재 회전 속도 | `Twist`의 `angular.z`에 대응 |

- **`Twist`와의 차이** — Twist는 "이렇게 움직여라"(명령), Pose는 "지금 이렇다"(상태)입니다. 같은 물리량이라도 명령용 타입과 상태용 타입이 분리되어 있습니다

**④ 궤적 비교**

```bash
ros2 service call /turtle2/set_pen turtlesim/srv/SetPen "{r: 255, g: 0, b: 0, width: 3, off: 0}"
ros2 service call /turtle3/set_pen turtlesim/srv/SetPen "{r: 0, g: 0, b: 255, width: 3, off: 0}"
```

- 세 개체를 서로 다른 색·다른 속도로 주행시키고 궤적을 비교합니다
- `clear`로 정리한 뒤 `reset`과의 차이를 관찰합니다 (clear = 궤적만 / reset = 개체 구성까지 초기 상태로)

**관찰 정리**

| 관찰 | 확인한 것 |
|------|------|
| 노드 1 : 토픽 세트 3 | 노드와 개체는 1:1이 아님 — 한 노드가 여러 대상을 관리할 수 있음 |
| 토픽 이름 = 통로의 구분자 | 발행 대상이 다르면 다른 개체가 반응 — Day 3 remapping·Day 6 토픽 계약의 근거 |
| 명령 타입 ↔ 상태 타입 | `Twist`(명령) / `Pose`(상태) 분리 — Day 8 실물도 같은 구도 |

---

## 5. 액션 CLI 실습

### 5.1 목표 전송과 인터페이스

```bash
ros2 action list                                     # 액션 목록 조회
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"
ros2 action send_goal --feedback /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 0}"
```

- `action send_goal` — 목표를 보내고 완료까지 대기하는 동작 (`--feedback` = 진행 피드백을 화면에 계속 표시)
- 서비스와의 차이 — teleport(서비스)는 즉시 완료되지만, rotate(액션)는 **과정이 보입니다**

```bash
ros2 action list -t                                       # 액션 목록 + 타입 병기
ros2 interface show turtlesim/action/RotateAbsolute       # 목표·결과·피드백 구조 출력
```

출력은 `---` 구분자 2개로 세 부분으로 나뉩니다:

```
float32 theta        # 목표(goal)
---
float32 delta        # 결과(result)
---
float32 remaining    # 피드백(feedback)
```

| 필드 | 구분 | 의미 |
|------|------|------|
| `theta` | 목표 | 도달할 **절대 각도** |
| `delta` | 결과 | 액션 시작 위치로부터 실제로 회전한 각도 변위 |
| `remaining` | 피드백 | 목표까지 남은 각도 — `--feedback` 옵션을 써야 표시됨 |

- 관찰 포인트 — 회전 중 `remaining`이 줄어들다가, 도달 시 `delta`가 출력됩니다

### 5.2 목표 취소

turtlesim의 teleop(teleoperation — 원격 조작) 노드는 화살표 키와 별개로 **액션 목표를 보내는 키**를 제공합니다.

```bash
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
```

- 사용 키 = 화살표가 아니라 **F 키 주변의 G · B · V · C · D · E · R · T**
- 각 키는 `rotate_absolute` 액션의 목표 절대 각도를 전송합니다. `G`가 기준(theta = 0.0)이고 나머지는 위치에 따라 **0.7854rad(45°)씩 반시계 방향**입니다
- **`F` 키 = 전송한 목표를 취소** — 회전 도중 즉시 정지합니다

turtlesim 터미널의 로그로 결과를 확인합니다:

| 조작 | 로그 | 목표 상태 |
|------|------|------|
| 목표 각도에 도달 | `Rotation goal completed successfully` | SUCCEEDED |
| 도달 전 `F` 입력 | `Rotation goal canceled` | CANCELED |

- 자율차 대응 — "목표 지점까지 주행" 중 사람이 정지를 지시하는 경로가 이 구조입니다

### 5.3 CLI 총괄

| 방식 | 조회 | 실행 | 오늘 실행한 것 |
|------|------|------|------|
| 토픽 | `topic list`·`echo`·`hz`·`bw` | `topic pub` | 다중 turtle 주행 |
| 서비스 | `service list`·`type`·`find` | `service call` | teleport·spawn·set_pen·clear |
| 액션 | `action list -t` | `action send_goal` | rotate_absolute (피드백·목표 취소) |

- 전부 `ros2 <대상> <동작>` 체계입니다 — 새 대상이 나와도 같은 방법으로 접근할 수 있습니다

---

## 6. rclpy — 코드로 진입

### 6.1 rclpy와 계층 구조

- **rclpy**(ROS Client Library for Python) — ROS2 기능을 Python에서 사용하게 하는 공식 라이브러리
- C++은 rclcpp를 사용하며, 이 과정은 Python 단독입니다

```python
import rclpy                    # ROS2 Python 라이브러리
from rclpy.node import Node     # 노드 클래스의 부모
```

하나의 노드는 **내 코드와 라이브러리 계층의 결합체**입니다:

```
내 Python 코드 (응용 층)
  → rclpy (클라이언트 층)
    → RCL(ROS Client Library — 언어 공통 핵심 층)
      → RMW(ROS Middleware — 미들웨어 연결 층)
        → DDS(Data Distribution Service — 네트워크 전송 층)
```

- 내가 작성하는 것은 응용 층뿐이며 나머지는 라이브러리가 담당합니다
- Python 노드와 C++ 노드가 통신되는 이유 — **DDS 계층에서 연결되기 때문**입니다
- CLI(`ros2 topic pub`)도 내부적으로 같은 계층·경로를 사용합니다

### 6.2 C에서 Python으로 — 오늘 코드에 나오는 차이

| # | C | Python |
|:--:|---|---|
| 1 | `{ }`로 블록 구분 | **들여쓰기로 블록 구분** — 줄 끝 `:` 다음은 반드시 들여쓰기 |
| 2 | `int a = 10;` 형 선언 | **선언 없이 대입** — `a = 10`. 타입은 값이 정함(`2` = 정수 · `2.0` = 실수) |
| 3 | 배열 `int a[5]` | **리스트** `a = [1, 2, 3]` — 크기 가변·타입 혼합 가능 |
| 4 | 구조체 `struct` | **딕셔너리** `{'키': 값}` — 이름으로 값을 꺼냄 |
| 5 | `#include <stdio.h>` | **`import`** — `import 모듈` 또는 `from 모듈 import 이름` |

- 세미콜론이 없습니다 — **줄바꿈이 문장의 끝**입니다

> **자주 하는 실수 —** 정수 `2`와 실수 `2.0`은 다른 값입니다. ROS2 메시지 필드는 대부분 실수형이라 `msg.linear.x = 2`는 타입 오류가 됩니다.

### 6.3 최소 노드

```python
import rclpy                               # ROS2 Python 라이브러리
from rclpy.node import Node                # 노드 클래스의 부모 — ①에서 상속

class HelloNode(Node):                     # ① Node를 상속한 클래스
    def __init__(self):                    # ② 객체 생성 시 1회 자동 실행
        super().__init__('hello_node')     # ③ 부모(Node) 초기화 — 이름 등록
        self.count = 0                     # ④ self. = 객체 자신의 변수
        self.timer = self.create_timer(1.0, self.say_hello)

    def say_hello(self):                   # ⑤ 메서드 — 클래스 안의 함수
        self.count += 1
        self.get_logger().info(f'hello {self.count}')   # ⑥ f-string / 로그 출력

def main(args=None):
    rclpy.init(args=args)
    node = HelloNode()                     # ⑦ 객체 생성 — 이 순간 ②③④ 실행
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

| 표시 | 문법 | 의미 |
|:--:|------|------|
| ① | `class HelloNode(Node):` | **상속** — Node의 통신 기능(창구·타이머·로그)을 물려받은 클래스 |
| ② | `def __init__(self):` | 객체 생성 시 1회 자동 실행되는 초기화 |
| ③ | `super().__init__('hello_node')` | 부모의 초기화 실행 — 노드 이름 등록이 여기서 발생 |
| ④ | `self.count = 0` | **self** = 객체 자신 — 자기 변수·메서드 앞에 항상 붙음 |
| ⑤ | `def say_hello(self):` | **메서드** — 클래스 안에 정의된 함수 (첫 인자는 항상 self) |
| ⑥ | `f'hello {self.count}'` | **f-string** — 변수를 문자열에 삽입 |
| ⑦ | `node = HelloNode()` | **객체 생성** — 이 순간 ②~④가 실행됨 |

- `super().__init__('이름')`을 빠뜨리면 노드 이름 등록이 되지 않아 실행 즉시 오류가 발생합니다

### 6.4 노드 프로그램의 생애주기

```python
def main(args=None):
    rclpy.init(args=args)     # 1. 초기화
    node = HelloNode()        # 2. 노드 생성
    rclpy.spin(node)          # 3. 실행 유지 — 콜백 처리 루프
    node.destroy_node()       # 4. 정리
    rclpy.shutdown()          # 5. 종료
```

| 단계 | 코드 | 하는 일 | 빠뜨리면 |
|------|------|------|------|
| 초기화 | `rclpy.init()` | ROS2 통신 계층 준비 | 노드 생성 시 오류 |
| 생성 | `node = HelloNode()` | 이름 등록·통신 창구 준비 | — |
| **유지** | **`rclpy.spin(node)`** | **콜백을 대기·처리하는 반복 루프** | 프로그램 즉시 종료 — 발행·구독 동작 없음 |
| 정리 | `node.destroy_node()` | 통신 자원 해제 | 종료는 되나 정리 생략 |
| 종료 | `rclpy.shutdown()` | 통신 계층 종료 | — |

`spin`의 실체는 **대기·처리 반복문**입니다.

- `spin(node)` 호출 = 그 줄에서 프로그램이 멈춘 채 "콜백 발생 확인 → 실행 → 다시 대기"를 무한 반복
- Day 1에서 `turtlesim_node` 실행 후 터미널이 멈춘 것처럼 보인 이유가 이것입니다
- 종료 방법 = `Ctrl+C`

값 하나만 확인하고 다음 코드로 넘어가야 할 때는 `spin_once`를 사용합니다:

```python
rclpy.spin_once(node)                    # 대기 중인 콜백을 1회 처리하고 반환
rclpy.spin_once(node, timeout_sec=1.0)   # 1초 안에 콜백이 없으면 그대로 반환
```

- Day 4에서 카메라의 **한 프레임만** 받아 처리하는 구조가 이 형태입니다

### 6.5 콜백과 타이머

메시지가 언제 도착할지 모르는 상황에서 프로그램을 작성하는 두 가지 방식:

| 방식 | 동작 | 평가 |
|------|------|------|
| 폴링(polling) | 반복문에서 "메시지 도착했는가?"를 계속 검사 | 실행 흐름이 검사 반복에 고정됨 |
| **이벤트 구동(콜백)** | 함수를 등록해 두면 **사건 발생 시 시스템이 대신 호출** | 노드는 spin으로 대기만 — 여러 콜백 병행 등록 가능 |

**콜백(callback)** = 등록해 두면 사건이 발생할 때 자동으로 호출되는 함수입니다. ROS2 노드는 전부 이 방식입니다.

- **호출의 역전** — 위 예제 어디에도 `say_hello()`를 호출하는 줄이 없습니다. 내 코드는 **등록만** 하고, 호출은 spin이 수행합니다

| 사건 | 콜백 | 용도 |
|------|------|------|
| 구독한 토픽에 메시지 도착 | 구독 콜백 | 센서 값·위치 수신 처리 |
| 지정한 시간 경과 | **타이머 콜백** | **주기적 발행 — 제어 루프** |

- 타이머 주기 0.1초 = 초당 10회 실행(10Hz) — 로봇 제어 루프의 표준 형태입니다
- 주기 선택 기준 — 너무 길면 반응이 늦고, 너무 짧으면 연산 부하가 증가합니다 (이 과정 = 0.1~0.5초)

### 6.6 실행 — 패키지 없이 python3로

```bash
mkdir ~/ros2_scripts                       # 스크립트 보관 폴더
cd ~/ros2_scripts
# 텍스트 에디터로 hello_node.py 저장
python3 hello_node.py
```

- 1초마다 `hello 1`·`hello 2` 로그 출력을 확인한 뒤 `Ctrl+C`로 종료합니다
- **빌드·등록 없이 실행됩니다** — ROS2 환경(`source`)만 되어 있으면 rclpy를 바로 사용할 수 있습니다
- 새 터미널에서 `ros2 node list`를 실행하면 `/hello_node`가 표시됩니다
- 이 방식의 한계(배포·`ros2 run` 불가)는 Day 3의 패키지가 해결합니다

---

## 7. 발행·구독 스크립트

### 7.1 발행 스크립트 — circle_driver.py

코드 작성 전에 사양을 먼저 정합니다:

| 항목 | 값 | 근거 |
|------|------|------|
| 노드 이름 | `circle_driver` | 기능을 알 수 있는 이름 |
| 발행 토픽 | `/turtle1/cmd_vel` | Day 1에서 관찰한 속도 명령 토픽 |
| 메시지 타입 | `geometry_msgs/msg/Twist` | 그 토픽의 타입 |
| 발행 주기 | 0.5초 (2Hz) | 원 주행에 충분한 주기 |
| 발행 값 | linear.x = 2.0 · angular.z = 1.0 | 반지름 2의 원 |

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist        # 발행할 메시지 타입 — 타입 경로의 /가 .으로

class CircleDriver(Node):
    def __init__(self):
        super().__init__('circle_driver')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)   # 발행 창구 (10 = 대기열 크기)
        self.timer = self.create_timer(0.5, self.timer_callback)          # 콜백은 이름만 전달 (괄호 없음)

    def timer_callback(self):
        msg = Twist()                      # 매 호출마다 새 메시지 생성
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CircleDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

> **Tip —** 메시지 객체의 구조가 분명하지 않으면 값을 넣기 전에 그대로 출력해 확인합니다.
>
> ```python
> msg = Twist()
> print(msg)      # linear·angular 아래에 각각 x·y·z가 있음이 그대로 출력됨
> ```

> **자주 하는 실수**
>
> - `create_timer(0.5, self.timer_callback())`처럼 **괄호를 붙이면** 등록이 아니라 즉시 호출이 되어 오류가 발생합니다. 콜백은 이름만 전달합니다
> - `msg.linear.x = 2`처럼 정수를 대입하면 타입 오류가 발생합니다. Twist의 필드는 실수형이므로 `2.0`으로 씁니다
> - `msg = Twist()`를 콜백 밖에 두면 이전 값이 남습니다. 매 호출마다 새로 생성합니다

### 7.2 실행과 관찰

```bash
python3 circle_driver.py                   # turtlesim이 원 궤적 주행 시작
```

새 터미널에서:

```bash
ros2 node list                             # /circle_driver 등장
ros2 topic info /turtle1/cmd_vel           # 발행 수 1 = 내 노드
ros2 topic echo /turtle1/cmd_vel           # 0.5초 간격 linear.x 2.0 — 설계 값 확인
rqt_graph                                  # circle_driver → /turtle1/cmd_vel → turtlesim
```

- Day 1 미니프로젝트의 `topic pub --rate` 명령과 동일한 동작입니다 — **발행 주체가 CLI에서 내 코드로 교체**된 것입니다

### 7.3 발행 대기열과 QoS

`create_publisher(Twist, '/turtle1/cmd_vel', 10)`의 세 번째 인자에 대한 설명입니다.

| 표기 | 정체 |
|------|------|
| `10` | **대기열 크기** — 상대가 즉시 수신하지 못할 때 보관하는 메시지 개수 |
| 실제 의미 | **QoS**(Quality of Service — 통신 품질 설정)의 축약 표기. 깊이만 지정하고 나머지는 기본값 적용 |

```bash
ros2 topic info /turtle1/cmd_vel --verbose    # 발행자·구독자별 QoS 설정 상세 출력
```

| 항목 | 기본값 | 의미 |
|------|------|------|
| Reliability | RELIABLE | 도착 보장 — 유실 시 재전송 |
| Durability | VOLATILE | 늦게 연결된 구독자에게 과거 메시지를 전달하지 않음 |
| History | KEEP_LAST | 최근 N개만 보관 |
| Depth | 10 | 그 N — 코드의 세 번째 인자가 지정한 값 |

- 대기열이 넘치면 오래된 것부터 폐기됩니다. 속도 명령은 최신 값이 유효하므로 이 동작이 적절합니다
- 자율차 대응 — 센서 데이터는 최신 값이 중요하고(작은 depth·유실 허용), 지도·설정값은 유실되면 안 됩니다

### 7.4 구독 스크립트 — pose_printer.py

발행과 통신 방향이 반대입니다. turtlesim이 **발행하는** 위치를 내가 **구독**합니다.

| 항목 | circle_driver | pose_printer |
|------|------|------|
| 역할 | 명령을 **보냄** | 상태를 **받음** |
| 통신 | `/turtle1/cmd_vel` **발행** | `/turtle1/pose` **구독** |
| 콜백 | 타이머 콜백 (주기 발행) | **구독 콜백** (메시지 도착 시) |
| 자율차 대응 | 모터 제어 노드의 원형 | 센서 수신 노드의 원형 (Day 8 초음파) |

```python
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PosePrinter(Node):
    def __init__(self):
        super().__init__('pose_printer')
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

    def pose_callback(self, msg):
        self.get_logger().info(f'x={msg.x:.2f}  y={msg.y:.2f}  theta={msg.theta:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = PosePrinter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

- `create_subscription(타입, 토픽, 콜백, 10)` — 구독 창구를 생성하며, 메시지 도착마다 `pose_callback`이 자동 호출됩니다
- `pose_callback(self, msg)`의 **msg = 도착한 메시지 객체**입니다
- `get_logger().info()` — ROS 표준 로그 출력. `print` 대신 사용하면 시간·노드명이 함께 기록되어 여러 노드 동시 실행 시 구분할 수 있습니다
- 타이머가 없는 이유 — 발행 시점은 내가 정하지만 **수신 시점은 상대가 정하기** 때문입니다

### 7.5 두 스크립트 동시 실행

터미널 2개로 `python3 circle_driver.py`와 `python3 pose_printer.py`를 동시에 실행합니다.

```
circle_driver ──/turtle1/cmd_vel──▶ turtlesim ──/turtle1/pose──▶ pose_printer
```

- 내 노드가 양쪽에서 turtlesim과 통신합니다 — 명령은 보내고 상태는 받는 구조입니다
- 자율차 대응 — 모터 제어 / 센서 수신이며, **둘을 한 노드에 합치면 판단 노드**가 됩니다

### 7.6 rqt 도구군

지금까지의 관찰은 전부 터미널 출력이었습니다. rqt는 같은 정보를 화면에 표시하는 플러그인 모음입니다.

```bash
rqt        # 통합 실행 — Plugins 메뉴에서 선택
```

| 플러그인 | 메뉴 경로 | 용도 |
|------|------|------|
| rqt_graph | Introspection > Node Graph | 노드·토픽 연결 관계 |
| rqt_console | Logging > Console | 로그 수집·필터 |
| rqt_plot | Visualization > Plot | 수치 토픽의 시간 변화 그래프 |
| Topic Monitor | Topics > Topic Monitor | 토픽 목록·타입·현재 값·발행 주기를 표로 동시 관찰 |
| Message Type Browser | Topics > Message Type Browser | 메시지 타입의 내부 구조 열람 |
| Service Caller | Services > Service Caller | 서비스를 화면에서 호출 — 타입을 고르면 입력란이 자동 생성 |

- 다만 **CLI를 먼저 익히는 것이 기준**입니다 — 실물 로봇은 화면 없이 원격으로 연결하는 경우가 많습니다(Day 8~9)

로그 수준:

```python
self.get_logger().info('정상 진행')
self.get_logger().warn('경계 근접')
self.get_logger().error('위치 수신 중단')
```

- rqt_plot 실습 — Topic 입력란에 `/turtle1/pose/x`·`/turtle1/pose/y`를 추가하고 circle_driver를 실행하면 두 값이 주기적으로 진동하는 그래프가 나타납니다(원 궤적의 수치 표현)

---

## 8. 미니프로젝트 — 도형 궤적 자동 주행

**문제 상황** — Day 1 도전 5에서는 직진 명령과 회전 명령을 사람이 번갈아 입력해 정사각형을 근사했습니다. 입력 시점이 수동이라 느리고 각도 오차가 누적됩니다.

**오늘의 해결** — 노드가 스스로 구간을 전환합니다. 타이머 콜백 안에 전환 논리를 배치합니다.

핵심 개념 = **상태 전환**: 노드가 현재 상태를 기억하고, 조건이 충족되면 다음 상태로 이동합니다.

```
직진 상태(linear.x = 2.0) ──2초 경과──▶ 회전 상태(angular.z = 1.57)
        ▲                                        │
        └────────────── 1초 경과 ─────────────────┘
```

구현 힌트:

| 필요한 것 | 방법 |
|------|------|
| 시간 경과 측정 | 타이머 주기 0.1초 + **틱 계수 변수**(`self.tick`) — 20틱 = 2초 |
| 현재 상태 기억 | **상태 변수**(`self.moving` — True 직진 / False 회전) |
| 상태 전환 | 콜백에서 틱이 목표에 도달하면 상태 반전 + 틱 초기화 |
| 상태별 발행 값 | 직진 = linear.x만 / 회전 = angular.z만 |

**문제** (`~/ros2_scripts/`에 작성 후 `python3`로 실행)

1. **정사각형 궤적 노드** `square_driver.py` — 직진 구간과 회전 구간을 타이머 콜백에서 전환
2. (도전) 회전 각도를 바꿔 **다른 다각형**(삼각형·육각형) 궤적으로 일반화
3. (도전) `pose_printer`를 확장해 **경계 접근 시 자동 정지**하는 노드 — 구독(위치)과 발행(속도)을 한 노드에서 결합

<details>
<summary><b>문제 (1) 정답 및 풀이 보기</b></summary>

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SquareDriver(Node):
    def __init__(self):
        super().__init__('square_driver')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.tick = 0          # 0.1초 단위 경과 계수
        self.moving = True     # True = 직진 구간 / False = 회전 구간

    def timer_callback(self):
        msg = Twist()
        if self.moving:
            msg.linear.x = 2.0             # 직진 2초 = 길이 4
            if self.tick >= 20:
                self.moving = False
                self.tick = 0
        else:
            msg.angular.z = 1.57           # 1.57rad/s × 1초 ≈ 90°
            if self.tick >= 10:
                self.moving = True
                self.tick = 0
        self.tick += 1
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SquareDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

- 구조 — 타이머(0.1초)마다 현재 구간을 확인해 직진/회전 명령을 발행하고, 구간 시간이 경과하면 상태를 전환합니다
- Day 1 CLI 방식과의 차이 — 사람이 명령을 번갈아 입력하지 않아도 노드가 스스로 전환합니다
- 시간 기반이라 오차가 누적됩니다. 각도를 정확히 맞추려면 위치(theta) 기반 제어가 필요합니다

</details>

<details>
<summary><b>문제 (2)·(3) 도전 정답 및 풀이 보기</b></summary>

**(2) 다각형 일반화** — 회전 구간의 목표 각도를 `2π ÷ 변 수`로 변경합니다:

```python
# 삼각형 = 외각 120° : angular.z = 2.09 (1초 기준)
# 육각형 = 외각 60°  : angular.z = 1.05 (1초 기준)
```

**(3) 경계 접근 시 자동 정지** — 구독(인식) + 발행(행동)의 결합:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class SafeDriver(Node):
    def __init__(self):
        super().__init__('safe_driver')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.safe = True

    def pose_callback(self, msg):
        self.safe = 1.5 < msg.x < 9.5 and 1.5 < msg.y < 9.5

    def timer_callback(self):
        msg = Twist()
        if self.safe:
            msg.linear.x = 2.0
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafeDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

- 구조 = **인식(구독 콜백이 안전 여부 갱신) → 판단(safe 변수) → 행동(타이머 콜백이 속도 발행)**
- 이 3단 구조가 최종 프로젝트 자율차의 최소형입니다. Day 3 미로가 이 정지를 회피로 확장하고, Day 6(표지판 반응)·Day 9(장애물 회피)가 같은 뼈대를 이어받습니다

</details>

---

## 9. 다음 시간

**Day 3 — 패키지·colcon + 멀티 노드 미로 자율주행**

- 워크스페이스·패키지 생성과 colcon 빌드 — 오늘 만든 스크립트를 실행 체계에 등록
- 파라미터 — 값을 코드에 고정하지 않고 실행 시점에 주입
- 멀티 노드 구성과 launch — 여러 노드를 한 번에 기동
- 미니프로젝트 — 미로 자율주행

오늘 작성한 `~/ros2_scripts/`의 스크립트(circle_driver·pose_printer·square_driver)가 다음 시간에 패키지로 옮길 대상입니다.
