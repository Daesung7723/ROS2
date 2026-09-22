# Day 6 — 실물 모듈 · 초음파 센서와 DC모터

**2026-09-28 · 한국폴리텍대학교 하이테크과정 ROS2**

이 자료는 수업의 개념 설명과 실습 절차·명령어를 복습용으로 정리한 것입니다. 복습 기준 = 이 자료 + 수업 중 필기.

---

## 목차

1. [오늘의 목표](#1-오늘의-목표)
2. [RPi5 GPIO 구조](#2-rpi5-gpio-구조)
3. [실습 ① — 초음파 센서 결선](#3-실습---초음파-센서-결선)
4. [실습 ② — 거리 퍼블리셔](#4-실습---거리-퍼블리셔)
5. [DC모터와 L298N](#5-dc모터와-l298n)
6. [실습 ③ — 모터 결선과 동작 확인](#6-실습---모터-결선과-동작-확인)
7. [실습 ④ — 모터 제어 노드](#7-실습---모터-제어-노드)
8. [미니프로젝트 — 수동 주행 차량](#8-미니프로젝트--수동-주행-차량)
9. [오늘의 요약](#9-오늘의-요약)
10. [보충](#10-보충)
11. [다음 시간](#11-다음-시간)

---

## 1. 오늘의 목표

Day 4·5는 RPi5에서 **카메라 영상과 AI 분류**를 다루었습니다. 오늘은 RPi5의 **GPIO 핀에 실물 부품을 연결**합니다 — 처음으로 코드로 실물 부품을 직접 제어합니다.

지난 시간과의 관계:

| 지난 내용 | 오늘과의 관계 |
|------|------|
| Day 1 자료 7장 · Day 2 자료 7장 — turtle에 `Twist` 발행 | 초음파 값으로 turtle을 움직여 확인(4.5) · 오후에는 같은 `Twist`로 모터 구동 |
| Day 3 자료 7장 — 커스텀 인터페이스 `my_msgs` | 거리 메시지 `DistMsg`를 같은 절차로 정의 — **RPi5에는 새로 만듦**(4.2) |
| Day 4 자료 3장 — 원격 데스크톱 연결 | turtlesim 창은 원격 데스크톱 화면에서 실행(Day 5 자료 2.5) |
| Day 5 자료 3장 — RPi5의 `my_car_pkg` | 오늘 작성하는 노드를 모두 이 패키지에 추가 |

오늘의 목적은 **부품마다 노드를 만들어 동작을 확인하는 것**입니다. 부품을 결합해 센서 값에 따라 자율 주행하는 차량은 Day 8(turtlesim 통합)·Day 9(실물 통합)에서 완성합니다.

| | 지금까지 | 오늘 |
|---|---|---|
| 행동 대상 | 화면 속 `turtle` | 오전 = turtle · 오후 = **바퀴 달린 차량** |
| 거리 입력 | 없음 | **초음파 센서 실측값** |
| 센서 값 확인 | — | `topic echo` + **turtle의 움직임**(4.5) |
| `Twist` 명령의 구독자 | turtlesim | turtlesim → **모터 노드**(7장) |

**오늘 완성할 것**
- ① 초음파 거리를 토픽으로 발행하는 **거리 퍼블리셔**(4.3)
- ② 거리 값에 반응하는 **turtlesim 확인 노드**(4.5)
- ③ 전진·후진·좌회전·우회전을 함수로 구현한 **동작 시험 노드**(6.4)
- ④ `/cmd_vel`을 바퀴 구동으로 변환하는 **모터 제어 노드**(7장)
- ⑤ ①과 ④를 결합한 **키보드 수동 주행 차량**(8장)

> **오늘의 순서 원칙**
>
> **결선이 끝나지 않은 상태에서 코드로 넘어가지 않습니다.** 부품마다 ⓐ 결선 → ⓑ 육안 점검 → ⓒ ROS2 없이 단품 스크립트로 통전 확인 → ⓓ ROS2 노드 작성의 순서를 지킵니다. 코드 오류와 배선 오류가 섞이면 원인을 찾을 수 없습니다.

오늘의 흐름 — **오전 = 초음파(센서) / 오후 = 모터(구동)**, 부품을 하나씩 완결하며 진행합니다.

| 구간 | 단계 | 장 | 내용 | 산출물 |
|:--:|:--:|:--:|------|------|
| 오전 | ① GPIO | 2 | RPi5 GPIO 구조 · gpiozero | 핀 번호 읽는 법 |
| 오전 | ② 초음파 결선 | 3 | HC-SR04 원리 · 분압 · 결선 · 통전 확인 | 동작하는 센서 |
| 오전 | ③ 거리 퍼블리셔 | 4 | `DistMsg` 정의 · `us_node` · **turtlesim으로 확인** | `/us_dist` 발행 · 반응하는 turtle |
| 오후 | ④ 모터 이론 | 5 | DC모터 · PWM · L298N · 전원 설계 | 결선 계획 |
| 오후 | ⑤ 모터 결선 | 6 | 결선 · 점검 · 단품 통전 확인 · **동작 함수** | 4방향으로 회전하는 바퀴 |
| 오후 | ⑥ 모터 노드 | 7 | `/cmd_vel` → 좌우 바퀴 변환 · 확인 5항 | `motor_node` |
| 오후 | ⑦ 미니프로젝트 | 8 | 키보드 수동 주행 | **움직이는 차량** |

- 9장 요약 이후의 **10장 보충**은 진도에 여유가 있을 때 다루는 독립 내용입니다

### 1.1 준비물 확인

| 품목 | 수량 | 확인 사항 |
|------|:--:|------|
| RPi5 (Day 4·5 상태 유지) | 1 | ROS2·`my_car_pkg` 동작 · 원격 데스크톱 연결(turtlesim 창) |
| **HC-SR04 초음파 센서 모듈** | 1 | 핀 4개 — VCC·Trig·Echo·GND · 이전 수업에서 사용한 모듈 |
| **PiCar-R5 차량 키트** | 1 | 조립 완료 여부 · 자동차 제어보드 · DC 모터 4 · 바퀴 4 |
| **7.4V 배터리** (키트 구성품) | 1 | **모터 전원** · 충전 상태 |
| 브레드보드·점퍼선 | 적정 | 수-암·수-수 혼용 |
| **저항 1kΩ · 2kΩ** | 각 1 | Echo 분압용(3.2) |

> **오늘 하루의 공통 원칙 — 전원을 끄고 배선합니다**
>
> **전원이 연결된 상태에서 선을 꽂거나 뽑으면 부품·보드가 손상될 수 있습니다.** 모든 배선 작업은 RPi5 전원과 배터리를 분리한 상태에서 수행하고, 육안 점검을 마친 뒤에 전원을 연결합니다.

---

## 2. RPi5 GPIO 구조

### 2.1 GPIO — 코드와 부품의 접점

**GPIO**(General-Purpose Input/Output — 범용 입출력) = 코드에서 값을 읽고 쓸 수 있는 **물리 핀**. RPi5 보드 가장자리의 40핀이 그것입니다.

| 핀 종류 | 개수 | 역할 |
|------|:--:|------|
| GPIO | 26 | 입력(센서 신호 읽기) 또는 출력(신호 내보내기) — 코드로 제어 |
| 5V 전원 | 2 | 부품 전원 공급 — HC-SR04가 사용 |
| 3.3V 전원 | 2 | 저전압 부품 전원 |
| **GND**(ground — 접지) | 8 | 전기 회로의 기준점 — **모든 부품과 공유** |

> **자주 하는 실수**
>
> **GPIO 핀의 허용 전압은 3.3V입니다.** 5V 신호를 GPIO 입력에 직접 연결하면 핀이 손상될 수 있습니다 — 3.2의 분압 회로가 이 문제의 해결책입니다.

**핀 번호 체계** — 물리 위치 번호와 GPIO 번호가 **다릅니다**:

| 체계 | 예 | 쓰임 |
|------|------|------|
| 물리 번호 | 1번(3.3V)·2번(5V)… 위치 순서 | 배선할 때 자리 찾기 |
| **BCM 번호**(Broadcom — 칩 기준 GPIO 번호) | GPIO23·GPIO24… | **코드에서 사용** — gpiozero의 기본 |

핀 배치는 명령으로 즉시 확인합니다:

```bash
pinout        # RPi 보드의 핀 배치도를 터미널에 출력 — gpiozero가 제공하는 도구
```

- 출력에서 물리 번호와 GPIO 번호의 대응을 확인 — 오늘 결선표(3.3·6.1)의 번호는 전부 **GPIO(BCM) 번호**이며, 배선할 때는 `pinout`으로 물리 위치를 찾습니다

### 2.2 RPi5의 특이점 — gpiozero를 사용하는 이유

RPi5는 GPIO 담당 회로가 **RP1이라는 별도 칩**으로 바뀌었습니다. 이전 세대에서 널리 쓰이던 라이브러리가 그대로 동작하지 않습니다.

| 라이브러리 | RPi5 |
|------|:--:|
| `RPi.GPIO` (구세대 표준) | ❌ **미동작** — RP1 구조를 지원하지 않음 |
| **`gpiozero`** (lgpio 백엔드) | ✅ **사용** — 부품 단위 클래스 제공 |

```bash
sudo apt install -y python3-gpiozero python3-lgpio
python3 -c "import gpiozero; print('ok')"      # 설치 확인
```

- gpiozero의 방식 — 핀 신호를 직접 다루지 않고 **부품 이름의 클래스**를 사용: `DistanceSensor`(초음파)·`Motor`(모터)·`LED` 등
- 인터넷 예제 중 `RPi.GPIO` 기반 코드는 RPi5에서 실행되지 않음 — **검색 결과를 선별하는 기준**으로 기억해 둘 것

### 2.3 확인 활동 — 개념 점검

결선에 들어가기 전 확인합니다.

| # | 질문 | 확인하려는 것 |
|:--:|------|------|
| 1 | GPIO 입력 핀에 5V 신호를 직접 연결하면? | 2.1 — 3.3V 한계·분압 필요 |
| 2 | 결선표의 "23"은 물리 23번 핀인가? | 2.1 — BCM 번호 ≠ 물리 번호·`pinout`으로 확인 |
| 3 | 인터넷의 `RPi.GPIO` 예제가 RPi5에서 실행되지 않는 이유는? | 2.2 — RP1 칩·gpiozero 사용 |
| 4 | 배선 작업 전 반드시 할 일은? | 1.1 — 전원 분리 |

---

## 3. 실습 ① — 초음파 센서 결선

### 3.1 HC-SR04 — 소리로 거리를 재는 원리

**HC-SR04** = 초음파를 쏘고 반사파가 돌아올 때까지의 **시간**으로 거리를 계산하는 센서.

| 순서 | 동작 |
|:--:|------|
| ① | 코드가 **Trig**(trigger) 핀에 짧은 신호를 보냄 |
| ② | 센서가 초음파(40kHz — 사람이 듣지 못하는 높은 소리)를 발사 |
| ③ | 반사파가 돌아오면 **Echo** 핀이 그 왕복 시간만큼 신호를 유지 |
| ④ | 시간 × 음속(약 340m/s) ÷ 2 = **거리** |

- ÷2인 이유 — 측정된 시간은 **왕복** 시간이므로
- 이 계산을 코드로 직접 작성하지 않습니다 — gpiozero의 `DistanceSensor`가 ①~④ 전체를 담당(3.4)

| 특성 | 값 | 영향 |
|------|------|------|
| 측정 범위 | 약 2cm ~ 4m | 지나치게 가까우면 측정 불가 |
| 측정 각도 | 약 15° | **정면의 한 방향만** — 여러 방향을 한 번에 측정하는 LiDAR와의 차이(Day 7에서 비교) |
| 반사면 | 단단한 면에 유리 | 천·경사면은 반사가 약해 값이 불안정(4.4에서 관찰) |

### 3.2 분압 회로 — Echo의 5V를 3.3V로

HC-SR04는 5V로 동작하므로 **Echo 출력도 5V**입니다. GPIO 허용 전압(3.3V — 2.1)을 넘으므로 저항 두 개로 전압을 나눠 연결합니다.

```
Echo ──[ 1kΩ ]──┬── GPIO24 (RPi5)
                │
              [ 2kΩ ]
                │
               GND
```

- 원리 — 5V가 두 저항에 비례 배분되어 GPIO에는 **5V × 2/(1+2) ≈ 3.3V**만 인가됨
- Trig는 분압이 필요 없음 — RPi5 → 센서 방향의 **출력**이며, 3.3V 신호로도 센서가 인식

### 3.3 결선표와 배선

**전원을 모두 분리한 상태**에서 아래 표대로 연결합니다.

| HC-SR04 | 연결 | RPi5 (BCM) |
|:--:|:--:|:--:|
| VCC | → | **5V** 핀 |
| Trig | → | **GPIO23** |
| Echo | → 1kΩ 경유 → | **GPIO24** (분압점) |
| GND | → | **GND** |
| (분압) 2kΩ | 분압점 → | GND |

**육안 점검** — 전원 연결 전에 확인:

| # | 점검 |
|:--:|------|
| 1 | VCC가 **5V 핀**에 연결되었는가 (3.3V 핀 아님 — 센서 동작 전압) |
| 2 | Echo가 **분압을 거쳐** GPIO24에 연결되었는가 (직결 금지) |
| 3 | GND가 RPi5 GND와 연결되었는가 |
| 4 | 저항 값이 맞는가 — 1kΩ(갈·검·빨) · 2kΩ(빨·검·빨) |

### 3.4 통전 확인 — ROS2 없이 단품으로

점검을 마쳤으면 전원을 연결하고, **ROS2 없이 gpiozero만으로** 센서가 동작하는지 확인합니다.

`~/us_test.py`:

```python
from gpiozero import DistanceSensor      # 초음파 센서 클래스 — 측정 절차 전체를 담당
from time import sleep

sensor = DistanceSensor(echo=24, trigger=23)    # BCM 번호로 지정
while True:
    print(f'{sensor.distance * 100:.1f} cm')    # distance는 m 단위 — cm로 환산 표시
    sleep(0.5)
```

```bash
python3 us_test.py      # 손을 가까이·멀리 하며 값 변화 확인 → Ctrl+C 종료
```

- 손을 움직일 때 값이 따라 변하면 → **단품 정상** · 4장으로 진행
- 값이 **100.0 부근에 고정**되면 → 반사파 미수신입니다. **전원을 분리하고 3.3 점검 2**(Echo 배선·분압)를 확인한 뒤 이 스크립트를 **다시** 실행
- 값이 **0 부근에 고정**되면 → Trig가 연결되지 않아 초음파가 발사되지 않는 상태입니다. **전원을 분리하고** Trig 배선을 확인한 뒤 **다시** 실행
- 실행 즉시 오류가 나오면 → gpiozero 미설치 또는 핀 번호 오기입니다. **2.2**의 설치·번호를 확인하고 **다시** 실행
- 값이 심하게 요동하면 → 반사면 문제입니다(3.1). **단단한 면을 정면에** 두고 다시 확인 → 그래도 요동하면 4장으로 진행하고 10.1(이동 평균)에서 다룹니다
- 위 조치로도 값이 나오지 않으면 → 교수에게 알림

- **단품이 동작해야 다음 단계로** — 여기서 확인된 배선은 4장 이후 의심 대상에서 제외됩니다

---

## 4. 실습 ② — 거리 퍼블리셔

### 4.1 계약 설계

노드를 작성하기 전에 **토픽 계약**을 먼저 정합니다.

- **토픽 계약** = 발행하는 노드와 구독하는 노드가 공유하는 약속 — 토픽 이름·메시지 타입·값의 의미와 단위·주기
- 계약이 같으면 **발행하는 노드나 구독하는 노드를 다른 노드로 바꾸어도 연결이 유지**됩니다(4.5에서 확인)

| 항목 | 값 |
|------|------|
| 토픽 이름 | `/us_dist` |
| 타입 | `my_msgs/msg/DistMsg` — 아래에서 정의 |
| 값의 의미 | 정면 장애물까지의 거리, **cm 단위** |
| 주기 | 10Hz (파라미터 `period` 0.1초) |
| 측정 불가일 때 | 상한값 부근이 발행됨 — 구독하는 노드가 상한 처리(4.4) |

> **단위를 계약에 명기합니다**
>
> ROS 표준 단위는 m입니다(Day 1 자료 2.3). 이 과정은 **cm를 사용**합니다 — 0~400의 정수 범위가 판독에 직관적이고, 2025년 검증 코드와 같은 규격이기 때문입니다. 표준과 다른 단위를 쓸 때의 규칙은 하나 — **메시지 정의와 계약에 단위를 명기**해 모든 노드가 같은 해석을 공유하는 것입니다(Day 3 자료 7.2).

### 4.2 DistMsg 정의 — RPi5에 인터페이스 패키지 만들기

> **선행 확인 — Day 3 자료 7장 커스텀 인터페이스**
>
> Day 3 자료 7장에서 **PC**(WSL)에 `my_msgs`를 만들었습니다. 빌드 결과는 만든 기기에 종속되므로(Day 5 자료 3.1) **RPi5에는 새로 만듭니다.** 절차는 Day 3 자료 7.2~7.4와 같습니다 — 그때의 필기와 Day 3 자료 7장을 참조합니다.

먼저 RPi5 워크스페이스에 무엇이 있는지 확인합니다:

```bash
ls ~/ros2_ws/src          # 패키지 폴더 목록
```

- `my_car_pkg`만 있으면 → 아래 **①부터** 수행(대부분 이 경우)
- `my_msgs`도 있으면 → **②부터** 수행(파일 추가·등록만)
- `my_car_pkg`가 없으면 → Day 5 자료 3.3의 절차로 **패키지를 먼저 만든 뒤** ①부터 수행

| 순서 | 작업 | 내용 |
|:--:|------|------|
| ① | 인터페이스 패키지 생성 | `ament_cmake` 패키지 `my_msgs` — 아래 명령 |
| ② | 메시지 정의 | `my_msgs/msg/DistMsg.msg` 작성 — 아래 한 줄 |
| ③ | `my_msgs/CMakeLists.txt` | `ament_package()` 위에 생성 규칙 2줄(Day 3 자료 7.3) |
| ④ | `my_msgs/package.xml` | 인터페이스 선언 3줄(Day 3 자료 7.3) |
| ⑤ | 사용하는 패키지의 의존 선언 | `my_car_pkg/package.xml`에 `<depend>my_msgs</depend>`(Day 3 자료 6장 규칙) |

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_msgs     # ① C++ 계열 빌드 타입 — 인터페이스 전용
mkdir my_msgs/msg
```

`my_msgs/msg/DistMsg.msg`:

```
float32 dist      # 정면 거리 [cm]
```

③ `CMakeLists.txt` — `ament_package()` 위:

```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "msg/DistMsg.msg")
```

④ `my_msgs/package.xml`:

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

- `rosidl`(ROS Interface Definition Language — 인터페이스 정의 언어) = `.msg` 파일을 Python·C++ 코드로 변환하는 생성 도구. ③④는 이 도구를 빌드에 연결하는 설정
- `my_msgs`가 이미 있었다면(② 시작) — ③의 `rosidl_generate_interfaces` 줄에 `"msg/DistMsg.msg"`를 **추가**합니다(메시지 파일을 추가할 때마다 이 줄에 파일명을 함께 적습니다 — Day 3 자료 7.3)

빌드하고 등록을 확인합니다:

```bash
cd ~/ros2_ws && colcon build          # my_msgs → my_car_pkg 순서로 빌드됨(⑤의 의존 선언)
source install/local_setup.bash
ros2 interface show my_msgs/msg/DistMsg
```

- `float32 dist`가 출력되면 → 4.3으로 진행
- 빌드 중 `rosidl` 관련 오류가 나오면 → ③의 두 줄과 ④의 세 줄을 확인하고 **빌드부터 다시** 실행
- 빌드는 성공했으나 `Unknown package 'my_msgs'`가 나오면 → `source install/local_setup.bash`를 실행하고 **`interface show`를 다시** 실행
- `.msg` 파일 이름이 소문자로 시작하면 생성이 실패합니다 → `DistMsg.msg`로 고치고 **빌드부터 다시** 실행

- 2025년 원 예제는 `rasp5_msg`라는 **별도 인터페이스 패키지**를 만들었습니다 — 이 과정은 `my_msgs`라는 이름을 사용합니다. 두 방식 모두 구조는 동일(인터페이스 전용 ament_cmake 패키지)

### 4.3 us_node — 거리 퍼블리셔 작성

`~/ros2_ws/src/my_car_pkg/my_car_pkg/us_node.py` — Day 5 자료 3장에서 만든 `my_car_pkg`의 **안쪽** 폴더에 작성합니다:

```python
import rclpy
from rclpy.node import Node
from my_msgs.msg import DistMsg
from gpiozero import DistanceSensor

class UsDistPublisher(Node):
    def __init__(self):
        super().__init__('us_dist_publisher')
        self.declare_parameter('period', 0.1)                     # ① 발행 주기(초)
        self.declare_parameter('max_cm', 200.0)                   # ② 측정 상한(cm)

        max_m = self.get_parameter('max_cm').value / 100.0
        self.sensor = DistanceSensor(echo=24, trigger=23,         # ③ 센서 객체 — 3.4와 같은 핀
                                     max_distance=max_m)
        self.pub = self.create_publisher(DistMsg, '/us_dist', 10)
        period = self.get_parameter('period').value
        self.timer = self.create_timer(period, self.on_timer)     # ④ 주기 발행
        self.get_logger().info('us_dist_publisher started')

    def on_timer(self):
        msg = DistMsg()
        msg.dist = self.sensor.distance * 100.0                   # ⑤ m → cm 환산
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = UsDistPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

| # | 하는 일 |
|:--:|------|
| ① | 발행 주기를 **파라미터로**(Day 3 자료 5.3) — 2025년 원 예제의 0.5초를 회피 제어에 맞춰 0.1초로 |
| ② | **측정 상한** — `DistanceSensor`는 `max_distance`(기본 1m)를 넘는 거리를 상한값으로 반환. 상한을 파라미터로 두어 코스에 맞게 조정 |
| ③ | 3.4 단품 스크립트와 **같은 핀 번호** — 단품에서 검증된 배선을 그대로 사용 |
| ④ | 타이머 주기 발행 — Day 2 자료 6.5의 제어 루프 형태 |
| ⑤ | m 단위(`distance`) → **cm 환산**(4.1 계약) |

`setup.py` 등록 후 빌드:

```python
'us_node = my_car_pkg.us_node:main',
```

> **자주 하는 실수**
>
> - 센서 객체를 콜백 안에서 만들면 매 주기 초기화가 반복되어 값이 출력되지 않습니다 — **`__init__`에서 한 번만** 생성합니다(Day 5 자료 10.3 모델 적재와 같은 원칙)
> - GPIO 접근 권한 오류가 나면 — 재로그인 후 재시도, 지속되면 `sudo` 실행으로 원인을 구분합니다

**코드 읽기 — Python 문법 ① 객체 생성과 인자** — Day 5 자료 5.3·10.3의 형태가 이 노드에도 그대로 나타납니다.

| 코드 | 뜻 |
|------|------|
| `from gpiozero import DistanceSensor` | 모듈에서 이름 하나만 가져오기 — `from … import …` 형태 |
| `class UsDistPublisher(Node):` | 괄호 = 상속 — `Node`를 물려받은 새 클래스 |
| `DistanceSensor(echo=24, trigger=23, …)` | **키워드 인자** — 순서가 아니라 이름으로 값을 지정 |
| `self.sensor = …` | `self.` = 이 객체의 변수 — `on_timer()`에서 같은 센서를 다시 사용 |
| `self.get_parameter('max_cm').value / 100.0` | 점 연결로 읽은 값을 그 자리에서 나눗셈 — 단위 환산 |

**코드 읽기 — Python 문법 ② 메시지 객체와 실수 연산**

| 코드 | 뜻 |
|------|------|
| `msg = DistMsg()` | 커스텀 메시지 객체 생성 — 필드는 기본값(`Twist()`·`Point()`와 같은 형태) |
| `msg.dist = self.sensor.distance * 100.0` | 점 연결로 읽은 값에 곱셈 후 필드에 대입 — `100.0`이 실수라 결과도 실수 |
| `f'{sensor.distance * 100:.1f} cm'` (3.4) | f-문자열 **형식 지정** — 소수 첫째 자리까지 |
| `while True:` … `sleep(0.5)` (3.4) | 무한 반복 + 지연 — ROS2 노드의 타이머가 대신하는 구조 |

- 표 ①·②의 형태는 **4.5·6.4·7.2의 노드와 이후 Day의 코드에도 다시 싣습니다** — 지금 전부 암기할 필요는 없으며, 코드를 읽을 때마다 표를 참조하며 확인합니다

### 4.4 실행과 관찰

```bash
cd ~/ros2_ws && colcon build && source install/local_setup.bash

# 터미널 1
ros2 run my_car_pkg us_node
# 터미널 2
ros2 topic type /us_dist        # ① 타입 확인 → my_msgs/msg/DistMsg
ros2 topic echo /us_dist        # ③ 값 확인 (② 구조 = 4.2의 interface show)
ros2 topic hz /us_dist          # 약 10Hz — period 설정과 대조 (Day 1 자료 6.3)
```

- `/us_dist`에 값이 출력되고 주기가 약 10Hz면 → 아래 측정 특성 실험으로 진행
- `ModuleNotFoundError: gpiozero`가 나오면 → 2.2의 설치를 확인하고 **빌드부터 다시** 실행
- `ModuleNotFoundError: my_msgs`가 나오면 → 4.2의 **`my_msgs`부터 다시** 빌드하고 `source` 실행
- `executable 'us_node' not found`가 나오면 → `setup.py` 등록을 확인하고 **빌드부터 다시** 실행
- GPIO 권한 오류(`Failed to add edge detection` 등)가 나오면 → 재로그인 후 **다시** 실행 → 지속되면 교수에게 알림
- 값이 계속 같은 수치로만 출력되면 → **3.4 단품 확인으로 돌아가** 배선을 먼저 검증

측정 특성을 실험으로 확인합니다 — **센서의 한계를 아는 것이 Day 9 회피 설계의 근거**:

| 실험 | 관찰 |
|------|------|
| 정면에 책·상자 | 거리에 따라 값이 안정적으로 변화 |
| 20cm → 5cm로 접근 | 최소 거리(약 2cm) 아래에서 값 불안정 |
| `max_cm`(2m) 밖 | **상한값 부근으로 고정** — "멀다"와 "없다"가 구분되지 않음 |
| 비스듬한 면·천 | 값 요동 — 반사 약화(3.1) |
| 센서를 좌우로 회전 | 약 15° 밖의 물체는 미검출 — **정면 한 방향만** 측정한다는 한계 |

- 구독하는 노드의 처리 원칙 — 상한 부근 값은 "장애물 없음"으로 해석(4.5의 판단 노드와 Day 9 회피 판단에 적용)
- 값의 요동을 줄이려면 — 10.1 보충(이동 평균 필터)

### 4.5 거리에 반응하는 turtlesim — 인식·판단·행동의 최소 구조

초음파 값이 정상적으로 발행되는지를 **수치가 아니라 움직임으로** 확인합니다. 거리를 읽어 turtle을 움직이는 노드를 하나 더 작성합니다.

| 거리 | turtle |
|------|------|
| 기준 이상 (멀다) | **전진** |
| 기준 미만 (가깝다) | **정지** |

- 손을 센서 앞에 가까이 대면 화면 속 turtle이 멈추고, 손을 치우면 다시 전진합니다
- 모터를 연결하기 전이므로 **turtlesim이 차량의 움직임을 대신합니다**

**인식 → 판단 → 행동** — 이 확인에 사용하는 세 노드의 구성은 자율 주행 시스템의 기본 구조와 같습니다.

```mermaid
flowchart LR
    A["us_node<br/>인식 — 거리 측정"] -->|"/us_dist · DistMsg(cm)"| B["dist_turtle<br/>판단 — 기준과 비교"]
    B -->|"/turtle1/cmd_vel · Twist"| C["turtlesim<br/>행동 — 이동"]
    style A fill:#6b8e5a,color:#fff
    style B fill:#4a7ba6,color:#fff
    style C fill:#a67b4a,color:#fff
```

| 단계 | 노드 | 하는 일 |
|------|------|------|
| 인식 | `us_node`(4.3) | 센서 값을 토픽으로 발행 |
| 판단 | `dist_turtle`(이 절) | 거리를 기준과 비교해 명령을 결정 |
| 행동 | turtlesim | 받은 명령대로 이동 — 오후에 **모터 노드로 교체**(8.1) |

이 구조의 두 원칙:

| 원칙 | 뜻 | 오늘의 사례 |
|------|------|------|
| **책임 분리** | 노드 하나는 한 가지 일만 담당 | 센서 노드는 판단하지 않고, 판단 노드는 GPIO를 다루지 않음 |
| **교체 가능** | 노드 사이의 연결은 토픽 계약(4.1)뿐 — 한 단계를 바꾸어도 나머지는 수정하지 않음 | 행동 단계의 turtlesim을 오후에 모터 노드로 교체(8.1) |

- 토픽 이름이 다른 노드로 교체할 때는 코드를 고치지 않고 **실행할 때 이름만 바꾸어** 연결합니다(remapping — 8.1)
- Day 8에서 이 구조에 카메라·AI 분류를 결합하고, Day 9에서 행동 단계를 실물 차량으로 바꿉니다

**설계** — 4.1과 같은 방식으로 계약부터 정합니다.

| 항목 | 값 |
|------|------|
| 노드 이름 | `dist_turtle` |
| 구독 | `/us_dist` (`my_msgs/msg/DistMsg`) — 4.1 계약 |
| 발행 | `/turtle1/cmd_vel` (`geometry_msgs/msg/Twist`) — Day 1 자료 7장·Day 2 자료 7장에서 사용한 토픽 |
| 파라미터 | `stop_cm` 정지 기준 거리(20.0) · `speed` 전진 속도(1.0) |
| 판단 규칙 | `dist`가 `stop_cm` 이상이면 전진 / 미만이면 정지 |

`~/ros2_ws/src/my_car_pkg/my_car_pkg/dist_turtle.py`:

```python
import rclpy
from rclpy.node import Node
from my_msgs.msg import DistMsg
from geometry_msgs.msg import Twist

class DistTurtle(Node):
    def __init__(self):
        super().__init__('dist_turtle')
        self.declare_parameter('stop_cm', 20.0)                  # ① 정지 기준 거리(cm)
        self.declare_parameter('speed', 1.0)                     # ② 전진 속도
        self.sub = self.create_subscription(DistMsg, '/us_dist', self.on_dist, 10)
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info('dist_turtle started')

    def on_dist(self, msg):
        cmd = Twist()                                            # ③ 모든 값 0 = 정지 명령
        if msg.dist >= self.get_parameter('stop_cm').value:      # ④ 판단 — 기준 이상이면
            cmd.linear.x = self.get_parameter('speed').value     #    전진 속도를 넣음
        self.pub.publish(cmd)                                    # ⑤ 거리 값이 올 때마다 발행

def main(args=None):
    rclpy.init(args=args)
    node = DistTurtle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

| # | 하는 일 |
|:--:|------|
| ① | 정지 기준을 **파라미터로**(Day 3 자료 5장) — 실행 중에 바꾸어 반응 거리를 확인 |
| ② | 전진 속도 — turtlesim 좌표 단위로 초당 1.0(Day 1 자료 7장의 `linear.x`와 같은 값) |
| ③ | `Twist()`의 필드는 모두 0 — 값을 넣지 않으면 그대로 **정지 명령** |
| ④ | **판단은 이 한 줄** — 계약의 cm 단위끼리 비교 |
| ⑤ | 구독 콜백 안에서 발행 — Day 5 자료 5.3 색상 검출 노드와 같은 형태. 거리 값이 10Hz로 발행되므로 명령도 10Hz |

`setup.py` 등록:

```python
'dist_turtle = my_car_pkg.dist_turtle:main',
```

> **자주 하는 실수**
>
> - `stop_cm`을 m 단위(0.2)로 설정하면 거리 값(cm)이 늘 기준 이상이 되어 **turtle이 멈추지 않습니다** — 계약의 단위는 cm입니다(4.1)
> - turtlesim을 VS Code(SSH) 터미널에서 실행하면 창이 열리지 않습니다 — **원격 데스크톱 화면의 터미널**에서 실행합니다

**코드 읽기 — Python 문법 ③ 조건 분기와 기본값** — 4.3 ①②의 형태도 다시 나타납니다.

| 코드 | 뜻 |
|------|------|
| `from geometry_msgs.msg import Twist` | 두 번째 메시지 타입 가져오기 — 구독 = `DistMsg` · 발행 = `Twist` |
| `cmd = Twist()` | 메시지 객체 생성 — 필드는 모두 기본값 0(4.3 ②의 `DistMsg()`와 같은 형태) |
| `def on_dist(self, msg):` | 구독 콜백 — 메시지가 도착할 때마다 호출되며 받은 메시지가 `msg`로 전달됨 |
| `if 조건:` + 들여 쓴 줄 | 조건이 참일 때만 들여 쓴 줄을 실행 — 거짓이면 건너뜀 |
| `>=` | 크거나 같다 — 비교 연산자(`<`·`>`·`<=`·`==`와 같은 종류) |
| `cmd.linear.x = …` | 점을 이어 **중첩 필드**에 대입(`Twist` 안의 `linear` 안의 `x`) |
| `self.get_parameter('speed').value` | 파라미터 값 읽기 — `param set`으로 바꾼 값이 다음 호출부터 반영 |

**실행** — turtlesim은 창을 여는 프로그램이므로 **원격 데스크톱 화면**의 터미널에서 실행합니다(Day 5 자료 2.5).

```bash
# 원격 데스크톱 화면의 터미널
ros2 run turtlesim turtlesim_node
```

```bash
# VS Code(SSH) 터미널 ① — 거리 퍼블리셔(4.4에서 실행 중이면 그대로 유지)
ros2 run my_car_pkg us_node
# 터미널 ② — 빌드 후 판단 노드
cd ~/ros2_ws && colcon build && source install/local_setup.bash
ros2 run my_car_pkg dist_turtle
# 터미널 ③ — 명령 확인(토픽 모니터)
ros2 topic echo /turtle1/cmd_vel
```

- 손을 20cm 안으로 가져가면 turtle이 멈추고 손을 치우면 다시 전진 → 정상 · 아래 관찰로 진행
- 창이 열리지 않고 표시 장치 오류가 나오면 → SSH 터미널에서 실행한 경우입니다. **원격 데스크톱 화면의 터미널에서 다시** 실행
- `Package 'turtlesim' not found`가 나오면 → `sudo apt install -y ros-jazzy-turtlesim` 후 **다시** 실행
- turtle이 움직이지 않으면 → 터미널 ③에 명령이 출력되는지 확인 → 출력되지 않으면 `ros2 topic echo /us_dist`로 거리 값을 확인(4.4) → 거리 값이 출력되면 터미널 ②의 오류를 확인하고 **빌드부터 다시**
- 거리와 무관하게 계속 멈춰 있으면 → 거리 값이 기준보다 작게 발행되는 상태입니다. `/us_dist` 값과 `stop_cm`(20)을 비교
- turtle이 화면 끝에 닿아 멈추면 → 정상 동작(화면 경계)입니다. `ros2 service call /reset std_srvs/srv/Empty`로 처음 위치로 되돌리고 계속 확인(Day 2 자료 4.4)

**관찰** — 실행 중에 판단 기준을 바꾸어 반응을 확인합니다.

| 실험 | 명령 | 관찰 |
|------|------|------|
| 기준 거리 변경 | `ros2 param set /dist_turtle stop_cm 40.0` | 더 먼 거리에서 정지 — 재실행 없이 반영 |
| 속도 변경 | `ros2 param set /dist_turtle speed 2.0` | 전진이 빨라짐 |
| 연결 확인 | 원격 데스크톱 화면에서 `rqt_graph` | 세 노드와 두 토픽 = 이 절 첫머리의 흐름도와 같은 연결 |

- 손을 댄 뒤 turtle이 멈출 때까지 **약간의 지연**이 있습니다 — 측정 주기(0.1초)·전달·판단이 차례로 누적된 결과입니다. 실물 차량은 이 지연 동안에도 계속 전진하므로 **기준 거리에 여유를 둡니다**(Day 9에서 계산)

**확인 활동 — 구조 점검**

| # | 질문 | 확인하려는 것 |
|:--:|------|------|
| 1 | 세 노드 중 GPIO를 다루는 노드는? | 책임 분리 — `us_node`만 |
| 2 | 정지 기준을 바꾸려면 어느 노드를? 실행 중이라면? | 판단 노드 · `param set` |
| 3 | turtle 대신 모터가 움직이게 하려면 `dist_turtle`의 코드를 고쳐야 하는가? | 교체 가능 — 행동 노드만 교체, 이름 차이는 remapping |
| 4 | 물체가 200cm보다 멀리 있을 때 turtle은? | 4.4 상한 처리 — 기준 이상이므로 전진 |

---

## 5. DC모터와 L298N

### 5.1 DC모터 — 방향과 속도

DC모터의 제어 요소는 둘뿐입니다.

| 제어 대상 | 방법 |
|------|------|
| **회전 방향** | 전류의 방향 — 두 단자의 극성을 바꾸면 역회전 |
| **회전 속도** | 공급 전력의 크기 — 전압(평균)이 높을수록 빠름 |

- 문제 ① — GPIO 핀은 모터를 돌릴 만한 전류를 내지 못함 → **별도 전원 + 중간 스위치 회로** 필요(5.3 L298N)
- 문제 ② — GPIO는 3.3V 켬/끔 두 상태뿐, 중간 전압이 없음 → **PWM**으로 평균을 만듦(5.2)

### 5.2 PWM — 켬·끔의 비율로 평균을 만들기

**PWM**(Pulse Width Modulation — 펄스 폭 변조) = 빠르게 켜고 끄기를 반복하며 **켜져 있는 시간의 비율**로 평균 전력을 조절하는 방식.

| 듀티 사이클(duty cycle — 켬 비율) | 평균 효과 | 모터 |
|:--:|:--:|------|
| 100% | 전체 전압 | 최고 속도 |
| 50% | 절반 | 중간 속도 |
| 20% | 1/5 | 저속 (모터에 따라 기동하지 못할 수 있음 — 10.3) |
| 0% | 0 | 정지 |

- 전환이 초당 수백~수천 회로 빠르므로 모터는 **평균값으로 동작** — 깜빡임이 아니라 속도 조절이 됨
- gpiozero에서는 `motor.forward(0.5)`처럼 **0.0~1.0의 값**으로 지정 — 내부에서 PWM으로 변환됨

### 5.3 L298N — H-브리지 드라이버

**L298N** = 방향 전환(H-브리지)과 전류 증폭을 담당하는 **모터 드라이버 보드**. 모터 2개를 독립 제어합니다.

**H-브리지** = 스위치 4개를 H자 모양으로 배치한 회로 — 켜는 쌍에 따라 모터에 흐르는 전류의 방향이 바뀝니다.

| IN1 | IN2 | 모터 A |
|:--:|:--:|------|
| 켬 | 끔 | 정회전 |
| 끔 | 켬 | **역회전** |
| 끔 | 끔 | 정지 |

| L298N 단자 | 역할 |
|------|------|
| IN1·IN2 / IN3·IN4 | 모터 A / B의 **방향** 지정 — GPIO 출력 연결 |
| **ENA / ENB** | 모터 A / B의 **속도** — PWM 신호 연결(점퍼 제거 후) |
| OUT1·2 / OUT3·4 | 모터 A / B 단자 |
| +12V · GND | **배터리 전원** 입력 |

### 5.4 전원 설계 — 분리와 공통 GND

> **모터 전원을 RPi5에서 끌어오지 않습니다**
>
> 모터는 기동 순간 큰 전류를 소비합니다. RPi5의 5V 핀에서 공급하면 **전압이 순간 강하해 RPi5가 재부팅**될 수 있습니다(10.4). 모터 전원은 **전용 배터리**로 분리합니다.

| 전원 | 공급 대상 |
|------|------|
| 배터리 (7~12V) | L298N +12V 단자 → 모터 |
| RPi5 어댑터 | RPi5 본체 |
| L298N 보드 로직 | 온보드 5V 생성(점퍼 유지) — RPi5 5V와 연결하지 않음 |

- **GND는 반드시 공통** — 배터리 GND·L298N GND·RPi5 GND를 한데 연결. 기준점이 다르면 GPIO 신호를 L298N이 읽지 못함
- 전원은 둘이지만 **기준점은 하나** — 오늘 배선에서 가장 자주 빠뜨리는 연결이 이 GND입니다

### 5.5 확인 활동 — 개념 점검

결선에 들어가기 전 확인합니다.

| # | 질문 | 확인하려는 것 |
|:--:|------|------|
| 1 | 모터의 회전 방향은 무엇으로 바꾸는가? | 5.1 — 전류 방향(IN1·IN2 조합) |
| 2 | GPIO가 중간 전압을 내지 못하면서도 속도를 조절할 수 있는 이유는? | 5.2 — PWM 듀티 사이클 |
| 3 | 모터 전원을 RPi5 5V에서 공급하면 생기는 문제는? | 5.4 — 전압 강하·재부팅 |
| 4 | 전원을 분리하면서도 GND를 공통으로 연결하는 이유는? | 5.4 — 신호 기준점 공유 |
| 5 | ENA의 점퍼를 제거하는 이유는? | 5.3 — PWM 신호를 연결하기 위해 |

---

## 6. 실습 ③ — 모터 결선과 동작 확인

### 6.1 결선표

**전원을 모두 분리한 상태**에서 연결합니다. 왼쪽 모터 = A(OUT1·2), 오른쪽 모터 = B(OUT3·4).

| L298N | 연결 | 대상 (BCM) |
|:--:|:--:|:--:|
| IN1 | → | **GPIO17** (왼쪽 방향 ①) |
| IN2 | → | **GPIO27** (왼쪽 방향 ②) |
| ENA | → | **GPIO12** (왼쪽 속도 — PWM) |
| IN3 | → | **GPIO5** (오른쪽 방향 ①) |
| IN4 | → | **GPIO6** (오른쪽 방향 ②) |
| ENB | → | **GPIO13** (오른쪽 속도 — PWM) |
| OUT1·OUT2 | → | 왼쪽 모터 두 단자 |
| OUT3·OUT4 | → | 오른쪽 모터 두 단자 |
| +12V | → | 배터리 **+** |
| GND | → | 배터리 **−** **그리고 RPi5 GND** (공통 — 5.4) |

- GPIO12·13을 속도(EN)에 배정한 이유 — RPi5에서 **PWM에 적합한 핀**
- ENA·ENB의 **점퍼 캡을 제거**한 뒤 연결(5.3) — 점퍼가 있으면 항상 최고 속도로 고정됨

### 6.2 육안 점검

| # | 점검 |
|:--:|------|
| 1 | 배터리 **극성** — +가 +12V 단자에 (역결선은 보드 손상 위험) |
| 2 | **공통 GND** — 배터리·L298N·RPi5의 GND가 이어져 있는가 |
| 3 | ENA·ENB **점퍼 제거** 후 GPIO12·13 연결 |
| 4 | 모터 단자가 OUT에, **GPIO 핀에 직접 연결된 모터가 없는가** |
| 5 | 좌우 모터가 A·B에 뒤바뀌지 않았는가 (차체 기준 왼쪽 = A) |

> **파손 위험 항목**
>
> - **배터리 역결선** — L298N 손상
> - **모터를 GPIO에 직결** — RPi5 손상
> - 통전 확인은 **차체를 들어 바퀴를 공중에 띄운 상태**에서 — 예상과 다르게 회전해도 사고가 없도록

### 6.3 통전 확인 — 단품 스크립트

배터리·RPi5 전원을 연결하고, **바퀴를 공중에 띄운 상태**에서 실행합니다.

`~/motor_test.py`:

```python
from gpiozero import Motor               # 모터 클래스 — 방향 핀 2 + 속도(PWM) 핀 1
from time import sleep

left  = Motor(forward=17, backward=27, enable=12)
right = Motor(forward=5,  backward=6,  enable=13)

left.forward(0.5);  right.forward(0.5);  sleep(2)    # ① 전진 (절반 속도)
left.stop();        right.stop();        sleep(1)
left.backward(0.5); right.backward(0.5); sleep(2)    # ② 후진
left.stop();        right.stop();        sleep(1)
left.backward(0.5); right.forward(0.5);  sleep(1)    # ③ 제자리 좌회전 — 왼쪽 뒤로·오른쪽 앞으로
left.stop();        right.stop();        sleep(1)
left.forward(0.5);  right.backward(0.5); sleep(1)    # ④ 제자리 우회전 — 왼쪽 앞으로·오른쪽 뒤로
left.stop();        right.stop()                     # ⑤ 정지
```

```bash
python3 motor_test.py
```

| 확인 | 왼쪽 바퀴 | 오른쪽 바퀴 | 차체의 움직임 (바닥에 두었을 때) |
|:--:|:--:|:--:|------|
| ① | 앞 | 앞 | 전진 |
| ② | 뒤 | 뒤 | 후진 |
| ③ | 뒤 | 앞 | 제자리에서 **왼쪽**으로 회전 |
| ④ | 앞 | 뒤 | 제자리에서 **오른쪽**으로 회전 |
| ⑤ | 정지 | 정지 | 정지 |

- 공중 시험이므로 차체는 움직이지 않습니다 — **바퀴의 회전 방향**을 표의 「왼쪽 바퀴」·「오른쪽 바퀴」 열과 대조합니다

- ①~⑤가 표대로 동작하면 → **단품 정상** · 6.4로 진행
- 한쪽이 **반대로** 회전하면 → 그 모터의 두 단자가 뒤바뀐 것입니다. **전원을 분리하고** OUT 두 선을 교환한 뒤 **다시** 실행
- ①②는 정상이지만 ③④의 좌우가 서로 바뀌면 → 좌우 모터가 A·B에 바뀌어 연결된 상태입니다(점검 5). **전원을 분리하고** 두 모터의 OUT 연결을 좌우 교환한 뒤 **다시** 실행
- 한쪽만 회전하면 → IN·EN 배선 누락 또는 점퍼 미제거입니다. **전원을 분리하고 6.1·점검 3**을 확인한 뒤 **다시** 실행
- 아무 반응이 없으면 → 배터리·공통 GND 미연결입니다. **전원을 분리하고 점검 1·2**를 확인한 뒤 **다시** 실행
- 회전하다 **RPi5가 재부팅**되면 → 모터 전원이 RPi5에서 공급되고 있습니다. **즉시 전원을 분리하고 5.4**(전원 분리)를 다시 확인 — 이 상태로 계속 실행하지 않습니다
- 위 조치로도 동작하지 않으면 → 교수에게 알림

- **여기서 방향이 맞아야 6.4·7장의 코드가 성립** — 단품에서 네 동작의 방향을 확정해 두면 노드의 오동작 원인에서 배선을 제외할 수 있습니다

### 6.4 동작 함수 노드 — 전진·후진·좌회전·우회전

6.3에서 확인한 네 동작을 **동작 하나에 함수 하나**로 정리하고, ROS2 노드에서 차례로 시험합니다.

| 함수 | 왼쪽 바퀴 | 오른쪽 바퀴 |
|------|:--:|:--:|
| `forward()` | 앞 | 앞 |
| `backward()` | 뒤 | 뒤 |
| `turn_left()` | 뒤 | 앞 |
| `turn_right()` | 앞 | 뒤 |
| `stop()` | 정지 | 정지 |

- 6.3 스크립트는 명령을 한 줄씩 나열했습니다 — 함수로 묶으면 **동작의 이름으로 호출**할 수 있습니다
- 7장의 모터 제어 노드는 이 네 함수를 **공식 하나로 일반화**합니다(7.1)

**설계**

| 항목 | 값 |
|------|------|
| 노드 이름 | `motion_test` |
| 입력 | 없음 — 코드에 정해 둔 시험 목록을 차례로 실행 |
| 출력 | gpiozero `Motor` 2개 — 6.3과 같은 핀 |
| 파라미터 | `speed` 시험 속도(0.5) · `step_sec` 동작 하나의 시간(2.0초) |
| 종료 | 목록을 모두 실행하면 정지 후 종료 · `Ctrl+C`에도 정지 |

`~/ros2_ws/src/my_car_pkg/my_car_pkg/motion_test.py`:

```python
import rclpy
from rclpy.node import Node
from gpiozero import Motor

class MotionTest(Node):
    def __init__(self):
        super().__init__('motion_test')
        self.declare_parameter('speed', 0.5)                      # ① 시험 속도(0~1)
        self.declare_parameter('step_sec', 2.0)                   # ② 동작 하나의 시간(초)
        self.left  = Motor(forward=17, backward=27, enable=12)    # ③ 6.3과 같은 핀
        self.right = Motor(forward=5,  backward=6,  enable=13)

        self.steps = [('forward', self.forward), ('stop', self.stop),       # ④ 시험 목록
                      ('backward', self.backward), ('stop', self.stop),
                      ('turn_left', self.turn_left), ('stop', self.stop),
                      ('turn_right', self.turn_right), ('stop', self.stop)]
        self.index = 0
        self.done = False
        step_sec = self.get_parameter('step_sec').value
        self.timer = self.create_timer(step_sec, self.next_step)  # ⑤ 일정 간격으로 다음 동작
        self.get_logger().info('motion_test started')

    def forward(self):                                            # ⑥ 동작 하나에 함수 하나
        s = self.get_parameter('speed').value
        self.left.forward(s);  self.right.forward(s)

    def backward(self):
        s = self.get_parameter('speed').value
        self.left.backward(s); self.right.backward(s)

    def turn_left(self):                                          # 제자리 좌회전
        s = self.get_parameter('speed').value
        self.left.backward(s); self.right.forward(s)

    def turn_right(self):                                         # 제자리 우회전
        s = self.get_parameter('speed').value
        self.left.forward(s);  self.right.backward(s)

    def stop(self):
        self.left.stop(); self.right.stop()

    def next_step(self):
        if self.index >= len(self.steps):                         # ⑦ 목록을 모두 실행했으면
            self.stop()
            self.done = True
            return
        name, action = self.steps[self.index]                     # ⑧ 이번 차례의 이름과 함수
        self.get_logger().info(f'step {self.index + 1}: {name}')
        action()                                                  # ⑨ 보관한 함수를 호출
        self.index += 1

def main(args=None):
    rclpy.init(args=args)
    node = MotionTest()
    try:
        while rclpy.ok() and not node.done:                       # ⑩ 목록이 끝날 때까지 반복
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()                                               # ⑪ 종료 시 정지 보장
        node.destroy_node()
        rclpy.shutdown()
```

| # | 하는 일 |
|:--:|------|
| ①② | 속도·시간을 **파라미터로** — 실행할 때 바꾸어 시험(Day 3 자료 5장) |
| ③ | 6.3 단품에서 확인한 핀을 그대로 사용 |
| ④ | **(이름, 함수)** 쌍을 순서대로 담은 시험 목록 — 동작 사이에 정지를 두어 하나씩 확인 |
| ⑤ | 타이머가 `step_sec`마다 `next_step`을 호출 — 첫 동작은 실행 2초 뒤 시작 |
| ⑥ | **동작 함수** — 좌우 바퀴의 조합을 이름 하나로 묶음(6.4 첫머리의 함수 표) |
| ⑦ | 목록을 모두 실행하면 정지하고 종료 표시(`done`) |
| ⑧⑨ | 목록에서 이번 차례를 읽어 로그로 이름을 출력하고 함수를 호출 |
| ⑩ | `done`이 참이 될 때까지 콜백을 하나씩 처리 — 끝나면 반복을 빠져나와 종료 |
| ⑪ | **종료 시 정지 보장** — 중간에 `Ctrl+C`를 눌러도 바퀴가 멈춤 |

**코드 읽기 — Python 문법 ④ 함수를 목록에 담기**

| 코드 | 뜻 |
|------|------|
| `def forward(self):` | 메서드 정의 — 동작 하나를 이름 하나로 묶음(4.3 ①의 `class`·`self.` 형태) |
| `self.left.forward(s);  self.right.forward(s)` | 세미콜론 = 한 줄에 두 문장 — 좌우 한 쌍의 짧은 동작에만 사용 |
| `('forward', self.forward)` | **튜플** — 값 두 개를 소괄호로 묶은 쌍(Day 5 자료 5.3·10.3의 형태) |
| `self.forward` (괄호 없음) | **함수 자체**를 값으로 다룸 — 호출하지 않고 목록에 보관 |
| `[ …, … ]` | **리스트** — 여러 값을 순서대로 담는 목록(Day 5 자료 5.3의 형태) |
| `self.steps[self.index]` | 대괄호 안의 번호(0부터)로 목록의 한 칸을 읽음(Day 5 자료 10.3의 번호 접근) |

**코드 읽기 — Python 문법 ⑤ 나누어 대입과 반복 조건**

| 코드 | 뜻 |
|------|------|
| `name, action = …` | 쌍의 두 값을 두 변수에 **나누어 대입** |
| `action()` | 괄호를 붙이면 호출 — 보관해 둔 함수를 이 시점에 실행 |
| `len(self.steps)` | 목록의 길이(항목 수) — 여기서는 8 |
| `self.index += 1` | 1 증가 — `self.index = self.index + 1`과 같음 |
| `f'step {self.index + 1}: {name}'` | f-문자열 — 중괄호 안의 식을 계산해 문자열 안에 표시(4.3 ②) |
| `while rclpy.ok() and not node.done:` | 두 조건이 모두 참인 동안 반복 — `not`은 참·거짓을 뒤집음 |
| `rclpy.spin_once(node)` | 콜백을 한 번만 처리하고 반환(Day 2 자료 6.4) — 반복을 끝낼 조건을 직접 정할 수 있음 |

- 표 ④·⑤의 형태는 **7.2와 이후 Day의 노드에도 다시 싣습니다** — 코드를 읽을 때마다 표를 참조합니다

**실행** — 바퀴를 공중에 띄운 상태에서 실행합니다.

`setup.py` 등록 후 빌드:

```python
'motion_test = my_car_pkg.motion_test:main',
```

```bash
cd ~/ros2_ws && colcon build && source install/local_setup.bash
ros2 run my_car_pkg motion_test
```

- 로그의 `step 1: forward` … `step 8: stop`에 맞추어 바퀴가 6.3의 표대로 회전하면 → 정상 · 관찰로 진행
- `executable 'motion_test' not found`가 나오면 → `setup.py` 등록을 확인하고 **빌드부터 다시** 실행
- 로그는 출력되지만 바퀴가 회전하지 않으면 → **6.3 단품 스크립트를 다시** 실행해 배선을 확인 → 단품이 정상이면 ③의 핀 번호를 6.3과 대조하고 **빌드부터 다시** 실행
- 좌회전과 우회전이 서로 바뀌면 → ⑥의 `turn_left`·`turn_right` 조합을 6.4 첫머리의 함수 표와 대조하고 **빌드부터 다시** 실행
- 멈추지 않고 계속 회전하면 → `Ctrl+C`(⑪이 정지를 보장) → 그래도 회전하면 **배터리를 분리**하고 교수에게 알림

**관찰** — 실행 중 다른 터미널에서 노드와 파라미터를 확인합니다.

```bash
ros2 node list                  # /motion_test 표시
ros2 param list /motion_test    # speed · step_sec 표시
```

> **자주 하는 실수**
>
> - 시험 목록에 `self.forward()`처럼 **괄호를 붙이면** 목록을 만드는 순간 함수가 실행되어 첫 동작이 곧바로 시작됩니다 — 목록에는 괄호 없이 함수 이름만 담습니다(코드 읽기 ④)

**변형 과제**

| # | 과제 | 확인할 것 |
|:--:|------|------|
| 1 | `--ros-args -p speed:=0.3 -p step_sec:=1.0`으로 실행(Day 3 자료 5.4) | 속도·시간이 바뀜 — 코드 수정 없이 |
| 2 | 시험 목록을 **전진 → 좌회전 → 전진**으로 바꾸어 실행 | 목록만 고치면 동작 순서가 바뀜 |
| 3 | 바닥에 내려놓고 저속으로 실행 | 네 동작의 **실제 차체 움직임** — 넓은 공간에서 |

- 3에서 `step_sec`을 바꾸면 회전 각도가 달라집니다 — 시간으로 정한 각도는 배터리 상태·바닥 마찰에 따라 달라지므로, 정확한 각도 제어에는 바퀴 회전량을 측정하는 센서(엔코더)가 필요합니다(Day 7에서 비교)

---

## 7. 실습 ④ — 모터 제어 노드

### 7.1 설계 — Twist를 두 바퀴로

6.4는 동작마다 함수를 하나씩 두었습니다. `Twist`는 **전진량(`linear.x`)과 회전량(`angular.z`) 두 값**으로 네 동작을 모두 표현합니다.

| 동작 (6.4 함수) | `linear.x` | `angular.z` | 왼쪽 바퀴 | 오른쪽 바퀴 |
|------|:--:|:--:|:--:|:--:|
| 전진 `forward()` | + | 0 | 앞 | 앞 |
| 후진 `backward()` | − | 0 | 뒤 | 뒤 |
| 좌회전 `turn_left()` | 0 | + | 뒤 | 앞 |
| 우회전 `turn_right()` | 0 | − | 앞 | 뒤 |
| 정지 `stop()` | 0 | 0 | 정지 | 정지 |

- 함수 넷을 **공식 하나**로 바꾸면 어떤 명령을 받아도 좌우 바퀴의 속도가 정해집니다 — 전진하며 조금씩 회전하는 중간 동작도 표현됩니다(7.1 변환 공식)

바퀴 2개로 방향을 바꾸는 방식 = **차동 구동**(differential drive — 좌우 바퀴의 속도 차로 회전).

| 항목 | 내용 |
|------|------|
| 노드 이름 | `motor_node` |
| 구독 | `/cmd_vel` (`geometry_msgs/msg/Twist`) — turtlesim과 **같은 메시지 타입** |
| 출력 | gpiozero `Motor` 2개 (토픽이 아니라 **GPIO 구동**) |
| 파라미터 | `max_linear`(정규화 기준)·`turn_gain`(회전 민감도)·`trim`(직진 보정 7.4) |
| 안전 | `/cmd_vel`이 **0.5초 이상 끊기면 정지** |

**변환 공식**:

```
left  = linear.x − angular.z × turn_gain
right = linear.x + angular.z × turn_gain
       → max_linear로 나눠 −1.0~1.0으로 정규화
```

- 부호 확인 — `angular.z` 양수 = 반시계 = **좌회전**(Day 1 자료 2.3 오른손 법칙) → 오른쪽 바퀴가 빨라져야 함 → `right`에 `+` ✓
- teleop(Day 1)과 4.5의 판단 노드가 보내는 `linear.x`(1.0\~2.0)는 turtlesim 규격 — `max_linear`로 나누어 모터의 0\~1 범위로 맞춤

### 7.2 코드 작성

`~/ros2_ws/src/my_car_pkg/my_car_pkg/motor_node.py`:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gpiozero import Motor

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')
        self.declare_parameter('max_linear', 2.0)                 # ① 정규화 기준
        self.declare_parameter('turn_gain', 0.6)
        self.declare_parameter('trim', 0.0)                       # 7.4 직진 보정

        self.left  = Motor(forward=17, backward=27, enable=12)    # ② 6.3과 같은 핀
        self.right = Motor(forward=5,  backward=6,  enable=13)

        self.cmd = Twist()                                        # ③ 최신 명령 보관
        self.last_received = self.get_clock().now()
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.timer = self.create_timer(0.05, self.drive)          # ④ 20Hz 구동 갱신
        self.get_logger().info('motor_node started')

    def on_cmd(self, msg):
        self.cmd = msg                                            # 보관만
        self.last_received = self.get_clock().now()

    def drive(self):
        elapsed = (self.get_clock().now() - self.last_received).nanoseconds / 1e9
        if elapsed > 0.5:                                         # ⑤ 신호 두절 → 정지
            self.left.stop(); self.right.stop()
            return

        gain = self.get_parameter('turn_gain').value
        trim = self.get_parameter('trim').value
        max_lin = self.get_parameter('max_linear').value

        l = (self.cmd.linear.x - self.cmd.angular.z * gain) / max_lin + trim   # ⑥ 변환
        r = (self.cmd.linear.x + self.cmd.angular.z * gain) / max_lin - trim
        self.set_wheel(self.left, l)
        self.set_wheel(self.right, r)

    def set_wheel(self, motor, v):
        v = max(-1.0, min(1.0, v))                                # ⑦ 범위 제한
        if v > 0.05:
            motor.forward(v)
        elif v < -0.05:
            motor.backward(-v)
        else:
            motor.stop()                                          # 미세 값은 정지 처리

def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.left.stop(); node.right.stop()                       # ⑧ 종료 시 정지 보장
        node.destroy_node()
        rclpy.shutdown()
```

| # | 하는 일 |
|:--:|------|
| ① | 정규화 기준·회전 민감도·보정을 **파라미터로** — 차량마다 값이 다름 |
| ② | 6.3 단품에서 검증된 핀·방향을 그대로 사용 |
| ③ | 구독 콜백은 **보관만** — 구동 계산은 ④의 타이머가 담당 |
| ④ | 구동 갱신은 타이머가 담당(20Hz) — 명령 수신과 구동 주기의 분리 |
| ⑤ | **신호 두절 시 정지** — 0.5초 넘게 새 명령이 없으면 정지. 명령이 끊겨도 차량이 계속 달리지 않게 하는 **안전 감시**(watchdog) |
| ⑥ | 차동 구동 변환(7.1) + 직진 보정(`trim` — 7.4) |
| ⑦ | −1.0~1.0 범위 제한 + **미세 값 정지 처리** — 낮은 듀티에서 모터가 소리만 내는 구간 회피(10.3) |
| ⑧ | **종료 시 정지 보장** — `Ctrl+C` 후 바퀴가 계속 도는 사고 방지 |

**코드 읽기 — Python 문법 ⑥ 범위 제한과 메서드 분리** — 6.4 ④의 메서드·세미콜론 형태가 다시 나타납니다.

| 코드 | 뜻 |
|------|------|
| `def set_wheel(self, motor, v):` | **메서드로 분리** — 같은 처리를 좌·우에 두 번 사용하기 위해. `self` 뒤가 실제 인자 |
| `max(-1.0, min(1.0, v))` | **클램프**(범위 제한) — 안쪽 `min`으로 위를 자르고 바깥 `max`로 아래를 자름 |
| `if v > 0.05:` `elif v < -0.05:` `else:` | 세 갈래 분기 — 양수·음수·그 사이(미세 값) |
| `motor.forward(v)` | 객체의 메서드 호출 — 인자로 속도(0~1)를 전달 |
| `node.left.stop(); node.right.stop()` | 세미콜론 = 한 줄에 두 문장 — 좌우 한 쌍의 짧은 동작에만 사용 |

**코드 읽기 — Python 문법 ⑦ 시간 비교와 산술식** — 4.5 ③의 중첩 필드 형태가 다시 나타납니다.

| 코드 | 뜻 |
|------|------|
| `self.get_clock().now()` | 노드의 현재 시각 — 명령을 받은 시각으로 보관 |
| `(now - last).nanoseconds / 1e9` | 두 시각의 차 → 나노초 → 초 환산(`1e9` = 10억) |
| `if elapsed > 0.5: … return` | 조기 종료 — 이후 구동 계산을 건너뜀 |
| `(a - b * gain) / max_lin + trim` | 괄호 우선 → 곱셈·나눗셈 → 덧셈 순으로 계산 |
| `self.cmd.linear.x` | 점을 이어 **중첩 필드**를 읽음(`Twist` 안의 `linear` 안의 `x`) |
| `self.cmd = msg` | 메시지 객체 자체를 보관 — 콜백은 보관만, 계산은 타이머가 |

- 표 ⑥·⑦의 항목은 **Day 8·9의 판단·회피 노드에도 같은 형태로 다시 싣습니다**

`setup.py` 등록 후 빌드:

```python
'motor_node = my_car_pkg.motor_node:main',
```

> **자주 하는 실수**
>
> - 구독 콜백(`on_cmd`)에서 바로 모터를 구동하면 명령이 끊겼을 때 호출 자체가 멈추므로 **신호 두절 정지(⑤)가 동작하지 않습니다** — 콜백은 보관만, 구동은 타이머가 담당합니다(③④)
> - `max_linear`를 명령 속도보다 작게 설정하면 계산값이 1.0을 넘고 **범위 제한(⑦)으로 1.0에 고정되어 늘 최고 속도**로 회전합니다 — teleop의 속도(2.0)에 맞춘 기본값을 유지합니다

### 7.3 단독 확인 — 손으로 명령을 주어

Day 1 자료 7장에서 turtlesim에 명령을 보낸 방식 그대로 — 명령을 보내는 노드 없이 `topic pub`으로 확인합니다. **바퀴는 공중에**.

```bash
# 터미널 1
ros2 run my_car_pkg motor_node
# 터미널 2 — 명령을 손으로 발행 (한 줄씩 실행 · 다음 줄 전에 Ctrl+C)
ros2 topic pub --rate 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"       # 1 전진
ros2 topic pub --rate 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -1.0}}"      # 2 후진
ros2 topic pub --rate 5 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 2.0}}"      # 3 제자리 좌회전
ros2 topic pub --rate 5 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: -2.0}}"     # 4 제자리 우회전
ros2 topic pub --once   /cmd_vel geometry_msgs/msg/Twist "{}"                        # 5 정지
```

**확인 5항** — 6.4 동작 함수와 같은 결과인지 확인합니다.

| # | 명령 | 정상 동작 |
|:--:|------|------|
| 1 | `linear.x` 양수 | 두 바퀴 앞으로·같은 속도 — `forward()`와 같음 |
| 2 | `linear.x` 음수 | 두 바퀴 뒤로 — `backward()`와 같음 |
| 3 | `angular.z` 양수 | 왼쪽 뒤로·오른쪽 앞으로 — `turn_left()`와 같음(7.1 부호) |
| 4 | `angular.z` 음수 | 왼쪽 앞으로·오른쪽 뒤로 — `turn_right()`와 같음 |
| 5 | 빈 명령 `{}` · 발행 중단 | **정지** — `--rate` 발행을 `Ctrl+C`로 멈춰도 0.5초 뒤 자동 정지(코드 ⑤) |

- 다섯 항목이 표대로 동작하면 → 7.4로 진행
- 3·4에서 소리만 나고 바퀴가 회전하지 않으면 → 듀티가 낮은 상태입니다(10.3). `angular.z`를 3.0으로 올려 **다시** 확인
- 바퀴가 전혀 회전하지 않으면 → `ros2 topic hz /cmd_vel`로 명령이 도착하는지 먼저 확인 → 명령은 도착하지만 회전하지 않으면 **6.3 단품 확인으로 돌아가** 배선을 검증
- 회전 방향이 표와 반대이면 → **6.3의 단품 방향**을 다시 확인(코드가 아니라 배선 문제일 때가 많습니다) → 단품이 정상이면 7.1의 부호를 확인하고 **빌드부터 다시** 실행
- 발행을 멈추어도 계속 회전하는 경우 → ⑤ 신호 두절 정지가 코드에 있는지 확인하고 **빌드부터 다시** 실행. 그 사이 **전원을 분리**해 정지시킵니다
- `executable 'motor_node' not found`가 나오면 → `setup.py` 등록을 확인하고 **빌드부터 다시** 실행
- `--rate 5`를 사용하는 이유 — `--once` 한 번이면 0.5초 뒤 안전 정지가 동작함. **⑤가 정상 동작한다는 증거**이기도 함

### 7.4 직진 보정 — trim

두 모터는 같은 신호에도 **같은 속도로 돌지 않습니다**(개체 차·기어 마찰). 전진 명령에 차량이 한쪽으로 휘면 `trim`으로 보정합니다.

```bash
ros2 param set /motor_node trim 0.03     # 오른쪽으로 휘면 양수 방향으로 조금씩
ros2 param set /motor_node trim -0.03    # 왼쪽으로 휘면 음수 방향
```

1. 전진 명령을 유지한 채(7.3) 바닥에서 2~3m 직진시켜 휘는 방향 확인
2. `param set`으로 0.01~0.05 단위 조정 → 재시험 반복
3. 맞춘 값을 `ros2 param dump /motor_node > motor.yaml`로 저장(Day 3 자료 5.6) — **차량 고유의 값**이므로 각자 보관

---

## 8. 미니프로젝트 — 수동 주행 차량

### 8.1 과제 — teleop 재사용, 구독자 교체

Day 1에서 turtlesim을 움직인 `turtle_teleop_key`를 **그대로** 사용합니다. teleop은 `/turtle1/cmd_vel`에 발행하고 모터 노드는 `/cmd_vel`을 구독하므로, 코드를 고치지 않고 **실행할 때 토픽 이름만 바꾸어** 연결합니다 — 이것을 **remapping**이라고 합니다.

| 부분 | 뜻 |
|------|------|
| `--ros-args` | 이후는 ROS2 실행 인자 — Day 3 자료 5.4의 `-p`와 같은 자리 |
| `-r` | remap — 토픽 이름 바꾸기 |
| `/turtle1/cmd_vel:=/cmd_vel` | `원래 이름:=새 이름` — 이 실행에서만 적용 |

- 이름만 바뀌며 노드의 코드·메시지 타입은 그대로 유지됩니다

```bash
# 터미널 1 — 차량
ros2 run my_car_pkg motor_node
# 터미널 2 — 센서
ros2 run my_car_pkg us_node
# 터미널 3 — 키보드 (발행 토픽을 /cmd_vel로 재지정)
ros2 run turtlesim turtle_teleop_key --ros-args -r /turtle1/cmd_vel:=/cmd_vel
# 터미널 4 — 거리 관찰
ros2 topic echo /us_dist
```

- 화살표 키에 바퀴가 반응하면 → 8.2 진행 절차로
- 키를 눌러도 반응이 없으면 → **터미널 3이 활성 창인지** 먼저 확인(teleop은 포커스가 있는 창의 입력만 받습니다) → 그래도 반응이 없으면 `ros2 topic echo /cmd_vel`로 명령 발행 여부를 확인
- 명령은 발행되지만 바퀴가 회전하지 않으면 → **7.3 단독 확인으로 돌아감**
- 거리 값이 출력되지 않으면 → 터미널 2의 `us_node` 실행 여부를 확인하고 **터미널 2부터 다시** 실행(4.4)

> **이 장면이 오늘의 결론입니다**
>
> 키보드 노드는 Day 1의 것, 메시지 타입(`Twist`)은 Day 2의 것입니다. **바뀐 것은 명령의 구독자 하나뿐** — 행동 단계의 turtlesim이 모터 노드로 교체되어 화면 속 turtle 대신 실물 차량이 움직입니다. 4.5에서 확인한 **교체 가능**이 이것입니다.

### 8.2 진행 절차

| 순서 | 단계 | 확인 |
|:--:|------|------|
| 1 | **공중 시험** — 바퀴를 띄운 채 화살표 키 4방향 | 전진·후진·좌우 회전 조합이 7.3과 일치 |
| 2 | **바닥 주행** — 넓은 공간에서 저속 주행 | 직진이 휘면 7.4 trim 재조정 |
| 3 | **센서 병행 관찰** — 벽으로 접근하며 터미널 4의 거리 값 확인 | 거리 감소가 실시간으로 표시 |

- 주행 속도가 부담스러우면 — `ros2 param set /motor_node max_linear 3.0` (나누는 기준값을 크게 하면 같은 명령에 느리게 반응)

### 8.3 원격 조종 — PC에서 차량으로

8.1에서는 teleop을 RPi5의 터미널에서 실행했습니다. 이번에는 **강의실 PC의 WSL2에서 직접** 실행합니다 — 노드가 두 기기에 나뉘어도 같은 도메인이면 연결된다는 것(Day 1 자료 2.5·Day 5 자료 2.6)을 조종으로 확인합니다.

- RPi5의 teleop(8.1 터미널 3)은 `Ctrl+C`로 **먼저 종료**합니다 — 한 토픽에 명령을 보내는 노드가 둘이면 두 명령이 섞여 전달됩니다

```bash
# 강의실 PC의 WSL 터미널에서 (원격 데스크톱 창이 아님)
export ROS_DOMAIN_ID=7        # RPi5와 같은 자기 번호 (Day 1 자료 2.5)
ros2 topic list               # RPi5의 /us_dist·/cmd_vel이 보이면 연결 성립
ros2 run turtlesim turtle_teleop_key --ros-args -r /turtle1/cmd_vel:=/cmd_vel
```

| 확인 | 내용 |
|------|------|
| `topic list`에 차량 토픽 표시 | **PC와 RPi5가 DDS로 연결됨** — 주소 입력 없이 자동 발견 |
| PC에서 화살표 키 | **차량이 움직임** — 키 입력은 PC에서, 구동은 RPi5에서 |
| PC에서 `topic echo /us_dist` | 차량의 센서 값도 PC에서 관찰 가능 |

- 원격 데스크톱과의 차이 — 원격 데스크톱은 **화면 전체**를 오가는 원격 조작이고, 이 방식은 **토픽만** 네트워크를 오가는 분산 실행. 조종 반응이 가볍고, 차량과 조종기가 완전히 분리됨
- `topic list`에 차량의 `/us_dist`·`/cmd_vel`이 출력되면 → 화살표 키로 조종을 확인
- 출력되지 않으면 → ① 두 기기가 **같은 Wi-Fi**인지 ② `echo $ROS_DOMAIN_ID`가 RPi5와 **같은 값**인지 확인하고 **`topic list`를 다시** 실행(Day 1 자료 2.5)
- ①②가 맞아도 출력되지 않으면 → **PC가 WSL2일 때의 네트워크 방식 제약**에 해당합니다(Day 5 자료 2.6 전제 ③). 교수에게 알리고, **조종은 RPi5 화면(원격 데스크톱)에서 진행**해 8.2까지는 그대로 완료합니다
- **이것이 실무의 로봇 운용 형태** — 로봇은 현장에, 조종·관찰은 관제 컴퓨터에서(Day 4 자료 2.2의 분리 구조)

### 8.4 단계별 과제

| 단계 | 과제 |
|:--:|------|
| **필수** | 키보드로 **전진·후진·좌우 회전** — 4방향 모두 의도대로 동작 |
| **도달** | 바닥 주행에서 **직진 보정(trim) 완료** + **원격 조종(8.3) 성립** — PC에서 조종·센서 관찰 |
| **도전** | 아래 ⓐ~ⓓ 중 선택 |

- ⓐ **4.5 판단 노드로 차량 정지** — teleop을 종료하고 `dist_turtle`을 `--ros-args -r /turtle1/cmd_vel:=/cmd_vel`로 실행 → 20cm 안에서 차량이 멈춤(**Day 9 회피의 최소형** · 공중 시험 후 넓은 공간에서 저속)
- ⓑ 세 노드를 launch 하나로 일괄 기동(Day 3 자료 8장)
- ⓒ 멈춘 뒤 **후진 → 회전 → 전진**으로 방향을 바꾸는 판단 노드 — 직전 상태를 기억하는 판단(Day 3 자료 9.2 미로의 상태 기계)
- ⓓ **자작 조종기** — 키 입력을 읽어 `Twist`를 발행하는 조종 노드를 직접 작성하고 `/cmd_vel`로 remapping해 기성 teleop을 대체(키 배치·속도 자작)

- **필수 단계는 전원 완료** — 4방향 주행이 성공하면 오늘 목표 달성
- **Day 9 대응** — 오늘의 세 노드(인식 `us_node` · 판단 `dist_turtle` · 행동 `motor_node`)가 **장애물 회피 자율주행의 최소형**입니다. Day 9에서 판단을 회피 로직으로 확장합니다

---

## 9. 오늘의 요약

| 항목 | 내용 |
|------|------|
| GPIO | 40핀·**3.3V 한계**·BCM 번호로 코드 작성(`pinout`으로 대조). RPi5 = RP1 칩 → **`RPi.GPIO` 미동작·gpiozero 사용** |
| 결선 원칙 | **전원 분리 상태 배선 → 육안 점검 → ROS2 없이 단품 통전 확인 → 노드 작성** — 배선 오류와 코드 오류의 분리 |
| 초음파 | HC-SR04 — 왕복 시간 × 음속 ÷ 2. **Echo 5V는 분압(1kΩ·2kΩ)** 후 GPIO 연결. 측정 각도 15°·상한 처리 |
| 거리 퍼블리셔 | RPi5에 `my_msgs` 신설 · `DistMsg`(cm — **단위는 계약에 명기**) → `/us_dist` 10Hz |
| 3단 구조 | 인식 `us_node` → 판단 `dist_turtle` → 행동 turtlesim — **책임 분리 · 교체 가능** |

| 항목 | 내용 |
|------|------|
| PWM | 켬 비율(듀티 사이클)로 평균 전력 조절 — gpiozero는 0.0~1.0 값으로 지정 |
| L298N | H-브리지 — IN 조합 = 방향 · EN = 속도(점퍼 제거 후 PWM). **모터 전원은 전용 배터리·GND는 공통** |
| 동작 함수 | `forward`·`backward`·`turn_left`·`turn_right`·`stop` — (이름, 함수) 목록으로 차례 시험 |
| 모터 노드 | 네 함수를 공식 하나로(`left = lin − ang·gain` / `right = lin + ang·gain`) · **신호 두절 0.5초 정지** |
| 직진 보정 | 모터 개체 차는 `trim` 파라미터로 — 값은 차량 고유·`param dump`로 보관 |
| 수동 주행 | Day 1 teleop + remapping — **명령의 구독자 교체만으로 turtlesim 대신 실물 차량이 움직임** |
| 원격 조종 | PC의 WSL2에서 teleop → 같은 Domain ID로 차량 구동 — 토픽만 오가는 분산 실행 |
| 다음 시간 | **SLAM 실습**(10/12 · 강의실 PC의 WSL2) — Gazebo·Nav2. 오늘의 세 노드는 **Day 9**에서 확장 |

---

## 10. 보충

> 본 과정을 마치고 시간이 남을 때 다루는 소재입니다. 각 항목은 서로 독립이며, 다음 시간의 선행 개념은 포함하지 않습니다.

### 10.1 측정값 필터링 — 이동 평균

`/us_dist`의 요동(4.4)을 완화하는 표준 수단 — **최근 N개의 평균**을 발행합니다.

```python
from collections import deque                      # 고정 길이 큐

# __init__
self.window = deque(maxlen=5)                      # 최근 5개 보관 — 넘치면 오래된 것 자동 제거
self.filt_pub = self.create_publisher(DistMsg, '/us_dist_filtered', 10)

# on_timer 말미
self.window.append(msg.dist)
filt = DistMsg()
filt.dist = sum(self.window) / len(self.window)
self.filt_pub.publish(filt)
```

| 창 크기 N | 성질 |
|:--:|------|
| 3 | 반응 빠름 · 요동 일부 잔존 |
| 10 | 매끄러움 · **반응 지연**(10 × 0.1초 = 최대 1초 전 값 포함) |

- Day 5 자료 11장(판정 안정화)과 같은 구조 — 문자열은 **연속 일치**, 수치는 **평균**이 기본 수단
- 원본 `/us_dist`와 필터 `/us_dist_filtered`를 함께 발행 — rqt_plot에 두 토픽을 겹쳐 효과를 눈으로 비교

### 10.2 표준 타입으로 발행 — sensor_msgs/Range

거리 센서에는 ROS 표준 타입이 이미 있습니다(센서 데이터용 표준 패키지 `sensor_msgs`).

```bash
ros2 interface show sensor_msgs/msg/Range
```

| 필드 | 내용 |
|------|------|
| `header` | 측정 시각·좌표계 이름 |
| `radiation_type` | 0 = 초음파 · 1 = 적외선 |
| `min_range` · `max_range` | 측정 한계 — **[m]** |
| `range` | 측정값 — **[m]** |

| 비교 | `DistMsg` (오늘) | `Range` (표준) |
|------|------|------|
| 장점 | 단순 — 학습에 적합 | **다른 도구·패키지가 그대로 해석** — 시각화·기록 호환 |
| 단위 | cm (계약 명기) | m (표준 고정) |

- 표준 타입이 있으면 표준을 사용하는 것이 실무 원칙 — 커스텀은 **표준으로 표현되지 않을 때**(Day 3 자료 7.1)
- 전환 실습 — `us_node`에 `Range` 발행을 추가하고 `rqt_plot`으로 관찰(단위 환산 주의)

### 10.3 모터 특성 — 최소 듀티와 데드존

낮은 듀티에서 모터는 **소리만 내고 회전하지 못합니다** — 기동에 필요한 힘(토크)에 미치지 못하기 때문입니다.

| 실험 | 방법 |
|------|------|
| 최소 듀티 찾기 | 6.3 스크립트의 값을 0.1부터 0.05씩 올리며 **바퀴가 실제로 회전하는 최솟값** 확인 |
| 부하 차이 | 공중 ↔ 바닥(하중)에서 최소 듀티가 달라짐을 확인 |

- 7.2 ⑦의 0.05 정지 처리 = 이 **데드존**(dead zone — 신호가 있어도 동작하지 않는 구간)의 최소 대응
- 개선 방향 — 명령을 `[최소 듀티, 1.0]` 구간으로 재배분하면 저속 제어가 정밀해짐 (도전 소재)

### 10.4 전원 문제 진단

실물 차량의 오동작 중 상당수는 코드가 아니라 **전원**이 원인입니다.

| 증상 | 원인 | 대책 |
|------|------|------|
| 모터 기동 순간 RPi5 재부팅 | 모터 전류로 인한 전압 강하 — 전원 미분리 | 5.4 전원 분리 재확인 |
| 주행 중 간헐 재부팅 | RPi5 어댑터 용량 부족 | 5V/5A 어댑터 사용(Day 3 자료 11.1) |
| 속도가 점점 느려짐 | **배터리 방전** — 전압 하락 | 배터리 교체·충전. `trim` 값도 달라질 수 있음 |
| 특정 동작에서만 정지 | 급가속·급반전 시 순간 전류 최대 | 명령 변화를 완만하게(가감속 완화 — Day 9 소재) |

- 진단 순서 — **배선(6.2) → 전원(이 표) → 코드** 순으로 확인. 코드는 마지막에 의심합니다

### 10.5 거리에 비례하는 속도 — 비례 제어

4.5 `dist_turtle`의 속도는 두 값뿐입니다 — 기준 거리 이상이면 `speed`, 미만이면 0. 이와 같이 명령이 켬·끔 두 값만 가지는 방식을 **온오프 제어**(on-off control)라고 합니다.

- 기준 거리까지 최고 속도로 전진하다가 한 번에 정지합니다
- turtle은 즉시 멈추지만, 실물 차량은 관성 때문에 정지 명령 뒤에도 조금 더 전진합니다
- 개선 방향 = **가까워질수록 속도를 줄여** 기준 거리에서 0이 되게 합니다

| 거리 | 4.5 `dist_turtle` | 이 절의 노드 |
|:--:|:--:|:--:|
| 80cm 이상 | 1.0 | 1.0 |
| 50cm | 1.0 | 0.5 |
| 35cm | 1.0 | 0.25 |
| 20cm 미만 | 0 | 0 |

**설계** — 4.5의 계약을 그대로 사용하고 파라미터 하나를 더합니다.

| 항목 | 값 |
|------|------|
| 노드 이름 | `dist_turtle_slow` |
| 구독 · 발행 | `/us_dist` → `/turtle1/cmd_vel` — 4.5와 동일 |
| 파라미터 | `stop_cm` 정지 거리(20.0) · `slow_cm` 감속 시작 거리(80.0) · `speed` 최고 속도(1.0) |
| 판단 규칙 | `slow_cm` 이상 = 최고 속도 / 두 거리 사이 = 거리에 비례 / `stop_cm` 미만 = 정지 |

- `stop_cm`·`speed`는 4.5와 같은 값 — 두 노드의 결과를 같은 조건으로 비교합니다
- `slow_cm`은 `stop_cm`보다 큰 값이어야 합니다 — 두 값의 차이가 감속 구간의 길이입니다

`~/ros2_ws/src/my_car_pkg/my_car_pkg/dist_turtle_slow.py`:

```python
import rclpy
from rclpy.node import Node
from my_msgs.msg import DistMsg
from geometry_msgs.msg import Twist

class DistTurtleSlow(Node):
    def __init__(self):
        super().__init__('dist_turtle_slow')
        self.declare_parameter('stop_cm', 20.0)                  # 정지 거리(cm) — 4.5와 같은 값
        self.declare_parameter('slow_cm', 80.0)                  # ① 감속을 시작하는 거리(cm)
        self.declare_parameter('speed', 1.0)                     # 최고 속도 — 4.5와 같은 값
        self.sub = self.create_subscription(DistMsg, '/us_dist', self.on_dist, 10)
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info('dist_turtle_slow started')

    def on_dist(self, msg):
        stop = self.get_parameter('stop_cm').value               # ② 파라미터를 지역 변수로
        slow = self.get_parameter('slow_cm').value
        top = self.get_parameter('speed').value
        ratio = (msg.dist - stop) / (slow - stop)                # ③ 감속 구간 안의 위치
        ratio = max(0.0, min(1.0, ratio))                        # ④ 0.0~1.0으로 제한
        cmd = Twist()
        cmd.linear.x = top * ratio                               # ⑤ 비율만큼의 속도
        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DistTurtleSlow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

| # | 하는 일 |
|:--:|------|
| ① | 감속 시작 거리 — 이보다 가까워지면 속도를 줄이기 시작 |
| ② | 세 값을 지역 변수로 먼저 읽어 식을 짧게 작성 — 매 호출마다 읽으므로 `param set`이 다음 호출부터 반영 |
| ③ | `stop_cm`에서 0.0 · `slow_cm`에서 1.0 — 50cm면 (50−20)÷(80−20) = 0.5 |
| ④ | `stop_cm`보다 가까우면 음수 · `slow_cm`보다 멀면 1보다 큰 값이 계산되므로 0.0~1.0으로 제한 |
| ⑤ | 최고 속도 × 비율 — 50cm에서 1.0 × 0.5 = 0.5 |

`setup.py` 등록:

```python
'dist_turtle_slow = my_car_pkg.dist_turtle_slow:main',
```

**코드 읽기 — Python 문법 ⑧ 지역 변수와 비율 계산** — 7.2 ⑥⑦의 클램프·괄호 계산이 다시 나타납니다.

| 코드 | 뜻 |
|------|------|
| `stop = self.get_parameter('stop_cm').value` | **지역 변수** — `self.` 없이 이 메서드 안에서만 사용 · 호출 종료 시 해제 |
| `(msg.dist - stop) / (slow - stop)` | 괄호 안을 먼저 계산한 뒤 나눗셈(7.2 ⑦) — 분모가 0이면 `ZeroDivisionError` |
| `max(0.0, min(1.0, ratio))` | 클램프(7.2 ⑥) — `min`으로 위를, `max`로 아래를 제한 |
| `ratio = … ratio …` | 오른쪽을 먼저 계산해 **같은 이름에 다시 대입** — 이전 값을 대체 |
| `cmd.linear.x = top * ratio` | 곱셈 결과를 중첩 필드에 대입(4.5 ③) |

**실행** — 4.5의 turtlesim과 `us_node`는 그대로 둡니다. `dist_turtle`은 **먼저 종료**합니다 — 두 노드가 같은 `/turtle1/cmd_vel`에 발행하면 두 명령이 번갈아 적용됩니다.

```bash
# 터미널 ② — dist_turtle을 Ctrl+C로 종료한 뒤 빌드
cd ~/ros2_ws && colcon build && source install/local_setup.bash
ros2 run my_car_pkg dist_turtle_slow
```

```bash
# 원격 데스크톱 화면의 터미널 — 그래프 창
ros2 run rqt_plot rqt_plot
```

- Topic 입력란에 `/turtle1/cmd_vel/linear/x`를 입력하고 추가(Day 2 자료 7.6 실습 2와 같은 방법)
- 그래프에는 **속도만** 표시 — 거리(0\~200cm)를 함께 표시하면 세로축 범위가 거리에 맞추어 설정되어 속도(0\~1.0)의 변화가 거의 드러나지 않습니다
- 거리 값은 다른 터미널의 `ros2 topic echo /us_dist`로 확인(4.4)

- 손을 가까이 가져가면 속도 그래프가 비스듬히 0까지 내려가고 turtle이 서서히 멈춤 → 정상 · 아래 관찰로 진행
- `No executable found`가 나오면 → `setup.py` 등록 줄을 확인하고 **빌드부터 다시**
- 속도 그래프가 두 값 사이를 오가면 → `dist_turtle`이 아직 실행 중입니다. 해당 터미널에서 Ctrl+C로 종료
- 노드가 `ZeroDivisionError`로 종료되면 → `slow_cm`을 `stop_cm`과 같은 값으로 설정한 경우입니다. 기본값으로 **다시 실행**
- 그래프 창이 열리지 않고 표시 장치 오류가 나오면 → 원격 데스크톱 화면의 터미널에서 **다시** 실행
- `Package 'rqt_plot' not found`가 나오면 → `sudo apt install -y ros-jazzy-rqt-plot` 후 **다시** 실행

**관찰** — 손을 80cm 거리에서 천천히 가까이 가져가며 그래프를 확인합니다.

| 실험 | 방법 | 관찰 |
|------|------|------|
| 천천히 접근 | 손을 80cm → 20cm로 천천히 이동 | 속도가 1.0에서 0까지 **비스듬히** 감소 · turtle이 서서히 정지 |
| 감속 구간 변경 | `ros2 param set /dist_turtle_slow slow_cm 40.0` | 40cm부터 감속 — 같은 접근에서 기울기가 가팔라짐 |
| 온오프 제어와 비교 | 이 노드 종료 → `dist_turtle` 실행 → 같은 동작 | 1.0에서 0으로 **한 번에** 바뀌는 계단 모양 |

- 두 노드의 결과가 한 그래프 창에 차례로 그려지므로 모양의 차이를 바로 비교할 수 있습니다

| 구분 | 온오프 제어 (4.5) | 비례 제어 (이 절) |
|------|------|------|
| 속도 명령 | `speed` 또는 0 — 두 값 | 거리에 따라 연속으로 변화 |
| 그래프 | 계단 모양 | 기울기가 있는 직선 |
| 정지 직전 속도 | 최고 속도 | 0에 가까움 |
| 실물 차량 | 관성으로 기준 거리를 지나쳐 정지 | 미리 감속해 지나치는 거리가 줄어듦 |

- 목표와 측정값의 차이(여기서는 `dist - stop_cm`)에 비례해 명령을 정하는 방식 = **비례 제어**(proportional control) — 피드백 제어(측정값을 명령에 반영하는 제어)의 기본 형태
- 이 노드도 remapping으로 실물 차량에 연결할 수 있습니다(8.4 도전 ⓐ와 같은 방법) — 공중 시험 후 넓은 공간에서 저속으로 확인

---

## 11. 다음 시간

**Day 7 — SLAM 실습**(10/12 · 10/5 휴강 후)

| 항목 | 내용 |
|------|------|
| 환경 | **강의실 PC의 WSL2** — Day 7 하루만 PC로 복귀하고, Day 8부터 다시 RPi5 |
| 도구 | Gazebo(3D 시뮬레이터) · RViz(시각화) · slam_toolbox · Nav2 |
| 내용 | TurtleBot3 모델로 지도를 작성하고, 그 지도 위에서 목표 지점까지 주행 |
| 연결 | 오늘 만든 세 노드(`us_node`·`dist_turtle`·`motor_node`)는 **Day 8·9**에서 확장합니다 |
