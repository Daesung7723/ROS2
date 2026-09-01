# Day 3 — 패키지와 미로 자율주행

**2026-09-07 · 한국폴리텍대학교 하이테크과정 ROS2**

이 자료는 수업의 개념 설명과 실습 절차·명령어를 복습용으로 정리한 것입니다. 복습 기준 = 이 자료 + 수업 중 필기.

---

## 목차

1. [오늘의 목표](#1-오늘의-목표)
2. [패키지와 워크스페이스 — 개념](#2-패키지와-워크스페이스--개념)
3. [실습 ① — 워크스페이스·패키지 만들기](#3-실습--워크스페이스패키지-만들기)
4. [실습 ② — 스크립트를 패키지로 옮기기](#4-실습--스크립트를-패키지로-옮기기)
5. [파라미터 — 값을 밖에서 주입](#5-파라미터--값을-밖에서-주입)
6. [package.xml — 의존성 선언](#6-packagexml--의존성-선언)
7. [커스텀 인터페이스 — 메시지 타입 직접 정의](#7-커스텀-인터페이스--메시지-타입-직접-정의)
8. [멀티 노드와 launch](#8-멀티-노드와-launch)
9. [미니프로젝트 — 미로 자율주행](#9-미니프로젝트--미로-자율주행)
10. [문제 해결 — 오류 진단](#10-문제-해결--오류-진단)
11. [다음 시간 준비 — RPi5 환경 구축 (과제)](#11-다음-시간-준비--rpi5-환경-구축-과제)

---

## 1. 오늘의 목표

Day 2에서 만든 스크립트를 **패키지 체계로 옮기고**, 여러 노드를 구성해 미로를 자율주행하는 것이 오늘의 목표입니다.

Day 2 스크립트 방식(`python3 파일명.py`)의 한계:

| 한계 | 내용 |
|------|------|
| 실행 방식 | 파일 위치를 알아야 실행 가능 — `ros2 run` 불가 |
| 배포·공유 | 파일을 낱개로 전달 — 의존 정보·실행 목록이 없음 |
| 설정값 변경 | 속도·반지름이 코드에 고정 — 값 하나 바꾸려면 코드 수정 |
| 다중 노드 | 터미널마다 파일 경로를 찾아 실행 — 일괄 실행 수단 없음 |

오늘의 해결 수단:

| 수단 | 해결하는 것 | 해당 절 |
|------|------|:--:|
| **패키지** | 배포 단위 — `ros2 run`으로 실행 | 2·3·4 |
| **colcon** | 빌드 — 코드를 실행 형태로 배치 | 2·3 |
| **파라미터** | 설정값을 코드 밖으로 분리 | 5 |
| **package.xml** | 의존 패키지 선언 — 다른 환경에서도 동작 | 6 |
| **커스텀 인터페이스** | 여러 값을 한 메시지로 묶어 전달 | 7 |
| **launch** | 여러 노드 일괄 실행 | 8 |

오늘 완성할 것 — **멀티 노드 구성으로 미로를 자율주행하는 turtlesim** (Day 6 판단 노드·Day 9 실물 회피 주행의 예행)

Day 2와의 관계:

| Day 2 내용 | 오늘과의 관계 |
|------|------|
| 통신 메커니즘 — 토픽·서비스·액션 CLI(Command Line Interface) 제어 | 미로 주행에서 코드·CLI로 재사용 |
| rclpy 노드 구조 — 생애주기·콜백·타이머·클래스 | 오늘 작성하는 모든 노드의 골격 |
| `.py` 스크립트 3종(circle_driver·pose_printer·square_driver) | 오늘 **패키지로 옮기는** 대상 |

수업 시작 시 환경 점검:

```bash
ros2 doctor                              # 설치 상태 재확인
ros2 run turtlesim turtlesim_node        # turtlesim 실행 유지 (오늘 내내 사용)
```

---

## 2. 패키지와 워크스페이스 — 개념

### 2.1 패키지 — 코드의 배포 단위

**패키지(package)** = 노드·설정·의존 정보를 하나로 담은 ROS2 코드의 기본 단위입니다.

| 패키지에 담기는 것 | 예 |
|------|------|
| 노드 코드 | `circle_driver.py` (Day 2 작성 — 오늘 옮김) |
| 패키지 정보·의존 선언 | `package.xml` |
| 실행 등록 | `setup.py` — `ros2 run`이 찾는 목록 |
| 인터페이스 정의 | 커스텀 메시지·서비스 (7장) |

- 패키지 = **공유·재사용의 단위** — ROS 생태계의 유통 단위가 바로 이것입니다
- **메타 패키지(meta package)** — 관련 패키지 여러 개를 묶어 한 번에 설치되게 하는 상위 묶음입니다(예: `desktop` 변형이 rqt·시각화 도구를 함께 설치). ROS2에서는 별도 종류가 아니라 **코드 없이 의존만 선언한 패키지**로 구현되며, 이 과정에서 직접 만들지는 않습니다

이미 패키지를 사용해 왔습니다 — Day 1 명령의 구조를 다시 봅니다:

```bash
ros2 pkg list | grep turtlesim      # 설치된 패키지 목록에서 turtlesim 확인
ros2 pkg executables turtlesim      # 패키지가 제공하는 실행 노드 목록
```

- `pkg executables` — 패키지에 등록된 실행 노드를 나열하는 동작입니다(`turtlesim_node`·`turtle_teleop_key`가 여기 있습니다)
- `ros2 run turtlesim turtlesim_node` = "**turtlesim 패키지**의 turtlesim_node 실행" — Day 1부터 쓴 명령의 앞자리가 패키지명이었던 이유입니다
- 오늘의 목표 = 이 목록에 **내 패키지가 등장**하게 만드는 것

### 2.2 워크스페이스 — 패키지 작업 공간

**워크스페이스(workspace)** = 패키지를 만들고 빌드하는 작업 폴더입니다.

```
~/ros2_ws/
├── src/       ← 소스 코드 (내가 작성하는 곳)
├── build/     ← 빌드 중간 산출물 (자동 생성)
├── install/   ← 실행 가능한 결과물 + 환경 스크립트 (자동 생성)
└── log/       ← 빌드 기록 (자동 생성)
```

| 원칙 | 이유 |
|------|------|
| 직접 편집하는 곳은 **src뿐** | build·install·log는 빌드가 만들고 관리 — 직접 수정하면 다음 빌드에서 덮어써짐 |
| 실행되는 것은 **install의 사본** | src 수정만으로는 실행에 반영되지 않음 → **재빌드 필요** |
| 위치·이름은 자유 | 이 과정은 `~/ros2_ws`로 통일 |

**두 개의 환경이 함께 동작합니다** — ROS2 본체와 내 워크스페이스:

| 층 | 위치 | 내용 | 관리 |
|------|------|------|------|
| ROS2 본체 | `/opt/ros/jazzy/` | rclpy·turtlesim 등 기본 패키지 | apt (수정하지 않음) |
| 내 워크스페이스 | `~/ros2_ws/` | my_first_pkg 등 내 패키지 | 내가 작성·빌드 |

```
/opt/ros/jazzy       ──source──┐
                               ├──▶ 셸 ──ros2 run──▶ 양쪽 패키지를 모두 찾음
~/ros2_ws/install    ──source──┘
```

- 두 층을 각각 `source`로 셸에 등록하면 `ros2 run`이 **양쪽 패키지를 모두** 찾습니다
- Day 1에서 본체를 등록했고, 오늘 3.4에서 내 워크스페이스를 추가 등록합니다

### 2.3 빌드 흐름 — colcon → source → run

**colcon** = ROS2 표준 빌드 도구입니다. 작업은 아래 순환으로 진행됩니다:

```
src에 코드 작성·수정 ──▶ colcon build ──▶ source install/local_setup.bash ──▶ ros2 run 실행
        ▲                                                                          │
        └──────────────────────────────────────────────────────────────────────────┘
```

| 단계 | 하는 일 | 왜 필요한가 |
|------|------|------|
| `colcon build` | src를 읽어 패키지별로 빌드 → install에 실행 형태로 배치 | 실행되는 것은 install의 사본 — src 수정은 빌드해야 반영 |
| `source install/local_setup.bash` | install의 등록 정보를 **현재 셸이 다시 읽음** | 새로 등록된 노드를 셸이 인식해야 `ros2 run`이 찾음 |
| `ros2 run 패키지 노드` | 등록된 내 노드 실행 | — |

- `colcon build`는 **항상 워크스페이스 최상위**(`~/ros2_ws`)에서 실행합니다 — src 내부에서 실행하면 잘못된 위치에 build 폴더가 생성됩니다
- source를 `.bashrc`에 등록해 두면 새 터미널은 자동입니다. 단 **빌드 직후의 현재 터미널**은 수동 source가 필요합니다
- 오늘 반복할 순환 = **수정 → 빌드 → (source) → 실행**

> **자주 하는 실수 —** 빌드 직후 같은 터미널에서 바로 `ros2 run`을 실행하면 새로 등록한 노드를 찾지 못합니다. `.bashrc` 등록은 새 터미널에만 적용되므로, 빌드한 터미널에서는 `source install/local_setup.bash`를 직접 실행해야 합니다.

---

## 3. 실습 ① — 워크스페이스·패키지 만들기

### 3.1 워크스페이스 생성

```bash
mkdir -p ~/ros2_ws/src    # 폴더 생성 (-p = 중간 경로까지 한 번에)
cd ~/ros2_ws
colcon build              # 첫 빌드 — src가 비어 있어도 실행 가능
ls                        # build·install·log 폴더 생성 확인
```

- 빈 빌드의 목적 — 2.2에서 배운 4폴더 구조가 **실제로 만들어지는 것**을 눈으로 확인하는 것입니다

**구조 확인 도구 — tree**

`ls`는 현재 폴더의 항목만 나열하므로 하위 폴더 **안쪽**은 보이지 않습니다. **tree**는 계층 구조를 들여쓰기로 한 번에 출력합니다.

```bash
sudo apt install tree     # 최초 1회 설치
tree -L 2                 # 현재 위치부터 2단계 깊이까지 표시
```

출력:

```
.
├── build
├── install
├── log
└── src
```

| 옵션 | 용도 |
|------|------|
| `-L N` | 깊이 N단계까지만 — 워크스페이스 조회의 기본 |
| `-d` | 폴더만 표시 — 전체 골격 파악용 |
| `-a` | 숨김 파일 포함 |

- `-L N`이 필요한 이유 — **빌드 산출물(build·install)은 파일이 수백 개**이므로, 깊이 제한 없이 실행하면 출력이 화면을 넘어갑니다
- 앞으로 패키지 구조를 확인할 때 계속 사용합니다(3.2·4.2)

### 3.2 패키지 생성

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_first_pkg
```

- `ros2 pkg create` — 패키지 골격 생성 (`pkg` 대상 + `create` 동작 + 옵션·이름 인자)
- `--build-type ament_python` — Python 패키지로 지정 (ament = ROS2 빌드 시스템)

생성된 구조 확인:

```bash
tree my_first_pkg
```

```
my_first_pkg/
├── my_first_pkg/        ← Python 코드가 들어갈 폴더 (패키지명과 동일 — 주의)
│   └── __init__.py
├── resource/
│   └── my_first_pkg     ← 패키지 등록 표시 파일
├── test/                ← 코드 검사용 자동 생성 (이 과정에서는 사용하지 않음)
├── package.xml          ← 패키지 정보·의존성 선언
├── setup.py             ← 실행 노드 등록 (entry_points)
└── setup.cfg
```

- **같은 이름의 폴더가 이중 구조**입니다 — 바깥 = 패키지 전체 / **안쪽 = 코드를 넣는 곳**(4장에서 스크립트를 옮길 위치)
- `ls`로는 두 이름이 같은 항목으로만 보이지만, tree는 **어느 쪽이 상위인지 들여쓰기로 구분**합니다. 이 구조를 혼동하면 4장에서 파일을 잘못된 위치에 배치하게 됩니다

### 3.3 생성된 파일의 역할

| 파일 | 역할 | 오늘 편집 여부 |
|------|------|:--:|
| `package.xml` | 패키지 이름·버전·설명·**의존 패키지 선언** | 6장에서 편집 |
| `setup.py` | **실행 노드 등록** — `ros2 run`이 참조하는 entry_points 목록 | 4.2에서 편집 |
| `setup.cfg` | 설치 경로 설정 | 편집하지 않음 |
| `my_first_pkg/__init__.py` | Python 패키지 표시 파일 | 편집하지 않음 |
| `resource/my_first_pkg` | 패키지 등록 표시 파일 — ROS2가 설치된 패키지를 찾는 근거 | 편집하지 않음 |
| `test/` | 코드 형식 검사 스크립트 (자동 생성) | 이 과정에서는 사용하지 않음 |

```bash
cat my_first_pkg/package.xml    # 내용 관찰 — name·version·description 확인
```

- 지금은 코드 없는 빈 패키지 상태입니다. 4장에서 **스크립트 이동 → setup.py 등록 → 빌드** 순서로 채웁니다

### 3.4 빌드와 등록

```bash
cd ~/ros2_ws
colcon build
echo "source ~/ros2_ws/install/local_setup.bash" >> ~/.bashrc    # 새 터미널마다 자동 등록
source ~/.bashrc
ros2 pkg list | grep my_first_pkg    # 내 패키지가 목록에 있는지 확인
```

- `grep` — 출력에서 특정 문자열만 추려 표시합니다(`|` = 앞 명령의 출력을 뒤 명령의 입력으로 연결)
- 등록 방식은 Day 1에서 ROS2 본체를 등록한 것과 동일한 구조입니다

> **Tip —** Python 패키지는 `colcon build --symlink-install`로 빌드하면 코드 수정 시 재빌드 없이 바로 반영됩니다(install에 복사 대신 원본 연결을 배치하는 옵션). 오늘처럼 코드를 자주 고치는 날에 유용합니다.

---

## 4. 실습 ② — 스크립트를 패키지로 옮기기

### 4.1 옮길 대상

| Day 2 스크립트 | 기능 | 옮긴 후 실행 |
|------|------|------|
| `circle_driver.py` | 타이머 콜백 → Twist 발행 (원 주행) | `ros2 run my_first_pkg circle_driver` |
| `pose_printer.py` | Pose 구독 → 위치 로그 | `ros2 run my_first_pkg pose_printer` |
| `square_driver.py` | 상태 전환 → 정사각형 궤적 | `ros2 run my_first_pkg square_driver` |

- 코드는 **한 글자도 바뀌지 않습니다** — 바뀌는 것은 위치(패키지 안)와 실행 방법(`python3` → `ros2 run`)뿐입니다
- 코드 내용·해설은 Day 2 자료 7장을 참조합니다

### 4.2 배치와 등록 — circle_driver 기준

Day 2 스크립트 파일을 패키지 **안쪽** 폴더로 복사합니다:

```bash
cp ~/ros2_scripts/circle_driver.py ~/ros2_ws/src/my_first_pkg/my_first_pkg/
tree ~/ros2_ws/src/my_first_pkg -L 2    # 파일이 안쪽 폴더에 놓였는지 확인
```

- 확인 지점 — `circle_driver.py`가 **안쪽** `my_first_pkg/` 아래에 있어야 합니다(3.2 이중 구조)
- **바깥 폴더에 복사하는 것이 이 단계에서 가장 흔한 실수**입니다 — 빌드는 성공하나 `ros2 run`이 노드를 찾지 못합니다. tree로 위치를 먼저 확인하면 빌드 전에 발견할 수 있습니다

`setup.py`의 `entry_points`에 실행 이름을 등록합니다:

```python
entry_points={
    'console_scripts': [
        'circle_driver = my_first_pkg.circle_driver:main',
    ],
},
```

- 형식 — `'실행이름 = 패키지.파일명:함수'` (`ros2 run`이 이 등록을 찾아 main을 호출합니다)

### 4.3 빌드와 실행

```bash
cd ~/ros2_ws
colcon build
source install/local_setup.bash
ros2 run my_first_pkg circle_driver    # turtlesim이 원 궤적 주행 시작
```

| 구분 | Day 2 (스크립트) | 오늘 (패키지) |
|------|------|------|
| 실행 | `python3 ~/ros2_scripts/circle_driver.py` | `ros2 run my_first_pkg circle_driver` |
| 전제 | 파일 경로를 알아야 함 | 셸 등록만 되어 있으면 어디서든 |
| 수정 반영 | 저장 즉시 | **재빌드 필요** (`--symlink-install`로 생략 가능) |

- 종료 = `Ctrl+C` (spin 루프 중단)

### 4.4 관찰

새 터미널에서 실행합니다:

```bash
ros2 node list                      # /circle_driver 표시 확인
ros2 topic info /turtle1/cmd_vel    # 발행 수 1 = 내 노드
rqt_graph                           # circle_driver → /turtle1/cmd_vel → turtlesim 연결 확인
```

- Day 2와 같은 동작·같은 그래프입니다 — **실행 경로만 패키지 체계로 바뀌었습니다**

---

## 5. 파라미터 — 값을 밖에서 주입

### 5.1 왜 파라미터인가

4장에서 옮긴 circle_driver는 궤적을 결정하는 두 값이 코드 안에 고정되어 있습니다:

```python
msg.linear.x = 2.0      # 전진 속도
msg.angular.z = 1.0     # 회전 속도 → 반지름 = 전진 ÷ 회전 = 2.0
```

| 하려는 것 | 지금 방식 | 문제 |
|------|------|------|
| 반지름을 바꿔 실험 | 코드 수정 → colcon build → 실행 | 시도 1회마다 빌드 대기 |
| 학생마다 다른 값 적용 | 각자 코드를 수정 | 코드가 서로 달라져 같은 파일을 공유할 수 없음 |
| 주행 중 조정 | 불가 — 종료·수정·재실행 | 움직이는 로봇을 정지시켜야 수정 가능 |

**파라미터(parameter)** = 노드가 외부에서 읽고 쓸 수 있도록 공개한 설정값입니다.

| 항목 | 내용 |
|------|------|
| 보유 주체 | 모든 노드가 자신의 **파라미터 서버**를 보유 — RCL(ROS Client Library)의 기본 기능 |
| 접근 방법 | 외부(CLI·다른 노드)가 **서비스 통신**으로 조회·설정 (Day 2 요청-응답 구조의 응용) |
| 다른 노드 접근 | 노드는 **파라미터 클라이언트**도 가질 수 있음 — 자기 값뿐 아니라 다른 노드의 값도 읽고 씀 |
| 값의 위치 | 코드가 아니라 **노드 실행 환경** — 코드 수정 없이 동작을 바꾸는 수단 |
| 저장 형식 | **yaml 파일** — 설정 전체를 파일로 저장·재사용 가능 (5.6) |

- 자율차 대응 — 주행 속도·회피 거리·조향 각도를 코드 밖으로 분리하면 **주행 중 튜닝**이 가능합니다(Day 8~9 실물 조정에서 필수)

### 5.2 CLI로 먼저 확인 — turtlesim의 파라미터

turtlesim은 배경색을 이미 파라미터로 공개하고 있습니다:

```bash
ros2 param list                            # 실행 중 노드들의 파라미터 목록
ros2 param get /turtlesim background_b     # 값 조회
ros2 param set /turtlesim background_b 0   # 값 설정 — 배경색 즉시 변경
```

- 확인 — turtlesim을 **재시작하지 않고** 색이 바뀝니다. 값이 코드 밖에 있기 때문입니다
- 오늘의 목표 = 내 노드의 속도·회전값도 이와 같은 방식으로 다루는 것

### 5.3 파라미터 선언 — circle_driver 개조

설계를 먼저 정합니다:

| 항목 | 값 | 근거 |
|------|------|------|
| 파라미터 이름 | `linear_speed` · `angular_speed` | 발행 값과 이름을 대응 |
| 타입 | 실수 | 기본값의 타입이 파라미터 타입을 결정 |
| 기본값 | 2.0 · 1.0 | 기존 고정값 그대로 — 개조 전후 동작 동일 |
| 읽는 시점 | 노드 생성 시 1회 | 실행 중 갱신은 5.5에서 추가 |

`~/ros2_ws/src/my_first_pkg/my_first_pkg/circle_driver.py` 수정:

```python
class CircleDriver(Node):
    def __init__(self):
        super().__init__('circle_driver')
        self.declare_parameter('linear_speed', 2.0)                # ① 선언 + 기본값
        self.declare_parameter('angular_speed', 1.0)
        self.linear  = self.get_parameter('linear_speed').value    # ② 값 읽기
        self.angular = self.get_parameter('angular_speed').value
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x  = self.linear        # ③ 고정값 → 읽어 온 설정값
        msg.angular.z = self.angular
        self.pub.publish(msg)
```

| # | 코드 | 하는 일 |
|:--:|------|------|
| ① | `declare_parameter(이름, 기본값)` | 이 노드가 해당 이름의 파라미터를 가진다고 **선언** — 선언하지 않은 이름은 외부에서 설정 불가 |
| ② | `get_parameter(이름).value` | 선언된 파라미터의 현재 값을 읽음 (`.value`를 생략하면 값이 아니라 파라미터 객체가 반환됨) |
| ③ | `self.linear` | 발행 값이 코드 상수가 아니라 설정값 변수 |

- 나머지 구조는 무변경입니다 — Day 2 circle_driver 골격 그대로입니다

### 5.4 실행 시 주입 — `--ros-args -p`

```bash
cd ~/ros2_ws && colcon build && source install/local_setup.bash
ros2 run my_first_pkg circle_driver                                       # 기본값 — 반지름 2.0
ros2 run my_first_pkg circle_driver --ros-args -p angular_speed:=0.5      # 반지름 4.0
ros2 run my_first_pkg circle_driver --ros-args -p linear_speed:=1.0 -p angular_speed:=2.0   # 반지름 0.5
```

- `--ros-args` — 뒤따르는 인자를 ROS2 실행 옵션으로 해석하라는 구분자입니다(프로그램 자체 인자와 분리하는 표시)
- `-p 이름:=값` — 파라미터 주입 표기입니다. 대입 기호가 `=`이 아니라 **`:=`**입니다
- `&&` — 앞 명령이 성공한 경우에만 뒤 명령을 실행하는 셸 연결 기호입니다
- 확인 — **코드는 동일한데 궤적이 달라집니다.** 세 번의 실험에 재빌드가 없었습니다

### 5.5 실행 중 변경 — 파라미터 콜백

5.4는 **시작 시점**에 한 번 정하는 방식입니다. 주행 중 변경을 반영하려면 변경 통지를 받아야 합니다:

```python
from rcl_interfaces.msg import SetParametersResult   # 파라미터 변경 결과 타입

        # __init__ 말미에 추가
        self.add_on_set_parameters_callback(self.param_callback)   # ④ 변경 콜백 등록

    def param_callback(self, params):        # ⑤ 외부에서 set 할 때 호출됨
        for p in params:
            if p.name == 'linear_speed':
                self.linear = p.value
            elif p.name == 'angular_speed':
                self.angular = p.value
        self.get_logger().info(f'speed = {self.linear} / {self.angular}')
        return SetParametersResult(successful=True)   # ⑥ 수락 응답
```

| # | 하는 일 |
|:--:|------|
| ④ | 변경 콜백 등록 — Day 2 "호출의 역전"과 같은 구조(등록만 하고 호출은 ROS2가 수행) |
| ⑤ | 변경된 파라미터 목록을 받아 내부 변수 갱신 |
| ⑥ | 수락 여부 응답 — `successful=False`를 반환하면 변경이 **거부**됨(값 검증 지점) |

- 확장 지점 — ⑥에서 범위를 검사하면(예: 속도 0~5) 잘못된 값의 주입을 노드가 막을 수 있습니다

### 5.6 관찰과 설정 저장

재빌드·실행 후 새 터미널에서:

```bash
ros2 param list /circle_driver                    # linear_speed·angular_speed 등장
ros2 param get /circle_driver linear_speed        # 현재 값 확인
ros2 param set /circle_driver angular_speed 2.0   # 주행 중 변경 — 궤적이 그 자리에서 좁아짐
ros2 param describe /circle_driver linear_speed   # 값이 아니라 사양(타입·범위·읽기 전용) 출력
```

- 5.2에서 turtlesim에 사용한 명령과 **완전히 동일**합니다 — 내 노드도 같은 방식으로 다뤄집니다
- `param describe`는 값이 거부될 때의 1차 확인 수단입니다 — 타입이 double인지 integer인지가 여기 표시됩니다
- `rqt_graph`로 보면 파라미터 설정 시점에 서비스 호출이 발생합니다 — 파라미터가 서비스 위에 구현되었음의 확인입니다

**설정 저장과 재사용** — `param set`으로 맞춘 값은 **노드를 종료하면 사라집니다.** 찾아낸 설정을 다음 실행에 이어 쓰려면 파일로 저장합니다.

```bash
ros2 param dump /circle_driver > circle_driver.yaml    # 현재 파라미터 전체를 yaml로 저장
cat circle_driver.yaml                                  # 내용 확인
```

```yaml
/circle_driver:
  ros__parameters:
    linear_speed: 1.0
    angular_speed: 2.0
```

```bash
ros2 run my_first_pkg circle_driver --ros-args --params-file ./circle_driver.yaml
```

| 방식 | 값의 위치 | 지속성 |
|------|------|:--:|
| `--ros-args -p` (5.4) | 명령 줄 | 그 실행 한정 |
| `param set` (5.6) | 실행 중 메모리 | 종료 시 소멸 |
| **`--params-file`** | **yaml 파일** | **영구 — 다시 쓰고 공유 가능** |

- 실무 흐름 — `param set`으로 **튜닝** → `param dump`로 **저장** → `--params-file`로 **재현**
- Day 8~9 대응 — 실물 차량의 회피 거리·모터 보정값을 이 방식으로 차량별 파일로 관리합니다

> **자주 하는 실수**
>
> - `ros2 param set ... 2`처럼 **정수로 입력**하면 실수 타입 파라미터가 거부합니다 — `2.0`으로 입력합니다
> - `declare_parameter` 없이 `get_parameter`를 호출하면 실행 즉시 예외가 발생합니다. **선언이 먼저**입니다
> - 콜백(5.5)을 등록하지 않으면 `param set`은 성공 응답을 반환하지만 **주행은 그대로**입니다 — 노드 내부 변수가 갱신되지 않기 때문입니다

### 5.7 변형 과제 — square_driver 파라미터화

Day 2 square_driver의 고정값을 파라미터로 분리합니다.

1. **(필수)** 파라미터 `side_ticks`(정수, 기본 20)·`turn_speed`(실수, 기본 1.57)를 선언하고 코드의 고정값을 치환
2. **(필수)** `--ros-args -p side_ticks:=40`으로 실행해 **두 배 크기**의 정사각형 확인
3. **(도전)** `turn_speed:=2.09`를 주입해 **삼각형**이 그려지는지 확인 — Day 2 도전 2를 코드 수정 없이 재현

<details>
<summary><b>정답 및 풀이 보기</b></summary>

```python
class SquareDriver(Node):
    def __init__(self):
        super().__init__('square_driver')
        self.declare_parameter('side_ticks', 20)      # 정수 — 직진 구간 틱 수
        self.declare_parameter('turn_speed', 1.57)    # 실수 — 회전 각속도
        self.side = self.get_parameter('side_ticks').value
        self.turn = self.get_parameter('turn_speed').value
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.tick = 0
        self.moving = True

    def timer_callback(self):
        msg = Twist()
        if self.moving:
            msg.linear.x = 2.0
            if self.tick >= self.side:        # 고정 20 → 파라미터
                self.moving, self.tick = False, 0
        else:
            msg.angular.z = self.turn         # 고정 1.57 → 파라미터
            if self.tick >= 10:
                self.moving, self.tick = True, 0
        self.tick += 1
        self.pub.publish(msg)
```

- 타입 주의 — `side_ticks`는 기본값을 `20`(정수)으로 선언했으므로 주입도 `40`(정수)입니다. `40.0`은 거부됩니다
- (3) 삼각형 — 회전 1초에 120°가 되려면 2.09rad/s입니다. Day 2에서는 **코드를 고쳐** 바꿨으나 지금은 **같은 노드에 다른 값을 주입**해 바꿉니다
- 미로(9장) 연결 — 주행 속도·후진 시간·벽 좌표를 파라미터로 두면 난이도를 코드 수정 없이 조정할 수 있습니다

</details>

---

## 6. package.xml — 의존성 선언

### 6.1 왜 선언하는가

5.5에서 새 import가 하나 늘었습니다:

```python
from rcl_interfaces.msg import SetParametersResult
```

- 지금 빌드가 성공하는 이유 = 내 컴퓨터에 `rcl_interfaces`가 ROS2 본체와 함께 이미 설치되어 있어서입니다
- 이 패키지를 다른 컴퓨터로 전달하면 — 그 환경에 무엇이 설치되어 있는지 알 수 없습니다
- `package.xml` = **"이 패키지는 무엇을 필요로 하는가"의 선언서**입니다. 설치 도구가 이 목록을 읽어 의존 패키지를 준비합니다

| 선언하지 않으면 | 선언하면 |
|------|------|
| 내 환경에서만 동작 — 다른 환경에서 실행 시 `ModuleNotFoundError` | 설치 도구가 목록을 참조해 사전 설치 |
| 무엇이 필요한지 코드 전체를 확인해야 파악 | 파일 하나로 확인 |

### 6.2 선언 추가

`~/ros2_ws/src/my_first_pkg/package.xml` — `<license>` 줄 아래에 추가합니다:

```xml
<depend>rclpy</depend>
<depend>geometry_msgs</depend>
<depend>turtlesim</depend>
<depend>rcl_interfaces</depend>
```

| 선언 | 대응하는 코드 |
|------|------|
| `rclpy` | `import rclpy` — 모든 노드의 기반 |
| `geometry_msgs` | `from geometry_msgs.msg import Twist` (circle_driver) |
| `turtlesim` | `from turtlesim.msg import Pose` (pose_printer) |
| `rcl_interfaces` | `from rcl_interfaces.msg import SetParametersResult` (5.5) |

- `<depend>` = 빌드·실행 양쪽 의존을 한 번에 선언하는 축약 표기입니다(`<build_depend>`+`<exec_depend>`에 해당)
- **규칙 — import가 늘면 선언도 늘립니다.** "복사 + 등록 + 재빌드"에 이어지는 네 번째 항목입니다

### 6.3 확인과 누락 재현

```bash
cd ~/ros2_ws
colcon build
ros2 pkg xml my_first_pkg     # package.xml 내용 출력 — depend 4종 확인
rosdep check my_first_pkg     # 선언된 의존이 설치되어 있는지 점검
```

- `rosdep` — 선언 목록을 읽어 의존 패키지를 점검·설치하는 도구입니다. 일괄 설치는 `rosdep install --from-paths src --ignore-src -y`

누락 상태를 재현해 확인합니다:

| 순서 | 조작 | 확인 |
|:--:|------|------|
| 1 | `rcl_interfaces` 선언 줄을 주석 처리(`<!-- ... -->`) | `colcon build` → `ros2 pkg xml`에서 해당 줄이 사라짐 |
| 2 | 그대로 노드 실행 | **정상 동작** — 내 환경에는 설치돼 있으므로 |
| 3 | 주석 해제 → 재빌드 | 선언 목록 복귀 |

- 핵심 — 선언을 지워도 **내 컴퓨터에서는 빌드·실행이 그대로 성공**합니다. 결함이 드러나는 시점은 다른 환경에 전달했을 때입니다
- **이 지연된 실패가 선언을 습관으로 만들어야 하는 이유**입니다 — 배포 자료를 각자 내려받아 빌드할 때 그대로 겪게 됩니다

---

## 7. 커스텀 인터페이스 — 메시지 타입 직접 정의

### 7.1 왜 직접 정의하는가

지금까지 사용한 `Twist`·`Pose`는 ROS2가 미리 정의해 둔 타입입니다. 자율차에서는 **여러 값을 묶어** 보내야 하는 상황이 생깁니다.

| 상황 | 필요한 데이터 | 표준 타입으로는 |
|------|------|------|
| 벽 감지 결과 전달 | 거리 + 통과 가능 여부 + 판정 | 세 토픽으로 나눠야 함 — **동시성이 깨짐** |
| 표지판 인식 결과 (Day 6) | 분류 이름 + 신뢰도 | 문자열·실수를 따로 발행 |
| 초음파 거리 (Day 8) | 거리 값 | `Float32`로 가능하나 의미가 드러나지 않음 |

- 한 메시지로 묶으면 **같은 시점의 값**임이 보장됩니다 — 따로 발행하면 도착 순서가 어긋날 수 있습니다

| 인터페이스 | 파일 | 쓰이는 통신 |
|------|------|------|
| msg | `*.msg` | 토픽 |
| srv | `*.srv` | 서비스 (요청/응답 2부) |
| action | `*.action` | 액션 (목표/결과/피드백 3부) |

### 7.2 인터페이스 전용 패키지 만들기

> **제약 —** Python 패키지(`ament_python`)에서는 인터페이스를 생성할 수 없습니다. **별도 CMake 패키지**가 필요합니다.

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_msgs   # C++ 계열 빌드 타입
mkdir my_msgs/msg
```

`my_msgs/msg/WallStatus.msg` 작성 — 타입과 필드명을 한 줄씩 적습니다:

```
float32 distance      # 가장 가까운 벽까지의 거리
bool    blocked       # 통과 가능 여부
string  action        # 'run' / 'back' / 'turn'
```

- 사용 가능한 자료형 — `bool`·`int32`·`float32`·`float64`·`string` + 배열 `float32[]`
- `#` 뒤는 주석입니다 — 각 필드의 의미와 **단위**를 반드시 적습니다(Day 1 표준 단위)

### 7.3 빌드 설정 — 두 파일 수정

`CMakeLists.txt`에 생성 규칙 추가 (`ament_package()` 위):

```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME} "msg/WallStatus.msg")
```

`package.xml`에 추가:

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

| 항목 | 하는 일 |
|------|------|
| `rosidl_generate_interfaces` | 어느 `.msg` 파일을 변환할지 지정 — **파일을 추가하면 이 줄도 늘립니다** |
| `rosidl` | 인터페이스 정의를 각 언어(Python·C++)의 코드로 **자동 변환**하는 도구 (IDL, Interface Definition Language — 인터페이스 정의 언어) |
| `member_of_group` | 이 패키지가 인터페이스 제공 패키지임을 선언 |

- 6장에서 배운 의존 선언의 연장입니다 — **인터페이스 패키지는 선언 항목이 다릅니다**

### 7.4 빌드·확인·사용

```bash
cd ~/ros2_ws && colcon build
source install/local_setup.bash
ros2 interface show my_msgs/msg/WallStatus    # 등록 확인
```

```python
from my_msgs.msg import WallStatus

self.pub = self.create_publisher(WallStatus, '/wall_status', 10)
msg = WallStatus()
msg.distance, msg.blocked, msg.action = 0.8, True, 'back'
self.pub.publish(msg)
```

```bash
ros2 topic echo /wall_status    # 세 필드가 한 묶음으로 출력됨
```

> **자주 하는 실수**
>
> - **사용하는 쪽 패키지에도 의존을 선언**해야 합니다 — `my_first_pkg/package.xml`에 `<depend>my_msgs</depend>`
> - 인터페이스를 고친 뒤에는 **`my_msgs`부터 빌드**해야 합니다. 사용하는 쪽만 빌드하면 옛 정의가 남습니다
> - `.msg` 파일 이름은 **대문자로 시작**합니다(`WallStatus.msg`) — 소문자면 생성이 실패합니다

**이후 연결** — 이 절이 뒤에서 쓰이는 곳:

| Day | 용도 |
|:--:|------|
| 6 | 인식 노드 → 판단 노드로 보내는 **분류 이름 + 신뢰도** |
| **8** | **RPi5 초음파 거리 퍼블리셔** — 거리 값을 담는 `.msg`를 정의해 실물 센서 노드에 사용 |
| 9 | 회피 통합에서 센서·판단 결과를 한 묶음으로 전달 |

---

## 8. 멀티 노드와 launch

### 8.1 남은 스크립트 일괄 이동

같은 절차의 반복입니다 — **파일 복사 + 한 줄 등록 + 재빌드**(앞으로 모든 노드 추가의 표준 절차):

```bash
cp ~/ros2_scripts/pose_printer.py  ~/ros2_ws/src/my_first_pkg/my_first_pkg/
cp ~/ros2_scripts/square_driver.py ~/ros2_ws/src/my_first_pkg/my_first_pkg/
```

`setup.py` — 등록 3종 완성:

```python
entry_points={
    'console_scripts': [
        'circle_driver = my_first_pkg.circle_driver:main',
        'pose_printer = my_first_pkg.pose_printer:main',
        'square_driver = my_first_pkg.square_driver:main',
    ],
},
```

```bash
cd ~/ros2_ws
colcon build
source install/local_setup.bash
ros2 pkg executables my_first_pkg    # 등록 3종 확인
```

### 8.2 멀티 노드 동시 실행

터미널 3개로 실행합니다 — turtlesim + 내 노드 2개:

```bash
# 터미널 1
ros2 run turtlesim turtlesim_node
# 터미널 2
ros2 run my_first_pkg circle_driver
# 터미널 3
ros2 run my_first_pkg pose_printer
```

```
circle_driver ──/turtle1/cmd_vel──▶ turtlesim ──/turtle1/pose──▶ pose_printer
```

- 내 노드가 **양쪽에서** turtlesim과 통신합니다 — 명령은 보내고 상태는 받는 구조입니다
- 자율차 대응 — 모터 제어 / 센서 수신이며, **둘을 한 노드에 합치면 판단 노드**가 됩니다(미니프로젝트의 maze_driver가 그 형태)
- **멀티 노드 설계 원칙** — 기능 하나당 노드 하나: 관찰(pose_printer)과 구동(circle_driver)을 분리하면 하나를 교체·정지해도 나머지가 동작합니다
- 확인된 불편 — **노드 하나당 터미널 하나**입니다. 노드가 늘수록 실행·종료 관리가 번거로워집니다 → launch가 해결합니다

### 8.3 launch 파일 — Python으로 실행 구성

**launch** = 여러 노드의 실행 구성을 파일로 선언해 **한 명령으로 일괄 실행**하는 체계입니다.

`~/ros2_ws/src/my_first_pkg/launch/demo_launch.py` 생성(`launch` 폴더 신규):

```python
from launch import LaunchDescription        # 실행 구성 선언
from launch_ros.actions import Node         # 실행할 노드 항목

def generate_launch_description():          # launch가 찾는 표준 함수 이름
    return LaunchDescription([
        Node(package='turtlesim',    executable='turtlesim_node'),
        Node(package='my_first_pkg', executable='circle_driver'),
        Node(package='my_first_pkg', executable='pose_printer'),
    ])
```

- `Node(package=, executable=)` — "어느 패키지의 어느 실행 이름을 실행하라"는 선언입니다 — `ros2 run`의 두 인자와 동일합니다

`setup.py`에 launch 폴더 배포 등록 (`data_files` 항목에 추가):

```python
import os
from glob import glob                       # glob = 패턴과 일치하는 파일 목록을 얻는 함수
# data_files 목록에 아래 항목 추가
(os.path.join('share', 'my_first_pkg', 'launch'), glob('launch/*_launch.py')),
```

```bash
cd ~/ros2_ws
colcon build
source install/local_setup.bash
ros2 launch my_first_pkg demo_launch.py    # 터미널 1개로 노드 3개 일괄 실행
```

- `ros2 launch <패키지> <launch 파일>` — launch 실행 명령입니다(`run` = 노드 1개 / `launch` = 구성 일괄)
- 종료 = `Ctrl+C` 한 번으로 **전체 노드 동시 종료** — 실행·종료 관리 문제가 해소됩니다
- Day 6(시뮬레이션 통합)·최종 프로젝트에서 노드 4~5개를 이 방식으로 운영합니다

### 8.4 launch에 파라미터 담기

5.4의 `--ros-args -p`는 명령 줄에 매번 값을 입력하는 방식입니다. launch 파일에 담으면 **실행 구성 전체가 파일 하나**로 정리됩니다:

```python
    return LaunchDescription([
        Node(package='turtlesim',    executable='turtlesim_node',
             parameters=[{'background_b': 100}]),                    # turtlesim 배경색
        Node(package='my_first_pkg', executable='circle_driver',
             parameters=[{'linear_speed': 1.0, 'angular_speed': 2.0}]),   # 반지름 0.5
        Node(package='my_first_pkg', executable='pose_printer'),
    ])
```

| 방식 | 값의 위치 | 적합한 상황 |
|------|------|------|
| `--ros-args -p` (5.4) | 명령 줄 | 값을 바꿔 가며 **실험**할 때 |
| `param set` (5.6) | 실행 중 | 동작을 보면서 **튜닝**할 때 |
| launch `parameters=` | launch 파일 | 확정된 설정으로 **반복 실행**할 때 |

- 확인 — 배경색과 궤적이 launch 파일의 값대로 시작됩니다. 실행 명령에는 값이 없습니다
- Day 6·Day 9 대응 — 카메라 해상도·회피 거리 같은 설정이 늘어나면 launch 파일이 **차량 전체의 설정 파일** 역할을 합니다

---

## 9. 미니프로젝트 — 미로 자율주행

### 9.1 과제 — 가상 벽 미로

- 미로 = **가상 벽**입니다. 실제 장애물이 아니라 **좌표 범위로 정의한 벽**이며, 위치(Pose)를 구독해 벽 영역 진입 여부를 코드가 판정합니다
- 목표 — 시작 지점(1.5, 1.5)에서 출발해 **목표 구역(우상단 8~9, 8~9)에 도달**
- Day 2 도전(경계 정지)의 확장입니다 — 정지가 아니라 **후진·회전으로 회피하고 계속 주행**합니다

| 구역 | 좌표 | 역할 |
|------|------|------|
| 외곽 경계 | x·y = 1.0 ~ 9.0 | 벗어나면 회피 |
| 가상 벽 ① | x 3~4 · y 1~6 | 세로 벽 (아래쪽) |
| 가상 벽 ② | x 6~7 · y 3~9 | 세로 벽 (위쪽) |
| 목표 구역 | x 8~9 · y 8~9 | 도달 시 정지·완주 |

### 9.2 상태 기계 — 3상태 순환

Day 2 정사각형(2상태·시간 조건)의 확장입니다 — 상태 3개 + **위치 감지 조건**:

```
RUN(전진 + 랜덤 조향) ──벽·경계 감지──▶ BACK(0.5초 후진) ──▶ TURN(랜덤 각도 회전)
   ▲                                                              │
   └──────────────────────────────────────────────────────────────┘
```

| Day 2 정사각형 | 오늘 미로 |
|------|------|
| 상태 2개 (직진/회전) | 상태 3개 (RUN/BACK/TURN) |
| 전환 조건 = **시간**(틱) | 감지 조건 = **위치**(가상 벽) + 시간(후진·회전 구간) |
| 발행만 (타이머) | **구독(Pose) + 판단 + 발행** 결합 |

- 인식 → 판단 → 행동의 3단 결합입니다 — **Day 6 판단 노드·Day 9 실물 회피와 같은 뼈대**이며, 센서가 Pose에서 카메라·초음파로 바뀔 뿐입니다

### 9.3 구현 힌트

| 필요한 것 | 방법 |
|------|------|
| 벽·경계 판정 | 사각형 범위 비교 함수 `hit()` — 현재 x·y가 벽 좌표 범위(여유 0.2 포함) 안인지 확인 |
| 상태 기억 | `self.state` (`'RUN'`·`'BACK'`·`'TURN'`) + 틱 계수(후진·회전 시간) |
| 목표 판정 | 타이머 콜백 첫머리에서 목표 구역 포함 여부 확인 → 정지·로그 |
| 조정 가능한 값 | 주행 속도·후진 시간을 **파라미터로 선언**(5.3) — 미로를 통과하지 못할 때 코드 수정 없이 조정 |
| 시작 배치 | **서비스 CLI 재사용**(Day 2) — teleport로 시작 지점 이동·clear로 궤적 정리 |

```bash
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 1.5, y: 1.5, theta: 0.7}"
ros2 service call /clear std_srvs/srv/Empty     # 배경 궤적 지우기
```

### 9.4 문제 (단계별 과제)

1. **(필수) 가상 벽 회피 주행** — `maze_driver.py`: RUN/BACK/TURN 상태 기계 + 벽·경계 감지. 패키지 등록·`ros2 run` 실행
2. **(도달 목표) 미로 완주** — 목표 구역 도달 시 정지·`goal!` 로그 + **launch로 pose_printer와 일괄 실행**
3. **(도전) 지도 기억 탐색** — 방문 구역을 기억해 미탐색 방향을 우선 선택

<details>
<summary><b>문제 (1)·(2) 정답 및 풀이 보기</b></summary>

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import random

WALLS = [(3.0, 4.0, 1.0, 6.0), (6.0, 7.0, 3.0, 9.0)]   # (x1, x2, y1, y2)
BOUND = (1.0, 9.0, 1.0, 9.0)                             # 외곽 경계
GOAL  = (8.0, 9.0, 8.0, 9.0)                             # 목표 구역

class MazeDriver(Node):
    def __init__(self):
        super().__init__('maze_driver')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.pose = None
        self.state = 'RUN'          # RUN / BACK / TURN
        self.tick = 0

    def pose_callback(self, msg):
        self.pose = msg

    def hit(self):                  # 벽·경계 판정
        x, y = self.pose.x, self.pose.y
        if not (BOUND[0] < x < BOUND[1] and BOUND[2] < y < BOUND[3]):
            return True
        for x1, x2, y1, y2 in WALLS:
            if x1 - 0.2 < x < x2 + 0.2 and y1 - 0.2 < y < y2 + 0.2:
                return True
        return False

    def timer_callback(self):
        if self.pose is None:       # 첫 위치 수신 전
            return
        if GOAL[0] < self.pose.x < GOAL[1] and GOAL[2] < self.pose.y < GOAL[3]:
            self.pub.publish(Twist())            # (2) 도달 — 정지
            self.get_logger().info('goal!')
            return
        msg = Twist()
        if self.state == 'RUN':
            if self.hit():
                self.state, self.tick = 'BACK', 0
            else:
                msg.linear.x = 1.5
                msg.angular.z = random.uniform(-1.5, 1.5)   # 랜덤 조향
        elif self.state == 'BACK':
            msg.linear.x = -1.5
            self.tick += 1
            if self.tick >= 5:                   # 0.5초 후진
                self.state, self.tick = 'TURN', 0
        else:                                    # TURN
            msg.angular.z = 2.0
            self.tick += 1
            if self.tick >= random.randint(5, 15):   # 랜덤 각도 회전
                self.state = 'RUN'
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MazeDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

- `import random` — 난수 라이브러리입니다(`uniform` = 범위 내 실수 / `randint` = 범위 내 정수)
- 구조 — 구독 콜백은 위치 저장만 하고, **판단·발행은 전부 타이머 콜백**(10Hz 제어 루프)에서 수행합니다
- 랜덤 조향·랜덤 회전의 의미 — 경로 계획 없이도 반복 시도로 언젠가 목표에 도달합니다(도전 3의 지도 기억이 이를 체계적 탐색으로 개선)
- (2) launch 일괄 실행 — 8.3의 demo_launch.py를 복사해 `maze_launch.py`를 작성하고(circle_driver 항목을 maze_driver로 교체) `ros2 launch my_first_pkg maze_launch.py`로 실행합니다

</details>

<details>
<summary><b>문제 (3) 도전 — 접근 방향 보기</b></summary>

- 화면을 격자(예: 1×1)로 나눠 **방문한 칸을 기억**한 뒤, RUN 상태의 조향을 "미방문 칸이 많은 방향"으로 편향시킵니다
- 완성 형태 = 격자 지도 클래스 + BFS(Breadth-First Search — 너비 우선 탐색)로 가장 가까운 미탐색 칸을 찾는 구현입니다
- 핵심 개념 = **상태(어디를 방문했는가)를 축적하면 같은 센서로도 더 정교한 판단이 가능**합니다 — Day 5 AI 분류가 인식을 강화하는 것과 대칭으로, 이쪽은 판단을 강화합니다

</details>

---

## 10. 문제 해결 — 오류 진단

실습 중 막혔을 때 확인하는 순서입니다.

**빌드·실행 오류**

| 증상 | 원인 | 조치 |
|------|------|------|
| `ros2 run`이 실행 이름을 찾지 못함 | `setup.py` entry_points 누락·오타 | 등록 줄 확인 → 재빌드 → `pkg executables`로 확인 |
| `ModuleNotFoundError` | import 대상 미설치 또는 package.xml 미선언 | 6장 선언 확인 · `rosdep install` |
| 빌드는 성공하나 동작이 이전 코드 | 빌드 후 source 미실행 | `source install/local_setup.bash` |
| 엉뚱한 위치에 build 폴더 생성 | `src` 안에서 colcon 실행 | 해당 폴더 삭제 후 최상위에서 재실행 |
| `SetuptoolsDeprecationWarning` | 도구 버전 경고 | 빌드에 영향 없음 — 무시 |

- 마지막 수단 — `build`·`install`·`log` 폴더를 삭제하고 전체 재빌드합니다. **src는 삭제하지 않습니다**

빌드 관련 유용한 옵션:

```bash
colcon build --packages-select my_first_pkg     # 지정 패키지만 빌드 — 패키지가 늘면 시간 절약
colcon build --symlink-install                  # 코드 수정 시 재빌드 생략
colcon build --event-handlers console_direct+   # 빌드 출력을 그대로 표시 — 오류 원인 확인용
```

**연결이 되지 않을 때 — rqt_graph 판독**

```bash
rqt_graph
```

기본 화면은 정상 연결만 표시하므로, 문제를 찾으려면 좌상단 체크박스를 **해제**합니다:

| 옵션 | 해제하면 보이는 것 |
|------|------|
| Dead sinks | 구독자가 없는 토픽 — 발행만 되고 아무도 받지 않는 상태 |
| Leaf topics | 발행자가 없는 토픽 — 구독 대상이 실재하지 않는 상태 |
| Debug | `/rosout`(로그 토픽)·파라미터 관련 내부 노드까지 표시 |

| 화면 상태 | 원인 |
|------|------|
| 내 노드가 표시되지 않음 | 노드 미실행 또는 `ROS_DOMAIN_ID` 불일치 |
| 노드는 있으나 연결선이 없음 | 토픽 이름 불일치(오타) 또는 QoS(Quality of Service — 통신 품질) 불일치 |
| 연결선은 있으나 동작이 없음 | 통신은 성립 — 콜백·발행 논리를 코드에서 확인 |
| 같은 이름의 노드가 둘 | 중복 실행 — 노드 이름 분리 필요 |

---

## 11. 다음 시간 준비 — RPi5 환경 구축 (과제)

Day 4부터 **실행 환경이 Raspberry Pi 5로 바뀝니다.** 카메라가 CSI(Camera Serial Interface) 방식이라 강의실 PC에는 연결되지 않기 때문입니다.

환경 구축은 **다음 시간까지 각자 수행**합니다. Day 1에서 WSL(Windows Subsystem for Linux)에 Ubuntu·ROS2를 설치한 절차와 같으며, 대상 기기만 바뀝니다.

| 항목 | 내용 |
|------|------|
| 기한 | **9/14(월) 수업 전** |
| 대상 | 각자 보유 Raspberry Pi 5 |
| 목표 상태 | Ubuntu 24.04 + ROS2 Jazzy 동작 + 강의실 PC에서 원격 연결 |

> **이 과제의 목적 —** 절차를 외우는 것이 아니라 **공식 문서를 찾아 그대로 따라가는 것**입니다. 현장에서 새 장비·새 배포판을 만났을 때 필요한 능력이며, Day 1에서 설치 기준을 공식 문서로 정한 이유와 같습니다.

### 11.1 준비물

| 품목 | 비고 |
|------|------|
| Raspberry Pi 5 | 본체 |
| microSD | 32GB 이상 권장 |
| 전원 어댑터 | **5V/5A** — 용량이 부족하면 부팅 중 재시작 |
| 모니터·키보드·마우스 | **초기 설정 1회만** 필요 — 원격 연결을 설정한 뒤에는 필요 없음 |
| 유선 랜 또는 Wi-Fi | 설치 파일 내려받기·원격 연결. **PC와 같은 네트워크** |

> **자주 하는 실수**
>
> - **전원 용량 부족** — 휴대폰 충전기로는 부팅이 불안정합니다
> - **ROS2 Jazzy는 Ubuntu 24.04 전용**입니다 — 22.04를 설치하면 Jazzy를 설치할 수 없습니다
> - CSI 카메라는 **전원을 끈 상태에서** 연결합니다(Day 4에서 사용)

### 11.2 단계 개요

상세 절차는 공식 문서를 따릅니다. 아래는 **무엇을 하는 단계인지**의 개요입니다.

| # | 단계 | 확인 지점 |
|:--:|------|------|
| ① | Raspberry Pi Imager로 **Ubuntu 24.04(64-bit)** 기록 | 부팅 후 로그인 화면 |
| ② | 초기 설정 — 사용자·네트워크 → `sudo apt update && sudo apt upgrade` | 네트워크 연결 |
| ③ | **ROS2 Jazzy 설치** — Day 1과 같은 단계 | 설치 오류 없음 |
| ④ | 환경 등록 — `.bashrc`에 `source /opt/ros/jazzy/setup.bash` + `ROS_DOMAIN_ID` | 새 터미널에서 `ros2` 인식 |
| ⑤ | **원격 연결** — `openssh-server` 설치 + 설정 → 공유 → 원격 데스크톱 켬 | PC에서 연결됨 |
| ⑥ | 완료 확인 — 11.3의 명령 4종 | 4개 모두 정상 |

- ③은 **Day 1에서 수행한 것과 같은 절차**입니다 — 그때의 필기를 그대로 참조합니다
- ④의 `ROS_DOMAIN_ID`는 **Day 1에서 정한 번호와 같은 값**을 사용합니다(값이 다르면 PC에서 RPi5의 토픽이 보이지 않습니다)
- 참조 문서 = ROS2 공식 설치 문서(Jazzy · Ubuntu 24.04) · Raspberry Pi 공식 OS 설치 문서

### 11.3 완료 확인

RPi5에서 아래 네 명령을 실행해 결과를 확인합니다.

```bash
lsb_release -a          # Ubuntu 24.04 확인
ros2 --help             # ROS2 명령 인식 확인
echo $ROS_DISTRO        # jazzy 출력 확인
ros2 topic list         # /parameter_events·/rosout 표시
```

원격 연결도 함께 확인합니다.

| 확인 | 정상 |
|------|------|
| VNC(Virtual Network Computing) 뷰어로 연결 | RPi5 바탕화면이 PC 화면에 표시 |
| `ssh 사용자명@주소` | 터미널 연결 |
| RPi5에서 `hostname -I` | 주소 출력 — **메모해 둘 것** |

- **주소는 재부팅하면 바뀔 수 있습니다** — 연결이 되지 않으면 주소부터 다시 확인합니다
- 이 네 명령은 **Day 4 첫 점검에서 그대로 다시 실행**합니다

### 11.4 막혔을 때

| 상황 | 조치 |
|------|------|
| 특정 단계에서 진행 불가 | 오류 메시지를 **그대로 기록**해 두고 다음 시간에 확인 |
| `ros2` 명령 미인식 | ④ 환경 등록 누락 — `.bashrc` 확인 |
| Ubuntu 버전 오설치 | 재설치 필요 — 시간이 걸리므로 **일찍 확인** |

- 미완이라도 **수업은 진행됩니다.** 이 과정은 개인 단위이므로 구축을 이어서 진행하면서 이론에는 동일하게 참여합니다
- 다만 카메라 실습이 뒤로 밀리므로 기한 내 완료를 권합니다

---

## 12. 오늘의 요약

| 항목 | 내용 |
|------|------|
| 패키지 | 코드의 배포 단위 — `ros2 pkg create` + `setup.py` entry_points 등록 = `ros2 run` 실행 단위 |
| 워크스페이스 | `~/ros2_ws`(src만 편집) + ROS2 본체의 이중 구조 / **수정 → colcon build → source → run** 순환 |
| 구조 확인 | `tree -L N` — 계층 출력. 패키지 이중 폴더 구조·파일 배치 확인의 표준 수단 |
| 옮기기 | 스크립트 → 패키지 = **파일 복사 + 한 줄 등록 + 재빌드** (코드 무변경 — 실행 체계만 교체) |
| 파라미터 | 설정값을 코드 밖으로 분리 — `declare_parameter` 선언 + `--ros-args -p` 주입 + `param set` 실행 중 변경 + `param dump` → yaml → `--params-file` 재사용 |
| 의존성 선언 | `package.xml` `<depend>` — import가 늘면 선언도 늘린다. 결함은 **다른 환경에 전달할 때** 드러남 |
| 커스텀 인터페이스 | 표준 타입으로 부족할 때 `.msg` 직접 정의 — **ament_cmake 전용 패키지** + `rosidl_generate_interfaces` + 사용 쪽 `<depend>` |
| 멀티 노드 | 기능 하나당 노드 하나 — 관찰·구동 분리, rqt_graph로 연결 확인 |
| launch | 실행 구성을 파일로 선언 — 여러 노드 일괄 실행·일괄 종료 + 파라미터 값 수록 |
| 미로 자율주행 | 가상 벽(좌표 판정) + 상태 기계(RUN/BACK/TURN) — **인식 → 판단 → 행동** 결합 |
| 이후 연결 | 미로의 뼈대 = Day 6 판단 노드·Day 9 실물 회피와 동일 — 센서만 Pose → 카메라·초음파로 교체 |
| 산출물 | my_first_pkg — circle_driver(파라미터화)·pose_printer·square_driver·**maze_driver·maze_launch.py** |

---

## 13. 다음 시간

**Day 4 — 카메라와 영상 처리** (9/14)

- 카메라 스택 빌드와 이미지 토픽 확인
- OpenCV 색상 검출 — HSV 색공간
- 미니프로젝트 — 라인 인식

**실행 환경이 Raspberry Pi 5로 바뀝니다.** 11장의 준비 과제를 완료한 상태로 참석합니다.
