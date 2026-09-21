# Day 5 — AI 분류와 추론 노드

**2026-09-21 · 한국폴리텍대학교 하이테크과정 ROS2**

이 자료는 수업의 개념 설명과 실습 절차·명령어를 복습용으로 정리한 것입니다. 복습 기준 = 이 자료 + 수업 중 필기.

---

## 목차

1. [오늘의 목표](#1-오늘의-목표)
2. [개발 환경 — PC에서 연결해 작업하기](#2-개발-환경--pc에서-연결해-작업하기)
3. [작업 공간과 패키지 만들기](#3-작업-공간과-패키지-만들기)
4. [영상 처리의 기초](#4-영상-처리의-기초)
5. [색상 검출 노드](#5-색상-검출-노드)
6. [왜 AI 분류인가](#6-왜-ai-분류인가)
7. [실습 ① — 학습 데이터 수집](#7-실습---학습-데이터-수집)
8. [실습 ② — 모델 학습과 검증](#8-실습---모델-학습과-검증)
9. [모델 배포 — TFLite](#9-모델-배포--tflite)
10. [실습 ③ — 추론 노드 작성](#10-실습---추론-노드-작성)
11. [미니프로젝트 — 표지판 인식](#11-미니프로젝트--표지판-인식)
12. [라인 인식 (자습)](#12-라인-인식-자습)
13. [오늘의 요약](#13-오늘의-요약)
14. [보충](#14-보충)
15. [다음 시간](#15-다음-시간)

---

## 1. 오늘의 목표

Day 4는 **카메라에서 영상이 출력되는 것**까지 확인했습니다. 오늘은 그 영상에서 **좌표를 산출하는 단계**부터 이어갑니다.

Day 4까지의 복습:

| 항목 | 내용 |
|------|------|
| 환경 전환 | Day 4부터 RPi5 — 카메라가 CSI(Camera Serial Interface) 방식이라 PC에 연결되지 않음 |
| 카메라 스택 | Raspberry Pi판 `libcamera` + `camera_ros` 소스 빌드 → `/camera/image_raw` 발행 |
| 이미지 토픽 | `sensor_msgs/msg/Image` — Header + 배열형 `data`. 640×480 컬러 ≈ 0.9MB/장 |
| 좌표계 | 영상 x는 오른쪽(+) / 로봇 회전 양수는 반시계 → **부호 반전** |
| 확인 | `rqt_image_view`로 영상 표시까지 확인 |

오늘 완성할 것 — 세 가지입니다.

| 산출물 | 내용 |
|------|------|
| 작업 환경 | PC에서 RPi5에 연결해 코드를 작성·빌드하는 구조 |
| `color_tracker` | 색으로 대상을 검출해 좌표를 발행하는 노드 |
| `sign_classifier` | 표지를 분류해 **안정된 판정**(`/sign_stable`)을 발행하는 노드 |

세 번째가 Day 6 판단 노드의 입력이 됩니다.

| 단계 | 장 | 내용 | 산출물 |
|:--:|:--:|------|------|
| ① 환경 | 2·3 | PC에서 연결해 작업하기 → 작업 공간·패키지 만들기 | `my_car_pkg` |
| ② 검출 | 4·5 | 색공간·마스킹 → 색상 검출 노드 | `color_tracker` |
| ③ 분류 | 6~10 | 규칙 ↔ 학습 → 촬영 → 학습·재학습 → 배포 → 추론 노드 | `sign_classifier` |
| ④ 안정화 | 11 | 매 프레임 달라지는 결과를 안정된 판정으로 | `/sign_stable` |

- **12장 라인 인식**은 수업에서 다루지 않습니다 — 자료를 보고 각자 학습합니다
- **카메라가 구동되지 않은 경우** — Day 4 자료 6장의 ⓐ(다른 기기의 영상 구독)·ⓑ(저장 영상 재생)로 `/camera/image_raw`를 확보한 뒤 위 흐름을 그대로 수행합니다
- **시간이 부족할 때 줄이는 곳** — 7장 촬영을 클래스당 50장으로, 8.5 재학습을 1회로 줄이고 10.4 부하 확인·10.5 관찰을 생략합니다. **11.4 필수까지는 수행**합니다

### 1.1 준비물 확인

| 품목 | 확인 사항 |
|------|----------|
| **Raspberry Pi 5** | Day 4 상태 유지 — 카메라 스택 빌드 완료·영상 확인 완료. **미완이면 Day 4 자료 6장의 ⓐ·ⓑ**로 영상을 확보한 뒤 진행 |
| **강의실 PC** | VS Code(원격 연결용) · 브라우저(학습용) · **웹캠**(내장 또는 USB — 7장 촬영용. 없으면 7.3의 분기) |
| 네트워크 | ① RPi5와 PC가 **같은 네트워크**(기기 간 통신) ② **외부 인터넷 접속**(VS Code 서버 설치·Teachable Machine·10.1 설치) — 둘은 별개 조건 |
| **표지 3종** | **직접 그린 표지** — 정지·직진 + 배경(표지 없음) |
| 휴대전화 조명 | 밝기를 달리해 촬영 조건을 바꿀 때 사용 |

> **Tip —** 오늘 사용하는 표지는 각자의 그림입니다. 공식 표지판은 이후 회차에서 배포하며, 그때 같은 절차로 다시 학습합니다. 학습 데이터가 바뀌면 모델도 다시 만들어야 한다는 것을 그 자리에서 확인합니다.

### 1.2 상태 점검 — 시작 전 10분

전원이 동시에 수행합니다. 오늘은 환경 → 패키지 → 코드 → 학습 → 추론이 이어지므로, 모두 같은 상태에서 시작하는지 먼저 확인합니다.

강의실 PC의 터미널에서 RPi5에 연결합니다.

```bash
ssh 사용자명@192.168.0.__        # SSH(Secure Shell) — 터미널 연결
```

연결된 뒤 RPi5에서 실행합니다.

```bash
echo $ROS_DISTRO                 # jazzy 출력
ros2 run camera_ros camera_node  # 카메라 노드 — 0: imx708_wide 확인
```

다른 터미널에서 확인합니다.

```bash
ros2 topic hz /camera/image_raw  # 약 30Hz
```

| 결과 | 상태 | 다음 |
|------|------|------|
| 주기가 출력됨 | 정상 | 2장으로 진행 |
| `ros2` 명령 미인식 | 경미 | `.bashrc`의 `source /opt/ros/jazzy/setup.bash` 확인 후 **다시** 연결 |
| `no cameras available` | 카메라 스택 | Day 4 자료 5.1 **⑤-2부터** 다시 수행 |
| `Package 'camera_ros' not found` | 환경 등록 또는 빌드 미완 | Day 4 자료 5.1 **⑤-1**(환경 등록) → 그래도 같으면 **④-1부터** 재빌드 |
| SSH 연결 실패 | 네트워크 | ① 주소 확인 후 **다시** 연결 → ② **RDP**로 접속해 `hostname -I` 확인 → ③ 교수에게 알림 |

- `hostname -I`는 **RPi5에서 실행하는 명령**입니다 — SSH가 연결되지 않은 상태에서는 RDP나 모니터로 접속해야 확인할 수 있습니다. RPi5는 **재부팅하면 주소가 바뀔 수 있습니다**
- 강의실 PC에서는 **브라우저로 웹캠이 열리는지** 함께 확인합니다 — 7장 촬영에 사용합니다(열리지 않으면 7.3의 분기)
- 카메라 스택 복구가 **오전 내에 끝나지 않으면** → Day 4 자료 6장(카메라를 사용할 수 없을 때)으로 전환해 영상을 확보하고 진행합니다

---

## 2. 개발 환경 — PC에서 연결해 작업하기

**학습 목표** — 개발 환경과 실행 환경이 분리되는 구조를 설명하고, 강의실 PC의 편집기로 RPi5의 코드를 작성·빌드할 수 있다.

### 2.1 왜 PC에서 연결해 작업하는가

실무의 로봇 개발은 대부분 이 구조입니다.

```
강의실 PC (작성·빌드 명령) ──SSH──▶ RPi5 (실행·센서 연결)
        ▲                              │
        └────────── 출력·로그 ──────────┘
```

| 역할 | 위치 | 이유 |
|------|------|------|
| 코드 작성·편집 | **강의실 PC** | 화면이 크고 키보드 입력이 편리함 |
| 빌드·실행 | **RPi5** | 카메라가 물리적으로 연결되어 있음 |

- Day 4에서는 RPi5 하나에서 모두 수행했습니다. 오늘부터 **작성과 실행을 나눕니다**
- Day 8부터 차량에 실린 RPi5도 같은 방식으로 다룹니다

### 2.2 연결 방식 세 가지

| 방식 | 보이는 것 | 이 과정에서 |
|------|------|:--:|
| **SSH** | 터미널 | **기본** — 작성·빌드·실행·수치 확인 |
| **RDP**(Remote Desktop Protocol) | 바탕화면 전체 | 그래픽 창이 필요할 때 |
| 모니터 직접 연결 | 바탕화면 전체 | 위 둘이 어려울 때 |

> **핵심 —** SSH 터미널에서는 그래픽 창이 열리지 않습니다. 오늘 다루는 구간은 모두 터미널로 진행되며, 창이 필요한 지점은 2.5에 모아 두었습니다.

### 2.3 VS Code 원격 연결

**①** 강의실 PC의 VS Code에서 확장 탭을 열고 **`Remote - SSH`**를 설치합니다.

- 설치가 끝나면 → ②로 진행
- 확장 탭이 보이지 않으면 `Ctrl + Shift + X`로 열고 **①을 다시** 수행
- 진행 표시만 계속되고 설치가 끝나지 않으면 → 마켓플레이스 접속 문제입니다. ①을 반복하지 말고 **⑤로 전환**하고 교수에게 알림

**②** `F1` → `Remote-SSH: Connect to Host` → `사용자명@192.168.0.__` 입력 → 암호를 입력합니다.

- **좌하단에 `SSH: 192.168.0.__` 표시**가 출력되면 → ③으로 진행
- 첫 연결에서는 RPi5에 VS Code 서버가 자동으로 설치됩니다(외부 인터넷 필요·수 분)
- 연결이 실패하면 → 주소·같은 네트워크인지 확인한 뒤 **②를 다시** 실행
- 표시가 없으면 로컬 창입니다 → **②를 다시** 실행
- 서버 설치 표시가 **수 분이 지나도 끝나지 않으면** → 원격 서버 내려받기가 진행되지 않는 상태입니다. ②를 반복하지 말고 **⑤로 전환**하고 교수에게 알림

**③** `파일 → 폴더 열기`에서 `/home/사용자명`을 엽니다.

- 좌측 탐색기에 RPi5의 폴더가 보이면 → ④로 진행

**④** 확장 탭에서 **Python**을 설치합니다.

- 설치 버튼이 **`Install in SSH: 192.168.0.__`**로 표시되는지 확인합니다 → 표시되면 원격에 설치되는 상태
- `Install` 그대로면 로컬에 설치되는 상태입니다 → 원격 창인지 확인하고 **④를 다시** 수행
- 설치되지 않아도 오늘 절차에는 지장이 없습니다(편집 보조 기능) → 2.4로 진행

> **Tip —** 확장이 로컬용과 원격용으로 나뉘는 구조는 Day 2 자료에서 WSL 연결로 다룬 것과 같습니다. 연결 대상만 WSL에서 RPi5로 바뀌었습니다.

**⑤ 원격 연결이 되지 않을 때** — ①·②가 끝내 진행되지 않아도 **SSH 터미널만으로** 오늘 절차를 그대로 수행할 수 있습니다. 편집기는 터미널에서 실행하는 `nano`를 사용합니다.

```bash
nano ~/ros2_ws/src/my_car_pkg/my_car_pkg/image_check.py
```

| 조작 | 방법 |
|------|------|
| 붙여넣기 | 터미널 창에서 마우스 오른쪽 버튼 클릭(또는 `Ctrl + Shift + V`) |
| 저장 | `Ctrl + O` → `Enter` |
| 종료 | `Ctrl + X` |

- 저장한 뒤 `ls`로 파일이 보이면 → 이후 절의 코드 작성·`setup.py` 수정을 모두 이 방법으로 수행
- 오늘 작성하는 파일은 `image_check.py`·`color_tracker.py`·`sign_classifier.py` 세 개입니다

### 2.4 터미널 운용

VS Code 안에서 터미널을 여러 개 열어 사용합니다(`Ctrl` + `` ` `` · 분할 아이콘).

| 터미널 | 용도 |
|:--:|------|
| ① | 카메라 노드 — 수업 내내 유지 |
| ② | **모니터** — `ros2 topic echo`로 메시지 확인 |
| ③ | 빌드·실행 명령 |

- Day 2·3에서 terminator로 하던 운용을 그대로 옮긴 것입니다 — 도구가 바뀌어도 방식은 같습니다
- 이 터미널은 RPi5의 셸입니다. `ls`·`colcon build`가 모두 RPi5에서 실행됩니다
- **2.3 ⑤로 진행한 경우** — VS Code 대신 PC의 터미널에서 `ssh`를 **세 번 연결**해 같은 3개 구성을 만듭니다

**터미널 ① — 카메라 노드 실행**

1.2에서 점검용으로 실행한 카메라 노드는 `Ctrl+C`로 종료하고, 터미널 ①에서 다시 실행합니다. 오늘 작성하는 노드는 모두 이 영상을 구독하므로 **수업이 끝날 때까지 이 터미널을 유지**합니다.

```bash
ros2 run camera_ros camera_node --ros-args -p width:=640 -p height:=480
```

- 로그에 `0: imx708_wide`가 출력되고 `configured with …`에 설정한 크기가 표시되면 → 그대로 두고 3장으로 진행
- `Package 'camera_ros' not found`가 나오면 → Day 4 자료 5.1 **⑤-1**(환경 등록)을 실행하고 **다시** 실행
- `no cameras available`이 나오면 → Day 4 자료 5.1 **⑤-2**(실행 경로 확인)부터 수행
- 해상도를 지정하는 이유 — 지정하지 않으면 기본값(800×600)으로 시작합니다. 5장의 면적 기준값이 **640×480 기준**이므로 크기를 맞춰 둡니다(3.4에서 실제 값을 확인)

### 2.5 그래픽 창이 필요한 도구

| 도구 | SSH | 대체 방법 |
|------|:--:|------|
| `rqt_image_view` | ❌ | 프레임을 파일로 저장해 **VS Code에서 열어 확인** |
| `rqt_graph` | ❌ | `ros2 node info`·`ros2 topic info`로 연결 확인 |
| `turtlesim_node` | ❌ | 12장에서 필요 — 그때 RDP 또는 모니터 사용 |

- SSH에서 실행하면 **표시 장치에 연결할 수 없다**는 오류가 출력됩니다. 고장이 아니라 화면이 없는 환경에서 창을 열려고 했기 때문입니다
- 창이 필요하면 → RDP로 연결해 그 화면에서 실행 → 그래도 어려우면 RPi5에 모니터를 연결

### 2.6 토픽 원격 관찰

RPi5에서 실행 중인 노드의 토픽을 **PC에서 직접 관찰**할 수도 있습니다. 조건이 맞을 때만 동작합니다.

| 전제 | 확인 방법 |
|------|------|
| ① PC와 RPi5가 같은 공유기(같은 서브넷)에 연결 | 두 기기의 주소 앞 세 자리 비교 — RPi5 `hostname -I` · PC `ipconfig` |
| ② 같은 `ROS_DOMAIN_ID` | 두 기기에서 `echo $ROS_DOMAIN_ID` |
| ③ PC가 WSL2이면 네트워크 방식 | 기본(NAT) 방식에서는 다른 기기의 토픽이 보이지 않을 수 있음 — 교수 확인 |

```bash
# PC(WSL)에서 — RPi5의 토픽이 출력되는지 확인
export ROS_DOMAIN_ID=<자신의 번호>   # Day 1에서 정한 번호 — RPi5와 같은 값
ros2 topic list
```

- RPi5에서 실행 중인 노드의 토픽(예: `/camera/image_raw`)이 출력되면 → 성공
- 출력되지 않으면 → 전제 ①·②를 확인한 뒤 **`ros2 topic list`를 다시** 실행
- ①·②가 맞아도 출력되지 않으면 → ③(WSL2 네트워크 방식·방화벽)에 해당 — 교수에게 알림

**카메라가 구동되지 않은 경우(Day 4 자료 6장 ⓐ)** — 정상인 다른 RPi5의 영상을 **자기 RPi5에서** 구독합니다. 두 기기가 모두 RPi5이므로 위 표의 전제 ③은 해당하지 않습니다.

| 순서 | 수행 위치 | 명령 |
|:--:|------|------|
| ① | 영상을 제공하는 RPi5 | `ros2 run camera_ros camera_node` — 수업 내내 유지 |
| ② | 두 RPi5 각각 | `echo $ROS_DOMAIN_ID` — **같은 번호인지 확인** |
| ③ | 자기 RPi5 | `ros2 topic hz /camera/image_raw` |

- 주기가 출력되면 → 이후 절을 **그대로** 수행합니다(구독하는 토픽 이름이 같으므로 코드는 동일)
- 출력되지 않으면 → ②의 번호를 다시 확인하고 **③을 다시** 실행 → 그래도 출력되지 않으면 교수에게 알림
- 번호는 **제공하는 쪽에 맞춥니다** — 개인 번호는 Day 1에서 정한 값이며, 이 경우에만 일시적으로 같게 둡니다

> **핵심 —** DDS는 같은 네트워크 안에서 멀티캐스트로 다른 기기를 자동으로 찾습니다. 공유기를 넘어서 찾는 것은 기본 동작이 아니며, 공유기의 무선 클라이언트 격리나 PC 방화벽이 켜져 있어도 토픽이 보이지 않습니다.

---

## 3. 작업 공간과 패키지 만들기

**학습 목표** — RPi5에 워크스페이스와 Python 패키지를 만들어 노드를 실행할 수 있다.

### 3.1 왜 다시 만드는가

오늘 작성하는 노드는 PC에서 만든 `my_first_pkg`가 아니라 **새 패키지**에 둡니다.

| 이유 | 내용 |
|------|------|
| 기기 종속 | 빌드 결과물(`build`·`install`)은 생성한 기기에 종속되어 그대로 사용할 수 없음 |
| **절차 숙달** | 워크스페이스 → 패키지 → 등록 → 빌드는 앞으로 계속 반복하는 절차 — 한 번 더 수행 |

- 이름은 **`my_car_pkg`** — PC의 `my_first_pkg`와 구분됩니다
- Day 4에서도 작업물을 옮기지 않고 RPi5에서 새로 만들어 빌드했습니다. 그때 만든 패키지가 있으면 그대로 두고, 오늘 코드는 `my_car_pkg`에 작성합니다
- 오늘부터 Day 12까지 이 패키지에 노드를 더해 갑니다

### 3.2 빌드 도구 확인

```bash
which colcon
```

- 경로가 출력되면 → 3.3으로 진행
- 출력이 없으면 아래를 실행한 뒤 **다시** 확인

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions ros-dev-tools
```

### 3.3 워크스페이스·패키지 생성

```bash
mkdir -p ~/ros2_ws/src                # 이미 있으면 그대로 사용됨
cd ~/ros2_ws && colcon build          # 4폴더(build·install·log·src) 생성 확인
cd src
ros2 pkg create --build-type ament_python my_car_pkg
```

```bash
cd ~/ros2_ws && tree -L 2             # 없으면 sudo apt install -y tree
```

- `build`·`install`·`log`·`src`가 보이고 `src/my_car_pkg`가 있으면 → 3.4로 진행
- `colcon: command not found`가 나오면 → **3.2를 다시** 수행
- 패키지 폴더 안에 **같은 이름의 폴더가 한 번 더** 있습니다 — 코드는 **안쪽**에 둡니다

### 3.4 첫 노드 — 영상 수신 확인

빌드 흐름을 확인하면서, 오늘 사용할 카메라 값도 함께 확인합니다. **터미널 ①의 카메라 노드가 실행 중인 상태에서 진행합니다**(2.4).

`~/ros2_ws/src/my_car_pkg/my_car_pkg/image_check.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ImageCheck(Node):
    def __init__(self):
        super().__init__('image_check')
        self.count = 0                                            # ① 수신 건수
        self.sub = self.create_subscription(                      # ② 영상 구독
            Image, '/camera/image_raw', self.on_image, 10)

    def on_image(self, msg):
        self.count += 1
        if self.count % 30 == 0:                                  # ③ 30장마다 한 번
            self.get_logger().info(
                f'{self.count} 장 — {msg.width}x{msg.height} {msg.encoding}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageCheck()
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
| ① | 수신 장 수 집계 변수 |
| ② | 이미지 토픽 구독 — **5장 색상 검출 노드와 같은 구조** |
| ③ | 매 장 출력하면 출력이 많아지므로 30장에 한 번만 |

**코드 읽기 — Python 문법**

| 코드 | 뜻 |
|------|------|
| `from sensor_msgs.msg import Image` | 모듈에서 이름 하나만 가져오기 |
| `class ImageCheck(Node):` | 괄호 = 상속 — `Node`를 물려받은 새 클래스 |
| `def __init__(self):` | 객체를 만들 때 한 번 실행되는 초기화 메서드 |
| `super().__init__('image_check')` | 부모의 초기화를 먼저 실행 — 노드 이름 등록 |
| `self.count = 0` | `self.` = 이 객체의 변수 — 다른 메서드에서도 같은 값 사용 |
| `self.count % 30 == 0` | `%` = 나머지 — 30으로 나눈 나머지가 0이면 30의 배수 |
| `f'{self.count} 장 — {msg.width}…'` | f-문자열 — 중괄호 안 값이 그 자리에 삽입 |
| `try:` … `except KeyboardInterrupt:` … `finally:` | `Ctrl+C`로 멈춰도 `finally`의 정리는 반드시 실행 |

- 이 표의 항목은 오늘과 이후 Day의 모든 노드에 **같은 형태로 반복**됩니다. 지금 전부 암기할 필요는 없으며, 코드를 읽을 때마다 표를 참조하십시오

> **출력에서 확인할 것 —** `msg.width`·`msg.height`가 오늘 다룰 영상의 실제 크기입니다. 파라미터를 주지 않으면 기본값으로 시작하므로, 5장에서 면적 기준을 정할 때 이 값을 근거로 삼습니다.

### 3.5 등록·빌드·실행

`setup.py`의 `console_scripts`에 등록합니다(Day 3 자료와 동일).

```python
'console_scripts': [
    'image_check = my_car_pkg.image_check:main',
],
```

```bash
cd ~/ros2_ws && colcon build
source install/local_setup.bash
echo "source ~/ros2_ws/install/local_setup.bash" >> ~/.bashrc    # 새 터미널에도 적용
ros2 run my_car_pkg image_check
```

- 30장마다 한 줄씩 출력되면 → 4장으로 진행
- `Package 'my_car_pkg' not found`가 나오면 → `source` 줄을 실행하고 **다시** 실행
- `executable 'image_check' not found`가 나오면 → `setup.py` 등록을 확인하고 **빌드부터 다시** 실행
- 아무것도 출력되지 않으면 → 터미널 ①의 카메라 노드가 실행 중인지 확인(2.4) 후 **다시** 실행
- **카메라를 확보하지 못한 상태**면 → `ros2 node list`·`ros2 node info /image_check`로 **구독자가 등록되었는지**까지 확인하고 4장으로 진행(영상은 Day 4 자료 6장 ⓐ·ⓑ로 확보한 뒤 이 절로 돌아옴)

> **자주 하는 실수 —** ① 코드를 바깥 폴더에 두는 것(`my_car_pkg/my_car_pkg/` 안쪽이 맞습니다) ② 빌드 후 `source` 누락 ③ `setup.py`의 쉼표·따옴표 빠짐 — 빌드가 실패하면 그 줄을 먼저 확인합니다.

> **Tip —** `colcon build --symlink-install`로 빌드하면 Python 코드는 수정 후 재빌드 없이 반영됩니다. 오늘처럼 코드를 자주 고치는 날에 유용합니다.

---

## 4. 영상 처리의 기초

**학습 목표** — 색으로 대상을 찾는 원리와 HSV 색공간을 쓰는 이유를 설명할 수 있다.

### 4.1 픽셀에서 좌표로

영상은 숫자의 배열일 뿐입니다. "빨간 공이 어디 있는가"를 알려면 **숫자에서 위치를 추출하는 절차**가 필요합니다.

| 단계 | 하는 일 | 결과 |
|:--:|------|------|
| ① 변환 | BGR → HSV | 색을 다루기 쉬운 형태로 |
| ② 마스킹 | 지정 색 범위만 남김 | **흑백 영상**(대상 = 흰색) |
| ③ 잡음 제거 | 작은 점 제거 | 깔끔한 덩어리 |
| ④ 좌표 산출 | 흰 영역의 중심 계산 | **(x, y) 숫자** |

- ④의 결과가 **토픽으로 발행할 값** — 여기서부터 다시 ROS2의 영역

### 4.2 왜 HSV인가

| 색공간 | 구성 | 성질 |
|------|------|------|
| **BGR**(Blue·Green·Red) | 파랑·초록·빨강의 세기 | 조명이 바뀌면 **세 값이 모두 변함** |
| **HSV**(Hue·Saturation·Value) | **H**(색상)·**S**(채도)·**V**(명도) | 조명이 바뀌어도 **H는 비교적 유지됨** |

빨간 공을 예로 들면:

| 조건 | BGR | HSV |
|------|------|------|
| 밝은 곳 | (30, 30, 220) | H≈0, S≈220, **V≈220** |
| 그늘 | (15, 15, 110) | H≈0, S≈220, **V≈110** |

- BGR에서는 세 값이 모두 절반이 되어 같은 색으로 보기 어렵습니다
- HSV에서는 **H(색상)가 그대로** — "빨강"이라는 판단이 유지됨

**H 값의 범위** (OpenCV 기준 0~179):

| 색 | H 대략값 |
|------|:--:|
| 빨강 | 0~10 · 170~179 (두 구간으로 나뉨) |
| 노랑 | 20~35 |
| 초록 | 40~80 |
| 파랑 | 90~130 |

> **자주 하는 실수 —** 빨강은 H 범위가 양끝 두 구간으로 나뉩니다. 한 범위만 쓰면 절반을 놓치므로 두 범위를 만들어 합쳐야 합니다. 5장의 코드는 `h_min`을 `h_max`보다 크게 주면 두 범위를 합쳐 잡습니다(5.4 「색 값 정하기 ③」).

### 4.3 잡음 제거 — 모폴로지

마스킹 결과에는 작은 흰 점이 섞입니다. 반사·그림자 때문입니다.

| 연산 | 하는 일 | 쓰는 때 |
|------|------|------|
| **침식**(erode) | 흰 영역을 깎음 | 작은 점 제거 |
| **팽창**(dilate) | 흰 영역을 불림 | 깎인 본체 복원 |
| **열림**(opening) | 침식 → 팽창 | **작은 점만 없애고 본체는 유지** — 주로 사용 |

- 실습에서는 `cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)` 한 줄로 처리
- 커널 크기가 크면 강하게 제거되지만 **작은 대상도 함께 사라짐**

### 4.4 무게중심 계산

흰 영역의 중심을 구하는 방법은 **모멘트**를 쓰는 것이 표준입니다.

```
cx = M['m10'] / M['m00']
cy = M['m01'] / M['m00']
```

| 기호 | 의미 |
|------|------|
| `M` | `cv2.moments(mask)`가 반환한 **딕셔너리** — 아래 키(문자열)로 각 값을 읽음 |
| `m00` | 흰 픽셀의 **총 개수**(면적) |
| `m10` | x좌표의 합 |
| `m01` | y좌표의 합 |

- `m00`이 0이면 **대상이 화면에 없음** — 나눗셈 전에 반드시 확인
- 면적(`m00`)은 **거리 추정에도 쓰임** — 가까울수록 커짐(5.3 ⑨에서 검출 판정에 사용)

### 4.5 확인 활동 — 개념 점검

코드를 쓰기 전에 구두로 확인합니다.

| # | 질문 | 확인하려는 것 |
|:--:|------|------|
| 1 | 640×480 컬러 영상 한 장은 `Twist` 메시지의 몇 배 크기인가? | 이미지 토픽의 부담 |
| 2 | 그늘로 들어가면 BGR과 HSV 중 어느 쪽이 더 크게 변하는가? | 4.2 — HSV를 쓰는 이유 |
| 3 | 마스크에 작은 흰 점이 많다. 어떤 연산을 쓰는가? | 4.3 — 열림 연산 |
| 4 | 대상이 화면 **오른쪽**에 있다. `angular.z`는 양수인가 음수인가? | Day 4 — **부호 반전** |
| 5 | `m00`이 0일 때 무엇을 확인하지 않으면 오류가 나는가? | 4.4 — 나눗셈 전 확인 |

- 4번이 오늘 가장 자주 틀리는 지점입니다 — **영상의 오른쪽(+)과 로봇의 반시계(+)가 반대**
- 이 부호를 **회전 명령에 실제로 적용하는 코드**는 12장(자습)과 Day 6의 판단 노드에 나타납니다. 오늘 5장의 `color_tracker`는 좌표만 발행하고 회전을 지시하지 않습니다

---

## 5. 색상 검출 노드

**학습 목표** — 이미지 토픽을 구독해 OpenCV로 처리하는 노드를 작성하고, 검출 결과를 좌표로 발행할 수 있다.

### 5.1 cv_bridge — ROS2와 OpenCV의 연결

ROS2의 `Image` 메시지와 OpenCV의 이미지 형식은 다릅니다. **`cv_bridge`**가 둘을 변환합니다.

```bash
sudo apt install -y ros-jazzy-cv-bridge python3-opencv
```

| 방향 | 함수 |
|------|------|
| ROS2 → OpenCV | `bridge.imgmsg_to_cv2(msg, 'bgr8')` |
| OpenCV → ROS2 | `bridge.cv2_to_imgmsg(frame, 'bgr8')` |

- `'bgr8'` = 변환 후 형식 지정. OpenCV의 기본 순서가 **BGR**이므로 이 값을 사용

**의존성 선언** — 패키지가 이 라이브러리를 사용한다는 것을 `package.xml`에도 적습니다(Day 3 자료와 같은 절차 · **import가 늘면 선언도 늘린다**).

`~/ros2_ws/src/my_car_pkg/package.xml`의 `<buildtool_depend>` 아래에 추가합니다.

```xml
<depend>rclpy</depend>
<depend>sensor_msgs</depend>
<depend>geometry_msgs</depend>
<depend>cv_bridge</depend>
```

- 저장한 뒤 → 5.2로 진행
- 이미 있는 줄은 다시 쓰지 않습니다(중복 선언 시 빌드 경고)
- 선언하지 않아도 이 기기에서는 빌드되지만, **다른 기기에서 받아 빌드할 때** 의존 패키지가 자동으로 설치되지 않습니다

### 5.2 노드 설계

| 항목 | 내용 |
|------|------|
| 노드 이름 | `color_tracker` |
| 구독 | `/camera/image_raw` (`sensor_msgs/msg/Image`) |
| 발행 | `/target_point` (`geometry_msgs/msg/Point`) — 검출 좌표 |
| 파라미터 | `h_min`·`h_max`·`s_min`·`v_min` — 색 범위를 밖에서 조정 |
| 동작 | 영상 수신 → HSV 변환 → 마스킹 → 열림 → 무게중심 → 발행 |

- 좌표 전달에 `Point`를 쓰는 이유 — x·y·z 세 실수를 담는 **표준 타입**이라 커스텀 정의가 불필요
- 파라미터로 색 범위를 빼는 이유 — **조명이 바뀌면 값을 다시 맞춰야 함**

### 5.3 코드 작성

`~/ros2_ws/src/my_car_pkg/my_car_pkg/color_tracker.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorTracker(Node):
    def __init__(self):
        super().__init__('color_tracker')
        self.bridge = CvBridge()                                  # ① 변환기

        self.declare_parameter('h_min', 40)                       # ② 색 범위 파라미터
        self.declare_parameter('h_max', 80)
        self.declare_parameter('s_min', 80)
        self.declare_parameter('v_min', 60)

        self.sub = self.create_subscription(                      # ③ 영상 구독
            Image, '/camera/image_raw', self.on_image, 10)
        self.pub = self.create_publisher(Point, '/target_point', 10)
        self.get_logger().info('color_tracker started')

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')            # ④ ROS2 → OpenCV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)              # ⑤ 색공간 변환

        h_min = self.get_parameter('h_min').value
        h_max = self.get_parameter('h_max').value
        s_min = self.get_parameter('s_min').value
        v_min = self.get_parameter('v_min').value

        lower = np.array([h_min, s_min, v_min])                   # ⑥ 마스킹
        upper = np.array([h_max, 255, 255])
        if h_min <= h_max:
            mask = cv2.inRange(hsv, lower, upper)
        else:                                                     # 빨강처럼 179와 0에 걸친 범위
            m1 = cv2.inRange(hsv, lower, np.array([179, 255, 255]))
            m2 = cv2.inRange(hsv, np.array([0, s_min, v_min]), upper)
            mask = cv2.bitwise_or(m1, m2)                         # 두 구간 합치기

        kernel = np.ones((5, 5), np.uint8)                        # ⑦ 잡음 제거
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        m = cv2.moments(mask, binaryImage=True)                   # ⑧ 무게중심
        point = Point()
        if m['m00'] > 500:                                        # ⑨ 대상 존재 판정
            point.x = m['m10'] / m['m00']
            point.y = m['m01'] / m['m00']
            point.z = m['m00']                                    # 면적 = 거리의 단서
        else:
            point.x, point.y, point.z = -1.0, -1.0, 0.0           # 미검출 표시

        self.pub.publish(point)

        rows, cols = hsv.shape[:2]                                # ⑩ 가운데 색 값 출력
        self.get_logger().info(f'center HSV = {hsv[rows // 2, cols // 2]}',
                               throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = ColorTracker()
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
| ① | `CvBridge` 생성 — ROS2 ↔ OpenCV 변환기 |
| ② | 색 범위를 **파라미터로 선언** — 조명이 바뀌면 코드 수정 없이 조정 |
| ③ | 이미지 토픽 구독 — 구조는 Day 2 자료와 동일 |
| ④ | 메시지를 OpenCV 이미지로 변환 |
| ⑤~⑦ | 4장의 절차를 코드로 — 변환 → 마스킹 → 잡음 제거 |
| ⑥ | `h_min`이 `h_max`보다 크면 **두 구간을 합쳐** 마스킹 — 179와 0에 걸친 빨강용 |
| ⑧ | 무게중심 계산 |
| ⑨ | **면적이 500 미만이면 미검출로 판정** — 작은 잡음을 대상으로 오인하지 않기 위함 |
| ⑩ | 화면 가운데의 HSV 값을 1초마다 출력 — 물체의 색 값을 재는 용도(5.4) |

**코드 읽기 — Python 문법 ① 클래스·객체**

| 코드 | 뜻 |
|------|------|
| `from sensor_msgs.msg import Image` | 모듈에서 이름 하나만 가져오기 |
| `class ColorTracker(Node):` | 괄호 = 상속 — `Node`를 상속받아 만든 새 클래스 |
| `def __init__(self):` | 객체를 만들 때 한 번 실행되는 초기화 메서드 |
| `super().__init__('color_tracker')` | 부모(`Node`)의 초기화를 먼저 실행 — 노드 이름 등록 |
| `self.bridge = CvBridge()` | `self.` = 이 객체의 변수 — 다른 메서드(`on_image`)에서도 같은 값 사용 |
| `def on_image(self, msg):` | 구독 콜백 메서드 — `msg`에 수신한 `Image`가 전달됨 |

**코드 읽기 — Python 문법 ② 자료형**

| 코드 | 뜻 |
|------|------|
| `self.get_parameter('h_min').value` | 점 연결 — 메서드가 반환한 객체의 `.value` 속성을 이어서 읽음 |
| `np.array([h_min, s_min, v_min])` | 리스트(대괄호)를 NumPy 배열로 변환 — OpenCV 함수의 입력 형식 |
| `np.ones((5, 5), np.uint8)` | 괄호 안 괄호 `(5, 5)` = 튜플 하나 — 배열 크기를 한 인자로 전달 |
| `cv2.moments(mask, binaryImage=True)` | 키워드 인자 — `True`면 흰 픽셀을 1로 셈 → `m00` = 픽셀 개수 |
| `m['m00']` | 딕셔너리 키 접근 — `cv2.moments()`가 반환한 딕셔너리 |
| `rows, cols = hsv.shape[:2]` | `shape` = (높이, 너비, 채널) · `[:2]` = 앞의 두 값만 잘라 동시 대입 |
| `hsv[rows // 2, cols // 2]` | `//` = 정수 나눗셈 · [행, 열] 위치 픽셀의 H·S·V 세 값 |

**코드 읽기 — Python 문법 ③ 흐름 제어**

| 코드 | 뜻 |
|------|------|
| `if m['m00'] > 500:` … `else:` | 조건 분기 — 들여쓰기가 같은 줄까지가 한 블록 |
| `if h_min <= h_max:` … `else:` | 범위가 0을 넘어가는지로 분기 — 빨강만 `else`로 감 |
| `cv2.bitwise_or(m1, m2)` | 두 마스크 중 **하나라도 흰색**이면 흰색 — 두 구간 합치기 |
| `info(..., throttle_duration_sec=1.0)` | 키워드 인자 — 1초에 한 번만 출력(매 프레임 출력 방지) |
| `point.x, point.y, point.z = -1.0, -1.0, 0.0` | 동시 대입 — 왼쪽 세 곳에 오른쪽 세 값을 순서대로 |
| `try:` … `except KeyboardInterrupt:` … `finally:` | Ctrl+C로 종료해도 `finally`의 정리 코드는 반드시 실행 |
| `def main(args=None):` | 기본값 인자 — 호출 시 값을 주지 않으면 `None` |

- 세 표의 항목은 Day 5·6·8·9의 모든 노드 코드에 **같은 형태로 반복**됩니다 — **이후 Day의 자료에도 같은 표를 다시 싣습니다.** 지금 전부 암기할 필요는 없으며, 코드를 읽을 때마다 표를 참조하며 확인하십시오

`setup.py`에 등록:

```python
'console_scripts': [
    ...
    'color_tracker = my_car_pkg.color_tracker:main',
],
```

### 5.4 실행과 관찰

```bash
cd ~/ros2_ws && colcon build && source install/local_setup.bash

# 터미널 ① — 2.4에서 실행해 둔 카메라 노드를 그대로 둡니다(종료됐으면 2.4의 명령으로 다시 실행)
# 터미널 ②
ros2 run my_car_pkg color_tracker
# 터미널 ③
ros2 topic echo /target_point
```

기본값은 **초록** 범위입니다. 초록색 물체가 있으면 먼저 관찰하고, 다른 색 물체를 쓰려면 아래 「내 물체의 색 값 정하기」로 값을 정한 뒤 관찰합니다.

- 터미널 ②에 `ModuleNotFoundError`(`cv2`·`cv_bridge`)가 나오면 → 5.1의 설치 명령을 실행하고 `package.xml` 선언까지 확인한 뒤 **빌드부터 다시** 실행
- 터미널 ③에 아무것도 출력되지 않으면 → 터미널 ①의 카메라 노드가 실행 중인지 확인(2.4) 후 **터미널 ②부터 다시** 실행
- **영상을 확보하지 못한 상태**면 → Day 4 자료 6장의 ⓐ·ⓑ로 `/camera/image_raw`를 먼저 세운 뒤 이 절로 돌아옴

| 조작 | 예상 출력 |
|------|------|
| 대상을 화면 **왼쪽**으로 | x가 작아짐 (0에 가까움) |
| 대상을 화면 **오른쪽**으로 | x가 커짐 (640에 가까움) |
| 대상을 **가까이** | z(면적)가 커짐 |
| 대상을 치움 | x = -1.0 (미검출) |

**내 물체의 색 값 정하기 ① — 물체 고르기**

| 조건 | 이유 |
|------|------|
| **단색** | 여러 색이 섞이면 한 범위로 잡히지 않음 |
| **무광** | 반사광 부분은 흰색이 되어 마스크에서 빠짐 |
| **배경과 다른 색** | 같은 색 배경은 함께 잡혀 구분 불가 |
| 손바닥 크기 | 너무 작으면 면적 500 미만 = 미검출 |

- 모양은 상관없습니다 — 색만 보고 찾습니다(속이 빈 고리 모양은 점이 빈 곳에 찍힘)

**내 물체의 색 값 정하기 ② — 값 재기**

1. 물체를 **화면 가운데**에 크게 비춤
2. 터미널 ②에 1초마다 출력되는 값을 읽음 — 예: `center HSV = [ 62 180 140]` → H 62 · S 180 · V 140
3. 아래 표로 파라미터를 계산

| 파라미터 | 정하는 법 | 예 (H 62 · S 180 · V 140) |
|------|------|:--:|
| `h_min` · `h_max` | H − 10 · H + 10 | 52 · 72 |
| `s_min` | S의 절반 (50보다 작으면 50) | 90 |
| `v_min` | V의 절반 (40보다 작으면 40) | 70 |

- V를 절반으로 잡는 이유 — **그늘에서는 V가 절반으로 떨어집니다**(4.2)
- 출력값이 매번 크게 달라지면 → 물체가 가운데에서 벗어났거나 반사광이 비친 상태 → 물체를 옮기고 **2부터 다시**

**내 물체의 색 값 정하기 ③ — 빨강(H가 0 근처)**

H는 179 다음이 0으로 이어집니다. 계산 결과가 0보다 작거나 179보다 크면 **180을 더하거나 빼서** 넣습니다.

| 측정 H | 계산 | 넣는 값 |
|:--:|------|------|
| 4 | 4 − 10 = −6 → **174** · 4 + 10 = 14 | `h_min 174` · `h_max 14` |
| 175 | 175 − 10 = 165 · 175 + 10 = 185 → **5** | `h_min 165` · `h_max 5` |

- `h_min`이 `h_max`보다 크면 코드 ⑥이 **두 구간을 합쳐** 잡습니다(5.3)

**내 물체의 색 값 정하기 ④ — 시작 값 참고**

재기 전에 먼저 넣어 볼 값입니다.

| 색 | `h_min` | `h_max` |
|------|:--:|:--:|
| 빨강 | 170 | 10 |
| 주황 | 10 | 20 |
| 노랑 | 20 | 35 |
| 초록 (기본값) | 40 | 80 |
| 파랑 | 90 | 130 |

- `s_min 80` · `v_min 60`은 공통 시작 값
- 파랑에 하늘색이 함께 잡히면 → `h_min`을 100으로 올리고 **다시** 확인

**색 범위 조정** — 정한 값을 실행 중인 노드에 넣습니다(코드 수정 없음). 대상을 비춰도 x = -1.0만 나오면 값을 다시 조정합니다:

```bash
ros2 param set /color_tracker h_min 35
ros2 param set /color_tracker h_max 90
ros2 param set /color_tracker s_min 60
```

- 대상을 움직여 x가 변하면 → 조정 완료. 맞춘 값은 `ros2 param dump /color_tracker > color.yaml`로 저장
- 여전히 -1.0이면 → 5.5 마스크 영상으로 원인을 확인한 뒤 **이 조정으로 돌아옴**

> **자주 하는 실수 —** ① 조명이 바뀌면 값이 맞지 않습니다(창가와 실내 조명 아래가 다릅니다) ② `m00 > 500`의 500은 해상도에 따라 달라집니다 — 3.4에서 확인한 실제 크기가 이보다 크면 같은 대상의 면적도 커지므로, 검출이 지나치게 민감하면 이 값을 올립니다 ③ 값을 정하는 방법 — `/target_point`의 `z`(면적)를 `echo`로 보면서 대상이 있을 때와 없을 때의 값을 비교해 그 사이로 정합니다.

### 5.5 마스크 영상 확인 (선택)

검출되지 않는 원인을 눈으로 확인하려면 마스크를 발행합니다.

```python
# __init__에 추가
self.mask_pub = self.create_publisher(Image, '/mask_view', 10)

# on_image 말미에 추가
self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, 'mono8'))
```

```bash
ros2 run rqt_image_view rqt_image_view      # /mask_view 선택
```

- 이 도구는 **창이 필요하므로 SSH 터미널에서는 열리지 않습니다**(2.5) → **RDP로 연결해 그 화면에서 실행**하거나, 아래처럼 한 장을 파일로 저장해 VS Code에서 엽니다

```bash
# 대체 — 마스크 한 장을 파일로 저장
ros2 run image_view image_saver --ros-args -r image:=/mask_view \
  -p filename_format:="mask_%04d.png"       # 없으면 sudo apt install -y ros-jazzy-image-view
```

- 추가 위치 — 첫 줄은 `self.pub = ...` 다음 줄에 **같은 들여쓰기**로, 둘째 줄은 `self.pub.publish(point)` 다음 줄에(들여쓰기가 어긋나면 다른 메서드에 속하게 됨)
- 대상이 **흰색으로 또렷하게** 보이면 범위가 맞은 상태 → 5.4 관찰로 돌아감
- 화면 전체가 흰색이면 범위가 지나치게 넓은 상태 → `s_min`·`v_min`을 높이고 **다시** 확인
- 화면 전체가 검으면 범위가 대상 색과 어긋난 상태 → `h_min`·`h_max`를 대상 색에 맞추고 **다시** 확인
- 작은 흰 점이 여러 개 남으면 → 코드 ⑦의 `(5, 5)`를 `(7, 7)`·`(9, 9)`로 늘리고 빌드부터 **다시** 실행

---

## 6. 왜 AI 분류인가

**학습 목표** — 규칙 기반과 학습 기반의 차이를 설명하고, 분류 문제의 입력과 출력을 말할 수 있다.

### 6.1 규칙을 사람이 쓸 수 없을 때

5장에서 우리는 규칙을 직접 썼습니다 — "H가 40~80이면 초록".

정지 표지판을 이렇게 쓰려면 어떤 규칙이 필요할까요?

| 시도 | 문제 |
|------|------|
| "빨간색이면 정지 표지판" | 빨간 캔·빨간 옷도 함께 검출됨 |
| "빨갛고 팔각형이면" | 각도가 틀어지면 팔각형으로 보이지 않음 |
| "빨갛고 팔각형이고 흰 글자가 있으면" | 거리·조명에 따라 글자가 뭉개짐 |

**규칙이 늘어날수록 예외도 늘어납니다.** 사람이 다 쓸 수 없습니다.

- 학습 기반은 규칙을 쓰지 않습니다 — **정답이 붙은 예시를 충분히 제시하면** 모델이 스스로 특징을 찾습니다
- 우리가 하는 일 = **좋은 예시를 준비하는 것**

### 6.2 분류 문제의 구조

```
이미지 1장 ──▶ 모델 ──▶ 클래스별 점수 ──▶ 최고 점수 = 판정
                        (정지 0.92 · 직진 0.05 · 배경 0.03)
```

| 용어 | 의미 | 이 실습에서 |
|------|------|------|
| **클래스** | 구분할 종류 | 정지 · 직진 · 배경 |
| **학습 데이터** | 정답이 붙은 예시 | 클래스별로 찍은 사진 |
| **모델** | 학습 결과물 | `model_unquant.tflite` 파일 |
| **추론** | 새 입력에 모델을 적용 | 카메라 영상 → 판정 |
| **신뢰도** | 그 판정의 점수 | 0.0~1.0 |

> **배경 클래스를 반드시 둡니다 —** 클래스가 "정지"와 "직진" 둘뿐이면, 모델은 아무것도 없는 화면도 둘 중 하나로 판정합니다. 점수를 나눌 곳이 그 둘뿐이기 때문입니다. **"배경"(해당 없음) 클래스**를 넣어야 "지금은 표지가 없다"를 표현할 수 있습니다.

### 6.3 학습 환경과 실행 환경

| 구간 | 어디서 | 무엇을 |
|------|------|------|
| **학습** | 강의실 PC의 **브라우저**(Teachable Machine) | 사진 수집 → 학습 → 모델 파일 생성 |
| **실행** | **RPi5**의 ROS2 노드 | 모델 파일을 불러 영상에 적용 |

- 두 환경은 **파일 하나(`model_unquant.tflite`)로 연결**됩니다 — 실제 파일 이름은 9.2의 반출 선택에 따라 정해집니다
- 학습에 GPU(Graphics Processing Unit)가 필요하지만 브라우저 서비스가 대신 처리 — 이 과정에서 학습용 하드웨어는 불필요
- 추론만 RPi5 CPU(Central Processing Unit)로 수행 — 가벼운 모델이라 가능

### 6.4 확인 활동 — 개념 점검

| # | 질문 | 확인하려는 것 |
|:--:|------|------|
| 1 | 빨간 정지 표지와 빨간 캔을 5장의 방법으로 구분할 수 있는가? | 6.1 — 규칙 기반의 한계 |
| 2 | 클래스를 `stop`·`go` 둘만 두면 빈 화면은 어떻게 판정되는가? | 6.2 — **`none` 클래스가 필요한 이유** |
| 3 | 학습은 어디서 하고 실행은 어디서 하는가? 둘을 잇는 것은 무엇인가? | 6.3 — 모델 파일 하나 |
| 4 | 신뢰도 0.55로 `stop`이 나왔다. 그대로 믿어도 되는가? | 6.2 — 하한 설정의 필요 |

- 2번이 오늘 설계의 핵심입니다 — **점수를 나눌 곳이 둘뿐이면 억지로 둘 중 하나가 됩니다**

---

## 7. 실습 ① — 학습 데이터 수집

**학습 목표** — 분류 목표에 맞는 클래스를 정의하고, 조건을 통제해 학습 데이터를 수집할 수 있다.

### 7.1 클래스 정의

| 클래스 | 대상 | 차량 동작(Day 6) |
|------|------|------|
| `stop` | 정지 표지 | 정지 |
| `go` | 직진 표지 | 주행 |
| `none` | **표지 없음**(배경) | 현 상태 유지 |

- 3종으로 시작 — 클래스가 늘면 데이터도 그만큼 늘어야 함
- 이름은 **영문 소문자**로 통일 — 코드에서 그대로 사용

### 7.2 촬영 조건 통제

**같은 조건으로만 찍으면 그 조건에서만 동작합니다.** 조건을 의도적으로 나눠 찍습니다.

| 조건 | 변화 범위 | 이유 |
|------|------|------|
| **거리** | 30cm · 60cm · 100cm | 차량이 접근하며 크기가 변함 |
| **각도** | 정면 · 좌 30° · 우 30° | 주행 중 비스듬히 보임 |
| **조명** | 실내등 · 창가 · 그늘 | 시간·위치에 따라 달라짐 |
| **배경** | 최소 2종 | 배경을 특징으로 학습하는 것을 방지 |

> **자주 하는 실수 —** 한 자리에서 연속으로만 찍으면 거의 같은 사진 100장이 됩니다. 장수는 채워지지만 학습에는 도움이 되지 않습니다. **촬영하는 동안 대상을 계속 움직이십시오** — 거리·각도를 바꾸며.

- 클래스당 **100장 내외** 권장 — Teachable Machine은 웹캠으로 연속 촬영을 지원
- `none` 클래스는 **표지가 없는 다양한 장면** — 빈 책상·사람·다른 물체

### 7.3 수집 절차

강의실 PC 브라우저에서 진행합니다.

1. **https://teachablemachine.withgoogle.com** 접속 → **Get Started** → **Image Project** → **Standard image model**
2. Class 1의 이름 부분을 눌러 `stop`으로 변경 → **Webcam** 선택
   - 브라우저가 카메라 권한을 물으면 → **허용**(최초 1회)
   - 촬영 화면이 표시되면 → 3으로 진행
   - 눌러도 화면이 표시되지 않으면 → 주소창 자물쇠 아이콘에서 카메라 권한을 **허용**으로 바꾸고 **2를 다시** 수행
   - **웹캠이 없으면** → **Upload** 탭으로 전환해 휴대전화로 찍은 사진을 올립니다(위 조건 통제는 그대로 적용) → 그래도 어려우면 14.1의 RPi5 카메라 경로
3. **Hold to Record**를 누른 채 대상을 움직이며 촬영(약 10초 = 100장 내외)
4. **Add a class**로 클래스를 추가해 `go`·`none`을 같은 방식으로 촬영(기본 화면에는 클래스가 2개만 있습니다)

| 확인 | 기준 |
|------|:--:|
| 클래스별 장수 | **비슷하게** — 한 클래스만 많으면 그쪽으로 치우침 |
| 미리보기 | 같은 그림이 반복되지 않는지 |
| 초점 | 흐린 사진이 섞이지 않았는지 |

> **촬영한 사진을 보관해 둡니다 —** 각 클래스 메뉴(⋮)의 **Download Samples**로 사진을 내려받아 보관하십시오. 공식 표지판이 배포되면 같은 절차로 다시 학습하는데, 그때 오늘 데이터와 비교하면 데이터가 바뀌면 모델이 어떻게 달라지는지를 직접 확인할 수 있습니다. Teachable Machine은 브라우저에서 동작하므로 **탭을 닫으면 작업 내용이 사라집니다.** 학습 프로젝트 자체를 남기려면 좌상단 메뉴에서 **Save project to Drive** 또는 **Download project as file**을 사용합니다.

---

## 8. 실습 ② — 모델 학습과 검증

**학습 목표** — 모델을 학습하고 결과를 즉시 검증하며, 오분류를 관찰해 데이터를 보강할 수 있다.

### 8.1 학습 실행

**Train Model** 버튼을 누릅니다.

| 설정 | 기본값 | 조정 |
|------|:--:|------|
| Epochs | 50 | 그대로 — 데이터가 적으면 늘려도 개선이 크지 않음 |
| Batch Size | 16 | 그대로 |
| Learning Rate | 0.001 | 그대로 |

- 위 세 설정은 **Advanced**를 펼쳐야 표시됩니다 — 기본 화면에는 **Train Model** 버튼만 있습니다. 오늘은 값을 바꾸지 않으므로 펼치지 않아도 됩니다
- **1~3분** 소요 — 이 대기 시간에 8.2를 진행합니다
- 학습이 끝날 때까지 **이 탭을 화면에 띄운 상태로 유지**해야 함 — 다른 탭으로 이동하면 브라우저가 학습을 늦추거나 멈추고, 창을 닫으면 처음부터 다시 수행
- 진행 막대가 멈춘 채 5분이 지나면 → 탭을 새로 고치지 말고 교수에게 알림(새로 고치면 촬영분이 사라집니다)

### 8.2 왜 1~3분 만에 끝나는가

이미지 인식 모델을 처음부터 학습하려면 수백만 장의 사진과 긴 계산 시간이 필요합니다. 우리는 클래스당 100장 내외로 몇 분 만에 마쳤습니다. **전이 학습(Transfer Learning)** 방식이기 때문입니다.

| 구간 | 역할 | 학습 여부 |
|------|------|------|
| **특징 추출부** | 윤곽·모서리·질감 등 **일반적인 시각 특징** 인식 | ❌ 고정 — 사전 학습 그대로 |
| **분류부** | 그 특징을 **내 클래스에 대응** | ✅ 내 사진으로만 재학습 |

- Teachable Machine은 대규모 데이터로 미리 학습된 모델(MobileNet 계열)을 내장하고, 마지막 분류부만 우리 사진으로 다시 학습합니다
- 그래서 **적은 데이터·짧은 시간**으로 실용 수준에 도달합니다
- 한계 역시 이 구조에 있습니다 — 특징 추출부가 일반 사진 기준이므로, 성격이 크게 다른 영상(열화상 등)에서는 정확도가 낮아집니다

### 8.3 즉시 검증

학습이 끝나면 우측 Preview에서 웹캠이 켜집니다. 대상을 비추며 확인합니다.

| 관찰 | 정상 |
|------|------|
| 정지 표지를 비춤 | `stop` 막대가 90% 이상 |
| 아무것도 없음 | `none`이 우세 |
| 대상을 천천히 치움 | 막대가 부드럽게 전환 |

- **여기서 끝내지 않습니다.** 다음 절이 오늘의 핵심입니다

### 8.4 오분류 유도

의도적으로 어려운 조건을 줍니다.

| 시도 | 예상 결과 |
|------|------|
| 표지를 **아주 멀리** | 신뢰도 하락 · 오분류 |
| **비스듬히** 크게 기울임 | 오분류 |
| **손으로 절반을 가림** | 오분류 |
| **찍지 않은 배경**에서 보여줌 | 오분류 |
| **다른 조명**(창가 등) | 신뢰도 하락 |

> **이것이 오늘의 학습 내용입니다 —** 오분류는 **모델의 결함이 아니라 데이터의 공백**입니다. 학습 데이터에 없던 조건이 들어왔기 때문입니다. **코드를 고쳐서 해결할 수 없습니다.**

### 8.5 데이터 보강과 재학습

오분류가 난 조건을 **추가로 촬영**해 다시 학습합니다.

1. 오분류가 발생한 조건(예: 멀리서·비스듬히)으로 해당 클래스에 **30~50장 추가**
2. **Train Model** 재실행
3. 같은 조건으로 다시 검증

| 항목 | 1차 | 2차(보강 후) |
|------|:--:|:--:|
| 정면·근거리 | 정상 | 정상 |
| 원거리 | 오분류 | **개선** |
| 비스듬히 | 오분류 | **개선** |

- **이 순환을 최소 1회 수행**합니다 — 첫 학습 결과로 끝내지 않습니다
- 실무의 AI 개발도 대부분 이 순환 — "모델을 바꾸는 것"보다 **"데이터를 채우는 것"**이 효과가 큽니다

> **Tip —** 어떤 조건에서 틀렸는지 기록해 두십시오. Day 6에서 차량이 오작동할 때 원인을 찾는 근거가 됩니다.

---

## 9. 모델 배포 — TFLite

**학습 목표** — 학습 결과를 RPi5에서 쓸 수 있는 형식으로 반출할 수 있다.

### 9.1 왜 TFLite인가

| 형식 | 용도 | 크기·속도 |
|------|------|------|
| 원본 모델 | 학습·연구 환경 | 큼 · 무거움 |
| **TFLite** | **소형 기기 실행 전용** | **작음 · 가벼움** |

- **TFLite** = TensorFlow Lite — 휴대기기·임베디드 보드에서 추론만 수행하도록 줄인 **파일 형식**(확장자 `.tflite`)
- RPi5에는 신경망 연산을 가속하는 장치가 없어 CPU로 처리하므로, 연산량을 줄인 이 형식이 사실상 필수
- 학습은 못 하고 **추론만** 가능 — 오늘의 용도에는 충분
- **형식과 실행기는 구분합니다** — 파일 형식은 `.tflite` 그대로이고, 이 파일을 읽어 실행하는 **실행기는 LiteRT**(패키지 `ai-edge-litert`)입니다. 설치는 10.1에서 다룹니다

### 9.2 반출

Teachable Machine의 **Export Model** → **Tensorflow Lite** 탭:

| 선택 | 값 | 이유 |
|------|------|------|
| Model conversion type | **Floating point** | 정확도 손실이 없는 기본 형식 — 오늘 규모에서는 속도 차이가 크지 않음 |
| 다운로드 | **Download my model** | 변환에 수십 초~수 분 소요 — 진행 표시가 끝날 때까지 탭 유지 |

- 변환이 끝나면 `converted_tflite.zip`이 내려받아집니다 → 아래 파일 구성을 확인한 뒤 9.3으로 진행
- 진행 표시가 몇 분이 지나도 끝나지 않으면 → 탭을 그대로 둔 채 **Download my model을 다시** 누름
- 그래도 내려받아지지 않으면 → 교수에게 알림

> **변환 형식을 바꾸면 파일 이름이 달라집니다 —** **Quantized**를 고르면 파일 이름이 `model.tflite`로 나와 9.3·10.3에 적힌 경로와 어긋납니다. **EdgeTPU**는 별도 가속기(Coral) 전용이라 RPi5에서 동작하지 않습니다. 오늘은 **Floating point**를 선택합니다.

받은 압축 파일(`converted_tflite.zip`)을 풀면 두 개가 나옵니다.

| 파일 | 내용 |
|------|------|
| `model_unquant.tflite` | 모델 본체 |
| `labels.txt` | **클래스 이름 목록** — 순서가 모델의 출력 순서 |

`labels.txt` 예시:

```
0 stop
1 go
2 none
```

- **순서가 중요합니다** — 모델의 출력은 번호이며, 이 파일이 번호를 이름으로 변환합니다

### 9.3 RPi5로 이동

내려받은 파일은 **Windows의 다운로드 폴더**에 있습니다. 2.3에서 연결한 **VS Code 원격 창**의 탐색기에는 RPi5의 폴더가 표시되므로, Windows 탐색기의 파일을 그 위로 **끌어다 놓으면 RPi5에 복사됩니다.** 별도의 전송 명령은 필요하지 않습니다.

**①** PC에서 압축을 풉니다.

- Windows 탐색기의 **다운로드** 폴더에서 `converted_tflite.zip`을 마우스 오른쪽 버튼으로 클릭 → **압축 풀기**
- 풀린 폴더에 `model_unquant.tflite`·`labels.txt` 두 파일이 보이면 → ②-1로 진행
- `converted_tflite.zip`이 보이지 않으면 → 브라우저의 내려받기 목록에서 저장 위치를 확인한 뒤 **①을 다시** 수행

**②-1** 받을 폴더를 만듭니다 — **VS Code 원격 창**에서 수행합니다.

- 좌측 탐색기에서 `ros2_ws/src/my_car_pkg` 폴더를 마우스 오른쪽 버튼으로 클릭 → **새 폴더** → 이름 `model`
- `my_car_pkg`는 같은 이름의 폴더가 바깥과 안쪽에 이중으로 있습니다 — **`setup.py`가 있는 바깥 폴더** 아래에 만듭니다
- `setup.py`와 같은 위치에 `model` 폴더가 보이면 → ②-2로 진행
- 좌하단에 `SSH: 192.168.0.__` 표시가 없으면 로컬 창입니다 → 2.3 ②로 연결한 뒤 **②-1을 다시** 수행
- 2.3 ⑤(SSH 터미널만 사용)로 진행 중이면 → **②-3**으로 진행

**②-2** 두 파일을 끌어다 놓습니다.

- Windows 탐색기에서 `model_unquant.tflite`·`labels.txt`를 함께 선택합니다
- VS Code 탐색기의 **`model` 폴더 위**로 끌어다 놓습니다

RPi5 터미널에서 확인합니다.

```bash
ls ~/ros2_ws/src/my_car_pkg/model
```

- 두 파일이 출력되면 → ③으로 진행
- `No such file or directory`가 나오면 → `model` 폴더를 안쪽 `my_car_pkg`에 만든 경우입니다. VS Code 탐색기에서 폴더를 바깥 `my_car_pkg`로 끌어 옮긴 뒤 **위 `ls`를 다시** 실행
- 압축 파일(`.zip`)을 그대로 옮겼으면 → VS Code 탐색기에서 삭제하고 **①부터 다시** 수행

**②-3** VS Code 원격 창을 사용하지 않는 경우 — 2.3 ⑤로 진행 중일 때만 수행합니다.

받을 폴더를 RPi5의 SSH 터미널에서 만듭니다.

```bash
mkdir -p ~/ros2_ws/src/my_car_pkg/model
```

PC에서는 ①에서 푼 폴더를 Windows 탐색기로 열고, **주소 표시줄에 `powershell`을 입력**해 그 폴더에서 PowerShell을 실행합니다.

```powershell
scp model_unquant.tflite labels.txt 사용자명@192.168.0.__:~/ros2_ws/src/my_car_pkg/model/
```

- RPi5에서 `ls ~/ros2_ws/src/my_car_pkg/model`로 두 파일이 출력되면 → ③으로 진행
- **보내는 쪽**에서 `No such file or directory`가 나오면 → PowerShell에서 `ls`로 두 파일이 현재 폴더에 있는지 확인하고 **`scp`를 다시** 실행
- **받는 쪽** 경로 오류(`scp: dest ... No such file`)가 나오면 → 위 **`mkdir`을 다시** 실행
- 연결이 거부되면 → 주소·같은 네트워크 확인 후 **`scp`를 다시** 실행
- `scp`를 찾을 수 없다고 나오면 → USB 메모리로 복사하고 교수에게 알림

**③ 패키지에 포함하기** — `setup.py`에 자료 파일 등록:

```python
data_files=[
    ...
    ('share/' + package_name + '/model', ['model/model_unquant.tflite',
                                          'model/labels.txt']),
],
```

| 코드 | 뜻 |
|------|------|
| `'share/' + package_name + '/model'` | 문자열 `+` = 이어 붙이기 — 설치될 폴더 경로를 구성 |
| `(경로, [파일 목록])` | 소괄호 = 튜플 한 쌍 — 앞은 설치 위치, 뒤는 그곳에 설치할 파일의 리스트 |

- `data_files=[...]`는 `setup.py` 상단에 **이미 있는 항목**입니다 — 기존 줄 아래에 위 한 쌍을 더합니다(`console_scripts`를 Day 3에서 추가한 것과 같은 방식)
- 이렇게 해야 `colcon build` 후 설치 경로에서 모델을 찾을 수 있음

> **자주 하는 실수 —** 모델 파일을 소스 폴더에만 두고 등록하지 않으면, 실행 시 "파일을 찾을 수 없음" 오류가 납니다. 빌드 결과물에는 등록된 것만 들어갑니다.

---

## 10. 실습 ③ — 추론 노드 작성

**학습 목표** — 모델을 불러 카메라 영상에 적용하는 노드를 작성하고, 판정 결과와 신뢰도를 토픽으로 발행할 수 있다.

### 10.1 실행 환경 준비

**①** 실행기를 설치합니다.

```bash
sudo apt install -y python3-pip
pip3 install ai-edge-litert --break-system-packages
```

| 패키지 | 역할 | 설치 시점 |
|------|------|------|
| `ai-edge-litert` | **추론 전용** 경량 실행기(LiteRT) — 학습 기능 없이 실행만 | 위 명령 |
| `python3-opencv` | 영상 전처리 | **5.1에서 설치** |
| `cv_bridge` | ROS2 ↔ OpenCV | **5.1에서 설치** |

- 아래 두 줄은 **5.1을 수행한 경우 이미 설치된 상태**입니다. 5장을 건너뛰었다면 여기서 먼저 실행합니다

```bash
sudo apt install -y ros-jazzy-cv-bridge python3-opencv
```

**②** 설치를 확인합니다.

```bash
python3 -c "from ai_edge_litert.interpreter import Interpreter; print('ok')"
```

- `ok`가 출력되면 → 아래 **NumPy 버전 확인**으로 진행
- `externally-managed-environment` 오류가 나오면 → `--break-system-packages`를 빠뜨린 것입니다. **①을 다시** 실행
- `Retrying`·`Read timed out`·`Temporary failure in name resolution`이 나오면 → 외부망 접속 문제입니다. ①을 반복하지 말고 **③ 오프라인 설치**로 진행
- `No matching distribution`이 나오면 → **③ 오프라인 설치**로 진행하고 교수에게 알림

**NumPy 버전 확인** — `ok`가 출력된 뒤 수행합니다.

```bash
python3 -c "import numpy; print(numpy.__version__)"
```

- `1.`로 시작하면(예: `1.26.4`) → 10.2로 진행
- `2.`로 시작하면 → 아래 명령으로 1.x로 낮춘 뒤 **위 버전 확인을 다시** 실행

```bash
pip3 install "numpy<2" --break-system-packages
```

- 실행기를 설치할 때 NumPy 2.x가 함께 설치되는 경우가 있습니다
- `apt`로 설치한 `cv_bridge`·`python3-opencv`는 **NumPy 1.x 기준으로 빌드**되어 있어, 2.x 상태에서는 10.4의 노드가 `numpy` 관련 오류로 종료됩니다
- 낮춘 뒤에도 `2.`로 출력되면 → 교수에게 알림

**③ 오프라인 설치** — ②에서 외부망 문제로 설치가 진행되지 않은 경우에만 수행합니다.

교수가 미리 받아 둔 설치 파일(`.whl`)을 USB 메모리 또는 9.3 ②와 같은 방식(VS Code 탐색기로 끌어다 놓기)으로 받은 뒤, 그 파일이 있는 폴더에서 실행합니다.

```bash
pip3 install ./ai_edge_litert-*.whl --break-system-packages
```

- 설치가 끝나면 → **②를 다시** 실행해 `ok`를 확인
- 그래도 실패하면 → 교수에게 알림. 이 경우 10~11장은 **옆자리 화면으로 동작을 확인**하고, 자신의 코드 작성·`setup.py` 등록·빌드까지는 그대로 수행합니다

> **형식과 실행기는 다릅니다 —** 모델 파일 형식은 `.tflite` 그대로이고, 그 파일을 읽어 실행하는 **실행기(런타임)의 이름**이 바뀐 것입니다. 구글은 TensorFlow Lite 런타임을 **LiteRT**로 개칭했고, 파이썬 패키지도 `tflite-runtime`에서 **`ai-edge-litert`**로 이어집니다. 인터넷의 예제 다수가 아직 `tflite-runtime`을 씁니다. **Ubuntu 24.04의 Python 3.12에는 그 패키지가 설치되지 않으므로**(지원 범위가 3.11까지) 위 패키지를 사용합니다. 코드에서 달라지는 것은 **import 줄과 클래스 이름 두 곳뿐**입니다.

### 10.2 노드 설계

| 항목 | 내용 |
|------|------|
| 노드 이름 | `sign_classifier` |
| 구독 | `/camera/image_raw` |
| 발행 | `/sign` (`std_msgs/msg/String`) — 판정 이름 |
| | `/sign_confidence` (`std_msgs/msg/Float32`) — 신뢰도 |
| 파라미터 | `threshold` — 이 값 미만이면 `none`으로 처리 |
| 처리 주기 | **매 프레임이 아니라 일정 간격** — RPi5 부하 관리 |

> **매 프레임 추론하지 않습니다 —** 카메라는 초당 30장을 보냅니다. RPi5 CPU로 30번 추론하면 다른 처리가 밀립니다. **초당 5회 정도**면 표지판 판정에 충분합니다.

### 10.3 코드 작성

`~/ros2_ws/src/my_car_pkg/my_car_pkg/sign_classifier.py`:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from ai_edge_litert.interpreter import Interpreter
import numpy as np
import cv2
import os

class SignClassifier(Node):
    def __init__(self):
        super().__init__('sign_classifier')
        self.bridge = CvBridge()
        self.declare_parameter('threshold', 0.7)                  # ① 신뢰도 하한
        self.declare_parameter('period', 0.2)                     # ② 추론 간격(초)

        share = get_package_share_directory('my_car_pkg')         # ③ 설치 경로 조회
        model_path = os.path.join(share, 'model', 'model_unquant.tflite')
        label_path = os.path.join(share, 'model', 'labels.txt')

        self.interpreter = Interpreter(model_path=model_path)          # ④ 모델 적재
        self.interpreter.allocate_tensors()
        self.in_detail = self.interpreter.get_input_details()[0]
        self.out_detail = self.interpreter.get_output_details()[0]

        with open(label_path) as f:                               # 라벨 읽기
            self.labels = [line.strip().split(' ', 1)[1] for line in f if line.strip()]

        self.latest = None                                        # ⑤ 최신 프레임 보관
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.on_image, 10)
        self.pub = self.create_publisher(String, '/sign', 10)
        self.conf_pub = self.create_publisher(Float32, '/sign_confidence', 10)

        period = self.get_parameter('period').value
        self.timer = self.create_timer(period, self.infer)        # ⑥ 주기 추론
        self.get_logger().info(f'sign_classifier ready — labels: {self.labels}')

    def on_image(self, msg):
        self.latest = self.bridge.imgmsg_to_cv2(msg, 'bgr8')      # 보관만

    def infer(self):
        if self.latest is None:
            return
        h = self.in_detail['shape'][1]                            # ⑦ 모델 입력 크기
        w = self.in_detail['shape'][2]
        img = cv2.resize(self.latest, (w, h))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        x = np.expand_dims(img.astype(np.float32) / 127.5 - 1.0, axis=0)   # ⑧ 정규화

        self.interpreter.set_tensor(self.in_detail['index'], x)
        self.interpreter.invoke()
        scores = self.interpreter.get_tensor(self.out_detail['index'])[0]

        idx = int(np.argmax(scores))                              # ⑨ 최고 점수
        conf = float(scores[idx])
        name = self.labels[idx] if conf >= self.get_parameter('threshold').value else 'none'

        self.pub.publish(String(data=name))
        self.conf_pub.publish(Float32(data=conf))

def main(args=None):
    rclpy.init(args=args)
    node = SignClassifier()
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
| ①② | 신뢰도 하한·추론 간격을 **파라미터로** — 현장에서 조정 |
| ③ | `get_package_share_directory` — **설치된 패키지의 경로**를 찾는 표준 함수(9.3에서 등록한 그 위치) |
| ④ | 모델을 메모리에 적재 — **한 번만** 수행(콜백 안에서 하면 매번 느려짐) |
| ⑤ | 구독 콜백은 **보관만** — 추론은 타이머가 담당 |
| ⑥ | 타이머로 주기 추론 — 카메라 속도와 추론 속도를 분리 |
| ⑦ | 모델이 요구하는 입력 크기(보통 224×224)로 축소 |
| ⑧ | 정규화 — Teachable Machine 모델은 **−1~1 범위**를 기대 |
| ⑨ | 최고 점수 선택 → **하한 미만이면 `none`** |

> **자주 하는 실수 —** ⑧의 정규화 범위를 −1~1이 아니라 0~1로 두거나, ⑦ 다음의 BGR→RGB 변환을 빠뜨리면 **오류 없이 실행되면서 판정만 특정 클래스로 치우칩니다.** 실행이 된다고 전처리가 맞는 것은 아닙니다 — 신뢰도가 계속 비정상이면 ⑦·⑧을 먼저 확인합니다.

**코드 읽기 — Python 문법 ④ 클래스와 객체** — 5.3의 형태가 이 노드에도 그대로 나타납니다.

| 코드 | 뜻 |
|------|------|
| `class SignClassifier(Node):` | 괄호 = 상속 — `Node`의 기능을 물려받은 새 클래스 |
| `def __init__(self):` | 객체를 만들 때 한 번 실행되는 초기화 메서드 |
| `super().__init__('sign_classifier')` | 부모(`Node`)의 초기화를 먼저 실행 — 노드 이름 등록 |
| `self.interpreter = ...` | `self.` = 이 객체의 변수 — `infer()`에서 같은 값을 다시 사용 |
| `def infer(self):` | 메서드 정의 — 타이머가 주기마다 호출 |

**코드 읽기 — Python 문법 ⑤ 파일 읽기와 문자열** — `labels.txt`에서 라벨 이름만 남기는 한 줄의 문법입니다.

| 코드 | 뜻 |
|------|------|
| `os.path.join(share, 'model', ...)` | 폴더·파일 이름을 경로 문자열로 결합 — `/`를 직접 붙이지 않음 |
| `with open(label_path) as f:` | 파일을 열고, 블록이 끝나면 자동으로 닫음 |
| `line.strip()` | 앞뒤 공백·줄바꿈 제거 |
| `line.split(' ', 1)[1]` | 공백으로 1회만 자른 뒤 두 번째 요소 — `0 stop` → `stop` |
| `[A for line in f if B]` | 리스트 컴프리헨션 — 각 줄 중 `B`가 참인 것만 `A`로 변환해 리스트로 수집 |

**코드 읽기 — Python 문법 ⑥ 배열 연산과 형변환** — `infer()`의 전처리 구간(⑦⑧)입니다.

| 코드 | 뜻 |
|------|------|
| `self.in_detail['shape'][1]` | 딕셔너리에서 얻은 값이 리스트이므로 번호로 다시 접근 — 연쇄 접근 |
| `img.astype(np.float32)` | 배열 전체의 자료형을 실수로 변환 |
| `... / 127.5 - 1.0` | 배열 전체에 한 번에 적용되는 연산 — 반복문 불필요 |
| `np.expand_dims(x, axis=0)` | `axis=0` = 키워드 인자 — 인자의 순서를 확인하지 않고 이름으로 지정 |
| `int(...)` · `float(...)` | 자료형 변환 — 인덱스는 정수, 메시지 필드는 실수 |

**코드 읽기 — Python 문법 ⑦ 조건과 객체 생성**

| 코드 | 뜻 |
|------|------|
| `if self.latest is None:` | `is None` = 값이 아직 없음의 표준 확인 — `== None`은 사용하지 않음 |
| `A if 조건 else B` | 조건부 표현식 — 한 줄로 값을 선택. 5.3 문법 ③의 `if`·`else` 블록과 뜻은 같고 결과가 값 |
| `String(data=name)` | 메시지 객체를 만들며 필드 값을 함께 지정 — 키워드 인자 |
| `f'... {self.labels}'` | f-문자열 — 따옴표 앞에 `f`, 중괄호 안 변수의 값이 그 자리에 삽입 |
| `try:` … `except:` … `finally:` | Ctrl+C로 종료해도 `finally`의 정리는 반드시 실행 |

- 위 네 표는 **Day 6·8·9의 코드에도 같은 형태로 다시 싣습니다** — 지금 전부 암기할 필요는 없습니다

`setup.py` 등록:

```python
'sign_classifier = my_car_pkg.sign_classifier:main',
```

### 10.4 실행과 관찰

```bash
cd ~/ros2_ws && colcon build && source install/local_setup.bash

# 터미널 ① — 2.4에서 실행해 둔 카메라 노드를 그대로 둡니다(종료됐으면 2.4의 명령으로 다시 실행)
# 터미널 ②
ros2 run my_car_pkg sign_classifier
# 터미널 ③
ros2 topic echo /sign
# 터미널 ④
ros2 topic echo /sign_confidence
```

터미널 ②에 `sign_classifier ready — labels: [...]`가 출력되고 `/sign`에 판정이 출력되면 → 아래 관찰로 진행합니다. 오류는 종류에 따라 돌아갈 단계가 다릅니다.

| 터미널 ②의 출력 | 조치 |
|------|------|
| `ModuleNotFoundError: ai_edge_litert` | **10.1 ①부터 다시** |
| `ModuleNotFoundError: cv_bridge`·`cv2` | 10.1 ①의 `apt` 실행 → **빌드부터 다시** |
| 메시지에 `NumPy`·`numpy`가 포함된 오류 | **10.1 ② NumPy 버전 확인** → `2.`이면 낮춘 뒤 **실행부터 다시** |
| `Could not open ...` · `FileNotFoundError` | **9.3 ③** `data_files` 등록·파일 이름 확인 → **빌드부터 다시** |
| `executable 'sign_classifier' not found` | 10.3 말미의 등록 확인 → **빌드부터 다시** |
| `IndexError` (라벨 읽기) | `labels.txt`가 `0 stop` 형태인지 확인 → 다르면 **9.2에서 다시 내려받기** |
| 정상 출력 후 `/sign` **무출력** | `ros2 topic hz /camera/image_raw` 확인 → 없으면 **2.4**에서 카메라 노드를 다시 실행(영상 미확보 시 **Day 4 자료 6장**) |
| 신뢰도가 계속 비정상 | **10.3 ⑦⑧**(입력 크기·정규화) 확인 → **빌드부터 다시** |

| 조작 | 예상 |
|------|------|
| 정지 표지를 비춤 | `/sign` = `stop`, 신뢰도 0.9 이상 |
| 치움 | `/sign` = `none` |
| 애매하게 반쯤 가림 | 신뢰도 하락 → 하한 미만이면 `none` |

**부하 확인**:

```bash
top                       # sign_classifier의 CPU 점유율
ros2 topic hz /sign       # 발행 주기 — period 설정과 일치하는지
```

- **Day 9 대응** — 실물에서는 이 노드와 모터 제어가 동시에 실행되어야 하므로 여유를 남겨야 함

### 10.5 실행 중 파라미터 조정

노드를 재시작하지 않고 파라미터를 바꿔 봅니다.

```bash
ros2 param list /sign_classifier                  # 이 노드의 파라미터 목록
ros2 param get /sign_classifier threshold         # 현재 값 확인
ros2 param set /sign_classifier threshold 0.9     # 실행 중 변경
```

| 실험 | 조작 | 관찰 |
|------|------|------|
| 하한 상향 | `threshold` 0.7 → **0.9** | 애매한 각도·거리의 표지가 `none`으로 판정됨 |
| 하한 하향 | `threshold` → **0.5** | 오판정 증가 — 14.2의 상충 관계를 실측 |
| 주기 변경 | `period` → **0.05** | **`ros2 topic hz /sign`이 그대로** — 반영되지 않음 |

**`threshold`는 즉시 반영되는데 `period`는 왜 그대로일까요?** 코드가 파라미터를 **언제 읽는가**의 차이입니다.

| 파라미터 | 코드가 읽는 시점 | 실행 중 변경 |
|------|------|:--:|
| `threshold` | **매 추론마다** `get_parameter` (⑨) | ✅ 반영 |
| `period` | **시작 시 한 번** — 타이머 생성에 사용 (⑥) | ❌ 미반영 |

- 반영 경로는 이로써 세 가지 — **시작 시 주입** · **매 주기 조회** · **변경 콜백**
- `period`를 실행 중에 반영하려면 변경 콜백으로 타이머를 다시 만들어야 합니다 — 이 노드에서는 재시작으로 충분
- **자유 조작** — `threshold`를 여러 값으로 조정하며, 11장에서 사용할 자신의 기준값을 미리 확인해 두십시오

---

## 11. 미니프로젝트 — 표지판 인식

**학습 목표** — 판정 결과를 실용 가능한 수준으로 안정화할 수 있다.

### 11.1 과제 — 흔들리는 판정의 안정화

`/sign`을 계속 관찰하면 문제가 보입니다.

| 증상 | 원인 |
|------|------|
| `stop`과 `none`이 빠르게 번갈아 나옴 | 경계 조건에서 신뢰도가 하한 근처를 오감 |
| 손이 지나가는 순간 오판정 | 순간적인 잘못된 입력 |
| 표지를 치웠는데 잠시 유지 | 마지막 값이 남음 |

**차량이 이 신호로 정지·주행을 결정한다면** 이 흔들림이 그대로 동작에 나타납니다. 안정화가 필요합니다.

### 11.2 설계 — 연속 확인

| 방법 | 원리 |
|------|------|
| **연속 N회 일치** | 같은 판정이 **N회 연속**될 때만 확정 |
| 다수결 | 최근 N회 중 **과반**을 채택 |
| 신뢰도 누적 | 최근 N회 신뢰도의 평균이 하한 이상일 때 |

이 실습은 **연속 N회 일치**를 사용합니다 — 가장 단순하고 효과가 분명합니다.

```
판정 순서:  stop stop none stop stop stop stop
N=3 확정:    -    -    -    -    -   stop  stop
```

- N이 크면 **안정적이지만 반응이 느림** — 표지를 지나칠 수 있음
- N이 작으면 빠르지만 흔들림 — **`period`와 함께 조정**(N=3, period=0.2 → 0.6초 후 확정)

### 11.3 구현 힌트

```python
# __init__
self.declare_parameter('stable_n', 3)
self.history = []
self.confirmed = 'none'
self.stable_pub = self.create_publisher(String, '/sign_stable', 10)

# infer 말미
self.history.append(name)
n = self.get_parameter('stable_n').value
if len(self.history) > n:
    self.history.pop(0)

if len(self.history) == n and len(set(self.history)) == 1:   # 전부 같으면
    if self.confirmed != self.history[0]:
        self.confirmed = self.history[0]
        self.get_logger().info(f'confirmed: {self.confirmed}')
self.stable_pub.publish(String(data=self.confirmed))
```

| 항목 | 내용 |
|------|------|
| `set(history)`의 길이가 1 | 최근 N회가 **모두 같은 값** |
| 변화 시에만 로그 | 매 주기 로그는 화면을 채워 관찰을 방해 |
| `/sign_stable` | Day 6의 판단 노드가 구독할 토픽 |

**코드 읽기 — 최근 N회를 유지하는 구조**

| 코드 | 뜻 |
|------|------|
| `self.history = []` | 빈 리스트 생성 — 판정 결과를 시간 순으로 보관 |
| `self.history.append(name)` | 리스트 맨 뒤에 추가 |
| `if len(...) > n: ... .pop(0)` | 개수가 넘치면 맨 앞(가장 오래된 값)을 제거 — 최근 N개만 유지 |
| `set(self.history)` | 중복을 제거한 모음 — 길이가 1이면 N개가 전부 같은 값 |
| `self.confirmed != self.history[0]` | `!=` = 다름 — 확정 값이 바뀐 순간에만 로그를 남기기 위한 비교 |

- `append`·`pop`·`len`·`set`은 ROS2 고유 문법이 아니라 **Python 기본 자료형의 기능**입니다 — Day 6 판단 노드에서 같은 형태로 다시 사용합니다

**추가 위치** — 위 두 조각을 10.3의 `sign_classifier.py`에 넣습니다.

| 조각 | 넣을 위치 |
|------|------|
| `# __init__` 4줄 | `self.conf_pub = self.create_publisher(...)` **다음 줄**에 **같은 들여쓰기**로 |
| `# infer 말미` 9줄 | `self.conf_pub.publish(Float32(data=conf))` **다음 줄**에 **같은 들여쓰기**로 |

- `if len(self.history) == n and ...` 안쪽의 `if`는 **한 단 더** 들여씁니다(들여쓰기 3단). 한 칸만 어긋나도 `IndentationError`가 나거나 다른 메서드에 속하게 됩니다

**실행과 확인**:

```bash
cd ~/ros2_ws && colcon build && source install/local_setup.bash

# 터미널 ②
ros2 run my_car_pkg sign_classifier
# 터미널 ③
ros2 topic echo /sign_stable
```

- `/sign_stable`에 값이 출력되면 → 11.4로 진행
- `IndentationError`·`SyntaxError`가 나오면 → 위 표의 들여쓰기를 확인하고 **빌드부터 다시** 실행
- `/sign`은 출력되는데 `/sign_stable`이 없으면 → `__init__`의 `stable_pub` 선언과 `infer` 말미의 발행 줄이 **둘 다** 들어갔는지 확인하고 **빌드부터 다시** 실행
- 확정이 전혀 바뀌지 않으면 → `stable_n`을 낮춰(`ros2 param set /sign_classifier stable_n 2`) 확인

### 11.4 단계별 과제

| 단계 | 과제 |
|:--:|------|
| **필수** | `/sign_stable`을 발행하고, 표지를 들었다 놓았을 때 **흔들림 없이 전환**되는지 확인 |
| **도달** | `stable_n`·`threshold`·`period`를 조정해(10.5) **반응 속도와 안정성의 균형**을 찾고 `param dump`로 저장 |
| **도전** | ⓐ 신뢰도 평균 방식으로 바꿔 비교 ⓑ 판정이 바뀐 시각을 로그에 남겨 **반응 지연을 측정** ⓒ 5장의 `color_tracker`와 함께 실행해 **CPU 여유** 확인 |

- **필수 단계는 전원 완료** — 안정화된 토픽 하나가 출력되면 성공
- **영상을 확보하지 못한 경우** — Day 4 자료 6장 ⓑ의 저장 영상 재생(14.6)으로 필수 단계를 수행합니다. 재생 중 표지가 화면에 들고 나는 구간이 그대로 전환 시험이 됩니다. 그래도 수행하지 못했으면 **다음 회차 전까지 카메라를 복구해 필수 단계를 마칩니다** — Day 6은 `/sign_stable`을 전제로 시작합니다
- Day 6 대응 — `/sign_stable`이 **판단 노드의 입력**이 됩니다. 오늘 그 계약을 확정합니다
- Day 9 대응 — 같은 노드가 실물 차량에서 그대로 동작합니다

---

## 12. 라인 인식 (자습)

수업 시간에는 다루지 않습니다. 4·5장의 색상 검출을 응용한 과제이며, 각자 자료를 보고 진행합니다.

### 12.1 과제 — 바닥의 선을 따라가려면

바닥에 붙인 색 테이프를 카메라로 보고, **차량이 어느 쪽으로 돌아야 하는지** 계산합니다.

```
┌─────────────────────┐
│                     │
│                     │   ← 상단은 멀리 = 무시
├─────────────────────┤
│         ██          │   ← 관심 영역(ROI): 화면 하단
└─────────────────────┘
     ↑        ↑
   화면중앙  선의 중심
```

| 항목 | 내용 |
|------|------|
| 입력 | `/camera/image_raw` |
| 출력 | `/cmd_vel` (`geometry_msgs/msg/Twist`) — turtlesim으로 검증 |
| 판단 | 선의 중심이 화면 중앙에서 **얼마나 벗어났는가** → 회전량 |

### 12.2 설계 — 관심 영역과 오차

**관심 영역(ROI, Region of Interest)** — 화면 전체를 보면 멀리 있는 선까지 섞여 판단이 불안정해집니다. **하단 1/3만** 사용합니다.

```python
h, w = frame.shape[:2]
roi = frame[int(h * 2 / 3):, :]        # 아래쪽 1/3
```

| 코드 | 문법 |
|------|------|
| `frame.shape` | (세로, 가로, 채널) 세 값의 튜플 — 640×480 컬러이면 `(480, 640, 3)` |
| `[:2]` | 슬라이스 — 앞 두 값만 `(480, 640)` |
| `h, w = …` | 동시 대입 — 두 값을 `h`·`w`에 나누어 담음 |
| `frame[int(h * 2 / 3):, :]` | 2차원 슬라이싱 `[행 범위, 열 범위]` — 행은 `h×2/3`부터 끝까지 · 열은 전체(`:`) |

> **자주 하는 실수 —** `h * 2 / 3`은 나눗셈 결과이므로 **실수**(320.0)입니다. 인덱스는 정수여야 하므로 `int()`로 변환하지 않으면 오류가 납니다. 실행 시 `TypeError: slice indices must be integers`가 나오면 → 슬라이싱 안의 `int()` 변환을 확인하고 **다시** 실행.

**오차 계산**:

```
error = cx - (w / 2)
```

| error | 의미 | 조치 |
|:--:|------|------|
| 0에 가까움 | 선이 중앙 | 직진 |
| **양수** | 선이 **오른쪽** | **오른쪽으로 회전** = `angular.z` **음수** |
| **음수** | 선이 왼쪽 | `angular.z` 양수 |

> **핵심 — 부호 —** **영상 좌표의 오른쪽(+)과 로봇 회전의 양수(반시계)가 반대 방향**입니다. 그래서 `angular.z = -error × 계수`로 부호를 뒤집습니다.

### 12.3 구현 힌트

```python
gain = self.get_parameter('gain').value       # 회전 민감도 — 파라미터로

twist = Twist()
if found:
    twist.linear.x = 1.0
    twist.angular.z = -error * gain           # ← 부호 반전
else:
    twist.linear.x = 0.0                      # 선을 잃으면 정지
    twist.angular.z = 0.5                     # 제자리 회전으로 탐색
self.pub.publish(twist)
```

| 항목 | 권장 |
|------|------|
| `gain` 초기값 | 0.005 정도에서 시작해 조정 |
| 선을 잃었을 때 | **정지 후 탐색** — 그대로 직진하면 이탈이 커짐 |
| 검증 | turtlesim을 실행하고 `/turtle1/cmd_vel`로 remap |

- turtlesim이 선의 반대 방향으로 회전하면 → `-error * gain`의 부호(`-`)를 확인하고 **다시** 실행

| 코드 | 뜻 |
|------|------|
| `twist = Twist()` | 빈 메시지 객체 생성 → `.linear.x`·`.angular.z` 속성에 값 대입 |
| `-error * gain` | 부호 곱 — `error` = +100(선이 오른쪽)·`gain` = 0.005이면 `angular.z` = **−0.5**(오른쪽 회전) |
| `if found:` | `found`는 참·거짓 값(불리언) — 검출 여부를 담아 두었다가 분기에 사용 |

### 12.4 단계별 과제

| 단계 | 과제 |
|:--:|------|
| **1단계** | 선의 중심 좌표를 구해 `error`를 로그로 출력 |
| **2단계** | `error`로 `Twist`를 발행해 **turtlesim이 선을 따라 움직이도록** |
| **심화** | ⓐ `gain`을 파라미터로 빼고 실행 중 조정 ⓑ 선을 잃었을 때 탐색 동작 추가 ⓒ 급커브에서 속도를 줄이도록 `linear.x`를 오차에 연동 |

- 자습 과제이므로 제출하지 않습니다 — **1단계까지만 수행해도** Day 6의 판단 노드를 이해하는 데 충분합니다

---

## 13. 오늘의 요약

| 항목 | 내용 |
|------|------|
| 규칙 ↔ 학습 | 색 검출은 규칙을 사람이 작성 / 분류는 **데이터로 학습**. "빨간 것"과 "정지 표지판"의 차이 |
| **핵심 원칙** | **코드가 아니라 데이터가 결과를 정한다** — 오분류는 모델 결함이 아니라 **데이터의 공백** |
| 클래스 설계 | 구분 대상 + **반드시 `none`(배경) 클래스** — 없으면 빈 화면도 억지로 분류됨 |
| 촬영 조건 | 거리·각도·조명·배경을 **의도적으로 나눠** 촬영. 한 자리 연속 촬영은 무의미 |
| 학습 순환 | 학습 → 검증 → **오분류 관찰 → 데이터 보강 → 재학습** (최소 1회) |
| 배포 | **TFLite** = 소형 기기 추론 전용 **형식**. `model_unquant.tflite` + `labels.txt`(순서 = 출력 순서) |
| 실행기 | **LiteRT**(`ai-edge-litert`) — 형식과 실행기는 별개. 코드에서는 `Interpreter` 하나만 씀 |
| 패키지 등록 | `setup.py`의 `data_files` — **코드 외 파일도 선언해야 빌드 결과물에 포함됨** |
| 추론 노드 | 모델 적재는 **한 번만** · 구독은 보관만 · **타이머로 주기 추론**(카메라 속도와 분리) |
| 안정화 | **연속 N회 일치**로 확정 — 흔들리는 판정을 그대로 차량에 넘기지 않음 |

---

## 14. 보충

본 과정을 마치고 시간이 남을 때 다루는 내용입니다. 각 항목은 서로 독립이며, 다음 시간의 선행 개념은 포함하지 않습니다.

### 14.1 RPi5 카메라로 학습 데이터 수집

실행할 카메라로 학습하면 정확도가 올라갑니다(렌즈 특성 일치).

**①** 저장 도구를 설치합니다 — RPi5에 기본 포함되지 않습니다.

```bash
sudo apt install -y ros-jazzy-image-view
```

**②** 프레임을 파일로 저장합니다.

```bash
# RPi5에서 — 현재 폴더에 저장됨
ros2 run image_view image_saver --ros-args -r image:=/camera/image_raw \
  -p filename_format:="stop_%04d.jpg" -p save_all_image:=true
```

- `ls *.jpg`로 파일이 보이면 → ③으로 진행
- `Package 'image_view' not found`가 나오면 → **①을 다시** 실행
- 파일이 생기지 않으면 → 터미널 ①의 카메라 노드가 실행 중인지 확인(2.4) 후 **②를 다시** 실행
- `save_all_image:=false`로 두면 **서비스를 호출할 때만 한 장씩** 저장됩니다

**③** 저장한 사진을 PC로 옮겨 Teachable Machine의 **Upload** 탭으로 올립니다.

### 14.2 신뢰도 하한을 정하는 법

| 하한 | 성질 |
|:--:|------|
| 낮음 (0.5) | 잘 반응하지만 **오판정 증가** |
| 높음 (0.9) | 확실할 때만 반응하지만 **놓치는 경우 증가** |

- 실측 방법 — 표지를 여러 조건에서 제시하며 `/sign_confidence`를 기록하고, **정답일 때의 최저값**과 **오답일 때의 최고값** 사이를 택함
- 자율차에서는 **잘못 멈추는 쪽이 안전**합니다 — 정지 표지를 못 봐서 지나치는 것보다, 없는데 멈추는 것이 낫습니다

### 14.3 클래스를 늘릴 때

| 추가 클래스 | 주의 |
|------|------|
| 좌회전·우회전 | **서로 대칭**이라 혼동이 잦음 — 데이터를 더 많이 |
| 속도 제한 | 숫자 구분은 저해상도에서 어려움 |

- 클래스가 늘면 각 클래스의 데이터도 함께 늘려야 함

### 14.4 추론 결과를 영상에 표시

```python
# __init__에 추가
self.debug_pub = self.create_publisher(Image, '/debug_view', 10)

# infer 말미에 추가
frame = self.latest.copy()                  # 원본을 건드리지 않도록 복사
cv2.putText(frame, f'{name} {conf:.2f}', (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
self.debug_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
```

- `rqt_image_view`로 `/debug_view`를 보면 **판정이 영상 위에 겹쳐** 표시됨
- 5.5의 마스크 발행과 같은 취지 — **진단 수단을 만들어 둡니다**

### 14.5 이 방식의 한계

| 한계 | 내용 |
|------|------|
| **위치를 모름** | 분류는 "무엇인가"만 답함 — **화면 어디에 있는지**는 알 수 없음 |
| 대상이 여럿일 때 | 구분 없이 하나의 판정으로 합쳐짐 |
| 학습 조건 밖 | 데이터에 없던 조건에서 급격히 나빠짐 |

- 위치까지 알려면 **객체 검출**(YOLO 등)이 필요 — 연산 부담이 훨씬 큼
- 위치가 필요하면 5장의 **색 검출과 병용**하는 방법이 현실적(색으로 위치·AI로 종류)

### 14.6 영상 저장과 재생

```bash
ros2 bag record /camera/image_raw -o run1      # 기록
ros2 bag play run1                              # 재생
```

- 이미지 토픽은 용량이 크므로 **짧게** 기록 — 30초면 수백 MB
- 재생 중에는 카메라 노드를 종료하고 진행합니다(같은 토픽이 겹침)

**카메라 없이 오늘을 진행할 때(Day 4 자료 6장 ⓑ)** — 교수가 준비한 기록 파일로 `/camera/image_raw`를 발행합니다.

| 순서 | 수행 위치 | 명령 |
|:--:|------|------|
| ① | PC → RPi5 | `run1` 폴더를 홈 폴더(`~`)로 옮김 — 9.3 ②-2(끌어다 놓기) 또는 ②-3(`scp -r run1 …:~/`) |
| ② | RPi5 | `ros2 bag play run1 --loop` — 반복 재생 |
| ③ | 다른 터미널 | `ros2 topic hz /camera/image_raw` |

- 주기가 출력되면 → **3.4로 돌아가** 이후 절을 그대로 수행합니다
- `Bag file does not exist`가 나오면 → `ls ~`로 폴더 이름을 확인하고 **②를 다시** 실행

### 14.7 다른 검출 방법

| 방법 | 원리 | 한계 |
|------|------|------|
| **색 검출**(4·5장) | HSV 범위 | 조명·같은 색 물체에 취약 |
| 윤곽선 검출 | 밝기 변화(`Canny`) | 색은 무시 — 형태만 |
| 템플릿 매칭 | 기준 이미지와 비교 | 크기·회전에 약함 |
| **AI 분류**(6~10장) | 학습된 모델 | 학습 데이터가 필요 |

- 색 검출은 **빠르고 단순** — 조건이 통제된 환경에서는 여전히 실용적

---

## 15. 다음 시간

**Day 6 — 인식·판단·행동 통합** (9/28)

`/sign_stable`을 받아 주행을 결정하는 **판단 노드**를 작성합니다. 오늘 만든 두 노드가 **인식** 계층이 되고, 그 위에 판단 계층을 얹어 turtlesim을 표지판에 반응시킵니다.

- 오늘 확정한 토픽 계약 — `/sign_stable`(`String`) · `/target_point`(`Point`)
- 준비 — 오늘 만든 `my_car_pkg`와 학습한 모델을 그대로 사용합니다

