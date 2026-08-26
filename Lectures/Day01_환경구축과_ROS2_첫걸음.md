# Day 1 — 환경 구축과 ROS2 첫걸음

**2026-08-24 · 한국폴리텍대학교 하이테크과정 ROS2**

이 자료는 수업에서 진행한 실습 절차와 명령어를 정리한 것입니다. 복습 기준 = 이 자료 + 수업 중 필기.

---

## 목차

1. [과정 안내](#1-과정-안내)
2. [오늘의 핵심 개념](#2-오늘의-핵심-개념)
3. [환경 구축](#3-환경-구축)
4. [Linux 기본 명령어](#4-linux-기본-명령어)
5. [ROS2 명령어 체계](#5-ros2-명령어-체계)
6. [turtlesim 실습](#6-turtlesim-실습)
7. [미니프로젝트 — CLI로 turtle 제어](#7-미니프로젝트--cli로-turtle-제어)
8. [다음 시간](#8-다음-시간)

---

## 1. 과정 안내

### 1.1 최종 목표

**개인별 자율차 1대 완성·시연** (Day 12)

- 플랫폼 — Raspberry Pi 5 + ROS2
- 주행 — 초음파센서 기반 장애물 회피
- 인식 — 카메라 + AI 표지판 인식·반응
- 언어 — Python

진행 원칙 — **시뮬레이션(turtlesim) 선행 검증 → 실물 전환**. 시뮬레이션에서 동작한 코드 구조가 실물에서 그대로 동작하는 경험이 이 과정의 핵심입니다.

### 1.2 12일 로드맵

| Day | 날짜 | 내용 |
|:--:|:--:|------|
| 1 | 8/24 | 환경 구축·ROS2 첫걸음 |
| 2 | 8/31 | turtlesim 종합 제어 — 토픽·서비스·액션 + rclpy 첫 코드 |
| 3 | 9/7 | 패키지·colcon — 멀티 노드 미로 자율주행 |
| 4 | 9/14 | 카메라·영상 처리 — 색상과 라인 인식 |
| 5 | 9/21 | AI 이미지 분류 — 표지판 인식 |
| 6 | 9/28 | 시뮬레이션 통합 — 표지판에 반응하는 turtle |
| — | 10/5 | 휴강 (개천절 대체공휴일) |
| 7 | 10/12 | SLAM 실습 — Gazebo에서 지도 작성과 목표 주행 |
| 8 | 10/19 | 실물 전환 — 초음파센서·DC모터 |
| 9 | 10/26 | 실물 통합 — 장애물 회피·표지판 반응 주행 |
| 10 | 11/2 | 최종 프로젝트 구현 ① |
| 11 | 11/9 | 최종 프로젝트 구현 ②·시연 리허설 |
| 12 | 11/16 | 최종 프로젝트 완성·시연 |

### 1.3 수업 방식과 평가

- 하루 구성 — **이론 강의 → 실습 → 미니프로젝트** (미니프로젝트 산출물 = 다음 수업의 재료)
- 개발 환경 — **각자 직접 구축** (환경 구축 능력 = 이 과정의 학습 내용)
- 평가 — **평소 수업 참여·수행 태도** 기준 (별도 제출물·지필시험 없음)

### 1.4 준비물

**Day 1~3 — 강의실 PC**

| 품목 | 확인 사항 |
|------|----------|
| 강의실 Windows PC | WSL 기설치 — 배포판·버전만 확인 |
| Ubuntu 배포판 | **Ubuntu 24.04** 필요 |
| Windows 버전 | Windows 11 권장(WSLg 내장) · Windows 10이면 X 서버 별도 설치 |

**Day 4부터 — Raspberry Pi 5**

| 품목 | 확인 사항 |
|------|----------|
| Raspberry Pi 5 | 전원 어댑터(5V/5A 권장)·방열 대책 |
| microSD 카드 | 32GB 이상·리더기 |
| 카메라 모듈 | **CSI**(Camera Serial Interface) 방식 — RPi 전용이라 PC에는 연결되지 않음 |
| 초음파센서 HC-SR04 · DC모터 · 모터 드라이버(L298N) | Day 8 |

> **RPi5 환경 구축은 Day 4 전 과제입니다.** 오늘 진행한 절차의 반복이므로 각자 수행합니다.

---

## 2. 오늘의 핵심 개념

### 2.1 ROS와 ROS2

- **ROS**(Robot Operating System) — 이름과 달리 운영체제가 아니라 Linux 위에서 동작하는 **로봇 소프트웨어 플랫폼**
- 통신 체계·개발 도구·패키지 생태계를 제공해, 개발자는 응용(서비스) 구현에 집중

| 구분 | ROS 1 | ROS 2 |
|------|------|------|
| 설계 전제 | 연구실 환경 | 산업·상용 환경 |
| 통신 계층 | 자체 개발 TCPROS — TCP(Transmission Control Protocol) 기반 · 마스터 구조 | **DDS** 기반 · OMG(Object Management Group — 국제 표준화 단체) 표준 |
| 강화 요소 | — | 실시간성·보안·다중 로봇 |
| 현재 위상 | 지원 종료 (Noetic, 2025-05) | 신규 개발 표준 |

- 이 과정의 사용 판 = **Jazzy Jalisco** — 2024-05 출시 · LTS(Long Term Support — 장기 지원판)로 2029년까지 지원 · **Ubuntu 24.04 대응**
- ROS2 LTS는 Ubuntu LTS와 연동해 출시됩니다 — Foxy↔20.04 · Humble↔22.04 · Jazzy↔24.04
- 릴리스 목록·지원 기간: https://docs.ros.org/en/rolling/Releases.html

### 2.2 노드와 토픽

**노드(Node)** = ROS2 프로그램을 구성하는 독립 실행 단위. 실행 중인 하나의 프로그램이며 각자 고유한 이름을 가집니다. **기능 하나당 노드 하나**로 나누는 것이 ROS 설계 방식입니다.

오늘 실행한 turtlesim의 구성:

| 노드 | 기능 | 통신 |
|------|------|------|
| `turtlesim_node` | turtle 화면 표시·이동 | `/turtle1/cmd_vel` 구독 · `/turtle1/pose` 발행 |
| `turtle_teleop_key` | 키 입력을 속도 명령으로 변환 | `/turtle1/cmd_vel` 발행 |

**토픽(Topic)** = 노드 사이의 기본 통신 통로(이름 있는 메시지 채널).

- **발행**(publish) — 노드가 토픽에 메시지를 내보내는 것
- **구독**(subscribe) — 노드가 토픽의 메시지를 받는 것
- 발행자와 구독자는 서로를 알지 못하며, **토픽 이름 + 메시지 타입만 일치하면 자동 연결**
- 성격 = **비동기·단방향** — 발행자는 응답을 기다리지 않고, 구독자가 없어도 발행 자체는 성립
- 연결 형태 = 1:N · N:1 · N:N 모두 가능. 한 노드가 발행자와 구독자를 겸할 수도 있음
- **ROS2 통신의 70% 이상이 토픽**입니다

속도 명령의 표준 타입 `geometry_msgs/msg/Twist`:

```
geometry_msgs/msg/Twist
├── linear  — x·y·z 직진 속도 [m/s]
└── angular — x·y·z 회전 속도 [rad/s]
```

- 평면 주행 로봇은 **linear.x**(전진·후진)와 **angular.z**(회전) 두 값만 사용하고 나머지는 0

토픽 외에 **서비스**(1회성 요청-응답)와 **액션**(장시간 작업·목표/피드백/결과)이 있습니다 — Day 2에서 다룹니다.

### 2.3 표준 단위와 좌표계

메시지는 이름과 자료형만 정의할 뿐 **단위는 담지 않습니다**. 단위가 어긋나면 오동작·사고로 이어지므로, ROS에는 개발 초기부터 표준 단위가 규정되어 있습니다.

| 물리량 | 표준 단위 |
|------|------|
| 길이 | m |
| 각도 | rad |
| 병진 속도 | m/s |
| 회전 속도 | rad/s |
| 시간 | s |

- `angular.z = 1.57`은 **초당 1.57라디안(약 90°)**이지 90이 아닙니다

| 대상 | 기본 좌표계 |
|------|------|
| 로봇 | **x 전방 · y 좌측 · z 상방** |
| 카메라 (컴퓨터 비전) | **z 전방 · x 우측 · y 하방** |

> **자주 하는 실수 —** 로봇과 카메라의 축 방향이 다릅니다. Day 4에서 영상 좌표를 주행 명령으로 변환할 때 이 차이를 감안하지 않으면 **회전 방향이 반대로 나옵니다.**

- 회전의 부호 = **오른손 법칙**. 제자리 좌회전(반시계) = `angular.z` 양수
- 시각화 도구의 축 색 = **x 빨강 · y 초록 · z 파랑** (RViz·Gazebo 공통)

### 2.4 메시지 기반 처리

노드들은 함수 호출이나 메모리 공유가 아니라 **정해진 형식의 메시지 교환**으로 협력합니다. 노드 사이의 약속은 **토픽 이름 + 메시지 타입** 두 가지뿐이며 내부 구현은 서로 무관합니다.

| 장점 | 내용 |
|------|------|
| 느슨한 결합 | 발행자·구독자가 서로를 알지 못해도 동작 — **시뮬레이션에서 실물로 옮길 수 있는 근거** |
| 언어 독립 | 메시지 형식만 일치하면 Python·C++ 노드 혼용 가능 |
| 분산 처리 | 여러 기기에 노드를 나누어 배치 — 네트워크 너머 노드와도 동일 방식 통신 |
| 관찰·기록 용이 | 흐르는 메시지를 CLI(Command Line Interface)로 관찰(`echo`)·기록/재생(bag) |
| 부분 장애 격리 | 한 노드가 정지해도 전체가 정지하지 않음 |

| 한계 | 보완 |
|------|------|
| 통신 오버헤드 — 고속 제어 루프에 불리 | IPC(Intra-Process Communication — 프로세스 내 통신) |
| 실시간성 보장 곤란 — 도착 시점이 비결정적 | DDS·QoS(Quality of Service — 통신 품질) 설정 |
| 실행 흐름 추적 복잡 | rqt_graph·로그·bag |

### 2.5 DDS와 Domain ID

- **DDS**(Data Distribution Service) — OMG 표준 출판/구독 통신 미들웨어. ROS2의 모든 토픽 통신이 이 위에서 동작하며, 마스터 없이 노드가 서로를 자동 발견합니다
- 전송 방식 = **UDP(User Datagram Protocol)/IP 기반의 신뢰성 있는 멀티캐스트**. 같은 도메인의 토픽들이 **DDS Global Space**라는 공통 공간에 놓입니다
- **`ROS_DOMAIN_ID`** = 이 Global Space를 번호로 분리하는 설정. 같은 번호끼리만 통신합니다

> **자주 하는 실수 —** 강의실 여러 대가 같은 네트워크를 사용합니다. Domain ID를 설정하지 않으면 다른 사람의 turtle이 함께 움직이는 혼선이 발생합니다. 반드시 각자 번호를 설정하세요.

### 2.6 개발 도구

| 도구 | 역할 | 이 과정에서 |
|------|------|------|
| CLI | 명령어로 노드·토픽 관찰·제어 | 오늘부터 매일 사용 |
| rqt 계열 | GUI(Graphical User Interface) 도구 모음 — rqt_graph·rqt_plot·rqt_bag | rqt_graph 오늘 사용 |
| RViz | 3D 시각화 — 센서 데이터·로봇 모델 표시 | Day 7 실습 |
| Gazebo | 물리엔진 3D 시뮬레이터 | Day 7 실습 |
| SLAM(Simultaneous Localization and Mapping) · Nav2 | 지도 작성 + 위치 추정 / 자율 내비게이션 | Day 7 실습 |

---

## 3. 환경 구축

### 3.1 환경 구조

| 구간 | 환경 | 내용 |
|:--:|------|------|
| **Day 1~3** | **강의실 PC + WSL2** | turtlesim·rclpy·패키지 — 시뮬레이션 기초 |
| **Day 4~12** | **Raspberry Pi 5** | 카메라(CSI)·AI·실물 차량 (Day 7 SLAM 실습만 PC) |

두 환경 모두 Ubuntu 24.04 + ROS2 Jazzy로 **동일**하므로 코드를 그대로 옮겨 실행할 수 있습니다.

**WSL**(Windows Subsystem for Linux) — Windows 위에서 Linux 실행 파일을 그대로 구동하는 호환 계층입니다. 가상머신과 달리 별도 OS 전체를 띄우지 않아 부담이 적고, **WSL2는 실제 Linux 커널을 탑재**해 대부분의 명령이 그대로 동작합니다.

### 3.2 WSL 상태 확인

Windows 터미널(또는 PowerShell)에서:

```powershell
wsl -l -v
```

출력 예시:

```
  NAME              STATE           VERSION
* Ubuntu-24.04      Stopped         2
```

| 열 | 확인할 것 |
|------|------|
| NAME | 배포판 이름 — 이 과정은 **Ubuntu 24.04** 필요 |
| STATE | 실행 상태 — `Stopped`여도 정상 |
| VERSION | **반드시 `2`** |

확인 결과에 따른 조치:

| 상태 | 조치 |
|------|------|
| Ubuntu-24.04 · VERSION 2 | 그대로 사용 |
| 목록에 Ubuntu-24.04가 없음 | `wsl --install -d Ubuntu-24.04` 실행 후 사용자 이름·암호 설정 |
| VERSION이 `1` | `wsl --set-version Ubuntu-24.04 2` |
| 다른 버전(22.04 등)만 있음 | 24.04를 추가 설치 |

> **주의 —** **ROS2 Jazzy는 Ubuntu 24.04 전용**입니다. 22.04만 있다면 Jazzy를 설치할 수 없으므로 24.04를 추가로 설치합니다. 배포판은 여러 개를 동시에 둘 수 있습니다.

**Ubuntu 24.04 설치 — 관리자 권한 PowerShell**

목록에 24.04가 없거나 다른 버전만 있을 때의 절차입니다.

```powershell
# 1) 시작 메뉴 → PowerShell 우클릭 → "관리자 권한으로 실행"
wsl --list --online                  # 설치 가능한 배포판 목록 — 이름의 정확한 표기 확인
wsl --install -d Ubuntu-24.04        # 설치 (-d = 배포판 지정)
```

- **WSL 자체가 처음 설치되는 경우 재부팅**이 필요합니다. 재부팅 후 Ubuntu가 자동 실행되며 사용자 이름·암호를 설정합니다
- 이미 WSL을 쓰고 있었다면 재부팅 없이 배포판만 추가됩니다

관리 명령:

| 명령 | 하는 일 |
|------|------|
| `wsl --status` | WSL 버전·기본 배포판·커널 버전 확인 |
| `wsl --update` | WSL 커널 갱신 — GUI 표시 문제의 1차 조치(3.3) |
| `wsl --set-default Ubuntu-24.04` | 기본 배포판 지정 — 이후 `wsl` 입력만으로 진입 |
| `wsl --set-version Ubuntu-24.04 2` | WSL2로 전환 |
| `wsl --shutdown` | 실행 중인 배포판 전체 종료 — 설정 변경 후 재시작에 사용 |
| `wsl --unregister Ubuntu-24.04` | **배포판 삭제 — 안의 파일이 전부 사라집니다.** 처음부터 다시 설치할 때만 |

> **자주 하는 실수**
>
> - **일반 권한 PowerShell에서는 설치가 실패합니다.** 반드시 관리자 권한으로 실행합니다
> - **BIOS에서 가상화가 꺼져 있으면** WSL2가 동작하지 않습니다(항목명은 제조사마다 Virtualization·SVM·VT-x 등)
> - **`wsl --install`을 단독으로 실행하면 기본 배포판이 설치**되어 24.04가 아닐 수 있습니다. `-d Ubuntu-24.04`로 명시합니다
> - 암호 설정 시 **입력해도 화면에 아무것도 표시되지 않습니다** — 정상 동작이므로 그대로 입력하고 Enter

Ubuntu를 실행한 뒤 패키지 목록을 갱신합니다:

```bash
sudo apt update && sudo apt upgrade -y
```

- WSL 설치 공식 안내: https://learn.microsoft.com/ko-kr/windows/wsl/install

### 3.3 GUI 프로그램 표시 확인

이 과정의 실습 대부분(`turtlesim`·rqt·RViz)은 **창을 띄우는 프로그램**입니다. Windows 11은 WSLg가 기본 내장되어 별도 설정 없이 창이 표시됩니다.

```bash
sudo apt install x11-apps -y
xeyes                     # 창이 뜨면 GUI 표시 정상
```

**첫 GUI 실행 — 경고 메시지와 지연**

> **첫 실행은 느리고, 경고가 함께 출력되는 것이 정상입니다.** WSLg가 그래픽 스택을 처음 초기화하므로 창이 뜨기까지 수 초에서 수십 초가 걸릴 수 있습니다. **경고가 나와도 창이 표시되면 정상**이므로 기다립니다.

- 무해한 경고의 예 — Qt 플랫폼·Wayland 관련 안내, 공유 메모리(MIT-SHM) 관련, 소프트웨어 렌더링(libGL) 관련
- 두 번째 실행부터는 초기화가 끝나 있어 빠르게 표시됩니다

**창이 끝내 뜨지 않을 때** — 위에서부터 순서대로 확인합니다.

| # | 확인·조치 | 명령 |
|:--:|------|------|
| 1 | 표시 대상이 지정되어 있는가 | `echo $DISPLAY` — **비어 있으면 WSLg가 동작하지 않는 상태** |
| 2 | **WSL 갱신 후 재시작** (가장 효과적) | 관리자 PowerShell에서 `wsl --update` → `wsl --shutdown` → Ubuntu 재실행 |
| 3 | GUI 자체가 되는가 | `xeyes` — 이것도 표시되지 않으면 개별 프로그램의 문제가 아님 |
| 4 | Windows 버전 | Windows 10이면 **X 서버(VcXsrv 등) 별도 설치** 필요 |
| 5 | 그래픽 드라이버 문제로 의심되면 | `export LIBGL_ALWAYS_SOFTWARE=1` 후 재실행 — 소프트웨어 렌더링으로 우회 |
| 6 | 그래도 표시되지 않으면 | `export QT_QPA_PLATFORM=xcb` 후 재실행 — Qt 표시 방식을 지정 |

- 5·6의 `export`는 **그 터미널에서만 유효**합니다. 계속 필요하면 `~/.bashrc`에 등록합니다(3.5 방식)

### 3.4 ROS2 Jazzy 설치

설치는 **공식 설치 문서를 직접 열어 그 가이드를 따라** 진행합니다:

- **https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html**
- 경로 — docs.ros.org → Jazzy → Installation → Ubuntu (deb packages)

> 학습 내용은 설치 명령 암기가 아니라 **공식 문서를 찾아 절차를 따라가는 능력**입니다. 다음 판이 나와도 문서 주소의 판 이름만 바뀌므로, 새 버전 설치를 스스로 수행할 수 있습니다.

단계 개요 — 실제 명령은 공식 문서에서 확인합니다.

| 단계 | 하는 일 | 비고 |
|:--:|------|------|
| 1 | 로케일 확인 — UTF-8 설정 | 공식 문서 Set locale 절 |
| 2 | apt 저장소 등록 — ROS2 패키지 출처를 시스템에 추가 | 판마다 변동이 잦은 구간 — 웹페이지의 최신 명령 사용 |
| 3 | ROS2 설치 — 패키지 변형 선택 | 이 과정 = desktop 변형 |
| 4 | 환경 설정 — `setup.bash`·Domain ID | 3.5 참조 |

```bash
sudo apt install ros-jazzy-desktop   # desktop 변형 — 데모·rqt·시각화 도구 포함
```

- `ros-jazzy-base` = GUI 도구 없는 최소판 — 이 과정에서는 사용하지 않습니다

### 3.5 환경 설정 — `.bashrc`와 Domain ID

새 터미널마다 ROS2 환경을 읽도록 `~/.bashrc`에 등록합니다:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=<개인 번호>" >> ~/.bashrc
source ~/.bashrc
```

| 항목 | 의미 |
|------|------|
| `setup.bash` | ROS2 명령·패키지를 셸에서 사용하게 하는 환경 스크립트 |
| `ROS_DOMAIN_ID` | 같은 네트워크의 ROS2 기기 구분 번호 (2.5 참조) |
| `~/.bashrc` | 사용자별 bash 설정 파일 — 터미널을 열 때마다 자동 실행됨 |
| `source` | 파일의 내용을 현재 셸에 즉시 반영 (`.` 명령과 동일) |

### 3.6 설치 확인

```bash
ros2 doctor        # 설치 상태·환경 진단
ros2 topic list    # 토픽 목록 조회
```

- 확인 기준 — `ros2` 명령이 인식되고 기본 토픽(`/parameter_events`·`/rosout`)이 표시되는 것

설치가 되었더라도 **두 노드가 실제로 통신되는지**는 별개입니다. 표준 데모로 확인합니다:

```bash
# 터미널 1
ros2 run demo_nodes_cpp talker

# 터미널 2
ros2 run demo_nodes_py listener
```

- talker가 문자열을 발행하고 listener가 수신·출력하면 DDS 통신까지 정상입니다
- 두 노드의 언어가 다릅니다(C++ ↔ Python) — 메시지 기반의 언어 독립을 여기서 확인할 수 있습니다
- 수신이 없으면 방화벽 또는 `ROS_DOMAIN_ID` 불일치를 먼저 점검합니다

> **Tip —** 오늘은 터미널을 3개까지 사용합니다. `sudo apt install terminator`로 설치하면 창 하나를 분할해 쓸 수 있어 화면 전환이 줄어듭니다(선택).

---

## 4. Linux 기본 명령어

**탐색·조회**

| 명령 | 기능 | 주요 옵션·예 |
|------|------|------|
| `ls` | 현재 디렉토리 목록 조회 | `-a` 숨김 포함 / `-l` 상세 / `-h` 크기 단위 / `-t` 시간순 |
| `cd` | 디렉토리 이동 | `cd ..` 상위 / `cd ~` 홈 |
| `pwd` | 현재 위치를 절대경로로 출력 | — |
| `cat` / `more` | 파일 내용 표시 / 분할 표시 | `cat 파일명` |
| `man` | 명령어 설명서 | `man ls` |
| `find` | 파일 찾기 | `find 경로 옵션` |

**파일 조작**

| 명령 | 기능 | 주요 옵션·예 |
|------|------|------|
| `cp` | 파일 복사 | `-r` 폴더 포함 / `-f` 강제 덮어쓰기 |
| `mv` | 파일 이동·이름 변경 | 폴더째 이동 — `-r` 불필요 |
| `rm` | 파일·디렉토리 삭제 — **주의** | `-r` 디렉토리 포함 / `-f` 강제 |
| `tar` | 폴더 압축 | `tar -cvzf 파일명.tar.gz 경로/` |
| `wget` | URL에서 파일 내려받기 | `wget [옵션] [URL]` |
| `clear` | 화면 내용 지우기 | — |

**시스템·권한**

| 명령 | 기능 | 주요 옵션·예 |
|------|------|------|
| `ps` | 실행 중인 프로세스 조회 | `ps -ef` 전체 상세 |
| `kill` | 프로세스 종료 | 종료 후 `ps`로 확인 |
| `sudo` | 관리자 권한으로 실행 | 최초 실행 시 비밀번호 입력 |
| `chmod` | 파일 권한 변경 | `chmod 744 파일` |
| `apt` | 패키지 관리 | `apt update` 목록 갱신 / `apt install` 설치 |

---

## 5. ROS2 명령어 체계

모든 ROS2 명령은 같은 구조입니다 — **`ros2 <대상> <동작> [인자] [옵션]`**

| 위치 | 역할 | 예 |
|------|------|------|
| `ros2` | ROS2 CLI 진입점 | — |
| 대상 | 다루는 리소스 종류 | `node`·`topic`·`service`·`action`·`interface`·`pkg`·`run` |
| 동작 | 대상에 수행할 하위 명령 | `list`·`info`·`echo`·`pub`·`call` |
| 인자·옵션 | 구체 대상·데이터·부가 설정 | `/turtle1/cmd_vel` · `"{...}"` · `--rate 1` |

분해 예:

- `ros2 node list` = ros2(CLI) + node(노드 대상) + list(목록 출력)
- `ros2 topic echo /turtle1/cmd_vel` = topic(토픽 대상) + echo(메시지 관찰) + 토픽 이름(인자)

계층마다 `--help`를 지원합니다:

```bash
ros2 --help              # 사용 가능한 대상 전체 목록
ros2 topic --help        # topic 대상의 동작 목록
ros2 topic pub --help    # pub 동작의 사용법·옵션
```

**Tab 자동완성** — `--help`보다 빠른 확인 경로:

```bash
ros2 <Tab><Tab>          # 사용 가능한 대상 전체가 나열됨
ros2 topic <Tab><Tab>    # topic 대상의 동작 목록
```

- 대상 전체 19종 — `action`·`bag`·`component`·`daemon`·`doctor`·`extension_points`·`extensions`·`interface`·`launch`·`lifecycle`·`multicast`·`node`·`param`·`pkg`·`run`·`security`·`service`·`topic`·`wtf`
- 이 과정에서 사용하는 것 = **run·node·topic·service·action·param·interface·pkg·launch·bag·doctor 11종**

---

## 6. turtlesim 실습

### 6.1 실행

터미널 2개를 사용합니다:

```bash
# 터미널 1 — 시뮬레이터
ros2 run turtlesim turtlesim_node

# 터미널 2 — 키보드 조작
ros2 run turtlesim turtle_teleop_key
```

- `ros2 run <패키지> <노드>` — 패키지에 등록된 노드 하나를 실행하는 명령
- 화살표 키로 turtle 이동을 확인합니다
- 동작 구조 — teleop(teleoperation — 원격 조작) 노드가 키 입력을 속도 메시지로 변환해 `/turtle1/cmd_vel`에 발행 → turtlesim 노드가 구독해 turtle 이동

> **창이 즉시 표시되지 않는 경우 —** `turtlesim_node`는 GUI 프로그램이라 **첫 실행에서 경고 메시지가 출력되고 창이 늦게 뜨는 일이 흔합니다.** 경고가 있어도 창이 표시되면 정상입니다. 끝내 표시되지 않으면 **3.3의 확인 순서 6단계**를 따릅니다(`echo $DISPLAY` → `wsl --update`·`wsl --shutdown` 재시작이 1차 조치).

### 6.2 노드 관찰

세 번째 터미널에서 실행합니다:

```bash
ros2 node list              # 실행 중인 노드 목록
ros2 node info /turtlesim   # 노드가 가진 토픽·서비스·액션 목록
```

- `node info` — 이 노드가 발행/구독하는 토픽과 그 외 통신 창구를 한눈에 확인
- 오늘은 이 중 **토픽**까지만 관찰합니다. 서비스·액션은 Day 2에서 다룹니다

### 6.3 토픽 관찰

```bash
ros2 topic list -t                  # 토픽 목록 조회 (-t = 메시지 타입 병기)
ros2 topic echo /turtle1/cmd_vel    # 토픽에 흐르는 메시지 실시간 관찰
ros2 topic info /turtle1/cmd_vel    # 토픽 정보 — 타입·발행/구독 수
ros2 interface show geometry_msgs/msg/Twist    # 타입의 내부 구조 출력
ros2 interface show turtlesim/msg/Pose         # 위치 메시지 구조
```

토픽의 **흐름 상태**를 측정하는 두 명령 — 값이 아니라 통신 자체를 관찰합니다:

```bash
ros2 topic hz /turtle1/pose      # 발행 주기 측정 (Hz — 초당 발행 횟수)
ros2 topic bw /turtle1/pose      # 대역폭 측정 (초당 전송량)
```

| 명령 | 확인하는 것 | 쓰는 상황 |
|------|------|------|
| `topic echo` | 메시지 **내용** | 값이 맞는지 |
| `topic hz` | 발행 **주기** | 센서가 규정 주기로 동작하는지 · 노드 지연 여부 |
| `topic bw` | **전송량** | 영상처럼 큰 데이터가 대역폭을 점유하는지 |

- `topic hz`로 turtlesim의 pose가 약 60Hz로 발행됨을 확인할 수 있습니다 — 화면 갱신 주기와 대응합니다
- Day 4 카메라·Day 8 초음파에서 재사용합니다 — 센서가 예상 주기로 동작하는지 확인하는 첫 진단 수단입니다

메시지 발행 명령:

```bash
ros2 topic pub <토픽> <타입> "<데이터>"
```

- 옵션 `--once` = 1회 발행 / `--rate N` = 초당 N회 반복
- Day 8 실물 차량에서도 같은 `Twist` 타입을 사용합니다

### 6.4 rqt_graph

```bash
rqt_graph
```

- 노드·토픽 연결 관계를 그래프로 확인합니다
- 오늘의 2노드 그래프가 최종 프로젝트에서 카메라·판단·센서·구동 노드 그래프로 확장됩니다

---

## 7. 미니프로젝트 — CLI로 turtle 제어

teleop 없이 `ros2 topic pub` 명령만으로 수행합니다.

1. turtle **직진**
2. turtle **제자리 회전**
3. 직진 + 회전 조합으로 **원 궤적 주행** (힌트: 발행 주기 옵션 `--rate`)
4. (도전) 속도 값을 바꿔 가며 **원 반지름 변화** 실험 — 반지름을 결정하는 요인 찾기
5. (도전) 직진과 90° 회전을 순서대로 실행해 **정사각형 궤적** 그리기

<details>
<summary><b>정답 및 풀이 보기</b></summary>

```bash
# (1) 직진 — linear.x만 지정
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}}"

# (2) 제자리 회전 — angular.z만 지정
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 1.57}}"

# (3) 원 궤적 — 직진+회전 주기 발행
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 1.0}}"

# (4) 도전 — linear.x 또는 angular.z 값을 바꿔 반복 실행·궤적 비교
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 1.0}}"

# (5) 도전 — 직진 → 90° 회전을 4회 반복 (두 명령을 번갈아 실행)
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}}"
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 1.57}}"
```

- 직진·회전 속도가 동시에 주어지면 원 궤적이 형성됩니다 — **반지름 = linear.x ÷ angular.z** (문제 4의 답: 2.0/1.0 → 반지름 2, 1.0/1.0 → 반지름 1)
- (5) `--once` 발행은 약 1초간 적용됩니다. `angular.z = 1.57`(rad/s)이면 약 90° 회전입니다. 오차가 누적되어 완전한 정사각형이 되지 않는 점도 관찰 대상입니다 — Day 2에서 코드로 개선합니다

</details>

---

## 8. 다음 시간

**Day 2 — turtlesim 종합 제어 + rclpy 첫 코드**

- 서비스·액션 개념과 CLI 제어 (teleport·spawn·kill·회전 목표)
- 패키지 없이 `.py` 스크립트로 첫 rclpy 노드 실행
- 미니프로젝트 — 도형 궤적 자동 주행 스크립트

오늘 CLI로 수행한 제어를 Python 코드로 옮기는 것이 다음 시간의 목표입니다.
