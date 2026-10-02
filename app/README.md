# 야붐카 ROS 웹 뷰어

ROS 2 Humble 기반 야붐카의 지도, 위치, 경로, 라이다, TF, 카메라와 상태 정보를 표시하는 FastAPI 웹 애플리케이션입니다. 웹에서 로봇별 rosbridge WebSocket 주소를 설정해 연결합니다.

## 주요 기능

- OccupancyGrid 지도와 Grid
- TF 및 로봇 위치·방향
- URDF·STL 기반 기체 모델
- LaserScan, Nav2 계획 경로, 압축 카메라, Odometry
- 배터리 전압 기반 추정 잔량
- 지도에서 초기 위치와 목적지 선택
- Nav2 자율주행 요청과 정지
- 로봇 3대의 개별 rosbridge 연결 설정
- Robot 1·2 동시 연결과 별도 URDF/TF 표시를 제공하는 ALL 보기
- ALL 보기의 이중 카메라와 두 로봇 동시 정지

## 시스템 구성

이 시스템은 로봇 PC와 웹 서버 PC를 분리해서 실행합니다.

- **로봇 PC**: ROS 2 토픽을 rosbridge WebSocket(`9090/tcp`)으로 제공합니다.
- **웹 서버 PC**: FastAPI 웹 화면을 `8081/tcp`로 제공합니다.
- **사용자 브라우저**: 웹 화면에 접속한 뒤 각 로봇 PC의 rosbridge 주소로 직접 연결합니다.

따라서 웹 서버 PC뿐 아니라 웹 화면을 사용하는 팀원 PC에서도 각 로봇 PC의 `9090/tcp`에 접근할 수 있어야 합니다.

## 1. 로봇 PC 설정

ROS 2 Humble이 설치된 각 로봇 PC에서 rosbridge를 설치합니다.

```bash
sudo apt update
sudo apt install -y ros-humble-rosbridge-server
```

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix rosbridge_server
```

로봇의 ROS 모듈을 실행한 뒤 rosbridge WebSocket을 시작합니다.

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  address:=0.0.0.0 \
  port:=9090
```

로봇 PC에서 UFW를 사용한다면 같은 LAN의 팀원 PC가 접속할 수 있도록 포트를 허용합니다.

```bash
sudo ufw allow 9090/tcp
```

`source /opt/ros/humble/setup.bash`와 rosbridge 실행은 새 터미널을 열거나 로봇 PC를 재시작한 뒤 다시 수행해야 합니다. `address:=0.0.0.0`은 rosbridge를 네트워크에 공개하므로 신뢰할 수 있는 내부망에서만 사용하세요.

## 2. 웹 서버 PC 설치

웹 관리자는 웹 서버 PC에서 다음 패키지와 Python 가상환경을 준비합니다.

```bash
sudo apt update
sudo apt install -y python3-venv ros-humble-rosbridge-server

cd ~/Future_drive_E
python3 -m venv app/.venv
app/.venv/bin/python -m pip install -r app/requirements.txt
```

## 3. 웹 서버 실행과 접속

```bash
cd ~/Future_drive_E
bash app/management/start-web.sh
```

웹 서버의 LAN IP는 다음 명령으로 확인합니다.

```bash
hostname -I
```

웹 서버 PC에서 UFW를 사용한다면 웹 포트를 허용합니다.

```bash
sudo ufw allow 8081/tcp
```

웹 서버는 기본적으로 `0.0.0.0:8081`에서 실행됩니다.

- 웹 서버 PC 자체에서 접속: `http://127.0.0.1:8081/`
- 같은 LAN의 팀원 PC에서 접속: `http://192.168.0.14:8081/`
- 오프라인 데모: `http://192.168.0.14:8081/?demo=1`


> 현재 웹 애플리케이션에는 사용자 인증이나 관리자/일반 사용자 권한 구분이 없습니다. 위 두 주소는 권한이 다른 주소가 아니라 접속 위치만 다릅니다.

## 웹에서 로봇 연결

1. Autonomy Studio 등에서 로봇의 Map, Localization, Planning, Vehicle 모듈을 실행합니다.
2. 해당 로봇 PC에서 rosbridge가 `0.0.0.0:9090`으로 실행 중인지 확인합니다.
3. 웹 상단에서 연결할 로봇을 선택합니다.
4. **연결 설정**에서 해당 로봇 PC의 주소(`ws://<로봇 PC LAN IP>:9090`)를 입력합니다.
5. **저장 및 연결**을 누릅니다.

예를 들어 로봇 PC의 LAN IP가 `192.168.0.20`이면 `ws://192.168.0.20:9090`을 입력합니다. `ALL` 보기를 사용하려면 먼저 Robot 1과 Robot 2에 각각 올바른 rosbridge 주소를 저장해야 합니다.

각 로봇의 rosbridge 주소는 브라우저 로컬 저장소에 개별 저장됩니다. HTTPS로 웹을 제공하는 경우에는 브라우저 보안 정책에 맞춰 `wss://` 주소가 필요합니다.

## 주요 파일과 실행 로그

- `app/app.py`: Uvicorn 실행 진입점
- `app/main.py`: FastAPI 앱
- `app/controllers/web_controller.py`: 화면·제어·연결 관리 API
- `app/statics/js/app.js`: ROS 구독·3D 화면·연결 UI
- `app/statics/js/navigation.js`: 초기 위치·주행·정지
- `app/management/start-web.sh`: FastAPI 웹 서버 실행
- `app/management/start-bridge.sh`: rosbridge 실행 보조 스크립트

## 기본 ROS 토픽

| 기능 | 토픽 |
|---|---|
| 지도 | `/map` |
| 라이다 | `/scan` |
| 경로 | `/plan` |
| 속도·위치 | `/odom` |
| TF | `/tf`, `/tf_static` |
| 카메라 | `/image_raw/compressed` |
| 기체 모델 | `/robot_description` |
| 위치 추정 | `/amcl_pose` |
| 초기 위치 | `/initialpose` |
| Nav2 액션 | `/navigate_to_pose` |
| 정지 속도 | `/cmd_vel` |
| 배터리 | `/battery` |

초기 위치는 선택한 rosbridge를 통해 `/initialpose`로 전송합니다. 목적지는 `/navigate_to_pose`, 정지는 Nav2 목표 취소와 `/cmd_vel` 속도 0으로 전송합니다.

ALL 보기는 Robot 1과 Robot 2의 기존 연결 상태를 재사용합니다. 단독 보기에서 주행 중인 로봇을 ALL로 전환해도 해당 WebSocket과 Navigation 객체를 닫지 않으며, 다른 로봇 연결만 추가합니다. ALL에서는 새로운 초기 위치·목적지·주행 명령을 비활성화하고 정지 버튼만 두 로봇에 전달합니다. 두 `/map` 메시지의 frame ID, 해상도, 크기, 원점을 비교해 공통 좌표계 여부도 화면에 표시합니다.

## 배터리 추정 잔량

`/battery` 값을 전압의 10배 값으로 해석합니다.

```text
전압 = data / 10
추정 잔량 = round(clamp(10 + (전압 - 6.5) × 60, 0, 100))
```

- 6.5 → 10%
- 7.4 → 64%
- 8.0 이상 → 100%

## 포함 라이브러리

- Three.js 0.170.0 — MIT
- roslibjs 1.4.1 — BSD-2-Clause
