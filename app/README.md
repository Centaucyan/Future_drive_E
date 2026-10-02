# 야붐카 ROS 웹 뷰어

ROS 2 Humble 기반 야붐카의 지도, 위치, 경로, 라이다, TF, 카메라와 상태 정보를 표시하는 FastAPI 웹 애플리케이션입니다. 웹에서 ROS Domain ID를 입력하면 FastAPI가 로봇별 rosbridge를 자동으로 실행합니다.

## 주요 기능

- OccupancyGrid 지도와 Grid
- TF 및 로봇 위치·방향
- URDF·STL 기반 기체 모델
- LaserScan, Nav2 계획 경로, 압축 카메라, Odometry
- 배터리 전압 기반 추정 잔량
- 지도에서 초기 위치와 목적지 선택
- Nav2 자율주행 요청과 정지
- 로봇 3대의 개별 Domain 설정

## 요구 환경과 설치

Ubuntu, ROS 2 Humble, Python 3.10 이상이 필요합니다.

```bash
sudo apt-get install -y ros-humble-rosbridge-server python3-venv
cd ~/Future_drive_E
python3 -m venv app/.venv
app/.venv/bin/python -m pip install -r app/requirements.txt
```

## 실행과 팀원 접속

```bash
cd ~/Future_drive_E
bash app/management/start-web.sh
```

웹 서버는 기본적으로 `0.0.0.0:8081`에서 실행됩니다.

- 서버 PC: `http://127.0.0.1:8081/`
- 같은 LAN의 팀원: `http://<서버 PC LAN IP>:8081/`
- 같은 Tailnet의 팀원: `http://<서버 PC Tailscale IP>:8081/`
- 오프라인 데모: `http://<서버 PC IP>:8081/?demo=1`

일반적인 사용에서는 `start-bridge.sh`를 별도로 실행하지 않습니다. FastAPI가 연결 요청을 받으면 rosbridge를 자동으로 준비합니다.

## 웹에서 로봇 연결

1. Autonomy Studio 등에서 로봇의 Map, Localization, Planning, Vehicle 모듈을 실행합니다.
2. 웹 상단에서 로봇을 선택합니다.
3. **연결 설정**에서 로봇의 ROS Domain ID를 입력합니다.
4. **저장 및 연결**을 누릅니다.

서버는 로봇 설정별로 9100번부터 WebSocket 포트를 할당합니다. 브라우저는 웹 서버 주소와 할당된 포트를 조합해 자동 접속하므로 팀원이 rosbridge 주소를 직접 입력할 필요가 없습니다.

로봇 IP와 SSH 정보는 입력하지 않습니다. 같은 LAN의 ROS 2 DDS 멀티캐스트에서 Domain ID가 일치하는 로봇을 찾습니다.

이 방식은 웹 서버 PC와 로봇이 같은 LAN에서 DDS 멀티캐스트를 사용할 수 있어야 합니다. Tailscale처럼 멀티캐스트가 전달되지 않는 환경에는 별도의 DDS Discovery Server 또는 피어 설정이 필요합니다.

## 주요 파일과 실행 로그

- `app/app.py`: Uvicorn 실행 진입점
- `app/main.py`: FastAPI 앱
- `app/controllers/web_controller.py`: 화면·제어·연결 관리 API
- `app/models/bridge_manager.py`: 로봇별 rosbridge와 DDS 설정 관리
- `app/statics/js/app.js`: ROS 구독·3D 화면·연결 UI
- `app/statics/js/navigation.js`: 초기 위치·주행·정지
- `app/runtime/`: 실행 중 생성되는 DDS 설정과 로그(Git 제외)

자동 연결이 실패하면 `app/runtime/rosbridge-<로봇>.log`를 확인합니다.

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
