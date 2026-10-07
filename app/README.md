# 야붐카 ROS 웹 뷰어

ROS 2 Humble 기반 야붐카의 지도, 위치, 경로, 라이다, TF, 카메라와 상태 정보를 표시하는 FastAPI 웹 애플리케이션입니다. 웹에서 로봇별 Autonomy Studio 브리지 주소를 설정해 지도·라이다·위치·경로와 제어 기능을 직접 연결합니다.

## 주요 기능

- OccupancyGrid 지도와 Grid
- TF 및 로봇 위치·방향
- URDF·STL 기반 기체 모델
- LaserScan, Nav2 계획 경로, 압축 카메라, Odometry
- 배터리 전압 기반 추정 잔량
- 지도에서 초기 위치와 목적지 선택
- Nav2 자율주행 요청과 정지
- 로봇 3대의 Autonomy Studio 브리지 개별 연결
- Robot 1·2 동시 연결과 별도 URDF/TF 표시를 제공하는 ALL 보기
- ALL 보기의 이중 카메라와 두 로봇 동시 정지

## 시스템 구성

Autonomy Studio가 제공하는 상태·제어 브리지를 웹에서 직접 사용합니다.

- **Autonomy Studio PC**: 지도·라이다·위치·경로를 `8765/ws/state`로 제공하고 초기 위치·목적지·정지 HTTP API를 제공합니다.
- **웹 서버 PC**: FastAPI 웹 화면을 `8081/tcp`로 제공합니다.
- **사용자 브라우저**: 로봇별 플랫폼 브리지 주소에 연결합니다. 별도 rosbridge 실행과 ROS Domain ID 입력은 필요하지 않습니다.

## 1. 로봇 데이터 준비

Autonomy Studio에서 로봇을 연결하고 Map, Localization, Planning, Vehicle, Camera 모듈을 실행합니다. 시각화 창은 열지 않아도 됩니다. 그다음 같은 PC에서 프로젝트의 독립 브리지를 실행합니다.

```bash
cd ~/Future_drive_E
bash app/management/start-platform-bridge.sh
```

스크립트는 Autonomy Studio가 해당 로봇 연결 때 생성한 최신 Fast DDS peer 프로필을 사용합니다. 최초 프로필 생성과 ROS 모듈 실행까지는 Autonomy Studio가 필요합니다.

브리지 상태는 다음 명령으로 확인합니다.

```bash
curl http://127.0.0.1:8765/health
```

`has_map`, `has_scan`, `has_pose`가 `true`이면 해당 데이터를 웹에서 받을 수 있습니다. 다른 PC의 브라우저가 연결하려면 플랫폼 PC의 `8765/tcp`에 접근할 수 있어야 합니다.

## 2. 웹 서버 PC 설치

웹 관리자는 웹 서버 PC에서 다음 패키지와 Python 가상환경을 준비합니다.

```bash
sudo apt update
sudo apt install -y python3-venv

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
- 같은 LAN의 팀원 PC에서 접속: `http://<웹-서버-PC-IP>:8081/`
- 오프라인 데모: `http://<웹-서버-PC-IP>:8081/?demo=1`


> 현재 웹 애플리케이션에는 사용자 인증이나 관리자/일반 사용자 권한 구분이 없습니다. 위 두 주소는 권한이 다른 주소가 아니라 접속 위치만 다릅니다.

## 웹에서 로봇 연결

1. Autonomy Studio에서 로봇을 연결하고 필요한 모듈을 실행합니다.
2. 해당 PC에서 `start-platform-bridge.sh`를 실행합니다.
3. 웹 상단에서 연결할 로봇을 선택합니다.
4. **연결 설정**에서 로봇 모듈을 실행 중인 PC의 브리지 주소를 입력합니다.
5. **저장 및 연결**을 누릅니다.

```text
http://<브리지-PC-IP>:8765
```

같은 PC에서 Autonomy Studio와 브라우저를 실행한다면 `http://127.0.0.1:8765`도 사용할 수 있습니다. 중앙 웹에서 여러 로봇을 동시에 보려면 각 로봇을 실행하는 플랫폼 PC의 LAN 주소를 Robot 1·2·3에 각각 저장합니다.

웹은 `/ws/state`에서 지도·라이다·위치·경로를 받고, `/initialpose`, `/navigate_to_pose`, `/cancel_navigation`, `/stop`으로 제어 요청을 보냅니다. `ALL` 보기를 사용하려면 Robot 1과 Robot 2의 브리지 주소를 먼저 저장해야 합니다.

## 주요 파일과 실행 로그

- `app/app.py`: Uvicorn 실행 진입점
- `app/main.py`: FastAPI 앱
- `app/controllers/web_controller.py`: 화면·제어·연결 관리 API
- `app/statics/js/app.js`: ROS 구독·3D 화면·연결 UI
- `app/statics/js/navigation.js`: 초기 위치·주행·정지
- `app/management/start-platform-bridge.sh`: 독립 상태·제어 브리지 실행
- `app/management/status-platform-bridge.sh`: 브리지 상태 확인
- `app/management/stop-platform-bridge.sh`: 브리지 종료
- `app/management/start-web.sh`: FastAPI 웹 서버 실행

## 플랫폼에서 사용하는 기본 ROS 토픽

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

초기 위치는 플랫폼 브리지의 `/initialpose`, 목적지는 `/navigate_to_pose`, 정지는 `/cancel_navigation`과 `/stop` API로 전송합니다.

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
