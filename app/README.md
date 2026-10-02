# 야붐카 ROS 웹 뷰어

ROS 2 Humble 기반 야붐카의 지도, 위치, 경로, 라이다, TF, 카메라와 상태 정보를 표시하는 FastAPI 웹 애플리케이션입니다. 브라우저는 `roslibjs`와 Three.js를 사용하며 PC의 rosbridge에 연결합니다.

## 주요 기능

- OccupancyGrid 지도와 Grid
- TF 및 로봇 위치·방향
- URDF·STL 기반 기체 모델
- LaserScan 라이다 점군
- Nav2 계획 경로
- 압축 카메라 영상
- Odometry 속도
- 배터리 추정 잔량
- 지도에서 초기 위치와 목적지 선택
- Nav2 자율주행 요청과 정지
- 로봇 3대의 개별 연결 설정

## 폴더 구조

```text
app/
├── app.py                         # Uvicorn 실행 진입점
├── main.py                        # FastAPI 앱·정적 파일 마운트
├── controllers/
│   └── web_controller.py          # 화면·설정·제어 API
├── models/
│   └── viewer_model.py            # 기본 연결 설정
├── views/
│   └── index.html                 # 웹 화면
├── statics/
│   ├── css/style.css
│   ├── js/
│   │   ├── app.js                 # ROS 구독·3D 화면·UI
│   │   ├── navigation.js          # 초기 위치·주행·정지
│   │   ├── model.js               # URDF/STL 로더
│   │   └── demo.js                # 오프라인 데모
│   ├── robot_models/              # 야붐카 URDF·STL
│   └── vendor/                    # Three.js·roslibjs
├── management/
│   ├── start-web.sh               # FastAPI 실행
│   ├── start-bridge.sh            # rosbridge 실행
│   ├── ros-env.sh                 # ROS Domain·DDS 환경
│   └── fastdds-pc.xml             # PC용 Fast DDS 설정
└── requirements.txt
```

## 요구 환경

- Ubuntu와 ROS 2 Humble
- Python 3.10 이상
- `ros-humble-rosbridge-server`
- Autonomy Studio의 `nav2-ros-bridge` 컨테이너

rosbridge가 없다면 설치합니다.

```bash
sudo apt-get install -y ros-humble-rosbridge-server
```

## Python 환경 설치

```bash
cd /home/hkit/Desktop/www/rviz-web-prototype/app
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

`.venv`, `log`, `__pycache__`, `*.pyc`는 Git에 포함하지 않습니다.

## 실행

터미널 1에서 rosbridge를 실행합니다.

```bash
bash /home/hkit/Desktop/www/rviz-web-prototype/app/management/start-bridge.sh
```

터미널 2에서 FastAPI를 실행합니다.

```bash
bash /home/hkit/Desktop/www/rviz-web-prototype/app/management/start-web.sh
```

접속 주소:

- 실제 ROS 연결: <http://127.0.0.1:8081/>
- 오프라인 데모: <http://127.0.0.1:8081/?demo=1>
- FastAPI 상태: <http://127.0.0.1:8081/api/health>

`8081: address already in use`가 나오면 웹 서버가 이미 실행 중인 것입니다. 기존 웹이 열리면 다시 실행하지 않아도 됩니다.

## 기본 연결 설정

| 항목 | 기본값 |
|---|---|
| rosbridge | `ws://localhost:9090` |
| 플랫폼 제어 API | `/api/control` |
| 기준 좌표계 | `map` |
| 기체 좌표계 | `base_footprint` |
| 지도 | `/map` |
| 라이다 | `/scan` |
| 경로 | `/plan` |
| 속도 | `/odom` |
| TF | `/tf`, `/tf_static` |
| 카메라 | `/image_raw/compressed` |
| 기체 모델 | `/robot_description` |
| 위치 추정 | `/amcl_pose` |
| 초기 위치 | `/initialpose` |
| Nav2 액션 | `/navigate_to_pose` |
| 정지 속도 | `/cmd_vel` |
| 배터리 | `/battery` |

각 로봇의 설정은 브라우저 `localStorage`에 저장됩니다. 로봇마다 연결 주소나 네임스페이스가 다르면 연결 설정에서 변경해야 합니다.

## 초기 위치 설정

1. 상단 **초기 위치**를 누릅니다.
2. 지도에서 실제 로봇 위치를 누른 채 로봇 전방으로 드래그합니다.
3. 주황색 위치와 방향을 확인합니다.
4. **초기 위치 적용**을 누릅니다.

브라우저는 `/api/control/initialpose`를 호출합니다. FastAPI Controller가 요청을 Autonomy Studio의 `http://127.0.0.1:8765/initialpose`로 전달하고, 플랫폼 브리지가 AMCL에 `/initialpose`를 발행합니다. 따라서 Autonomy Studio의 Map·Localization 모듈과 `nav2-ros-bridge`가 실행 중이어야 합니다.

적용 여부는 `/amcl_pose`와 `map → odom → base_footprint` TF로 확인합니다. 초기 위치 설정은 로봇 이동 명령이 아닙니다.

## 목적지와 정지

- **목적지**를 누르고 지도에서 위치를 선택합니다. 드래그하면 도착 방향을 정할 수 있습니다.
- **자율주행**을 누르면 `/navigate_to_pose`로 목표를 전달합니다.
- **정지**는 Nav2 목표 취소와 `/cmd_vel` 속도 0 전송을 수행합니다.

정지는 웹 수준의 주행 취소이며 하드웨어 비상정지가 아닙니다.

## 배터리 추정 잔량

`/battery`의 `std_msgs/msg/UInt16` 값을 사용합니다.

```text
전압 = data / 10
추정 잔량 = round(clamp(10 + (전압 - 6.5) × 60, 0, 100))
```

- 6.5 → 10%
- 7.4 → 64%
- 8.0 이상 → 100%

화면에는 전압을 표시하지 않고 추정 퍼센트만 표시합니다. 실제 SOC 측정값이 아닌 선형 추정치입니다.

## DDS 설정

현재 기본값:

- ROS Domain: `77`
- RMW: `rmw_fastrtps_cpp`
- PC Tailscale 주소: `100.64.0.16`
- 로봇 DDS 주소: `100.64.0.21`

네트워크나 로봇이 바뀌면 `management/ros-env.sh`와 `management/fastdds-pc.xml`을 함께 수정해야 합니다.

## 문제 확인

rosbridge 포트:

```bash
ss -ltn | grep 9090
```

플랫폼 제어 API:

```bash
curl http://127.0.0.1:8765/health
```

AMCL과 TF:

```bash
ros2 topic info /initialpose -v
ros2 topic info /amcl_pose -v
ros2 run tf2_ros tf2_echo map base_footprint
```

웹 화면은 `file:///.../index.html`로 직접 열지 않습니다. 반드시 <http://127.0.0.1:8081/>로 접속합니다.

## 포함 라이브러리

- Three.js 0.170.0 — MIT
- roslibjs 1.4.1 — BSD-2-Clause
