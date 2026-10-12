# Future_drive Project (모빌리티 E)

ROS 2 Humble 기반의 **자율주행 모빌리티 통합 시스템** 프로젝트입니다.  
임베디드 하드웨어 제어 및 2D SLAM/자율주행(**EP**), AI 기반 실시간 객체·차선 인지 및 센서 퓨전 충돌 방지(**SP**), 그리고 멀티 로봇 3D 웹 관제 및 원격 제어 플랫폼(**AP**)의 3개 파트로 구성되어 있습니다.

---

## 👥 지도 강사 및 팀원

| 구분 | 이름 | 담당 파트 및 주요 역할 |
| :--- | :--- | :--- |
| **지도강사** | 박승휘 | 프로젝트 총괄 지도 |
| **반장 / 부반장** | 권동재 / 김태형 | 프로젝트 매니징 및 통합 조율 |
| **AP (Application Platform)** | 김동규, 정원혁 | FastAPI & Three.js 기반 멀티 로봇 3D 웹 뷰어 및 제어 대시보드 |
| **SP (Smart Perception)** | 정진한, 조현준, 최지호 | YOLO 객체 검출, LiDAR-카메라 거리 추정, 차선 인식 및 충돌 정지 |
| **EP (Embedded Platform)** | 신현수, 정윤상, 조용만 | 4WD 모터/엔코더 제어, LiDAR 연동, Cartographer SLAM 및 Nav2 주행 |

---

## 1. 개요 (Overview)

### 핵심 기능
* **AP (웹 관제 및 제어 플랫폼)**
  * OccupancyGrid 지도(`/map`), URDF/STL 3D 기체 모델, TF, LiDAR(`/scan`), Nav2 경로(`/plan`), 오도메트리(`/odom`), 배터리 잔량(`/battery`) 실시간 시각화
  * 웹 브라우저에서 초기 위치 지정(`/initialpose`), 목적지 자율주행 요청(`/navigate_to_pose`), 긴급 정지(`/stop`) 원격 제어
  * 로봇 3대 개별 연결 및 Robot 1·2 동시 관제(**ALL 보기**: 이중 카메라 스트림, 공통 좌표계 검증, 동시 정지)
  * 플랫폼 브리지 카메라 프레임을 웹 서버에서 실시간 YOLO 추론 후 스트리밍 제공
* **SP (AI 비전 인지 및 안전 제어)**
  * 커스텀 학습 YOLO 모델(`futuredrive_yolo26n_best.pt`) 기반 **4륜차·2륜차·보행자** 실시간 검출
  * **LiDAR + 단안 카메라 센서 퓨전**: 바운딩 박스와 LiDAR 스캔 각도 매칭 및 핀홀 카메라 모델을 결합한 정밀 거리·상대속도 추정
  * **충돌 방지 제어**: 충돌 위험 판단(`/collision_warning`) 시 Nav2 주행 명령(`/cmd_vel_nav`)을 차단하고 즉시 정지 명령(`/cmd_vel`) 발행
  * **차선 검출**: Bird's-Eye View 원근 변환, HLS/LAB 색상 필터 및 Sliding Window 다항식 피팅 기반 차선 중심·오프셋 추출(`/lane/result`)
* **EP (임베디드 하드웨어 제어 및 SLAM/Nav2)**
  * 차량 하드웨어 프로파일 지원 (`hkit4`: Arduino UNO + RPLIDAR C1 / `hkit5`: OpenCR + RPLIDAR A1)
  * 4WD 모터 시리얼 제어 및 휠 엔코더 피드백 기반 오도메트리(`/odom`, `odom → base_link` TF) 생성
  * Cartographer 기반 2D SLAM 지도 작성 및 Nav2 자율주행 스택 연동
  * SP 객체 인식 결과(`/yolo/detections`) 연동 장애물 대응 (보행자/2륜차 정지 대기, 4륜차 후진-회전-전진 회피 주행)

---

### 시스템 아키텍처

```text
┌──────────────────────────────────────────────────────────────────────────────┐
│                        AP: Web Control & 3D Viewer (app/)                    │
│  [Browser (Three.js)] ◀──(HTTP/WS 8081)──▶ [FastAPI Server + YOLO Processor] │
└───────────────────────────────────────────────────────▲──────────────────────┘
                                                        │ (Bridge 8765/tcp)
┌───────────────────────────────────────────────────────▼──────────────────────┐
│                         ROS 2 Humble Ecosystem                               │
│                                                                              │
│  ┌─────────────────────────────────┐      ┌───────────────────────────────┐  │
│  │   SP: Perception (ros2_ws/)     │      │  EP: SLAM & Nav2 (ep_part/)   │  │
│  │                                 │      │                               │  │
│  │ • camera_node / yahboom_camera  │      │ • Cartographer 2D SLAM (/map) │  │
│  │ • yolo_node / lidar_yahboom     │      │ • Nav2 Stack (/plan)          │  │
│  │   ├─ /yolo/detections           │─────▶│ • avoidance_node              │  │
│  │   └─ /collision_warning         │      │   (/cmd_vel_avoid)            │  │
│  │ • collision_stop_node           │◀─────│   (/cmd_vel_nav)              │  │
│  │ • lane_node (/lane/result)      │      └───────────────▲───────────────┘  │
│  └────────────────┬────────────────┘                      │                  │
│                   │ /cmd_vel                              │ /scan, /odom, TF │
│                   ▼                                       │                  │
│  ┌────────────────────────────────────────────────────────┴───────────────┐  │
│  │                  EP: Hardware Bringup (my_robot_bringup)               │  │
│  │ • cmd_vel_serial_bridge (Arduino UNO) / cmd_vel_bridge (OpenCR)        │  │
│  │ • rplidar_ros (RPLIDAR C1 / A1) & Static TF Publisher                  │  │
│  └────────────────────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

### 디렉터리 구조

```text
Future_drive_E/
├── app/                                  # [AP] FastAPI 기반 3D 로봇 관제 웹 애플리케이션
│   ├── app.py / main.py                  # Uvicorn 실행 진입점 및 FastAPI 앱 설정
│   ├── controllers/web_controller.py     # 화면 렌더링, 로봇 제어, 브리지 연결 및 YOLO API
│   ├── models/                           # 브리지 매니저, 뷰어 설정, 웹 YOLO 프레임 처리기
│   ├── statics/                          # Three.js 3D 뷰어, 네비게이션 제어 JS/CSS, URDF/STL 모델
│   ├── views/index.html                  # 메인 웹 대시보드 템플릿
│   ├── management/                       # 웹 서버 및 플랫폼 브리지 시작/상태/종료 쉘 스크립트
│   └── README.md                         # AP 파트 상세 매뉴얼
│
├── ros2_ws/                              # [SP] AI 객체/차선 인식 및 충돌 방지 ROS 2 워크스페이스
│   ├── models/                           # 커스텀 학습 YOLO 가중치 (futuredrive_yolo26n_best.pt 등)
│   └── src/yolo_detector/                # YOLO 객체 검출 및 센서 퓨전 패키지
│       ├── config/yaboom_camera_info.yaml# 야붐카 카메라 내부 파라미터 및 거리 추정 설정
│       ├── launch/                       # 일반/야붐카 통합 실행 런치 파일
│       ├── yolo_detector/                # 카메라, YOLO, LiDAR 퓨전, 충돌 정지, 차선 인식 노드
│       └── README.md                     # SP 파트 상세 매뉴얼
│
├── ep_part/                              # [EP] 하드웨어 제어, SLAM 및 자율주행 워크스페이스
│   ├── hardware/                         # 마이크로컨트롤러 펌웨어 및 로봇 Bringup
│   │   ├── config.ini                    # 차량 프로파일 설정 (hkit4 / hkit5)
│   │   ├── futuredrive_4wd_firmware/     # Arduino UNO 4WD 제어 펌웨어 (.ino)
│   │   ├── opnecr_firmware/              # OpenCR 4WD 모터 제어 및 엔코더 펌웨어 (.ino)
│   │   └── ros2_ws/src/my_robot_bringup/ # 모터 브리지, 회피 주행, 카메라, 통합 Bringup 패키지
│   ├── ep_ws/src/arduino_cartography/    # Cartographer 2D SLAM, Nav2 설정, URDF 및 맵 패키지
│   ├── Dockerfile / entrypoint.sh        # Raspberry Pi ROS 2 & 카메라 컨테이너 환경
│   └── README.md                         # EP 파트 상세 매뉴얼
│
├── documents/
│   └── Topic_name.xlsx                   # 파트 간 ROS 2 토픽 인터페이스 정의서
└── how_to_use_yolo_dectector.txt         # SP 파트 노드 실행 요약 가이드
```

---

## 2. 개발 환경 (Development Environment)

* **OS:** Ubuntu 22.04 LTS (PC & Raspberry Pi)
* **Middleware:** ROS 2 Humble Hawksbill (`ROS_DOMAIN_ID=42` 기본 사용, Fast DDS)
* **Language:** Python 3.10+, C/C++ (Arduino / OpenCR Firmware), JavaScript (ES6+)
* **Web & Visualization:** FastAPI, Uvicorn, Three.js (`0.170.0`), roslibjs (`1.4.1`), RViz2
* **AI & Computer Vision:** PyTorch, Ultralytics YOLOv8, OpenCV
* **SLAM & Navigation:** Google Cartographer ROS, Navigation2 (Nav2)
* **Hardware:**
  * **hkit1:** Yahboom ROS2 로봇 플랫폼
  * **hkit2:** Yahboom ROS2 로봇 플랫폼
  * **hkit3:** Yahboom ROS2 로봇 플랫폼
  * **hkit4:** Raspberry Pi, Arduino UNO (`/dev/ttyACM0`, 9600bps), RPLIDAR C1 (`/dev/ttyUSB0`, 460800bps)
  * **hkit5:** Raspberry Pi, OpenCR (`/dev/ttyACM0`, 115200bps), RPLIDAR A1 (`/dev/ttyUSB0`, 115200bps)

---

## 3. 사전 설치 (Pre-installed)

### 공통 ROS 2 환경 설정
```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
```

### AP Part (웹 관제 서버)
```bash
sudo apt update
sudo apt install -y python3-venv

cd ~/Future_drive_E
python3 -m venv app/.venv
app/.venv/bin/python -m pip install -r app/requirements.txt
# requirements.txt: fastapi, uvicorn, numpy==1.26.4, ultralytics==8.4.164
```

### SP Part (AI 객체/차선 인식)
```bash
python3 -m pip install \
  numpy==1.26.4 \
  ultralytics==8.4.164 \
  opencv-python==4.10.0.84 \
  torch==2.14.0 \
  torchvision==0.29.0
```

### EP Part (하드웨어 Bringup & SLAM/Nav2)
```bash
sudo apt update
sudo apt install -y \
  ros-humble-cartographer \
  ros-humble-cartographer-ros \
  ros-humble-rviz2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-map-server \
  ros-humble-vision-msgs \
  ros-humble-usb-cam \
  ros-humble-compressed-image-transport \
  ros-humble-rplidar-ros \
  python3-serial
```

---

## 4. 파트별 README.md

각 파트의 세부 파라미터와 문제 해결 방법은 파트별 상세 문서를 참고하세요.
* 📘 **AP 상세 문서:** [`app/README.md`](./app/README.md) / [`app/management/README.md`](./app/management/README.md)
* 📗 **SP 상세 문서:** [`ros2_ws/src/yolo_detector/README.md`](./ros2_ws/src/yolo_detector/README.md)
* 📙 **EP 상세 문서:** [`ep_part/README.md`](./ep_part/README.md)

---

## 5. 주요 ROS 2 토픽 요약

| 파트 | 토픽명 | 메시지 타입 | 설명 |
| :---: | :--- | :--- | :--- |
| **EP** | `/scan` | `sensor_msgs/msg/LaserScan` | RPLIDAR 2D 스캔 데이터 |
| **EP** | `/odom` | `nav_msgs/msg/Odometry` | 휠 엔코더 기반 오도메트리 |
| **EP** | `/map` | `nav_msgs/msg/OccupancyGrid` | Cartographer 생성 2D 격자 지도 |
| **EP/SP** | `/cmd_vel` | `geometry_msgs/msg/Twist` | 최종 차량 모터 제어 속도 명령 |
| **EP** | `/cmd_vel_avoid` | `geometry_msgs/msg/Twist` | 장애물 회피 노드 속도 명령 |
| **SP** | `/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 차량 카메라 압축 영상 입력 |
| **SP** | `/yolo/detections` | `vision_msgs/msg/Detection2DArray` | 객체 검출 및 추정 거리(`position.z`) 결과 |
| **SP** | `/yolo/detections_json` | `std_msgs/msg/String` | 객체 검출 및 거리/속도 JSON 데이터 |
| **SP** | `/yolo/result_image` | `sensor_msgs/msg/Image` | 바운딩 박스가 시각화된 결과 영상 |
| **SP** | `/collision_warning` | `std_msgs/msg/String` | 충돌 위험 여부(`danger`), 거리, 상대속도 JSON |
| **SP** | `/lane/result` | `std_msgs/msg/Float32MultiArray` | 차선 검출 유효성, 중심 좌표, 오프셋 및 차선 좌표 |
| **AP** | `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 웹 뷰어 초기 위치 설정 |
| **AP** | `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | 웹 뷰어 목적지 자율주행 액션 |
| **AP** | `/battery` | `std_msgs/msg/UInt16` | 배터리 전압 데이터 (웹 잔량 추정용) |

---

## 6. Git Commit Message Prefix Rule

* `[feature]` : 기능 추가 및 개선
* `[bugfix]` : 오류 해결
* `[ignore]` : 기능 동작에 영향 없는 문서 파일 수정 등
* `[test]` : 기능 테스트를 위한 commit
