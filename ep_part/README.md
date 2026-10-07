# Future Drive E — EP

ROS 2 기반 차량 구동, 엔코더 오도메트리, LiDAR 연동 및 Cartographer 2D SLAM 패키지입니다.

> **Status:** Development  
> **ROS:** Humble  
> **Default Domain ID:** `42`

## 담당 기능

- Arduino UNO 기반 4WD 모터 제어
- OpenCR 기반 4WD 모터 제어
- 휠 엔코더 기반 오도메트리 생성
- ROS 2 `/cmd_vel` 차량 제어
- RPLIDAR C1 및 RPLIDAR A1 연동
- Cartographer 기반 2D 지도 생성
- RViz 지도 시각화
- 지도 및 Cartographer 상태 저장

## EP 팀원

- 신현수
- 정윤상
- 조용만

---

## 시스템 구조

```text
/cmd_vel
    │
    ▼
Motor Bridge ──▶ Arduino UNO / OpenCR ──▶ 4WD Motors
    ▲                                        │
    └──────────── Encoder Feedback ◀─────────┘
                      │
                      ├──▶ /odom
                      └──▶ odom → base_link

RPLIDAR C1 / A1 ──▶ /scan
                       │
                       ▼
                  Cartographer
                       │
                       ├──▶ /map
                       └──▶ map → odom
```

---

## 차량 프로파일

| 항목 | hkit4 | hkit5 |
|---|---|---|
| 모터 제어 보드 | Arduino UNO | OpenCR |
| 모터 장치 | `/dev/ttyACM0` | `/dev/ttyACM0` |
| 모터 통신속도 | `9600` | `115200` |
| 모터 브리지 | `cmd_vel_serial_bridge` | `cmd_vel_bridge` |
| LiDAR | RPLIDAR C1 | RPLIDAR A1 |
| LiDAR 장치 | `/dev/ttyUSB0` | `/dev/ttyUSB0` |
| LiDAR 통신속도 | `460800` | `115200` |
| LiDAR Frame | `laser_frame` | `lidar_frame` |

프로파일 설정 파일:

```text
hardware/config.ini
```

```ini
[robot]
profile = hkit5
```

지원하는 값은 `hkit4`, `hkit5`입니다. 프로파일을 변경한 뒤에는 차량 bringup을 다시 실행해야 합니다.

---

## 프로젝트 구조

```text
ep_part/
├── README.md
├── entrypoint.sh
├── hardware/
│   ├── config.ini
│   ├── futuredrive_4wd_firmware/
│   │   └── futuredrive_4wd.ino
│   ├── opnecr_firmware/
│   │   └── opencr_motor.ino
│   └── ros2_ws/src/my_robot_bringup/
│       ├── launch/ep.launch.py
│       └── my_robot_bringup/
│           ├── cmd_vel_serial_bridge.py
│           ├── cmd_vel_bridge.py
│           ├── avoidance_node.py
│           └── camera_node.py
└── ep_ws/src/arduino_cartography/
    ├── launch/futuredrive_hkit.launch.py
    ├── params/futuredrive_hkit.lua
    ├── package.xml
    └── setup.py
```

다음 디렉터리는 ROS 2 빌드 결과이므로 Git에 포함하지 않습니다.

```text
build/
install/
log/
```

---

# 빠른 시작

## 1. 공통 ROS 2 환경 설정 — Ubuntu PC

모든 ROS 2 터미널에서 실행합니다.

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
```

차량과 Ubuntu PC는 반드시 같은 Domain ID를 사용해야 합니다.

## 2. 장치 확인 — Raspberry Pi

```bash
lsusb
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

예상 장치:

```text
/dev/ttyUSB0   # LiDAR
/dev/ttyACM0   # Arduino UNO 또는 OpenCR
```

## 3. 차량 Bringup 실행 — Raspberry Pi

`hardware/config.ini`에서 차량 프로파일을 선택한 뒤 차량 bringup을 실행합니다.

차량 launch 파일:

```text
hardware/ros2_ws/src/my_robot_bringup/launch/ep.launch.py
```

프로파일별 실행 노드:

| hkit4 | hkit5 |
|---|---|
| `rplidar_node` | `rplidar_node` |
| `base_link_to_laser_frame` | `base_link_to_lidar_frame` |
| `cmd_vel_serial_bridge` | `cmd_vel_bridge` |
|  | `avoidance_node` |

## 4. 차량 토픽 확인 — Ubuntu PC

```bash
ros2 topic list | grep -E '^/(cmd_vel|odom|scan|tf|tf_static)$'
```

예상 토픽:

```text
/cmd_vel
/odom
/scan
/tf
/tf_static
```

## 5. 센서 데이터 확인 — Ubuntu PC

LiDAR:

```bash
timeout 5s ros2 topic hz /scan
ros2 topic echo /scan --once
```

정상적인 경우 `/scan`이 약 9~10Hz로 발행되고 `ranges`에 거리값이 출력됩니다.

오도메트리:

```bash
ros2 topic echo /odom --once
```

## 6. Cartographer 실행 — Ubuntu PC

```bash
cd ~/Future_drive_E/ep_part/ep_ws

source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch arduino_cartography futuredrive_hkit.launch.py
```

정상적으로 시작되면 다음 메시지가 출력됩니다.

```text
Added trajectory with ID '0'
Inserted submap (0, 0)
```

---

# ROS 2 구성

## 주요 토픽

| 토픽 | 타입 | 설명 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 차량 속도 명령 |
| `/odom` | `nav_msgs/msg/Odometry` | 엔코더 오도메트리 |
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR 스캔 |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Cartographer 지도 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 동적 TF |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 정적 TF |
| `/arduino/command` | `std_msgs/msg/String` | hkit4 Arduino 명령 |
| `/arduino/feedback` | `std_msgs/msg/String` | hkit4 Arduino 피드백 |
| `/yolo/detections` | `vision_msgs/msg/Detection2DArray` | 객체 감지 결과 |
| `/cmd_vel_avoid` | `geometry_msgs/msg/Twist` | 회피 속도 명령 |

## TF 구조

### hkit4

```text
map → odom → base_link → laser_frame
```

### hkit5

```text
map → odom → base_link → lidar_frame
```

| TF | 담당 |
|---|---|
| `map → odom` | Cartographer |
| `odom → base_link` | 엔코더 오도메트리 브리지 |
| `base_link → laser_frame` | hkit4 Static TF Publisher |
| `base_link → lidar_frame` | hkit5 Static TF Publisher |

`map → base_link`는 정적 TF로 고정하지 않습니다.

---

# 빌드

## 필수 패키지 — Ubuntu PC

```bash
sudo apt update

sudo apt install -y \
  ros-humble-cartographer \
  ros-humble-cartographer-ros \
  ros-humble-rviz2 \
  ros-humble-nav2-map-server \
  ros-humble-vision-msgs \
  python3-serial
```

## 차량 패키지

```bash
cd ~/Future_drive_E/ep_part/hardware/ros2_ws

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

확인:

```bash
ros2 pkg list | grep my_robot_bringup
```

## Cartographer 패키지

```bash
cd ~/Future_drive_E/ep_part/ep_ws

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

확인:

```bash
ros2 pkg list | grep arduino_cartography
```

---

# 차량 제어

## hkit4 — Arduino UNO

펌웨어:

```text
hardware/futuredrive_4wd_firmware/futuredrive_4wd.ino
```

ROS 2 브리지:

```text
hardware/ros2_ws/src/my_robot_bringup/my_robot_bringup/cmd_vel_serial_bridge.py
```

| 동작 | 명령 |
|---|---|
| 전진 | `W` |
| 후진 | `S` |
| 좌회전 | `A` |
| 우회전 | `D` |
| 정지 | `X` |

Arduino IDE Serial Monitor와 ROS 2 브리지는 `/dev/ttyACM0`을 동시에 사용할 수 없습니다.

차량 bringup을 실행하기 전에 Serial Monitor를 닫아야 합니다.

## hkit5 — OpenCR

펌웨어:

```text
hardware/opnecr_firmware/opencr_motor.ino
```

ROS 2 브리지:

```text
hardware/ros2_ws/src/my_robot_bringup/my_robot_bringup/cmd_vel_bridge.py
```

OpenCR 명령 형식:

```text
V,<LEFT_SPEED>,<RIGHT_SPEED>
```

정지 명령:

```text
V,0,0
```

OpenCR에서 전달한 엔코더 값을 이용해 `/odom`과 `odom → base_link` TF를 생성합니다.

---

# LiDAR

## hkit4 — RPLIDAR C1

| 항목 | 값 |
|---|---|
| 장치 | `/dev/ttyUSB0` |
| 통신속도 | `460800` |
| Frame ID | `laser_frame` |
| 측정 주기 | 약 `10 Hz` |
| 하드웨어 최대 거리 | `16.0 m` |

C1에 `115200`을 사용하면 다음 오류가 발생할 수 있습니다.

```text
SL_RESULT_OPERATION_TIMEOUT
```

## hkit5 — RPLIDAR A1

| 항목 | 값 |
|---|---|
| 장치 | `/dev/ttyUSB0` |
| 통신속도 | `115200` |
| Frame ID | `lidar_frame` |
| 측정 주기 | 약 `10 Hz` |
| 하드웨어 최대 거리 | 약 `12.0 m` |

---

# Cartographer

## 실행 노드

`futuredrive_hkit.launch.py`는 다음 노드를 실행합니다.

- `cartographer_node`
- `cartographer_occupancy_grid_node`
- `rviz2`

## 설정 파일

```text
ep_ws/src/arduino_cartography/params/futuredrive_hkit.lua
```

현재 주요 설정:

```lua
map_frame = "map"
tracking_frame = "base_link"
published_frame = "odom"
odom_frame = "odom"

provide_odom_frame = false
use_odometry = false
use_imu_data = false

TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 12.0
```

현재 설정 파일의 최대 사용 거리는 `12.0m`입니다.

설정을 변경한 뒤에는 패키지를 다시 빌드합니다.

```bash
cd ~/Future_drive_E/ep_part/ep_ws

colcon build --symlink-install
source install/setup.bash
```

---

# RViz

| 항목 | 설정값 |
|---|---|
| Fixed Frame | `map` |
| Map Topic | `/map` |
| LaserScan Topic | `/scan` |
| View Type | `TopDownOrtho` |

그래픽 드라이버 문제가 발생하면 소프트웨어 렌더링으로 실행합니다.

```bash
LIBGL_ALWAYS_SOFTWARE=1 rviz2
```

---

# 상태 확인

## TF

공통:

```bash
timeout 5s ros2 run tf2_ros tf2_echo map odom
timeout 5s ros2 run tf2_ros tf2_echo odom base_link
```

hkit4:

```bash
timeout 5s ros2 run tf2_ros tf2_echo base_link laser_frame
```

hkit5:

```bash
timeout 5s ros2 run tf2_ros tf2_echo base_link lidar_frame
```

처음 한 번 `frame does not exist`가 출력된 뒤 변환값이 반복 출력되면 정상입니다.

## 오도메트리

위치:

```bash
ros2 topic echo /odom --once --field pose.pose.position
```

회전:

```bash
ros2 topic echo /odom --once --field pose.pose.orientation
```

차량을 손으로 이동하면 실제 위치와 오도메트리가 달라질 수 있습니다. 오도메트리 시험은 모터를 구동하여 진행합니다.

---

# 지도 작성

지도 품질을 높이기 위해 다음 사항을 지킵니다.

1. 차량을 천천히 이동합니다.
2. 급출발과 급정지를 피합니다.
3. 회전할 때 바퀴 미끄러짐을 최소화합니다.
4. 벽과 장애물이 LiDAR 높이에 들어오도록 합니다.
5. 지나간 장소를 다시 통과해 Loop Closure를 유도합니다.
6. 지도 작성 중 차량을 손으로 옮기지 않습니다.
7. 한 장소에서 빠르게 반복 회전하지 않습니다.

---

# 지도 저장

저장 디렉터리:

```bash
MAP_DIR="$HOME/Future_drive_E/maps"
mkdir -p "$MAP_DIR"
```

## Occupancy Grid

```bash
ros2 run nav2_map_server map_saver_cli \
  -f "$MAP_DIR/futuredrive_map"
```

생성 파일:

```text
futuredrive_map.yaml
futuredrive_map.pgm
```

## Cartographer 상태

```bash
ros2 service call /write_state \
  cartographer_ros_msgs/srv/WriteState \
  "{filename: '${MAP_DIR}/futuredrive_map.pbstream', include_unfinished_submaps: true}"
```

생성 파일:

```text
futuredrive_map.pbstream
```

---

# 문제 해결

## `/scan`이 보이지 않는 경우

```bash
lsusb
ls -l /dev/ttyUSB* 2>/dev/null
```

통신속도:

```text
hkit4 / RPLIDAR C1: 460800
hkit5 / RPLIDAR A1: 115200
```

## `/odom`이 보이지 않는 경우

```bash
ls -l /dev/ttyACM* 2>/dev/null
```

hkit4에서는 Arduino IDE Serial Monitor가 닫혀 있는지 확인합니다.

## Cartographer 중복 실행

```bash
ros2 node list | grep cartographer
```

다른 터미널이나 PC에서 Cartographer가 실행되고 있는지 확인합니다.

ROS 그래프 정보가 남아 있다면 daemon을 다시 시작합니다.

```bash
ros2 daemon stop
ros2 daemon start
```

## RViz 그래픽 오류

다음 메시지는 그래픽 드라이버 관련 오류입니다.

```text
active samplers with a different type refer to the same texture image unit
```

RViz 화면이 정상적으로 표시된다면 지도 생성 자체의 오류는 아닐 수 있습니다.

```bash
LIBGL_ALWAYS_SOFTWARE=1 rviz2
```

---

# 종료 순서

각 프로그램은 실행 중인 터미널에서 `Ctrl+C`로 종료합니다.

1. 차량 제어 프로그램 종료
2. 차량 정지 확인
3. RViz와 Cartographer 종료
4. 차량 bringup 종료
5. Raspberry Pi 정상 종료
6. 차량 전원 분리

Raspberry Pi는 파일시스템 손상을 방지하기 위해 정상 종료 후 전원을 분리합니다.

---

# 개발 참고사항

다음 항목은 차량 통합 과정에서 추가 확인이 필요합니다.

- 차량별 LiDAR 설치 위치에 맞는 Static TF 보정
- 차량별 휠 크기 및 휠 간격 보정
- `/cmd_vel_avoid`와 최종 `/cmd_vel` 연결 방식
- Docker 이미지에 차량 ROS 2 워크스페이스를 포함하는 방법
- `entrypoint.sh` 설정 파일 경로 일반화
- Cartographer의 엔코더 오도메트리 사용 여부
- `package.xml` 런타임 의존성 보완