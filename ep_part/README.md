# Future Drive E — EP

ROS 2 기반 차량 구동, 엔코더 오도메트리, LiDAR 연동, Cartographer 2D SLAM 및 Nav2 자율주행 프로젝트입니다.

> **Status:** Development  
> **ROS:** ROS 2 Humble  
> **ROS_DOMAIN_ID:** hkit4는 `40`, hkit5는 `50`

Ubuntu PC와 Raspberry Pi의 차량 컨테이너는 같은 차량의 `ROS_DOMAIN_ID`를 사용해야 합니다.

## 담당 기능

- Arduino UNO 및 OpenCR 기반 4WD 모터 제어
- ROS 2 `/cmd_vel` 기반 차량 속도 제어
- 휠 엔코더 기반 오도메트리 생성
- `odom → base_link` TF 발행
- RPLIDAR C1 및 RPLIDAR A1 연동
- Cartographer 기반 2D 지도 생성
- OccupancyGrid 지도 저장
- 저장 지도 기반 AMCL 위치 추정
- Nav2 기반 자율주행
- RViz2 지도 및 로봇 상태 시각화
- Raspberry Pi 카메라 영상 발행

## EP 팀원

- 신현수
- 정윤상
- 조용만

---

## 시스템 구조

```text
Ubuntu PC
├── Cartographer 2D SLAM
│   ├── 입력: /scan, TF
│   └── 출력: /map, map → odom
│
├── Map Server + AMCL + Nav2
│   ├── 입력: 저장 지도, /scan, /odom, TF
│   └── 출력: /cmd_vel
│
└── RViz2
    └── 지도, LiDAR, 로봇 위치 및 주행 상태 시각화

Raspberry Pi Docker
├── RPLIDAR ─────────────────────────▶ /scan
├── Motor Bridge ◀─────────────────── /cmd_vel
│   ├── Arduino UNO 또는 OpenCR 제어
│   ├── 엔코더 오도메트리 ──────────▶ /odom
│   └── 동적 TF ─────────────────────▶ odom → base_link
└── Static TF ───────────────────────▶ base_link → LiDAR frame
```

현재 Cartographer 설정은 `use_odometry = false`입니다. 따라서 Cartographer는 LiDAR의 `/scan`과 TF를 기반으로 지도를 생성합니다.

모터 브리지가 발행하는 `/odom`은 차량 이동 상태 확인과 Nav2 위치 추정 및 주행에 사용합니다.

---

## 실행 환경

| 구분 | 환경 |
|---|---|
| Ubuntu 개발 PC | Ubuntu 22.04, ROS 2 Humble |
| Raspberry Pi 호스트 | Debian 13 (Trixie), aarch64 |
| Raspberry Pi Docker | Ubuntu 22.04, ROS 2 Humble |
| 컨테이너 이미지 | `ros:humble_ep_v01` |
| 배포 방식 | Docker |
| 형상 관리 | Git, GitHub |

Raspberry Pi 호스트에는 ROS 2 Humble이 직접 설치되어 있지 않습니다. 차량 ROS 2 노드는 Ubuntu 22.04 기반 Docker 컨테이너 안에서 실행합니다.

---

## 작업 폴더

| 환경 | 작업 폴더 | 역할 |
|---|---|---|
| Ubuntu 개발 PC | `~/Future_drive_E/ep_part/ep_ws` | Cartographer, 지도 저장, AMCL, Nav2, RViz2 |
| Raspberry Pi 호스트 | `~/ep_part` | Dockerfile, entrypoint, 차량 워크스페이스 |
| Raspberry Pi 호스트 워크스페이스 | `~/ep_part/ros2_ws` | 차량 ROS 2 패키지 |
| Raspberry Pi Docker | `/root/ep_part/ros2_ws` | 모터·LiDAR·카메라 노드 빌드 및 실행 |

---

## 차량 프로파일

| 항목 | hkit4 | hkit5 |
|---|---|---|
| ROS Domain ID | `40` | `50` |
| 모터 제어 보드 | Arduino UNO | OpenCR |
| 모터 장치 | `/dev/ttyACM0` | `/dev/ttyACM0` |
| 제어 보드 통신속도 | `9600` baud | `115200` baud |
| 모터 브리지 | `cmd_vel_serial_bridge` | `cmd_vel_bridge` |
| 모터 명령 형식 | `W/S/A/D/X` | `V,<LEFT_SPEED>,<RIGHT_SPEED>` |
| LiDAR | RPLIDAR C1 | RPLIDAR A1 |
| LiDAR 장치 | `/dev/ttyUSB0` | `/dev/ttyUSB0` |
| LiDAR 통신속도 | `460800` baud | `115200` baud |
| LiDAR Frame | `laser_frame` | `lidar_frame` |
| 카메라 노드 | 실행 | 실행 |
| 회피 노드 | 실행하지 않음 | 실행 |

### 프로파일 설정

프로젝트 저장소의 기본 설정 파일은 다음 위치에 있습니다.

```text
ep_part/hardware/config.ini
```

실제 차량 실행 시 `entrypoint.sh`는 Raspberry Pi 호스트의 `~/config.ini`를 컨테이너의 `/root/config.ini`에 연결합니다.

hkit4:

```ini
[robot]
profile = hkit4
```

hkit5:

```ini
[robot]
profile = hkit5
```

프로파일을 변경한 뒤에는 차량 bringup을 다시 시작해야 합니다.

---

## 프로젝트 구조

```text
ep_part/
├── README.md
├── Dockerfile
├── entrypoint.sh
├── default.rviz
├── hardware/
│   ├── config.ini
│   ├── futuredrive_4wd_firmware/
│   │   └── futuredrive_4wd/
│   │       └── futuredrive_4wd.ino
│   ├── opnecr_firmware/
│   │   └── opencr_motor.ino
│   └── ros2_ws/src/my_robot_bringup/
│       ├── launch/
│       │   └── ep.launch.py
│       └── my_robot_bringup/
│           ├── cmd_vel_serial_bridge.py
│           ├── cmd_vel_bridge.py
│           ├── avoidance_node.py
│           └── camera_node.py
└── ep_ws/src/arduino_cartography/
    ├── launch/
    │   ├── futuredrive_hkit.launch.py
    │   └── nav2_hkit5.launch.py
    ├── maps/
    │   ├── hkit_map.yaml
    │   ├── hkit_map.pgm
    │   ├── hkit_map_test.yaml
    │   └── hkit_map_test.pgm
    ├── params/
    │   ├── futuredrive_hkit.lua
    │   └── nav2_params.yaml
    ├── rviz/
    │   └── nav2_view.rviz
    ├── package.xml
    └── setup.py
```

ROS 2 빌드 결과인 `build/`, `install/`, `log/`는 소스 코드가 아니므로 Git 저장소에 포함하지 않습니다.

---

# 설치

## 1. Ubuntu PC 패키지 설치

```bash
sudo apt update

sudo apt install -y \
  ros-humble-usb-cam \
  ros-humble-compressed-image-transport \
  ros-humble-rplidar-ros \
  ros-humble-vision-msgs \
  ros-humble-cartographer \
  ros-humble-cartographer-ros \
  ros-humble-rviz2 \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-map-server \
  ros-humble-tf2-tools \
  python3-serial \
  python3-opencv \
  python3-numpy
```

ROS 2 환경을 현재 터미널에 적용합니다.

```bash
source /opt/ros/humble/setup.bash
```

새 터미널에서도 자동으로 적용하려면 다음 명령을 실행합니다.

```bash
grep -qxF 'source /opt/ros/humble/setup.bash' ~/.bashrc || \
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc

source ~/.bashrc
```

`ros-humble-usb-cam`, `ros-humble-rplidar-ros`, `python3-serial`은 PC에서 해당 하드웨어 노드를 직접 실행하지 않는다면 필수는 아닙니다.

Raspberry Pi 차량 컨테이너에 필요한 패키지는 프로젝트의 `Dockerfile`에서 설치합니다.

---

# Raspberry Pi 차량 Bringup

## 1. 차량 설정 확인

Raspberry Pi 호스트에서 실행합니다.

```bash
cat ~/config.ini
```

설정 파일이 없다면 차량에 맞게 생성합니다.

hkit4:

```bash
printf '[robot]\nprofile = hkit4\n' > ~/config.ini
```

hkit5:

```bash
printf '[robot]\nprofile = hkit5\n' > ~/config.ini
```

## 2. 장치 확인

```bash
lsusb
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/media* /dev/video* 2>/dev/null
```

일반적인 장치 구성:

```text
/dev/ttyUSB0   LiDAR
/dev/ttyACM0   Arduino UNO 또는 OpenCR
```

장치 번호는 연결 순서에 따라 달라질 수 있으므로 실행 전에 확인합니다.

## 3. Docker 이미지 빌드

이미지가 없는 경우 Raspberry Pi 호스트에서 실행합니다.

```bash
cd ~/ep_part
docker build -t ros:humble_ep_v01 .
```

## 4. 차량 컨테이너 실행

```bash
cd ~/ep_part
chmod +x entrypoint.sh
./entrypoint.sh
```

`entrypoint.sh`는 호스트 이름을 기준으로 Domain ID를 설정합니다.

```text
hkit4 → ROS_DOMAIN_ID=40
hkit5 → ROS_DOMAIN_ID=50
```

정상적으로 실행되면 프롬프트가 다음과 같이 바뀝니다.

```text
root@hkit4:~#
```

또는:

```text
root@hkit5:~#
```

## 5. 컨테이너 내부 빌드

```bash
cd /root/ep_part/ros2_ws
source /opt/ros/humble/setup.bash

colcon build
source install/setup.bash
```

## 6. 차량 Bringup 실행

```bash
ros2 launch my_robot_bringup ep.launch.py
```

프로파일별 실행 노드:

| hkit4 | hkit5 |
|---|---|
| RPLIDAR 노드 | RPLIDAR 노드 |
| `base_link → laser_frame` 정적 TF | `base_link → lidar_frame` 정적 TF |
| Arduino 모터 브리지 | OpenCR 모터 브리지 |
| — | 회피 노드 |
| — | 카메라 노드 |

---

# Ubuntu PC 연결

## hkit4 연결

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=40
```

## hkit5 연결

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=50
```

PC와 차량 컨테이너의 Domain ID가 다르면 서로의 노드와 토픽이 보이지 않습니다.

ROS 2 데몬에 이전 Domain 정보가 남아 있으면 다음 명령으로 재시작합니다.

```bash
ros2 daemon stop
ros2 daemon start
```

---

# 차량 상태 확인

## 노드와 토픽

```bash
ros2 node list

ros2 topic list | grep -E \
'^/(cmd_vel|odom|scan|map|tf|tf_static|arduino/command|arduino/feedback|ultrasonic|emergency_stop)'
```

## LiDAR

```bash
timeout 5s ros2 topic hz /scan
ros2 topic echo /scan --once
```

정상적인 경우 약 9~10 Hz로 `/scan`이 발행됩니다.

## 오도메트리

```bash
ros2 topic echo /odom --once
```

차량을 움직였을 때 위치 또는 방향 값이 변해야 합니다.

## TF

공통:

```bash
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

Cartographer 실행 후:

```bash
timeout 5s ros2 run tf2_ros tf2_echo map odom
```

---

# ROS 2 주요 토픽

| 토픽 | 차량 | 타입 | 설명 |
|---|---|---|---|
| `/cmd_vel` | 공통 | `geometry_msgs/msg/Twist` | 모터 브리지 입력 |
| `/odom` | 공통 | `nav_msgs/msg/Odometry` | 엔코더 오도메트리 |
| `/scan` | 공통 | `sensor_msgs/msg/LaserScan` | LiDAR 스캔 |
| `/map` | PC | `nav_msgs/msg/OccupancyGrid` | Cartographer 또는 map_server 지도 |
| `/tf` | 공통 | `tf2_msgs/msg/TFMessage` | 동적 TF |
| `/tf_static` | 공통 | `tf2_msgs/msg/TFMessage` | 정적 TF |
| `/arduino/command` | hkit4 | `std_msgs/msg/String` | Arduino에 전달한 명령 |
| `/arduino/feedback` | hkit4 | `std_msgs/msg/String` | Arduino 피드백 |
| `/ultrasonic/range` | hkit4 | `sensor_msgs/msg/Range` | 후방 초음파 거리 |
| `/ultrasonic` | hkit5 | `sensor_msgs/msg/Range` | 초음파 거리 |
| `/emergency_stop` | hkit5 | `std_msgs/msg/Bool` | 비상정지 상태 |
| `/image_raw/compressed` | hkit5 | `sensor_msgs/msg/CompressedImage` | 카메라 JPEG 영상 |
| `/yolo/detections` | hkit5 | `vision_msgs/msg/Detection2DArray` | 객체 감지 결과 입력 |
| `/cmd_vel_avoid` | hkit5 | `geometry_msgs/msg/Twist` | 회피 노드 속도 출력 |

`/cmd_vel_avoid`가 발행된다고 자동으로 모터가 움직이는 것은 아닙니다. 회피 명령을 모터 브리지에 적용하려면 remap 또는 velocity mux 구성이 필요합니다.

---

# TF 구조

hkit4:

```text
map → odom → base_link → laser_frame
```

hkit5:

```text
map → odom → base_link → lidar_frame
```

| TF | 담당 |
|---|---|
| `map → odom` | 지도 작성 중 Cartographer, 저장 지도 주행 중 AMCL |
| `odom → base_link` | 엔코더 오도메트리 브리지 |
| `base_link → laser_frame` | hkit4 정적 TF |
| `base_link → lidar_frame` | hkit5 정적 TF |

`map → base_link`를 정적 TF로 고정하지 않습니다.

지도 작성 중에는 Cartographer가 `map → odom`을 담당하고, 저장 지도 기반 주행 중에는 AMCL이 담당합니다. Cartographer와 AMCL을 동시에 실행하지 않습니다.

---

# 차량 제어

## hkit4 — Arduino UNO

펌웨어:

```text
hardware/futuredrive_4wd_firmware/futuredrive_4wd/futuredrive_4wd.ino
```

ROS 2 브리지:

```text
hardware/ros2_ws/src/my_robot_bringup/my_robot_bringup/cmd_vel_serial_bridge.py
```

브리지는 `/cmd_vel`을 다음 Arduino 문자 명령으로 변환합니다.

| 동작 | Arduino 명령 |
|---|---|
| 전진 | `W` |
| 후진 | `S` |
| 좌회전 | `A` |
| 우회전 | `D` |
| 정지 | `X` |

Arduino IDE Serial Monitor와 ROS 2 브리지는 `/dev/ttyACM0`을 동시에 사용할 수 없습니다. 차량 bringup을 실행하기 전에 Serial Monitor를 닫습니다.

## hkit5 — OpenCR

펌웨어:

```text
hardware/opnecr_firmware/opencr_motor.ino
```

ROS 2 브리지:

```text
hardware/ros2_ws/src/my_robot_bringup/my_robot_bringup/cmd_vel_bridge.py
```

OpenCR 속도 명령:

```text
V,<LEFT_SPEED>,<RIGHT_SPEED>
```

정지 명령:

```text
V,0,0
```

OpenCR은 직접 `W/S/A/D/X` 명령도 처리할 수 있지만, ROS 2 주행에서는 모터 브리지가 `/cmd_vel`을 좌·우 바퀴 속도로 계산해 `V` 형식으로 전달합니다.

OpenCR은 Dynamixel2Arduino를 통해 모터를 제어하고 엔코더 틱을 다음 형식으로 반환합니다.

```text
F,<TIME_MS>,<LEFT_TICK>,<RIGHT_TICK>
```

브리지는 이 값을 이용해 `/odom`과 `odom → base_link` TF를 생성합니다.

---

# 키보드 주행

`teleop_twist_keyboard`을 사용하면 키보드 입력이 `/cmd_vel`로 발행됩니다.

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=40
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

hkit5는 Domain ID를 `50`으로 변경합니다.

주요 키:

| 동작 | 키 |
|---|---|
| 전진 | `i` |
| 후진 | `,` |
| 좌회전 | `j` |
| 우회전 | `l` |
| 정지 | `k` |

`teleop_twist_keyboard`에서는 `W`가 아니라 `i`를 사용합니다. hkit4 브리지가 수신한 `/cmd_vel`을 내부적으로 Arduino의 `W/S/A/D/X` 명령으로 변환합니다.

안전을 위해 첫 시험은 바퀴를 바닥에서 띄우거나 주변 공간을 확보한 상태에서 진행합니다.

---

# LiDAR

| 차량 | 모델 | 장치 | 통신속도 | Frame | 예상 주기 |
|---|---|---|---:|---|---:|
| hkit4 | RPLIDAR C1 | `/dev/ttyUSB0` | `460800` | `laser_frame` | 약 10 Hz |
| hkit5 | RPLIDAR A1 | `/dev/ttyUSB0` | `115200` | `lidar_frame` | 약 10 Hz |

통신속도가 맞지 않으면 다음과 같은 문제가 발생할 수 있습니다.

```text
SL_RESULT_OPERATION_TIMEOUT
```

또는 `/scan`이 발행되지 않을 수 있습니다.

---

# Cartographer 2D SLAM

## PC 워크스페이스 빌드

```bash
cd ~/Future_drive_E/ep_part/ep_ws
source /opt/ros/humble/setup.bash

colcon build
source install/setup.bash
```

## Cartographer 실행

hkit4:

```bash
export ROS_DOMAIN_ID=40
ros2 launch arduino_cartography futuredrive_hkit.launch.py
```

hkit5:

```bash
export ROS_DOMAIN_ID=50
ros2 launch arduino_cartography futuredrive_hkit.launch.py
```

정상적으로 시작되면 다음과 비슷한 메시지가 출력됩니다.

```text
Added trajectory with ID '0'
Inserted submap (0, 0)
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
```

현재 Cartographer는 `/odom`을 센서 입력으로 사용하지 않고 `/scan`과 TF를 이용해 지도를 생성합니다.

지도 작성 중에는 차량을 천천히 움직이고 급회전, 급정지 및 바퀴 미끄러짐을 줄입니다.

---

# 지도 저장

Cartographer가 `/map`을 발행하는 상태에서 실행합니다.

```bash
ros2 run nav2_map_server map_saver_cli \
  -f /home/hkit/Future_drive_E/ep_part/ep_ws/src/arduino_cartography/maps/hkit_map_test
```

정상 완료 시 다음 파일이 생성됩니다.

```text
hkit_map_test.yaml
hkit_map_test.pgm
```

지도 저장이 끝나면 Cartographer를 종료합니다. 저장 지도 기반 위치 추정에서는 Cartographer와 AMCL을 동시에 실행하지 않습니다.

---

# 저장 지도 기반 위치 추정 및 Nav2

아래 예시는 hkit5의 Domain ID `50`을 사용합니다. hkit4에서 실행하려면 `40`으로 변경합니다.

## 1. AMCL과 Map Server 실행

```bash
source /opt/ros/humble/setup.bash
source /home/hkit/Future_drive_E/ep_part/ep_ws/install/setup.bash
export ROS_DOMAIN_ID=50

ros2 launch nav2_bringup localization_launch.py \
  map:=/home/hkit/Future_drive_E/ep_part/ep_ws/src/arduino_cartography/maps/hkit_map_test.yaml \
  params_file:=/home/hkit/Future_drive_E/ep_part/ep_ws/src/arduino_cartography/params/nav2_params.yaml \
  use_sim_time:=False
```

AMCL과 Map Server는 Nav2 주행 중에도 계속 실행합니다.

## 2. Nav2 실행

다른 PC 터미널에서 실행합니다.

```bash
source /opt/ros/humble/setup.bash
source /home/hkit/Future_drive_E/ep_part/ep_ws/install/setup.bash
export ROS_DOMAIN_ID=50

ros2 launch arduino_cartography nav2_hkit5.launch.py
```

`nav2_hkit5.launch.py`는 Nav2 navigation 노드를 실행합니다. Map Server와 AMCL은 포함하지 않으므로 localization launch를 먼저 실행해야 합니다.

---

# RViz2

## 현재 SLAM 확인용 설정

현재 프로젝트 루트의 `default.rviz`를 사용할 수 있습니다.

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=50

ros2 run rviz2 rviz2 \-d /home/hkit/Future_drive_E/ep_part/ep_ws/src/arduino_cartography/rviz/nav2_view.rviz
```

hkit4에서는 Domain ID를 `40`으로 변경합니다.

현재 설정:

- Fixed Frame: `map`
- Map Topic: `/map`
- LaserScan Topic: `/scan`
- Camera Topic: `/image_raw/compressed`

그래픽 드라이버 문제가 발생하면 소프트웨어 렌더링으로 실행합니다.

```bash
LIBGL_ALWAYS_SOFTWARE=1 rviz2 \
    -d /home/hkit/Future_drive_E/ep_part/ep_ws/src/arduino_cartography/rviz/nav2_view.rviz
```

## Nav2 전용 RViz 설정 

```

Nav2 전용 설정에는 다음 항목을 포함합니다.

- Map
- TF
- LaserScan
- RobotModel
- Global Costmap
- Local Costmap
- Global Plan
- Local Plan
- 2D Pose Estimate
- Nav2 Goal

저장 지도를 늦게 실행한 RViz에서도 받으려면 Map display의 Durability Policy를 `Transient Local`로 설정합니다.

파일 추가 후 실행 명령:

```bash
rviz2 \
  -d /home/hkit/Future_drive_E/ep_part/ep_ws/src/arduino_cartography/rviz/nav2_view.rviz
```

RViz에서:

1. Fixed Frame을 `map`으로 설정합니다.
2. **2D Pose Estimate**로 로봇의 실제 초기 위치와 방향을 지정합니다.
3. 위치 추정이 안정되는지 확인합니다.
4. **Nav2 Goal**로 목적지를 지정합니다.

---

# 권장 실행 순서

```text
1. Raspberry Pi 호스트에서 장치 확인
2. Raspberry Pi에서 ./entrypoint.sh 실행
3. Docker 컨테이너에서 차량 bringup 실행
4. Ubuntu PC에서 차량 토픽과 TF 확인
5. Cartographer로 지도 작성
6. map_saver_cli로 지도 저장
7. Cartographer 종료
8. Map Server와 AMCL 실행
9. Nav2 실행
10. RViz2에서 초기 자세 및 목표 위치 지정
```

---

# 문제 해결

## 차량 노드가 보이지 않음

```bash
echo $ROS_DOMAIN_ID
ros2 node list
```

hkit4는 `40`, hkit5는 `50`을 사용합니다. PC와 차량 컨테이너의 Domain ID가 동일한지 확인합니다.

필요하면 ROS 2 데몬을 재시작합니다.

```bash
ros2 daemon stop
ros2 daemon start
```

## `/scan`이 발행되지 않음

```bash
ls -l /dev/ttyUSB* 2>/dev/null
ros2 topic hz /scan
```

확인 항목:

- LiDAR USB 연결
- 컨테이너 장치 접근 권한
- 차량별 baud rate
- 동일 장치를 다른 프로세스가 사용 중인지 확인

## `/odom`이 발행되지 않음

```bash
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
```

확인 항목:

- `/dev/ttyACM0` 연결
- 모터 브리지 노드 실행 여부
- 제어 보드 펌웨어
- 엔코더 피드백 수신 여부

hkit4:

```bash
ros2 topic echo /arduino/feedback
```

## 키보드 명령은 발행되지만 바퀴가 움직이지 않음

```bash
ros2 topic echo /cmd_vel
```

hkit4는 다음 명령도 확인합니다.

```bash
ros2 topic echo /arduino/command
```

`/cmd_vel`과 `/arduino/command`가 정상이라면 Arduino 전원, 모터 전원, 모터 드라이버 및 배선을 확인합니다.

## `/map` 토픽은 있지만 지도가 보이지 않음

```bash
ros2 node list | grep -E 'map_server|cartographer'
ros2 topic info /map --verbose
ros2 topic echo /map --once
```

SLAM 중에는 Cartographer가 `/map`을 발행하고, 저장 지도 주행 중에는 Map Server가 발행해야 합니다.

## Cartographer가 중복 실행됨

```bash
ros2 node list | grep cartographer
ros2 topic info /submap_list -v
```

같은 Domain ID에서 Cartographer가 두 개 실행되면 중복 노드와 submap 오류가 발생할 수 있습니다. 기존 Cartographer를 종료한 뒤 하나만 실행합니다.

## Nav2에서 `odom` TF를 찾지 못함

```bash
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
```

차량 브리지의 `/odom`, `odom → base_link`, 동일한 Domain ID 및 DDS 네트워크 연결을 확인합니다.

## RViz2 그래픽 오류

```bash
LIBGL_ALWAYS_SOFTWARE=1 rviz2
```

RViz 창이 정상적으로 열리고 지도와 센서 데이터가 표시된다면 GLSL 관련 경고는 그래픽 드라이버 호환성 문제일 수 있습니다.

---

# 종료 순서

1. 차량을 정지합니다.
2. 키보드 주행 노드를 `Ctrl+C`로 종료합니다.
3. Nav2와 AMCL을 종료합니다.
4. Cartographer 또는 RViz2를 종료합니다.
5. Raspberry Pi 차량 bringup을 `Ctrl+C`로 종료합니다.
6. 컨테이너에서 `exit`를 입력합니다.
7. Raspberry Pi 운영체제를 정상 종료한 뒤 전원을 분리합니다.

```bash
sudo shutdown -h now
```
