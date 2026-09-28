# 🎯 YOLO 객체 검출 ROS2 패키지

카메라 또는 영상 파일을 입력으로 받아 YOLOv8 모델을 통해 실시간 객체 검출을 수행하고,
결과를 ROS2 토픽으로 발행하는 프로토타입 시스템입니다.

---

## 📐 시스템 아키텍처

```
┌──────────────────┐     /camera/image_raw      ┌──────────────────┐
│                  │  ─────────────────────────▶ │                  │
│  camera_node     │    (sensor_msgs/Image)      │   yolo_node      │
│  (카메라/영상)    │                             │  (YOLO 객체검출)  │
└──────────────────┘                             └────────┬─────────┘
                                                          │
                                          ┌───────────────┼──────────────────┐
                                          │               │                  │
                                          ▼               ▼                  ▼
                                 /yolo/detections  /yolo/detections_json  /yolo/result_image
                                 (Detection2DArray)  (String/JSON)       (sensor_msgs/Image)
                                                          │                  │
                                                          ▼                  ▼
                                                   ┌──────────────────────────┐
                                                   │   visualization_node     │
                                                   │  (결과 시각화 + 로그)     │
                                                   └──────────────────────────┘
```

---

## 🔧 노드 설명

### 1. `camera_node` (카메라 퍼블리셔)
- **역할**: 웹캠 또는 영상 파일에서 프레임을 캡처하여 ROS2 토픽으로 발행
- **발행 토픽**: `/camera/image_raw` (`sensor_msgs/Image`)
- **파라미터**:
  | 파라미터 | 기본값 | 설명 |
  |---------|--------|------|
  | `video_source` | `0` | 웹캠 인덱스 또는 영상 파일 경로 |
  | `frame_rate` | `30.0` | 프레임 발행 주기 (Hz) |
  | `frame_width` | `640` | 프레임 너비 |
  | `frame_height` | `480` | 프레임 높이 |
  | `loop_video` | `True` | 영상 파일 루프 재생 여부 |

### 2. `yolo_node` (YOLO 객체 검출기)
- **역할**: 카메라 이미지를 구독하여 YOLO 모델로 추론, 결과를 발행
- **구독 토픽**: `/camera/image_raw` (`sensor_msgs/Image`)
- **발행 토픽**:
  | 토픽 | 메시지 타입 | 설명 |
  |------|------------|------|
  | `/yolo/detections` | `vision_msgs/Detection2DArray` | 표준 검출 결과 |
  | `/yolo/detections_json` | `std_msgs/String` | JSON 형식 검출 결과 |
  | `/yolo/result_image` | `sensor_msgs/Image` | 바운딩 박스가 그려진 이미지 |
- **파라미터**:
  | 파라미터 | 기본값 | 설명 |
  |---------|--------|------|
  | `model_name` | `yolov8n.pt` | YOLO 모델 파일 |
  | `confidence_threshold` | `0.5` | 검출 신뢰도 임계값 |
  | `device` | `cpu` | 추론 장치 (`cpu` / `cuda`) |
  | `max_det` | `50` | 최대 검출 객체 수 |

### 3. `visualization_node` (시각화)
- **역할**: 검출 결과 이미지를 화면에 표시하고 로그 출력
- **구독 토픽**: `/yolo/result_image`, `/yolo/detections_json`

---

## 🚀 실행 방법

### 사전 준비
```bash
# 의존성 설치
pip3 install ultralytics opencv-python

# 빌드
cd ~/future/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select yolo_detector --symlink-install
source install/setup.bash
```

### 방법 1: 런치 파일로 전체 실행 (권장)
```bash
# 웹캠 사용 (기본)
ros2 launch yolo_detector yolo_detection_launch.py

# 영상 파일 사용
ros2 launch yolo_detector yolo_detection_launch.py video_source:=/path/to/video.mp4

# YOLO 모델 변경 (더 정확하지만 느림)
ros2 launch yolo_detector yolo_detection_launch.py model_name:=yolov8s.pt

# 신뢰도 임계값 변경
ros2 launch yolo_detector yolo_detection_launch.py confidence:=0.3

# GPU 사용 (CUDA 필요)
ros2 launch yolo_detector yolo_detection_launch.py device:=cuda
```

### 방법 2: 노드 개별 실행
```bash
# 터미널 1: 카메라 노드
ros2 run yolo_detector camera_node

# 터미널 2: YOLO 검출 노드
ros2 run yolo_detector yolo_node

# 터미널 3: 시각화 노드
ros2 run yolo_detector visualization_node
```

### 방법 3: 파라미터와 함께 개별 실행
```bash
# 영상 파일 사용
ros2 run yolo_detector camera_node --ros-args -p video_source:="/path/to/video.mp4"

# 더 큰 모델 사용
ros2 run yolo_detector yolo_node --ros-args -p model_name:="yolov8m.pt" -p confidence_threshold:=0.3
```

---

## 📊 토픽 확인

```bash
# 토픽 목록 확인
ros2 topic list

# 검출 결과 확인 (JSON)
ros2 topic echo /yolo/detections_json

# 검출 결과 확인 (Detection2DArray)
ros2 topic echo /yolo/detections

# 토픽 발행 빈도 확인
ros2 topic hz /camera/image_raw
ros2 topic hz /yolo/detections
```

---

## 📝 JSON 검출 결과 예시

```json
{
  "timestamp": 1695878400.123,
  "inference_time_ms": 45.2,
  "num_detections": 3,
  "detections": [
    {
      "class_id": 0,
      "class_name": "person",
      "confidence": 0.92,
      "bbox": {"x1": 100.0, "y1": 50.0, "x2": 300.0, "y2": 400.0},
      "center": {"x": 200.0, "y": 225.0}
    },
    {
      "class_id": 67,
      "class_name": "cell phone",
      "confidence": 0.78,
      "bbox": {"x1": 250.0, "y1": 200.0, "x2": 310.0, "y2": 350.0},
      "center": {"x": 280.0, "y": 275.0}
    }
  ]
}
```

---

## 🧩 YOLO 모델 종류

| 모델 | 크기 | 속도 | 정확도 | 권장 사용 |
|------|------|------|--------|----------|
| `yolov8n.pt` | 6MB | 가장 빠름 | 보통 | 실시간 / 임베디드 |
| `yolov8s.pt` | 22MB | 빠름 | 좋음 | 일반 용도 |
| `yolov8m.pt` | 52MB | 보통 | 매우 좋음 | 높은 정확도 |
| `yolov8l.pt` | 87MB | 느림 | 우수 | 고성능 환경 |
| `yolov8x.pt` | 131MB | 가장 느림 | 최고 | GPU 환경 |

> 모델은 첫 실행 시 자동으로 다운로드됩니다.
