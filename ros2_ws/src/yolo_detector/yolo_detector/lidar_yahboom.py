#!/usr/bin/env python3
"""
YOLO 객체 검출 노드
- /camera/image_raw 토픽을 구독하여 YOLO 모델로 객체 검출
- 검출 결과를 /yolo/detections (vision_msgs/Detection2DArray) 토픽으로 발행
- 검출 결과가 그려진 이미지를 /yolo/result_image (sensor_msgs/Image) 토픽으로 발행
"""

from networkx.generators import spectral_graph_forge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import cv2
import json
import time
import math


class YoloDetector(Node):
    """YOLO 모델을 이용한 실시간 객체 검출 ROS2 노드"""

    def __init__(self):
        super().__init__('yolo_detector')

        # ── 파라미터 선언 ──
        self.declare_parameter('video_source', '0')     # 웹캠 인덱스 또는 영상 파일 경로
        self.declare_parameter('model_name', 'models/futuredrive_yolo26n_aug_v1_best.pt')   # YOLO 학습모델 사용시 모델명 변경
        self.declare_parameter('confidence_threshold', 0.5)     # 신뢰도 임계값
        self.declare_parameter('device', 'cpu')                 # 'cpu' 또는 'cuda'
        self.declare_parameter('input_topic', '/image_raw/compressed')
        self.declare_parameter('max_det', 50)    
        # LiDAR / 카메라 설정
        self.declare_parameter('camera_fov_deg', 72.0)
        self.declare_parameter('lidar_angle_offset_deg', 0.0)
        self.declare_parameter('lidar_window_deg', 3.0)               # 최대 검출 수

        # 파라미터 값 가져오기
        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        device = self.get_parameter('device').get_parameter_value().string_value
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.max_det = self.get_parameter('max_det').get_parameter_value().integer_value
        video_source = self.get_parameter('video_source').value
        self.camera_fov_deg = self.get_parameter(
            'camera_fov_deg'
        ).value

        self.lidar_angle_offset_deg = self.get_parameter(
            'lidar_angle_offset_deg'
        ).value

        self.lidar_window_deg = self.get_parameter(
            'lidar_window_deg'
        ).value


        if str(video_source).isdigit():
            video_source = int(video_source)

        self.cap = cv2.VideoCapture(video_source)

        # ── YOLO 모델 로드 ──
        self.get_logger().info(f'🔄 YOLO 모델 로딩 중: {model_name}')
        self.model = YOLO(model_name)
        self.model.to(device)
        self.get_logger().info(f'✅ YOLO 모델 로드 완료 (디바이스: {device})')

        # ── CvBridge 초기화 ──
        self.bridge = CvBridge()



        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.latest_scan = None

        # ── 구독자: 카메라 이미지 ──
        self.subscription = self.create_subscription(
            CompressedImage,
            input_topic,
            self.image_callback,
            10
        )
        self.get_logger().info(f'📥 구독 토픽: {input_topic}')

        # ── 퍼블리셔: 검출 결과 ──
        # 1) Detection2DArray (표준 vision_msgs 형식)
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/yolo/detections',
            10
        )

        # 2) JSON 형식 검출 결과 (디버깅/간편 사용용)
        self.json_pub = self.create_publisher(
            String,
            '/yolo/detections_json',
            10
        )

        # 3) 검출 결과 시각화 이미지
        self.result_image_pub = self.create_publisher(
            Image,
            '/yolo/result_image',
            10
        )

        self.get_logger().info('📤 발행 토픽: /yolo/detections, /yolo/detections_json, /yolo/result_image')

        # ── 통계 ──
        self.inference_count = 0
        self.total_inference_time = 0.0

        self.get_logger().info('🚀 YoloDetector 노드 시작!')

    def scan_callback(self, msg):
        self.latest_scan = msg

    def get_lidar_distance(self, bbox_center_x, image_width):

        if self.latest_scan is None:
            return None

        scan = self.latest_scan

        # bbox 중심 x → 카메라 수평각
        camera_angle_deg = (
            (bbox_center_x / image_width) - 0.5
        ) * self.camera_fov_deg

        # 카메라와 LiDAR 방향 차이 보정
        lidar_angle_deg = (
            camera_angle_deg +
            self.lidar_angle_offset_deg
        )

        # bbox 방향 주변 ±3도 탐색
        angle_min = math.radians(
            lidar_angle_deg - self.lidar_window_deg
        )

        angle_max = math.radians(
            lidar_angle_deg + self.lidar_window_deg
        )

        # LaserScan index로 변환
        start_idx = int(
            (angle_min - scan.angle_min)
            / scan.angle_increment
        )

        end_idx = int(
            (angle_max - scan.angle_min)
            / scan.angle_increment
        )

        distances = []

        for i in range(start_idx, end_idx + 1):

            if 0 <= i < len(scan.ranges):

                r = scan.ranges[i]

                if (
                    math.isfinite(r)
                    and scan.range_min <= r <= scan.range_max
                ):
                    distances.append(r)

        if not distances:
            return None

        return min(distances)

    def image_callback(self, msg: Image):
        """카메라 이미지 수신 시 YOLO 추론 수행"""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_image is None:
                self.get_logger().error("❌ CompressedImage 디코딩 실패")
                return
        except Exception as e:
            self.get_logger().error(f'❌ 이미지 변환 실패: {e}')
            return

        # ── YOLO 추론 ──
        start_time = time.time()
        results = self.model(
            cv_image,
            conf=self.conf_threshold,
            max_det=self.max_det,
            verbose=False
        )
        inference_time = time.time() - start_time

        self.inference_count += 1
        self.total_inference_time += inference_time

        # ── 결과 처리 ──
        result = results[0]
        boxes = result.boxes

        # 1) Detection2DArray 메시지 생성
        detection_array_msg = Detection2DArray()
        detection_array_msg.header = msg.header

        # 2) JSON 결과 리스트
        json_detections = []

        # 3) 시각화를 위한 이미지 복사
        annotated_image = cv_image.copy()

        if boxes is not None and len(boxes) > 0:
            for box in boxes:
                # 바운딩 박스 좌표 (xyxy → center + size)
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                w = x2 - x1
                h = y2 - y1

                distance = self.get_lidar_distance(
                    cx,
                    cv_image.shape[1]
                )

                confidence = float(box.conf[0].cpu().numpy())
                class_id = int(box.cls[0].cpu().numpy())
                class_name = self.model.names[class_id]

                # Detection2D 메시지 구성
                detection = Detection2D()
                detection.bbox.center.position.x = float(cx)
                detection.bbox.center.position.y = float(cy)
                detection.bbox.size_x = float(w)
                detection.bbox.size_y = float(h)

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(class_id)
                hypothesis.hypothesis.score = confidence
                detection.results.append(hypothesis)

                detection_array_msg.detections.append(detection)

                # JSON 결과 추가
                json_detections.append({
                    'class_id': class_id,
                    'class_name': class_name,
                    'confidence': round(confidence, 3),
                    'bbox': {
                        'x1': round(float(x1), 1),
                        'y1': round(float(y1), 1),
                        'x2': round(float(x2), 1),
                        'y2': round(float(y2), 1),
                    },
                    'center': {
                        'x': round(float(cx), 1),
                        'y': round(float(cy), 1),
                    },
                    'lidar_distance_m': (
                        round(distance, 2)
                        if distance is not None
                        else None
                    ),
                })

                # ── 시각화: 바운딩 박스 + 라벨 그리기 ──
                color = self._get_color(class_id)
                cv2.rectangle(
                    annotated_image,
                    (int(x1), int(y1)),
                    (int(x2), int(y2)),
                    color, 2
                )

                label = f'{class_name} {confidence:.2f}'
                if distance is not None:
                    label += f" | {distance:.2f} m"
                
                label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.rectangle(
                    annotated_image,
                    (int(x1), int(y1) - label_size[1] - 10),
                    (int(x1) + label_size[0], int(y1)),
                    color, -1
                )
                cv2.putText(
                    annotated_image,
                    label,
                    (int(x1), int(y1) - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 255, 255), 2
                )

        # ── FPS 정보 표시 ──
        fps = 1.0 / inference_time if inference_time > 0 else 0
        fps_text = f'FPS: {fps:.1f} | Objects: {len(json_detections)}'
        cv2.putText(
            annotated_image,
            fps_text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8, (0, 255, 0), 2
        )

        # ── 토픽 발행 ──
        # Detection2DArray 발행
        self.detection_pub.publish(detection_array_msg)

        # JSON 결과 발행
        json_msg = String()
        json_msg.data = json.dumps({
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'inference_time_ms': round(inference_time * 1000, 1),
            'num_detections': len(json_detections),
            'detections': json_detections
        }, ensure_ascii=False)
        self.json_pub.publish(json_msg)

        # 시각화 이미지 발행
        result_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
        result_msg.header = msg.header
        self.result_image_pub.publish(result_msg)

        # 주기적 로그
        if self.inference_count % 30 == 0:
            avg_time = self.total_inference_time / self.inference_count
            self.get_logger().info(
                f'📊 추론 #{self.inference_count} | '
                f'검출: {len(json_detections)}개 | '
                f'추론시간: {inference_time*1000:.1f}ms | '
                f'평균: {avg_time*1000:.1f}ms | '
                f'FPS: {1.0/avg_time:.1f}'
            )

    def _get_color(self, class_id: int) -> tuple:
        """클래스 ID별 고유 색상 생성"""
        colors = [
            (255, 0, 0), (0, 255, 0), (0, 0, 255),
            (255, 255, 0), (255, 0, 255), (0, 255, 255),
            (128, 0, 0), (0, 128, 0), (0, 0, 128),
            (128, 128, 0), (128, 0, 128), (0, 128, 128),
            (255, 128, 0), (255, 0, 128), (128, 255, 0),
            (0, 255, 128), (128, 0, 255), (0, 128, 255),
            (255, 128, 128), (128, 255, 128),
        ]
        return colors[class_id % len(colors)]

    def get_lidar_distance(self, bbox_center_x, image_width):

        if self.latest_scan is None:
            return None

        scan = self.latest_scan

        camera_fov_deg = 90.0

        center_angle = (
            (bbox_center_x / image_width) - 0.5
        ) * camera_fov_deg

        # bbox 주변 ±3도 사용
        angle_min = math.radians(center_angle - 3)
        angle_max = math.radians(center_angle + 3)

        start_idx = int(
            (angle_min - scan.angle_min)
            / scan.angle_increment
        )

        end_idx = int(
            (angle_max - scan.angle_min)
            / scan.angle_increment
        )

        distances = []

        for i in range(start_idx, end_idx + 1):

            if 0 <= i < len(scan.ranges):

                r = scan.ranges[i]

                if (
                    not math.isnan(r)
                    and not math.isinf(r)
                    and scan.range_min <= r <= scan.range_max
                ):
                    distances.append(r)

        if not distances:
            return None

        return min(distances)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 사용자에 의해 종료됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
