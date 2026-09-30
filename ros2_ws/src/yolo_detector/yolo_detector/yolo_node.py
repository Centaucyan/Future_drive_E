#!/usr/bin/env python3
"""
YOLO 객체 검출 노드
- /camera/image_raw 토픽을 구독하여 YOLO 모델로 객체 검출
- 검출 결과를 /yolo/detections (vision_msgs/Detection2DArray) 토픽으로 발행
- 검출 결과가 그려진 이미지를 /yolo/result_image (sensor_msgs/Image) 토픽으로 발행
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import json
import time


class YoloDetector(Node):
    """YOLO 모델을 이용한 실시간 객체 검출 ROS2 노드"""

    def __init__(self):
        super().__init__('yolo_detector')

        # ── 파라미터 선언 ──
        self.declare_parameter('model_name', 'models/futuredrive_yolo26n_hardneg_v2_best.pt')   # YOLO 학습모델 사용시 모델명 변경
        self.declare_parameter('confidence_threshold', 0.5)     # 신뢰도 임계값
        self.declare_parameter('four_wheeler_confidence', 0.45)
        self.declare_parameter('two_wheeler_confidence', 0.25)
        self.declare_parameter('person_confidence', 0.30)
        self.declare_parameter('confirmation_frames', 2)
        self.declare_parameter('max_missed_frames', 2)
        self.declare_parameter('temporal_iou_threshold', 0.3)
        self.declare_parameter('device', 'cpu')                 # 'cpu' 또는 'cuda'
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('max_det', 50)                   # 최대 검출 수

        # 파라미터 값 가져오기
        model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.conf_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.class_thresholds = {
            0: self.get_parameter('four_wheeler_confidence').value,
            1: self.get_parameter('two_wheeler_confidence').value,
            2: self.get_parameter('person_confidence').value,
        }
        self.confirmation_frames = self.get_parameter('confirmation_frames').value
        self.max_missed_frames = self.get_parameter('max_missed_frames').value
        self.temporal_iou_threshold = self.get_parameter('temporal_iou_threshold').value
        device = self.get_parameter('device').get_parameter_value().string_value
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.max_det = self.get_parameter('max_det').get_parameter_value().integer_value
        # ── YOLO 모델 로드 ──
        self.get_logger().info(f'🔄 YOLO 모델 로딩 중: {model_name}')
        self.model = YOLO(model_name)
        self.model.to(device)
        self.get_logger().info(f'✅ YOLO 모델 로드 완료 (디바이스: {device})')

        # ── CvBridge 초기화 ──
        self.bridge = CvBridge()

        # ── 구독자: 카메라 이미지 ──
        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            1
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
        self.temporal_tracks = []

        self.get_logger().info('🚀 YoloDetector 노드 시작!')
        self.get_logger().info(
            '🎯 클래스별 임계값: '
            f'four_wheeler={self.class_thresholds[0]:.2f}, '
            f'two_wheeler={self.class_thresholds[1]:.2f}, '
            f'person={self.class_thresholds[2]:.2f}'
        )

    def image_callback(self, msg: Image):
        """카메라 이미지 수신 시 YOLO 추론 수행"""
        try:
            # ROS2 Image → OpenCV 이미지 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'❌ 이미지 변환 실패: {e}')
            return

        # ── YOLO 추론 ──
        start_time = time.time()
        results = self.model(
            cv_image,
            # 클래스별 후처리를 위해 가장 낮은 임계값 이상의 후보를 받는다.
            conf=min(self.class_thresholds.values()),
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
        image_height, image_width = annotated_image.shape[:2]

        display_class_names = {
            'four_wheeler': '4wheel',
            'two_wheeler': '2wheel',
            'person': 'person',
        }
        font_scale = 0.4
        font_thickness = 1
        box_thickness = 1
        padding = 3

        frame_detections = []
        if boxes is not None and len(boxes) > 0:
            for box in boxes:
                # 바운딩 박스 좌표 (xyxy → center + size)
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                confidence = float(box.conf[0].cpu().numpy())
                class_id = int(box.cls[0].cpu().numpy())
                class_name = self.model.names[class_id]

                class_threshold = self.class_thresholds.get(
                    class_id, self.conf_threshold
                )
                if confidence < class_threshold:
                    continue

                frame_detections.append({
                    'class_id': class_id,
                    'class_name': class_name,
                    'confidence': confidence,
                    'bbox': [float(x1), float(y1), float(x2), float(y2)],
                })

        stable_detections = self._update_temporal_tracks(frame_detections)

        for item in stable_detections:
            x1, y1, x2, y2 = item['bbox']
            confidence = item['confidence']
            class_id = item['class_id']
            class_name = item['class_name']
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            w = x2 - x1
            h = y2 - y1

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
                }
            })

            # ── 시각화: 바운딩 박스 + 라벨 그리기 ──
            color = self._get_color(class_id)
            cv2.rectangle(
                annotated_image,
                (int(x1), int(y1)),
                (int(x2), int(y2)),
                color, box_thickness
            )

            display_name = display_class_names.get(class_name, class_name)
            label = f'{display_name} {confidence:.2f}'
            label_size, baseline = cv2.getTextSize(
                label,
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale,
                font_thickness
            )

            label_width = label_size[0] + (padding * 2)
            label_height = label_size[1] + baseline + (padding * 2)
            label_left = max(
                0,
                min(int(x1), image_width - label_width)
            )
            label_right = min(
                image_width - 1,
                label_left + label_width
            )
            box_top = max(0, min(int(y1), image_height - 1))

            if box_top - label_height >= 0:
                label_bottom = box_top
                label_top = label_bottom - label_height
            else:
                label_top = box_top
                label_bottom = min(
                    image_height - 1,
                    label_top + label_height
                )

            cv2.rectangle(
                annotated_image,
                (label_left, label_top),
                (label_right, label_bottom),
                color, -1
            )
            cv2.putText(
                annotated_image,
                label,
                (
                    label_left + padding,
                    label_top + padding + label_size[1]
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                font_scale,
                (255, 255, 255),
                font_thickness,
                cv2.LINE_AA
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

    @staticmethod
    def _bbox_iou(box_a, box_b):
        """두 xyxy 바운딩 박스의 IoU를 계산한다."""
        ax1, ay1, ax2, ay2 = box_a
        bx1, by1, bx2, by2 = box_b
        ix1, iy1 = max(ax1, bx1), max(ay1, by1)
        ix2, iy2 = min(ax2, bx2), min(ay2, by2)
        intersection = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
        area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
        area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
        union = area_a + area_b - intersection
        return intersection / union if union > 0.0 else 0.0

    def _update_temporal_tracks(self, detections):
        """연속 프레임 확인으로 순간 오검출과 짧은 검출 누락을 완화한다."""
        unmatched_tracks = set(range(len(self.temporal_tracks)))

        for detection in detections:
            best_index = None
            best_iou = self.temporal_iou_threshold
            for index in unmatched_tracks:
                track = self.temporal_tracks[index]
                if track['class_id'] != detection['class_id']:
                    continue
                iou = self._bbox_iou(track['bbox'], detection['bbox'])
                if iou >= best_iou:
                    best_iou = iou
                    best_index = index

            if best_index is None:
                self.temporal_tracks.append({
                    **detection,
                    'hits': 1,
                    'missed': 0,
                })
                continue

            track = self.temporal_tracks[best_index]
            track.update(detection)
            track['hits'] += 1
            track['missed'] = 0
            unmatched_tracks.remove(best_index)

        for index in unmatched_tracks:
            self.temporal_tracks[index]['missed'] += 1

        self.temporal_tracks = [
            track for track in self.temporal_tracks
            if track['missed'] <= self.max_missed_frames
        ]
        return [
            track for track in self.temporal_tracks
            if track['hits'] >= self.confirmation_frames
        ]


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
