"""SP YOLO 결과 Topic을 구독하여 공용 VisionState에 전달합니다."""
import json

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

import cv2


class SPVisionReceiver(Node):

    def __init__(self, vision_state):
        super().__init__("sp_vision_receiver")

        self.vision_state = vision_state
        self.bridge = CvBridge()

        self.latest_detections = []
        self.latest_jpeg = None

        # YOLO 결과 이미지
        self.image_sub = self.create_subscription(
            Image,
            "/yolo/result_image",
            self.image_callback,
            10
        )

        # YOLO detection JSON
        self.detection_sub = self.create_subscription(
            String,
            "/yolo/detections_json",
            self.detection_callback,
            10
        )

    # SP가 이미 박스와 라벨을 그린 결과를 JPEG로 변환하여 저장합니다.
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            success, encoded = cv2.imencode(".jpg", cv_image)
            if not success:
                self.get_logger().error("YOLO 결과 이미지 JPEG 인코딩 실패")
                return
            self.latest_jpeg = encoded.tobytes()
            self.vision_state.update(self.latest_detections, self.latest_jpeg)
        except Exception as error:
            # 잘못된 프레임이 들어와도 다음 메시지 수신을 계속합니다.
            self.get_logger().error(f"YOLO 결과 이미지 변환 실패: {error}")

    # JSON에서 검출 목록을 읽고 가장 최근 영상과 함께 웹 공유 상태를 갱신합니다.
    def detection_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                raise ValueError("검출 JSON의 최상위 값은 객체여야 합니다")
            detections = data.get("detections", [])
            if not isinstance(detections, list):
                raise ValueError("detections는 목록이어야 합니다")
            self.latest_detections = detections
            self.vision_state.update(self.latest_detections, self.latest_jpeg)
        except (ValueError, TypeError) as error:
            # 잘못된 JSON은 기록만 남기고 마지막 정상 결과를 유지합니다.
            self.get_logger().error(f"YOLO 검출 JSON 처리 실패: {error}")
