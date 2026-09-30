#!/usr/bin/env python3
"""
시각화 노드
- /yolo/result_image를 구독하여 화면에 표시
- /yolo/detections_json을 구독하여 검출 로그 출력
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json


class VisualizationNode(Node):
    """YOLO 검출 결과를 시각화하는 노드"""

    def __init__(self):
        super().__init__('visualization_node')

        self.bridge = CvBridge()

        # ── 구독자: 검출 결과 이미지 ──
        self.image_sub = self.create_subscription(
            Image,
            '/yolo/result_image',
            self.image_callback,
            10
        )

        # ── 구독자: JSON 검출 결과 ──
        self.json_sub = self.create_subscription(
            String,
            '/yolo/detections_json',
            self.json_callback,
            10
        )

        self.get_logger().info('🖥️  VisualizationNode 시작 - 결과 시각화 중...')

    def image_callback(self, msg: Image):
        """검출 결과 이미지 수신 및 화면 표시"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow('YOLO Detection Result', cv_image)
            key = cv2.waitKey(1) & 0xFF

            # 'q' 키를 누르면 종료
            if key == ord('q'):
                self.get_logger().info('🛑 사용자가 종료를 요청했습니다.')
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f'❌ 시각화 오류: {e}')

    def json_callback(self, msg: String):
        """JSON 검출 결과 로그 출력"""
        try:
            data = json.loads(msg.data)
            num = data.get('num_detections', 0)
            inference_ms = data.get('inference_time_ms', 0)

            if num > 0:
                detections = data.get('detections', [])
                objects = [f"{d['class_name']}({d['confidence']})" for d in detections]
                self.get_logger().info(
                    f'🎯 검출 {num}개 ({inference_ms}ms): {", ".join(objects)}'
                )
        except Exception as e:
            self.get_logger().error(f'❌ JSON 파싱 오류: {e}')

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 사용자에 의해 종료됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
