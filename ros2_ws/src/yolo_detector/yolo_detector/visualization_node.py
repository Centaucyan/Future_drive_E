#!/usr/bin/env python3
"""
시각화 노드
- /yolo/result_image를 구독하여 화면에 표시
- /yolo/detections_json을 구독하여 검출 로그 출력
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32MultiArray
from cv_bridge import CvBridge
import cv2
import json
import numpy as np


class VisualizationNode(Node):
    """YOLO 검출 결과를 시각화하는 노드"""

    def __init__(self):
        super().__init__('visualization_node')

        self.bridge = CvBridge()

        self.lane_data=None

        # OpenCV 창 크기 설정
        cv2.namedWindow('YOLO Detection Result',cv2.WINDOW_NORMAL)
        cv2.resizeWindow('YOLO Detection Result',960,540)

        # ── 구독자: 검출 결과 이미지 ──
        self.image_sub = self.create_subscription(
            Image,
            '/yolo/result_image',
            self.image_callback,
            1
        )

        # ── 구독자: JSON 검출 결과 ──
        self.json_sub = self.create_subscription(
            String,
            '/yolo/detections_json',
            self.json_callback,
            1
        )

        # ── 구독자: 차선 검출 결과 ──
        self.lane_sub = self.create_subscription(
            Float32MultiArray,
            '/lane/result',
            self.lane_callback,
            1
        )

        self.get_logger().info('🖥️  VisualizationNode 시작 - 결과 시각화 중...')

    def image_callback(self,msg: Image):
        """YOLO 검출 결과 이미지에 차선 표시"""
        try:
            cv_image=self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

            if self.lane_data is not None and self.lane_data[0] > 0.5:
                left_points=[]
                right_points=[]

                # [valid, center, offset, left_bottom, right_bottom] 이후 차선 좌표
                for i in range(20):
                    j=5+i*4
                    left_points.append([
                        int(self.lane_data[j]),
                        int(self.lane_data[j+1])
                    ])
                    right_points.append([
                        int(self.lane_data[j+2]),
                        int(self.lane_data[j+3])
                    ])

                cv2.polylines(
                    cv_image,[np.array(left_points)],
                    False,(0,255,0),3
                )
                cv2.polylines(
                    cv_image,[np.array(right_points)],
                    False,(0,255,0),3
                )

            cv2.imshow('YOLO Detection Result',cv_image)
            key=cv2.waitKey(1)&0xFF

            # 'q' 키를 누르면 종료
            if key==ord('q'):
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
                objects = []
                for d in detections:
                    dist_str = f", {d['distance_m']}m" if 'distance_m' in d else ""
                    objects.append(f"{d['class_name']}({d['confidence']}{dist_str})")
                self.get_logger().info(
                    f'🎯 검출 {num}개 ({inference_ms}ms): {", ".join(objects)}'
                )
        except Exception as e:
            self.get_logger().error(f'❌ JSON 파싱 오류: {e}')

    def lane_callback(self, msg):
        if msg.data[0] > 0.5:
            self.lane_data=msg.data

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