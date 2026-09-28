#!/usr/bin/env python3
"""
카메라/영상 퍼블리셔 노드
- 카메라(웹캠) 또는 영상 파일에서 프레임을 읽어 ROS2 토픽으로 발행
- 발행 토픽: /camera/image_raw (sensor_msgs/Image)
"""

from networkx.generators import spectral_graph_forge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraPublisher(Node):
    """카메라 또는 영상 파일에서 프레임을 캡처하여 ROS2 토픽으로 발행하는 노드"""

    def __init__(self):
        super().__init__('camera_publisher')

        # ── 파라미터 선언 ──
        self.declare_parameter('video_source', "0")       # 0: 웹캠, 또는 영상 파일 경로
        self.declare_parameter('frame_rate', 30.0)       # 발행 프레임 레이트 (Hz)
        self.declare_parameter('frame_width', 640)       # 프레임 너비
        self.declare_parameter('frame_height', 480)      # 프레임 높이

        # 파라미터 값 가져오기
        video_source = self.get_parameter('video_source').value

        # 숫자로 입력되면 웹캠 장치 번호로 변환
        if isinstance(video_source, str) and video_source.isdigit():
            video_source = int(video_source)

        frame_rate = self.get_parameter('frame_rate').value
        frame_width = self.get_parameter('frame_width').value
        frame_height = self.get_parameter('frame_height').value

        # ── 퍼블리셔 생성 ──
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)

        # ── OpenCV 비디오 캡처 설정 ──
        self.bridge = CvBridge()

        # video_source가 숫자면 카메라, 문자열이면 영상 파일
        if isinstance(video_source, int):
            self.cap = cv2.VideoCapture(video_source)
            self.get_logger().info(f'📷 카메라 장치 [{video_source}] 을 열었습니다.')
        else:
            self.cap = cv2.VideoCapture(video_source)
            self.get_logger().info(f'🎬 영상 파일 [{video_source}] 을 열었습니다.')

        if not self.cap.isOpened():
            self.get_logger().error('❌ 비디오 소스를 열 수 없습니다!')
            return

        # 카메라 해상도 설정
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f'📐 프레임 해상도: {actual_w}x{actual_h}')

        # 영상 파일인 경우 루프 재생 여부
        self.is_video_file = isinstance(video_source, str)
        self.declare_parameter('loop_video', True)
        self.loop_video = self.get_parameter('loop_video').get_parameter_value().bool_value

        # ── 타이머로 주기적 프레임 발행 ──
        timer_period = 1.0 / frame_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.frame_count = 0
        self.get_logger().info(f'✅ CameraPublisher 시작 (발행 주기: {frame_rate} Hz)')

    def timer_callback(self):
        """타이머 콜백: 프레임을 읽어 ROS2 토픽으로 발행"""
        ret, frame = self.cap.read()

        if not ret:
            if self.is_video_file and self.loop_video:
                # 영상 파일 루프 재생
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().warn('⚠️ 영상 파일 재시작 실패')
                    return
                self.get_logger().info('🔄 영상 파일 처음부터 다시 재생')
            else:
                self.get_logger().warn('⚠️ 프레임을 읽을 수 없습니다.')
                return

        # OpenCV 이미지 → ROS2 Image 메시지 변환
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'

        self.publisher_.publish(msg)
        self.frame_count += 1

        if self.frame_count % 100 == 0:
            self.get_logger().info(f'📤 {self.frame_count}번째 프레임 발행 완료')

    def destroy_node(self):
        """노드 종료 시 카메라 리소스 해제"""
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info('📷 카메라 리소스 해제 완료')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 사용자에 의해 종료됨')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
