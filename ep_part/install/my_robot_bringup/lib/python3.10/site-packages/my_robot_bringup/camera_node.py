import cv2
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


class CameraNode(Node):
    """USB 카메라/영상 파일 -> 압축 이미지(기본) 및 원본 이미지(옵션) 발행.

    - /image_raw/compressed (CompressedImage) : lane_detection_node 입력, 네트워크 전송용
    - /camera/image_raw (Image, bgr8)         : yolo_node 입력 (publish_raw:=true 일 때)
    """

    def __init__(self):
        super().__init__('camera_publisher')
        self.declare_parameter('video_source', '0', ParameterDescriptor(dynamic_typing=True))
        self.declare_parameter('frame_rate', 15.0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('loop_video', True)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('publish_compressed', True)
        self.declare_parameter('publish_raw', False)
        self.declare_parameter('compressed_topic', '/image_raw/compressed')
        self.declare_parameter('raw_topic', '/camera/image_raw')
        # 실제 캘리브레이션 값이 있을 때만 켜세요 (yolo_node가 이 값으로 거리 계산)
        self.declare_parameter('publish_camera_info', False)
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('fx', 554.0)
        self.declare_parameter('fy', 554.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)

        g = self.get_parameter
        self.w = g('frame_width').value
        self.h = g('frame_height').value
        self.quality = g('jpeg_quality').value
        self.loop = g('loop_video').value
        self.frame_id = g('frame_id').value
        self.pub_comp = g('publish_compressed').value
        self.pub_raw = g('publish_raw').value
        self.pub_info = g('publish_camera_info').value

        # launch에서 "'0'"처럼 따옴표가 붙어 와도 처리
        src = str(g('video_source').value).strip().strip("'\"")
        self.is_file = not src.isdigit()
        self.src = src if self.is_file else int(src)
        self.cap = None
        self.open_capture()

        if self.pub_comp:
            self.comp_pub = self.create_publisher(CompressedImage, g('compressed_topic').value, 2)
        if self.pub_raw:
            self.raw_pub = self.create_publisher(Image, g('raw_topic').value, 2)
        if self.pub_info:
            self.info_pub = self.create_publisher(CameraInfo, g('camera_info_topic').value, 2)
            self.info = CameraInfo()
            self.info.width, self.info.height = self.w, self.h
            fx, fy, cx, cy = (g(k).value for k in ('fx', 'fy', 'cx', 'cy'))
            self.info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            self.info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
            self.info.distortion_model = 'plumb_bob'
            self.info.d = [0.0] * 5

        self.fail = 0
        self.create_timer(1.0 / max(1.0, g('frame_rate').value), self.tick)

    def open_capture(self):
        if self.cap is not None:
            self.cap.release()
        if self.is_file:
            self.cap = cv2.VideoCapture(self.src)
        else:
            self.cap = cv2.VideoCapture(self.src, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'BGR3'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
        if not self.cap.isOpened():
            self.get_logger().error(f'카메라/영상 열기 실패: {self.src}')

    def tick(self):
        ok, frame = self.cap.read() if self.cap.isOpened() else (False, None)
        if not ok:
            if self.is_file and self.loop:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                return
            self.fail += 1
            if self.fail % 30 == 1:
                self.get_logger().warn('프레임 읽기 실패, 재연결 시도')
            if self.fail % 15 == 0:
                self.open_capture()
            return
        self.fail = 0

        if frame.shape[1] != self.w or frame.shape[0] != self.h:
            frame = cv2.resize(frame, (self.w, self.h))

        stamp = self.get_clock().now().to_msg()

        if self.pub_comp:
            ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, int(self.quality)])
            if ok:
                m = CompressedImage()
                m.header.stamp, m.header.frame_id = stamp, self.frame_id
                m.format = 'jpeg'
                m.data = buf.tobytes()
                self.comp_pub.publish(m)

        if self.pub_raw:
            m = Image()
            m.header.stamp, m.header.frame_id = stamp, self.frame_id
            m.height, m.width = frame.shape[0], frame.shape[1]
            m.encoding = 'bgr8'
            m.is_bigendian = 0
            m.step = frame.shape[1] * 3
            m.data = frame.tobytes()
            self.raw_pub.publish(m)

        if self.pub_info:
            self.info.header.stamp, self.info.header.frame_id = stamp, self.frame_id
            self.info_pub.publish(self.info)


def main():
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.cap is not None:
            node.cap.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
