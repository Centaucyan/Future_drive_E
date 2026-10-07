import subprocess
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import CameraInfo, CompressedImage, Image


class CameraNode(Node):
    """rpicam-vid에서 MJPEG를 받아 ROS 이미지 토픽으로 발행."""

    def __init__(self):
        super().__init__('camera_publisher')

        self.declare_parameter('frame_rate', 15.0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('publish_compressed', True)
        self.declare_parameter('publish_raw', False)
        self.declare_parameter('compressed_topic', '/image_raw/compressed')
        self.declare_parameter('raw_topic', '/camera/image_raw')
        self.declare_parameter('publish_camera_info', False)
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('fx', 554.0)
        self.declare_parameter('fy', 554.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)

        get_param = self.get_parameter

        self.width = int(get_param('frame_width').value)
        self.height = int(get_param('frame_height').value)
        self.frame_rate = float(get_param('frame_rate').value)
        self.jpeg_quality = int(get_param('jpeg_quality').value)
        self.frame_id = get_param('frame_id').value

        self.publish_compressed = get_param('publish_compressed').value
        self.publish_raw = get_param('publish_raw').value
        self.publish_camera_info = get_param('publish_camera_info').value

        self.process = None
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.frame_sequence = 0
        self.published_sequence = 0
        self.fail_count = 0

        if self.publish_compressed:
            self.compressed_pub = self.create_publisher(
                CompressedImage,
                get_param('compressed_topic').value,
                2,
            )

        if self.publish_raw:
            self.raw_pub = self.create_publisher(
                Image,
                get_param('raw_topic').value,
                2,
            )

        if self.publish_camera_info:
            self.info_pub = self.create_publisher(
                CameraInfo,
                get_param('camera_info_topic').value,
                2,
            )
            self.camera_info = CameraInfo()
            self.camera_info.width = self.width
            self.camera_info.height = self.height

            fx = float(get_param('fx').value)
            fy = float(get_param('fy').value)
            cx = float(get_param('cx').value)
            cy = float(get_param('cy').value)

            self.camera_info.k = [
                fx, 0.0, cx,
                0.0, fy, cy,
                0.0, 0.0, 1.0,
            ]
            self.camera_info.p = [
                fx, 0.0, cx, 0.0,
                0.0, fy, cy, 0.0,
                0.0, 0.0, 1.0, 0.0,
            ]
            self.camera_info.distortion_model = 'plumb_bob'
            self.camera_info.d = [0.0] * 5

        self.start_camera()

        self.create_timer(
            1.0 / max(1.0, self.frame_rate),
            self.tick,
        )

    def start_camera(self):
        self.stop_camera()

        command = [
            'rpicam-vid',
            '--timeout', '0',
            '--nopreview',
            '--codec', 'mjpeg',
            '--width', str(self.width),
            '--height', str(self.height),
            '--framerate', str(self.frame_rate),
            '--output', '-',
        ]

        try:
            self.process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                bufsize=0,
            )
        except Exception as exc:
            self.process = None
            self.get_logger().error(f'rpicam-vid 실행 실패: {exc}')
            return

        self.get_logger().info('rpicam-vid 카메라 스트림 시작')

        reader = threading.Thread(
            target=self.read_mjpeg_stream,
            args=(self.process,),
            daemon=True,
        )
        reader.start()

    def stop_camera(self):
        process = self.process
        self.process = None

        if process is not None:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()

            if process.stdout is not None:
                process.stdout.close()

    def read_mjpeg_stream(self, process):
        buffer = bytearray()

        try:
            while process.poll() is None:
                chunk = process.stdout.read(4096)
                if not chunk:
                    break

                buffer.extend(chunk)

                while True:
                    start = buffer.find(b'\xff\xd8')
                    if start < 0:
                        # JPEG 시작 마커 조각이 남을 수 있어 마지막 바이트는 보존
                        buffer = buffer[-1:]
                        break

                    if start > 0:
                        del buffer[:start]

                    end = buffer.find(b'\xff\xd9', 2)
                    if end < 0:
                        break

                    jpeg_bytes = bytes(buffer[:end + 2])
                    del buffer[:end + 2]

                    encoded = np.frombuffer(jpeg_bytes, dtype=np.uint8)
                    frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)

                    if frame is not None:
                        with self.frame_lock:
                            self.latest_frame = frame
                            self.frame_sequence += 1

        except Exception as exc:
            self.get_logger().warning(f'MJPEG 스트림 읽기 종료: {exc}')

    def tick(self):
        if self.process is None or self.process.poll() is not None:
            self.fail_count += 1
            if self.fail_count % 15 == 0:
                self.get_logger().warning('rpicam-vid 재시작')
                self.start_camera()
            return

        with self.frame_lock:
            if (
                self.latest_frame is None
                or self.frame_sequence == self.published_sequence
            ):
                frame = None
            else:
                frame = self.latest_frame.copy()
                self.published_sequence = self.frame_sequence

        if frame is None:
            self.fail_count += 1
            if self.fail_count % 30 == 1:
                self.get_logger().warning('카메라 프레임을 받지 못했습니다')
            if self.fail_count % 15 == 0:
                self.start_camera()
            return

        self.fail_count = 0

        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height))
        
        frame = cv2.rotate(frame, cv2.ROTATE_180)

        stamp = self.get_clock().now().to_msg()

        if self.publish_compressed:
            ok, encoded = cv2.imencode(
                '.jpg',
                frame,
                [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
            )
            if ok:
                msg = CompressedImage()
                msg.header.stamp = stamp
                msg.header.frame_id = self.frame_id
                msg.format = 'jpeg'
                msg.data = encoded.tobytes()
                self.compressed_pub.publish(msg)

        if self.publish_raw:
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            msg.height = frame.shape[0]
            msg.width = frame.shape[1]
            msg.encoding = 'bgr8'
            msg.is_bigendian = 0
            msg.step = frame.shape[1] * 3
            msg.data = frame.tobytes()
            self.raw_pub.publish(msg)

        if self.publish_camera_info:
            self.camera_info.header.stamp = stamp
            self.camera_info.header.frame_id = self.frame_id
            self.info_pub.publish(self.camera_info)

    def destroy_node(self):
        self.stop_camera()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()