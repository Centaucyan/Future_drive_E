# app/models/image_stream_model.py

import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImageRawSubscriber(Node):
    def __init__(self):
        super().__init__("fastapi_image_raw_subscriber")

        self.bridge = CvBridge()

        self.latest_frame = None
        self.frame_lock = threading.Lock()

        self.subscription = self.create_subscription(
            Image,
            "/image_raw",
            self.image_callback,
            10,
        )

        self.get_logger().info("Subscribed to /image_raw")

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )

            success, jpeg = cv2.imencode(
                ".jpg",
                cv_image,
                [cv2.IMWRITE_JPEG_QUALITY, 80],
            )

            if not success:
                return

            with self.frame_lock:
                self.latest_frame = jpeg.tobytes()

        except Exception as e:
            self.get_logger().error(
                f"image_raw convert error: {e}"
            )

    def get_frame(self):
        with self.frame_lock:
            return self.latest_frame


class ImageStreamModel:
    def __init__(self):
        self.node = None
        self.ros_thread = None
        self.running = False

    def start(self):
        if self.running:
            return

        self.running = True

        if not rclpy.ok():
            rclpy.init()

        self.node = ImageRawSubscriber()

        self.ros_thread = threading.Thread(
            target=self._spin,
            daemon=True,
        )
        self.ros_thread.start()

    def _spin(self):
        try:
            rclpy.spin(self.node)
        except Exception as e:
            print(f"ROS spin error: {e}")

    def stop(self):
        self.running = False

        if self.node:
            self.node.destroy_node()
            self.node = None

        if rclpy.ok():
            rclpy.shutdown()

    def generate_mjpeg(self):
        while self.running:
            if self.node is None:
                time.sleep(0.1)
                continue

            frame = self.node.get_frame()

            if frame is None:
                time.sleep(0.03)
                continue

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n"
                + frame
                + b"\r\n"
            )

            # 웹 전송 FPS 제한이 필요하면 조절
            time.sleep(0.03)


image_stream_model = ImageStreamModel()