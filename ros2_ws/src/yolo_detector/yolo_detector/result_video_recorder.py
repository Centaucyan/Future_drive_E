#!/usr/bin/env python3
"""Save /yolo/result_image directly to a timestamp-aligned AVI file."""

import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class ResultVideoRecorder(Node):
    def __init__(self):
        super().__init__('result_video_recorder')
        self.declare_parameter('image_topic', '/yolo/result_image')
        self.declare_parameter('output_path', '/tmp/yolo_result.avi')
        self.declare_parameter('output_fps', 30.0)

        topic = self.get_parameter('image_topic').value
        self.output_path = os.path.abspath(self.get_parameter('output_path').value)
        self.output_fps = float(self.get_parameter('output_fps').value)
        if self.output_fps <= 0.0:
            raise ValueError('output_fps must be greater than zero')

        output_dir = os.path.dirname(self.output_path)
        os.makedirs(output_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.writer = None
        self.first_stamp = None
        self.last_frame = None
        self.written_frames = 0
        self.subscription = self.create_subscription(Image, topic, self.image_callback, 10)
        self.get_logger().info(f'녹화 대기: {topic} -> {self.output_path}')

    @staticmethod
    def _stamp_seconds(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'이미지 변환 실패: {exc}')
            return

        height, width = frame.shape[:2]
        if self.writer is None:
            fourcc = cv2.VideoWriter_fourcc(*'MJPG')
            self.writer = cv2.VideoWriter(
                self.output_path, fourcc, self.output_fps, (width, height)
            )
            if not self.writer.isOpened():
                self.get_logger().error(f'영상 파일을 열 수 없음: {self.output_path}')
                self.writer = None
                return
            self.first_stamp = self._stamp_seconds(msg)
            self.get_logger().info(
                f'녹화 시작: {width}x{height} @ {self.output_fps:.1f} FPS'
            )

        stamp = self._stamp_seconds(msg)
        target_index = max(0, int(round((stamp - self.first_stamp) * self.output_fps)))

        # 처리 지연으로 토픽 프레임이 빠졌다면 직전 프레임을 복제해 실제 시간을 유지한다.
        while self.last_frame is not None and self.written_frames < target_index:
            self.writer.write(self.last_frame)
            self.written_frames += 1

        if self.written_frames <= target_index:
            self.writer.write(frame)
            self.written_frames += 1
        self.last_frame = frame

    def destroy_node(self):
        if self.writer is not None:
            self.writer.release()
            duration = self.written_frames / self.output_fps
            self.get_logger().info(
                f'녹화 저장 완료: {self.output_path} '
                f'({self.written_frames} frames, {duration:.1f}s)'
            )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ResultVideoRecorder()
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
