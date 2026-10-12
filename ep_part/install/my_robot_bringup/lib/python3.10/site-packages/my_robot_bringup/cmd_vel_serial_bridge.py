#!/usr/bin/env python3

import math
import time
from typing import Optional

import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

import serial
from serial import SerialException


class CmdVelSerialBridge(Node):
    """Translate ROS 2 cmd_vel messages into Arduino W/S/A/D/X commands."""

    def __init__(self) -> None:
        super().__init__('cmd_vel_serial_bridge')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('cmd_timeout_sec', 0.75)
        self.declare_parameter('heartbeat_sec', 1.0)
        self.declare_parameter('linear_deadband', 0.02)
        self.declare_parameter('angular_deadband', 0.05)
        self.declare_parameter('rear_stop_distance_m', 0.10)
        self.declare_parameter('rear_release_distance_m', 0.12)
        self.declare_parameter('ultrasonic_timeout_sec', 1.0)
        self.declare_parameter('wheel_diameter_m', 0.065)
        self.declare_parameter('wheel_separation_m', 0.115)
        self.declare_parameter('encoder_timeout_sec', 0.75)
        self.declare_parameter('tf_time_offset_sec', 0.10)

        self.serial_port = str(self.get_parameter('serial_port').value)
        self.baud_rate = int(self.get_parameter('baud_rate').value)
        self.cmd_timeout_sec = float(
            self.get_parameter('cmd_timeout_sec').value
        )
        self.heartbeat_sec = float(
            self.get_parameter('heartbeat_sec').value
        )
        self.linear_deadband = float(
            self.get_parameter('linear_deadband').value
        )
        self.angular_deadband = float(
            self.get_parameter('angular_deadband').value
        )
        self.rear_stop_distance_m = float(
            self.get_parameter('rear_stop_distance_m').value
        )
        self.rear_release_distance_m = float(
            self.get_parameter('rear_release_distance_m').value
        )
        self.ultrasonic_timeout_sec = float(
            self.get_parameter('ultrasonic_timeout_sec').value
        )
        self.wheel_diameter_m = float(
            self.get_parameter('wheel_diameter_m').value
        )
        self.wheel_separation_m = float(
            self.get_parameter('wheel_separation_m').value
        )
        self.encoder_timeout_sec = float(
            self.get_parameter('encoder_timeout_sec').value
        )
        self.tf_time_offset_sec = float(
            self.get_parameter('tf_time_offset_sec').value
        )

        self.serial_connection: Optional[serial.Serial] = None
        self.serial_receive_buffer = bytearray()
        self.serial_ready_at = 0.0
        self.last_connect_attempt = 0.0
        self.last_cmd_vel_time = 0.0
        self.last_serial_send_time = 0.0
        self.desired_command = 'X'
        self.last_sent_command: Optional[str] = None
        self.reverse_turn_warning_shown = False
        self.last_ultrasonic_time = 0.0
        self.rear_distance_m = float('inf')
        self.rear_obstacle_blocked = False
        self.reverse_safety_message_shown = False
        self.encoder_rpm = [0.0, 0.0, 0.0, 0.0]
        self.encoder_data_received = False
        self.last_encoder_time = 0.0
        self.last_odometry_time = time.monotonic()
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        self.command_publisher = self.create_publisher( #아두이노 명령 토픽
            String,
            '/arduino/command',
            10,
        )
        self.feedback_publisher = self.create_publisher( #아두이노 연결 관련해서 피드백 주는 토픽
            String,
            '/arduino/feedback',
            10,
        )
        self.ultrasonic_publisher = self.create_publisher( #초음파 토픽
            Range,
            '/ultrasonic/range',
            10,
        )
        self.odometry_publisher = self.create_publisher( #오돔 토픽 발행
            Odometry,
            '/odom',
            20,
        )
        self.transform_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # A short timer gives quick response while the heartbeat remains slow.
        self.create_timer(0.05, self.timer_callback)

        self.get_logger().info(
            f'Bridge ready: /cmd_vel -> {self.serial_port} '
            f'({self.baud_rate} baud)'
        )

    def cmd_vel_callback(self, message: Twist) -> None:
        self.last_cmd_vel_time = time.monotonic()
        self.desired_command = self.twist_to_command(
            message.linear.x,
            message.angular.z,
        )

    def twist_to_command(self, linear_x: float, angular_z: float) -> str:
        linear_active = abs(linear_x) > self.linear_deadband
        angular_active = abs(angular_z) > self.angular_deadband

        if not linear_active and not angular_active:
            return 'X'

        # The current Arduino protocol has no separate reverse-curve command.
        # For safety, reverse cmd_vel requests remain straight backward.
        if linear_x < -self.linear_deadband:
            if angular_active and not self.reverse_turn_warning_shown:
                self.get_logger().warning(
                    'Reverse turning is not supported by the current Arduino '
                    'W/S/A/D/X protocol; sending S.'
                )
                self.reverse_turn_warning_shown = True
            return 'S'

        self.reverse_turn_warning_shown = False

        # ROS uses positive angular.z for left turns.
        if angular_z > self.angular_deadband:
            return 'A'
        if angular_z < -self.angular_deadband:
            return 'D'
        if linear_x > self.linear_deadband:
            return 'W'

        return 'X'

    def timer_callback(self) -> None:
        now = time.monotonic()

        if self.serial_connection is None:
            if now - self.last_connect_attempt >= 2.0:
                self.last_connect_attempt = now
                self.connect_serial()
            self.update_odometry(now)
            return

        if now < self.serial_ready_at:
            self.update_odometry(now)
            return

        self.read_serial_feedback()

        if now - self.last_cmd_vel_time > self.cmd_timeout_sec:
            command = 'X'
        else:
            command = self.desired_command

        # The ultrasonic sensor faces backward. Only reverse motion is blocked.
        if command == 'S' and self.reverse_is_unsafe(now):
            command = 'X'
            if not self.reverse_safety_message_shown:
                self.get_logger().warning(
                    'REAR SAFETY STOP: reverse motion is blocked.'
                )
                self.reverse_safety_message_shown = True
        else:
            self.reverse_safety_message_shown = False

        command_changed = command != self.last_sent_command
        # Moving commands are repeated to refresh the Arduino safety timer.
        # X only needs to be sent once, which avoids repeated STOP logs.
        heartbeat_due = (
            command != 'X'
            and now - self.last_serial_send_time >= self.heartbeat_sec
        )

        if command_changed or heartbeat_due:
            self.send_serial_command(command)

        self.update_odometry(now)

    def reverse_is_unsafe(self, now: float) -> bool:
        """Return True when rear sensor data is stale or an obstacle is close."""
        if now - self.last_ultrasonic_time > self.ultrasonic_timeout_sec:
            return True

        return self.rear_obstacle_blocked

    def connect_serial(self) -> None:
        try:
            self.serial_connection = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=0,
                write_timeout=0.2,
            )
            # Opening an Uno serial port normally resets the board.
            self.serial_ready_at = time.monotonic() + 2.0
            self.serial_receive_buffer.clear()
            self.last_sent_command = None
            self.get_logger().info(f'Connected to {self.serial_port}')
        except (SerialException, OSError) as error:
            self.serial_connection = None
            self.get_logger().warning(
                f'Waiting for {self.serial_port}: {error}'
            )

    def disconnect_serial(self, error: Exception) -> None:
        self.get_logger().error(f'Serial connection lost: {error}')
        if self.serial_connection is not None:
            try:
                self.serial_connection.close()
            except (SerialException, OSError):
                pass
        self.serial_connection = None
        self.serial_receive_buffer.clear()
        self.last_sent_command = None

    def send_serial_command(self, command: str) -> None:
        if self.serial_connection is None:
            return

        try:
            self.serial_connection.write((command + '\n').encode('ascii'))
            self.serial_connection.flush()
            self.last_sent_command = command
            self.last_serial_send_time = time.monotonic()

            message = String()
            message.data = command
            self.command_publisher.publish(message)

            self.get_logger().info(f'Arduino command: {command}')
        except (SerialException, OSError) as error:
            self.disconnect_serial(error)

    def read_serial_feedback(self) -> None:
        if self.serial_connection is None:
            return

        try:
            # Non-blocking serial reads may split one Arduino line into several
            # chunks. Keep incomplete bytes until the terminating newline.
            available = self.serial_connection.in_waiting
            if available > 0:
                received = self.serial_connection.read(min(available, 4096))
                self.serial_receive_buffer.extend(received)

            if len(self.serial_receive_buffer) > 8192:
                self.get_logger().warning(
                    'Serial receive buffer overflow; discarding partial data.'
                )
                self.serial_receive_buffer.clear()
                return

            for _ in range(20):
                newline_index = self.serial_receive_buffer.find(b'\n')
                if newline_index < 0:
                    break

                raw_line = bytes(
                    self.serial_receive_buffer[:newline_index]
                )
                del self.serial_receive_buffer[:newline_index + 1]
                line = raw_line.decode('utf-8', errors='replace').strip()
                if not line:
                    continue

                message = String()
                message.data = line
                self.feedback_publisher.publish(message)

                if line.startswith('ULTRASONIC_CM:'):
                    self.publish_ultrasonic_range(line)
                elif line.startswith('ENCODER_RPM:'):
                    self.update_encoder_rpm(line)
                else:
                    self.get_logger().info(f'Arduino feedback: {line}')
        except (SerialException, OSError) as error:
            self.disconnect_serial(error)

    def update_encoder_rpm(self, line: str) -> None:
        try:
            values = [
                float(value)
                for value in line.split(':', 1)[1].split(',')
            ]
        except (IndexError, ValueError):
            self.get_logger().warning(
                f'Invalid encoder RPM message from Arduino: {line}'
            )
            return

        if len(values) != 4:
            self.get_logger().warning(
                f'Expected four encoder RPM values: {line}'
            )
            return

        self.encoder_rpm = values
        self.last_encoder_time = time.monotonic()
        self.encoder_data_received = True

    def update_odometry(self, now: float) -> None:
        dt = now - self.last_odometry_time
        self.last_odometry_time = now

        if not self.encoder_data_received or dt <= 0.0:
            return

        if now - self.last_encoder_time > self.encoder_timeout_sec:
            wheel_rpm = [0.0, 0.0, 0.0, 0.0]
        else:
            wheel_rpm = self.encoder_rpm

        # M1/M4 are left wheels and M2/M3 are right wheels.
        left_rpm = (wheel_rpm[0] + wheel_rpm[3]) / 2.0
        right_rpm = (wheel_rpm[1] + wheel_rpm[2]) / 2.0
        wheel_circumference = math.pi * self.wheel_diameter_m
        left_velocity = left_rpm * wheel_circumference / 60.0
        right_velocity = right_rpm * wheel_circumference / 60.0
        linear_velocity = (left_velocity + right_velocity) / 2.0
        angular_velocity = (
            right_velocity - left_velocity
        ) / self.wheel_separation_m

        delta_yaw = angular_velocity * dt
        heading_midpoint = self.odom_yaw + delta_yaw / 2.0
        self.odom_x += linear_velocity * math.cos(heading_midpoint) * dt
        self.odom_y += linear_velocity * math.sin(heading_midpoint) * dt
        self.odom_yaw += delta_yaw
        self.odom_yaw = math.atan2(
            math.sin(self.odom_yaw),
            math.cos(self.odom_yaw),
        )

        clock_now = self.get_clock().now()
        stamp = clock_now.to_msg()
        quaternion_z = math.sin(self.odom_yaw / 2.0)
        quaternion_w = math.cos(self.odom_yaw / 2.0)

        message = Odometry() #여기 아래가 오돔을 어떻게 발행하는지
        message.header.stamp = stamp
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_link'
        message.pose.pose.position.x = self.odom_x
        message.pose.pose.position.y = self.odom_y
        message.pose.pose.orientation.z = quaternion_z
        message.pose.pose.orientation.w = quaternion_w
        message.twist.twist.linear.x = linear_velocity
        message.twist.twist.angular.z = angular_velocity
        message.pose.covariance[0] = 0.02
        message.pose.covariance[7] = 0.02
        message.pose.covariance[14] = 1000000.0
        message.pose.covariance[21] = 1000000.0
        message.pose.covariance[28] = 1000000.0
        message.pose.covariance[35] = 0.05
        message.twist.covariance[0] = 0.03
        message.twist.covariance[7] = 0.03
        message.twist.covariance[14] = 1000000.0
        message.twist.covariance[21] = 1000000.0
        message.twist.covariance[28] = 1000000.0
        message.twist.covariance[35] = 0.08
        self.odometry_publisher.publish(message)

        transform = TransformStamped()
        transform.header.stamp = (
            clock_now + Duration(seconds=self.tf_time_offset_sec)
        ).to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.odom_x
        transform.transform.translation.y = self.odom_y
        transform.transform.rotation.z = quaternion_z
        transform.transform.rotation.w = quaternion_w
        self.transform_broadcaster.sendTransform(transform)

    def publish_ultrasonic_range(self, line: str) -> None:
        try:
            distance_cm = float(line.split(':', 1)[1])
        except (IndexError, ValueError):
            self.get_logger().warning(
                f'Invalid ultrasonic message from Arduino: {line}'
            )
            return

        message = Range()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'rear_ultrasonic_link'
        message.radiation_type = Range.ULTRASOUND
        message.field_of_view = 0.26
        message.min_range = 0.02
        message.max_range = 4.0

        if distance_cm < 0.0:
            message.range = float('inf')
        else:
            message.range = distance_cm / 100.0

        self.last_ultrasonic_time = time.monotonic()
        self.rear_distance_m = message.range

        # Hysteresis prevents rapid stop/release switching around exactly 10 cm.
        if message.range <= self.rear_stop_distance_m:
            self.rear_obstacle_blocked = True
        elif message.range > self.rear_release_distance_m:
            self.rear_obstacle_blocked = False

        self.ultrasonic_publisher.publish(message)

    def destroy_node(self) -> bool:
        if self.serial_connection is not None:
            try:
                self.serial_connection.write(b'X\n')
                self.serial_connection.flush()
                self.serial_connection.close()
            except (SerialException, OSError):
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelSerialBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
