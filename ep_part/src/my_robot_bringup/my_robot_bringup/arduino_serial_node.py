import math
import time

import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('wheel_radius', 0.032)
        self.declare_parameter('wheel_separation', 0.165)
        self.declare_parameter('ticks_per_rev', 4096.0)
        self.declare_parameter('rotation_scale', 1.0)
        self.declare_parameter('max_raw', 250)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        p = self.get_parameter
        self.r = p('wheel_radius').value
        self.sep = p('wheel_separation').value
        self.m_per_tick = 2.0 * math.pi * self.r / p('ticks_per_rev').value
        self.rot_scale = p('rotation_scale').value
        self.max_raw = p('max_raw').value
        self.odom_frame = p('odom_frame').value
        self.base_frame = p('base_frame').value

        self.ser = serial.Serial(p('port').value, 115200, timeout=0)
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        self.buf = b''

        self.x = self.y = self.th = 0.0
        self.prev = None   # (ms, left_tick, right_tick)

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.range_pub = self.create_publisher(Range, 'ultrasonic', 10)
        self.estop_pub = self.create_publisher(Bool, 'emergency_stop', 10)
        self.tf_br = TransformBroadcaster(self)
        self.create_subscription(Twist, 'cmd_vel', self.cb_cmd, 10)
        self.create_timer(0.005, self.read_serial)

    # ---- cmd_vel -> V,좌,우 ----
    def to_raw(self, wheel_v):
        rpm = wheel_v / self.r * 60.0 / (2.0 * math.pi)
        return rpm / 0.229

    def cb_cmd(self, msg):
        v, w = msg.linear.x, msg.angular.z
        l = self.to_raw(v - w * self.sep / 2.0)
        r = self.to_raw(v + w * self.sep / 2.0)
        m = max(abs(l), abs(r))
        if m > self.max_raw:
            s = self.max_raw / m
            l, r = l * s, r * s
        self.ser.write(f'V,{int(l)},{int(r)}\n'.encode())

    # ---- 수신 ----
    def read_serial(self):
        n = self.ser.in_waiting
        if n:
            self.buf += self.ser.read(n)
        while b'\n' in self.buf:
            line, self.buf = self.buf.split(b'\n', 1)
            self.handle_line(line.decode(errors='ignore').strip())

    def handle_line(self, line):
        try:
            if line.startswith('F,'):
                _, ms, l, r = line.split(',')
                self.update_odom(int(ms), int(l), int(r))
            elif line.startswith('U,'):
                _, ms, mm, es = line.split(',')
                self.publish_range(int(mm))
                self.estop_pub.publish(Bool(data=bool(int(es))))
        except ValueError:
            pass

    def publish_range(self, mm):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ultrasonic_link'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26
        msg.min_range = 0.02
        msg.max_range = 2.0
        msg.range = float('inf') if mm < 0 else mm / 1000.0
        self.range_pub.publish(msg)

    # ---- odom ----
    def update_odom(self, ms, l, r):
        if self.prev is None:
            self.prev = (ms, l, r)
            return
        pms, pl, pr = self.prev
        self.prev = (ms, l, r)
        dt = ((ms - pms) & 0xFFFFFFFF) / 1000.0
        if dt <= 0.0:
            return

        dl = (l - pl) * self.m_per_tick
        dr = (r - pr) * self.m_per_tick
        ds = (dl + dr) / 2.0
        dth = (dr - dl) / self.sep * self.rot_scale

        self.x += ds * math.cos(self.th + dth / 2.0)
        self.y += ds * math.sin(self.th + dth / 2.0)
        self.th += dth

        now = self.get_clock().now().to_msg()
        qz, qw = math.sin(self.th / 2.0), math.cos(self.th / 2.0)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = ds / dt
        odom.twist.twist.angular.z = dth / dt
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.1
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[35] = 0.1
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_br.sendTransform(tf)


def main():
    rclpy.init()
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.ser.write(b'V,0,0\n')
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()