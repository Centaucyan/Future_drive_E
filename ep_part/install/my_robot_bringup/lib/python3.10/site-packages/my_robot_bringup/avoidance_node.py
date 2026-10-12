import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from vision_msgs.msg import Detection2DArray


class AvoidanceNode(Node):
    """SP팀 감지 결과 -> 사람: 정지 유지 / 차량: 후진 -> 회전 -> 전진"""

    def __init__(self):
        super().__init__('avoidance_node')
        # 회피 동작
        self.declare_parameter('backup_dist', 0.15)     # N: 후진 거리(m)
        self.declare_parameter('turn_angle', 1.57)      # rad (약 90도)
        self.declare_parameter('turn_dir', 1)           # +1 좌, -1 우
        self.declare_parameter('forward_dist', 0.30)
        self.declare_parameter('backup_speed', 0.08)
        self.declare_parameter('turn_speed', 0.8)
        self.declare_parameter('forward_speed', 0.10)
        self.declare_parameter('cooldown', 2.0)
        # 대응 정책
        self.declare_parameter('person_stop_dist', 0.8)
        self.declare_parameter('vehicle_avoid_dist', 0.4)
        self.declare_parameter('min_conf', 0.0)            # SP(yolo)가 이미 클래스별 임계값 적용
        self.declare_parameter('detections_topic', '/yolo/detections')
        # yolo class_id: 0=4륜차, 1=2륜차, 2=사람
        self.declare_parameter('stop_class_ids', [2, 1])   # 정지하고 대기
        self.declare_parameter('avoid_class_ids', [0])     # 후진-회전-전진 회피
        self.declare_parameter('sp_timeout', 0.5)
        self.declare_parameter('require_sp', True)

        g = self.get_parameter
        self.backup_dist = g('backup_dist').value
        self.turn_angle = g('turn_angle').value
        self.turn_dir = g('turn_dir').value
        self.forward_dist = g('forward_dist').value
        self.backup_speed = g('backup_speed').value
        self.turn_speed = g('turn_speed').value
        self.forward_speed = g('forward_speed').value
        self.cooldown = g('cooldown').value

        self.state = 'IDLE'
        self.obstacle = False
        self.person_stop = False
        self.rear_blocked = False
        self.last_sp = None
        self.x = self.y = self.th = 0.0
        self.ref_x = self.ref_y = self.ref_th = 0.0
        self.cooldown_until = self.get_clock().now()

        self.create_subscription(Detection2DArray, self.get_parameter('detections_topic').value,
                                 self.cb_det, 10)
        self.create_subscription(Bool, 'emergency_stop', self.cb_estop, 10)
        self.create_subscription(Odometry, 'odom', self.cb_odom, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel_avoid', 10)
        self.create_timer(0.05, self.step)

    def cb_det(self, msg):
        self.last_sp = self.get_clock().now()   # 빈 배열도 '데이터 살아있음'으로 취급
        g = self.get_parameter
        min_conf = g('min_conf').value
        stop_ids = set(int(x) for x in g('stop_class_ids').value)
        avoid_ids = set(int(x) for x in g('avoid_class_ids').value)
        stop_d = g('person_stop_dist').value
        avoid_d = g('vehicle_avoid_dist').value

        person_stop = vehicle_avoid = False
        for det in msg.detections:
            for res in det.results:
                try:
                    cid = int(res.hypothesis.class_id)
                except ValueError:
                    continue
                if res.hypothesis.score < min_conf:
                    continue
                dist = res.pose.pose.position.z      # yolo_node: 추정 거리(m)
                if cid in stop_ids and dist < stop_d:
                    person_stop = True
                elif cid in avoid_ids and dist < avoid_d:
                    vehicle_avoid = True
        self.person_stop = person_stop
        self.obstacle = vehicle_avoid and not person_stop

    def cb_estop(self, msg):
        self.rear_blocked = msg.data

    def cb_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.th = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)

    def set_state(self, s):
        self.state = s
        self.ref_x, self.ref_y, self.ref_th = self.x, self.y, self.th
        self.get_logger().info(f'state -> {s}')

    def moved(self):
        return math.hypot(self.x - self.ref_x, self.y - self.ref_y)

    def turned(self):
        d = self.th - self.ref_th
        return abs(math.atan2(math.sin(d), math.cos(d)))

    def sp_age(self, now):
        """마지막 SP 수신 후 경과 시간(s). 한 번도 못 받았으면 None"""
        if self.last_sp is None:
            return None
        return (now - self.last_sp).nanoseconds * 1e-9

    def sp_lost(self, now):
        # require_sp가 False면 SP 끊김은 정지 사유가 아님 (개발/테스트용)
        if not self.get_parameter('require_sp').value:
            return False
        age = self.sp_age(now)
        return age is None or age > self.get_parameter('sp_timeout').value

    def step(self):
        now = self.get_clock().now()

        # 감지 데이터가 끊기면 오래된 사람/차량 상태를 해제 (정지가 영구히 남는 것 방지)
        age = self.sp_age(now)
        if age is None or age > self.get_parameter('sp_timeout').value:
            self.person_stop = False
            self.obstacle = False

    # 사람 감지 또는 SP 끊김 -> 정지 유지 (0 속도를 계속 발행해 다른 명령을 덮어씀)
        if self.person_stop or self.sp_lost(now):
            if self.state != 'IDLE':
                self.state = 'IDLE'
                self.get_logger().warn('회피 중단: 사람 감지 또는 SP 끊김')
            self.pub.publish(Twist())
            return

        cmd = Twist()

        if self.state == 'IDLE':
            if self.obstacle and now > self.cooldown_until:
                self.set_state('TURN' if self.rear_blocked else 'BACKUP')
            return   # IDLE에서는 발행하지 않음 (다른 명령에 양보)

        if self.state == 'BACKUP':
            if self.rear_blocked or self.moved() >= self.backup_dist:
                self.set_state('TURN')
            else:
                cmd.linear.x = -self.backup_speed

        elif self.state == 'TURN':
            if self.turned() >= self.turn_angle:
                self.set_state('FORWARD')
            else:
                cmd.angular.z = self.turn_dir * self.turn_speed

        elif self.state == 'FORWARD':
            if self.moved() >= self.forward_dist or self.obstacle:
                self.cooldown_until = now + Duration(seconds=self.cooldown)
                self.state = 'IDLE'
                self.get_logger().info('state -> IDLE')
            else:
                cmd.linear.x = self.forward_speed

        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = AvoidanceNode()
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
