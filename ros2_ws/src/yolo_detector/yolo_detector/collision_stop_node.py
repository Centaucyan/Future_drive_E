import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class CollisionStopNode(Node):

    def __init__(self):
        super().__init__('collision_stop_node')

        # 현재 충돌 위험 상태
        self.danger = False
        self.ex_danger = False

        # 마지막으로 받은 Nav2 속도 명령
        self.last_cmd = Twist()

        # /cmd_vel_nav 수신
        self.cmd_vel_nav_sub = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.cmd_vel_nav_callback,
            10
        )

        # /collision_warning 수신
        self.collision_sub = self.create_subscription(
            String,
            '/collision_warning',
            self.collision_warning_callback,
            10
        )

        # 최종 차량 제어 명령
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.get_logger().info(
            '🛡️ Collision Stop Node 시작'
        )
        self.get_logger().info(
            '📥 /cmd_vel_nav, /collision_warning 구독'
        )
        self.get_logger().info(
            '📤 /cmd_vel 발행'
        )

    def cmd_vel_nav_callback(self, msg):
        """
        Nav2의 정상 주행 명령을 받는다.
        danger 상태에 따라 전달 또는 정지한다.
        """

        self.last_cmd = msg

        if self.danger:
            self.publish_stop()
        else:
            self.cmd_vel_pub.publish(self.last_cmd)

    def collision_warning_callback(self, msg):
        """
        lidar_yahboom.py에서 보내는 JSON을 처리한다.
        """
        
        try:
            data = json.loads(msg.data)

            self.danger = bool(data.get('danger', False))

            distance = data.get('distance', -1.0)
            relative_speed = data.get('relative_speed', 0.0)
            object_class = data.get('object_class', '')

            if self.danger:

                self.get_logger().warn(
                    f'🚨 비상정지! '
                    f'object={object_class}, '
                    f'distance={distance:.2f}m, '
                    f'relative_speed={relative_speed:.2f}m/s'
                )

                self.publish_stop()

            else:
                if self.ex_danger == True and self.danger == False:
                    self.get_logger().info(
                        '✅ 충돌 위험 해제'
                    )


                # danger=false이면 마지막 Nav2 명령 전달
                self.cmd_vel_pub.publish(self.last_cmd)

            self.ex_danger = self.danger

        except json.JSONDecodeError:
            self.get_logger().error(
                f'❌ /collision_warning JSON 파싱 실패: {msg.data}'
            )

        except Exception as e:
            self.get_logger().error(
                f'❌ collision warning 처리 오류: {e}'
            )

    def publish_stop(self):
        """
        모든 속도를 0으로 만들어 비상정지 명령을 보낸다.
        """

        stop_cmd = Twist()

        stop_cmd.linear.x = 0.0
        stop_cmd.linear.y = 0.0
        stop_cmd.linear.z = 0.0

        stop_cmd.angular.x = 0.0
        stop_cmd.angular.y = 0.0
        stop_cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(stop_cmd)


def main(args=None):

    rclpy.init(args=args)

    node = CollisionStopNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info(
            '🛑 Collision Stop Node 종료'
        )

    finally:
        # 종료 시에도 정지 명령
        node.publish_stop()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()