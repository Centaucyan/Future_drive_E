import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class ROSModel(Node):

    def __init__(self):

        super().__init__('fastapi_ros_node')

        self.message = "ROS 메시지 대기 중..."

        self.subscription = self.create_subscription(
            String,
            '/web_message',
            self.message_callback,
            10
        )

        self.get_logger().info("ROS Model Start")


    def message_callback(self, msg):

        self.message = msg.data

        self.get_logger().info(
            f"Received: {self.message}"
        )


    def get_message(self):

        return self.message