"""실제 AP 서버의 ROS2 테스트 제어 통신을 담당합니다."""
from rclpy.node import Node
from std_msgs.msg import String


# 실제 HMI 서버의 테스트 제어 발행 모델
class HMIControlModel(Node):
    """기존 hmi_control_test 노드와 STOP 테스트 Topic을 유지합니다."""

    def __init__(self):
        super().__init__("hmi_control_test")
        self.publisher = self.create_publisher(
            String, "/ap_test/control/request", 10
        )

    def publish_stop(self):
        # 실제 로봇 제어가 아닌 테스트 메시지만 발행합니다.
        msg = String()
        msg.data = "STOP"
        self.publisher.publish(msg)
