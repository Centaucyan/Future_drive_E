from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 1. YOLO 노드
        Node(
            package='yolo_detector',
            executable='lidar_yahboom',
            name='lidar_yahboom',
            output='screen',
            prefix=[
                'gnome-terminal --title="[Node 1] Yolo detector" -- '
            ],
        ),

        # 2. 충돌 정지 노드
        Node(
            package='yolo_detector',
            executable='collision_stop_node',
            name='collision_stop_node',
            output='screen',
            prefix=[
                'gnome-terminal --title="[Node 2] Collision stop node" -- '
            ],
        ),
    ])