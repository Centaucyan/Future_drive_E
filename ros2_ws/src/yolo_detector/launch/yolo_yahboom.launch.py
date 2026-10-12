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
            arguments=[
                '--ros-args',
                '--params-file',
                'src/yolo_detector/config/yaboom_camera_info.yaml'
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

        Node(
            package='yolo_detector',
            executable='lane_node',
            name='lane_node',
            output='screen',
            prefix=[
                'gnome-terminal --title="[Node 3] lane_node" -- '
            ],
        ),
    ])