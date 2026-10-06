import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('my_robot_navigation')
    default_params = os.path.join(pkg, 'config', 'slam_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('slam_params', default_value=default_params),
        # Humble 기준: async_slam_toolbox_node를 일반 노드로 실행
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[LaunchConfiguration('slam_params'),
                        {'use_sim_time': False}],
            output='screen',
        ),
    ])
