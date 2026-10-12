#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'arduino_cartography'

    package_share = get_package_share_directory(package_name)
    cartographer_share = get_package_share_directory('cartographer_ros')

    use_sim_time = LaunchConfiguration('use_sim_time')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        arguments=[
            '-configuration_directory',
            os.path.join(package_share, 'params'),
            '-configuration_basename',
            'futuredrive_cartographer.lua',
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom'),
        ],
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '1.0',
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
        ),
        cartographer_node,
        occupancy_grid_node,
        rviz_node,
    ])