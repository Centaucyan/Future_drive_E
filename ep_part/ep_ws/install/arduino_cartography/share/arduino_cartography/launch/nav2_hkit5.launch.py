#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory('arduino_cartography')
    nav2_share = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    nav2_navigation_launch = os.path.join(
        nav2_share,
        'launch',
        'navigation_launch.py',
    )

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='실제 로봇에서는 false',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                package_share,
                'params',
                'nav2_params.yaml',
            ),
            description='Nav2 파라미터 파일 경로',
        ),
        nav2_navigation,
    ])