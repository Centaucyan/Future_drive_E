#!/usr/bin/env python3

from configparser import ConfigParser
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node


config_path = Path("/root/config.ini")

if not config_path.is_file():
    raise FileNotFoundError(f"설정 파일을 찾을 수 없습니다: {config_path}")

config = ConfigParser()
config.read(config_path)

profile = config.get("robot", "profile", fallback="").strip().lower()


def generate_launch_description():
    # 패키지 이름과 설치된 공유 디렉토리 경로
    package_name = 'my_robot_bringup'
    pkg_share = get_package_share_directory(package_name)

    # hkit4 노드
    rplidar_cmd = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 460800,
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True,
        }],
        output='screen',
    )

    # LiDAR 고정 TF
    laser_static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_frame',
        output='screen',
        arguments=[
            '--x', '0.0',
            '--y', '0.08',
            '--z', '0.13',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser_frame',
        ],
    )

    arduino_bridge_node = Node(
        package=package_name,
        executable='cmd_vel_serial_bridge',
        name='cmd_vel_serial_bridge',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyACM0',
            'baud_rate': 9600,
        }],
    )

    # hkit5 노드
    avoidance_node = Node(
        package=package_name,
        executable='avoidance_node',
        name='avoidance_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyACM0',
            'baud_rate': 115200,
            'require_sp': False,
        }],
    )

    cmd_bridge_node = Node(
        package=package_name,
        executable='cmd_vel_bridge',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyACM0',
        }],
    )

    rplidar_cmd_2 = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'lidar_frame',
            'angle_compensate': True,
        }],
        output='screen',
    )

    laser_static_tf_node_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_lidar_frame',
        arguments=[
            '--x', '0.03',
            '--y', '0.09',
            '--z', '0.11',
            '--yaw', '0',
            '--pitch', '0',
            '--roll', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'lidar_frame',
        ],
    )

    # 프로파일별 실행 노드 목록
    ld_hkit4 = LaunchDescription()
    ld_hkit4.add_action(rplidar_cmd)
    ld_hkit4.add_action(laser_static_tf_node)
    ld_hkit4.add_action(arduino_bridge_node)

    ld_hkit5 = LaunchDescription()
    ld_hkit5.add_action(avoidance_node)
    ld_hkit5.add_action(rplidar_cmd_2)
    ld_hkit5.add_action(laser_static_tf_node_2)
    ld_hkit5.add_action(cmd_bridge_node)

    if profile == "hkit4":
        return ld_hkit4
    elif profile == "hkit5":
        return ld_hkit5
    else:
        raise ValueError(f"지원하지 않는 profile입니다: {profile!r}")