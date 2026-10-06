#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 패키지 이름 설정: 우리가 만든 패키지 이름을 변수로 지정
    package_name = 'my_robot_bringup'
    # 패키지가 설치된 실제 경로(공유 디렉토리)를 가져옵니다.
    pkg_share = get_package_share_directory(package_name)

    # 3. RPLidar C1 드라이버 노드: 하드웨어 라이다로부터 데이터를 받아오는 역할
    rplidar_cmd = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB0', # 라이다가 연결된 USB 포트 (확인 결과 ttyUSB0)
            'serial_baudrate': 460800,     # C1 모델의 통신 속도
            'frame_id': 'laser_frame',           # 라이다 데이터의 기준점 이름 정의
            'inverted': False,             # 라이다 설치 방향 반전 여부
            'angle_compensate': True,      # 회전 속도에 따른 각도 보정 활성화
        }],
        output='screen'
    )

        # LiDAR 고정 TF
    # 아래 6개 값은 실제 LiDAR 장착 위치와 각도로 변경해야 함.
    laser_static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_frame',
        output='screen',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
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


    # 실행할 명령 목록(LaunchDescription) 생성 및 노드 추가
    ld = LaunchDescription()

    ld.add_action(rplidar_cmd)      # 라이다 켜기
    ld.add_action(laser_static_tf_node)   # 좌표 중심 잡기
    ld.add_action(arduino_bridge_node)   # 좌표 중심 잡기

    return ld