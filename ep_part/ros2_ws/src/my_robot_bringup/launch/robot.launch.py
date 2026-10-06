import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('my_robot_bringup')
    mux_yaml = os.path.join(pkg, 'config', 'twist_mux.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('require_sp', default_value='true'),
        # 라이다 (기본 꺼짐: 연결 후 lidar:=true)
        DeclareLaunchArgument('lidar', default_value='false'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_baud', default_value='115200'),
        # base_link 기준 라이다 위치 (m, rad) - 실측값으로 수정
        DeclareLaunchArgument('laser_x', default_value='0.0'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.15'),
        DeclareLaunchArgument('laser_yaw', default_value='0.0'),
        # 카메라 (기본 꺼짐: 연결 후 camera:=true)
        DeclareLaunchArgument('camera', default_value='false'),
        DeclareLaunchArgument('video_source', default_value='0'),
        DeclareLaunchArgument('publish_raw', default_value='false'),

        Node(
            package='my_robot_bringup',
            executable='cmd_vel_bridge',
            parameters=[{'port': LaunchConfiguration('port')}],
            output='screen',
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            parameters=[mux_yaml],
            remappings=[('cmd_vel_out', 'cmd_vel')],
        ),
        Node(
            package='my_robot_bringup',
            executable='avoidance_node',
            parameters=[{'require_sp': LaunchConfiguration('require_sp')}],
            output='screen',
        ),
        # 후방 초음파 TF (x는 실측값으로)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['-0.10', '0', '0.05', '3.14159', '0', '0',
                       'base_link', 'ultrasonic_link'],
        ),

        # ---- 라이다 ----
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'lidar.launch.py')),
            launch_arguments={
                'lidar_port': LaunchConfiguration('lidar_port'),
                'lidar_baud': LaunchConfiguration('lidar_baud'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('lidar')),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[LaunchConfiguration('laser_x'), LaunchConfiguration('laser_y'),
                       LaunchConfiguration('laser_z'), LaunchConfiguration('laser_yaw'),
                       '0', '0', 'base_link', 'laser'],
            condition=IfCondition(LaunchConfiguration('lidar')),
        ),

        # ---- 카메라 ----
        Node(
            package='my_robot_bringup',
            executable='camera_node',
            parameters=[{
                'video_source': ParameterValue(LaunchConfiguration('video_source'), value_type=str),
                'publish_raw': LaunchConfiguration('publish_raw'),
            }],
            output='screen',
            condition=IfCondition(LaunchConfiguration('camera')),
        ),
    ])
