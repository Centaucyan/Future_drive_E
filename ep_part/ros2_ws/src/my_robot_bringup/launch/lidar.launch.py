from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """RPLiDAR(A1/A2/A3 등) 기준. 다른 모델이면 이 파일의 Node만 교체."""
    return LaunchDescription([
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_baud', default_value='115200'),  # A1: 115200, A2M12/A3: 256000
        DeclareLaunchArgument('lidar_frame', default_value='laser'),
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='lidar',
            parameters=[{
                'serial_port': LaunchConfiguration('lidar_port'),
                'serial_baudrate': LaunchConfiguration('lidar_baud'),
                'frame_id': LaunchConfiguration('lidar_frame'),
                'angle_compensate': True,
            }],
            output='screen',
        ),
    ])
