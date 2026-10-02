#!/usr/bin/env python3
"""
YOLO 객체 검출 시스템 런치 파일
- camera_node: 카메라/영상에서 프레임 캡처 → /camera/image_raw 발행
- yolo_node: /camera/image_raw 구독 → YOLO 검출 → /yolo/detections 발행
- visualization_node: /yolo/result_image 구독 → 결과 시각화
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── 런치 인자 선언 ──
    video_source_arg = DeclareLaunchArgument(
        'video_source',
        default_value="'0'",
        description='비디오 소스: 0=웹캠, 또는 영상 파일 경로'
    )

    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='models/futuredrive_yolo26n_hardneg_v2_best.pt',
        description='YOLO 모델 이름 (yolov8n.pt, yolov8s.pt, yolov8m.pt 등)'
    )

    frame_rate_arg = DeclareLaunchArgument(
        'frame_rate',
        default_value='30.0',
        description='카메라 프레임 레이트 (Hz)'
    )

    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cpu',
        description='추론 디바이스: cpu 또는 cuda'
    )

    # ── 거리 추정 런치 인자 ──
    distance_method_arg = DeclareLaunchArgument(
        'distance_method',
        default_value='ground',
        description='거리 추정 방식: ground (바닥 접점 지면 투영) 또는 bbox_height (높이 기반)'
    )
    camera_height_arg = DeclareLaunchArgument(
        'camera_height',
        default_value='0.6',
        description='카메라 지면 장착 높이 (m)'
    )
    camera_pitch_deg_arg = DeclareLaunchArgument(
        'camera_pitch_deg',
        default_value='0.0',
        description='카메라 하향 피치 각도 (도, degree)'
    )
    focal_length_y_arg = DeclareLaunchArgument(
        'focal_length_y',
        default_value='554.0',
        description='카메라 세로 초점거리 (픽셀)'
    )
    real_height_four_wheeler_arg = DeclareLaunchArgument(
        'real_height_four_wheeler',
        default_value='1.5',
        description='4륜차 실제 높이 (m)'
    )
    real_height_two_wheeler_arg = DeclareLaunchArgument(
        'real_height_two_wheeler',
        default_value='1.2',
        description='2륜차 실제 높이 (m)'
    )
    real_height_person_arg = DeclareLaunchArgument(
        'real_height_person',
        default_value='1.7',
        description='사람 실제 키 (m)'
    )

    # ── 노드 정의 ──
    camera_node = Node(
        package='yolo_detector',
        executable='camera_node',
        name='camera_publisher',
        output='screen',
        parameters=[{
            'video_source': LaunchConfiguration('video_source'),
            'frame_rate': LaunchConfiguration('frame_rate'),
            'frame_width': 640,
            'frame_height': 480,
            'loop_video': True,
        }],
    )

    yolo_node = Node(
        package='yolo_detector',
        executable='yolo_node',
        name='yolo_detector',
        output='screen',
        parameters=[{
            'model_name': LaunchConfiguration('model_name'),
            'four_wheeler_confidence': 0.75,
            'two_wheeler_confidence': 0.30,
            'person_confidence': 0.30,
            'confirmation_frames': 2,
            'max_missed_frames': 2,
            'temporal_iou_threshold': 0.3,
            'device': LaunchConfiguration('device'),
            'input_topic': '/camera/image_raw',
            'max_det': 50,
            'distance_method': LaunchConfiguration('distance_method'),
            'camera_height': LaunchConfiguration('camera_height'),
            'camera_pitch_deg': LaunchConfiguration('camera_pitch_deg'),
            'focal_length_y': LaunchConfiguration('focal_length_y'),
            'real_height_four_wheeler': LaunchConfiguration('real_height_four_wheeler'),
            'real_height_two_wheeler': LaunchConfiguration('real_height_two_wheeler'),
            'real_height_person': LaunchConfiguration('real_height_person'),
        }],
    )

    visualization_node = Node(
        package='yolo_detector',
        executable='visualization_node',
        name='visualization_node',
        output='screen',
    )

    return LaunchDescription([
        # 런치 인자
        video_source_arg,
        model_name_arg,
        frame_rate_arg,
        device_arg,
        distance_method_arg,
        camera_height_arg,
        camera_pitch_deg_arg,
        focal_length_y_arg,
        real_height_four_wheeler_arg,
        real_height_two_wheeler_arg,
        real_height_person_arg,
        # 노드
        camera_node,
        yolo_node,
        visualization_node,
    ])
