import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'yolo_detector'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=[
        'setuptools',
        'ultralytics',
        'opencv-python',
    ],
    zip_safe=True,
    maintainer='hkit',
    maintainer_email='chlwlgh99@gmail.com',
    description='YOLO 객체 검출 ROS2 패키지 - 카메라/영상 입력 기반 실시간 객체 검출',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_node = yolo_detector.camera_node:main',
            'lane_node = yolo_detector.lane_node:main',
            'yolo_node = yolo_detector.yolo_node:main',
            'visualization_node = yolo_detector.visualization_node:main',
            'result_video_recorder = yolo_detector.result_video_recorder:main',
            'yahboom_yolo = yolo_detector.yahboom_yolo:main',
            'lidar_yahboom = yolo_detector.lidar_yahboom:main',
        ],
    },
)
