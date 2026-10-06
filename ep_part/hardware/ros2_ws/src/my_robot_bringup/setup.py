import os # 운영체제의 파일 경로 기능을 사용하기 위해 불러옴
from glob import glob # 폴더 내의 여러 파일들을 패턴(*.py 등)으로 찾기 위해 불러옴
from setuptools import find_packages, setup

package_name = 'my_robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hkit',
    maintainer_email='hkit@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cmd_vel_serial_bridge = my_robot_bringup.cmd_vel_serial_bridge:main',
            'avoidance_node = my_robot_bringup.avoidance_node:main',
            'cmd_vel_bridge = my_robot_bringup.cmd_vel_bridge:main',
        ],
    },
)
