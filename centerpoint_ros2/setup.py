from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'centerpoint_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='CenterPoint 3D Object Detection Node for ROS2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # '실행명령어 = 패키지폴더명.파이썬파일명:main'
            'centerpoint_node = centerpoint_ros2.centerpoint_node:main'
        ],
    },
)