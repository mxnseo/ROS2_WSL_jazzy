# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim


import os  # 운영체제 경로 및 환경변수 사용을 위한 모듈

# ROS 2 패키지 경로를 찾기 위한 함수 임포트
from ament_index_python.packages import get_package_share_directory
# 런치 파일 구성을 위한 필수 클래스들 임포트
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 운영체제 환경변수에서 'TURTLEBOT3_MODEL' (burger, waffle, waffle_pi)을 가져옴
TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
# 운영체제 환경변수에서 'ROS_DISTRO' (humble, foxy 등)를 가져옴
ROS_DISTRO = os.environ.get('ROS_DISTRO')


def generate_launch_description():
    # LaunchConfiguration 설정: 외부에서 인자로 받을 수 있는 변수 정의
    # 'use_sim_time' 변수 정의 (기본값 false). 시뮬레이션(Gazebo) 사용 시 true로 설정해야 함.
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 'map' 변수 정의. 기본값으로 turtlebot3_navigation2 패키지 내의 map/map.yaml 경로를 잡음.
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'map',
            'map.yaml'))

    # 파라미터 파일 이름 생성
    param_file_name = TURTLEBOT3_MODEL + '.yaml'
    
    # ROS 버전에 따른 파라미터 파일 경로 설정
    if ROS_DISTRO == 'humble':
        param_dir = LaunchConfiguration(
            'params_file',
            default=os.path.join(
                get_package_share_directory('turtlebot3_navigation2'),
                'param',
                ROS_DISTRO,
                param_file_name))
    else:
        param_dir = LaunchConfiguration(
            'params_file',
            default=os.path.join(
                get_package_share_directory('turtlebot3_navigation2'),
                'param',
                param_file_name))

    # 실제 Nav2 기능을 실행할 'nav2_bringup' 패키지의 launch 폴더 경로를 찾음
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    # RViz 설정 파일 경로를 찾음 (tb3_navigation2.rviz)
    rviz_config_dir = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'rviz',
        'tb3_navigation2.rviz')

    # LaunchDescription 반환: 실제로 실행할 액션들의 리스트
    return LaunchDescription([
        # [인자 선언] 터미널에서 'map:=/my/map.yaml' 처럼 경로를 바꿀 수 있게 선언함
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        # [인자 선언] 터미널에서 'params_file:=/my/param.yaml' 처럼 파라미터를 바꿀 수 있게 선언함
        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        # [인자 선언] 시뮬레이션 시간 사용 여부 인자 선언
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # [외부 런치 파일 포함]'nav2_bringup' 패키지의 'bringup_launch.py'를 실행함.
        # 이때 우리가 위에서 설정한 map, use_sim_time, params_file 정보를 넘겨줌.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        # [노드 실행] 시각화 도구인 RViz2를 실행함.
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            # 위에서 찾은 .rviz 설정 파일을 로드해서 화면 구성을 불러옴
            arguments=['-d', rviz_config_dir],
            # 시뮬레이션 시간 동기화 설정
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])