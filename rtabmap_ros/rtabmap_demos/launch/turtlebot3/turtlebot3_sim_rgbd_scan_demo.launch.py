# Requirements:
#   Install Turtlebot3 packages
#   Modify turtlebot3_waffle SDF:
#     1) Edit /opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf
#     2) Add
#          <joint name="camera_rgb_optical_joint" type="fixed">
#            <parent>camera_rgb_frame</parent>
#            <child>camera_rgb_optical_frame</child>
#            <pose>0 0 0 -1.57079632679 0 -1.57079632679</pose>
#            <axis>
#              <xyz>0 0 1</xyz>
#            </axis>
#          </joint> 
#     3) Rename <link name="camera_rgb_frame"> to <link name="camera_rgb_optical_frame">
#     4) Add <link name="camera_rgb_frame"/>
#     5) Change <sensor name="camera" type="camera"> to <sensor name="camera" type="depth">
#     6) Change image width/height from 1920x1080 to 640x480
#     7) Note that we can increase min scan range from 0.12 to 0.2 to avoid having scans 
#        hitting the robot itself
# Example:
#   $ ros2 launch rtabmap_demos turtlebot3_sim_rgbd_scan_demo.launch.py
#
#   Teleop:
#     $ ros2 run turtlebot3_teleop teleop_keyboard

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import os

# 런치 동작을 설정하는 함수 / (터미널 실행할 떄 변수 값에 접근 가능함 (직접 수정 안 해도 됨))
def launch_setup(context, *args, **kwargs):
    if not 'TURTLEBOT3_MODEL' in os.environ: # 환경 변수 설정이 없으면 기본값
        os.environ['TURTLEBOT3_MODEL'] = 'waffle' # waffle로 설정됨

    # Directories --> 디렉터리 찾는 부분
    pkg_turtlebot3_gazebo = get_package_share_directory( # 터틀봇 시뮬레이터 관련 경로
        'turtlebot3_gazebo')
    pkg_nav2_bringup = get_package_share_directory( # 네비게이션 관련 경로
        'nav2_bringup')
    pkg_rtabmap_demos = get_package_share_directory( # 현재 패키지 경로
        'rtabmap_demos')

    world = LaunchConfiguration('world').perform(context) # 사용자가 입력한 world 인자 값을 읽어옴
    
    nav2_params_file = PathJoinSubstitution( # Nav2의 .yaml 경로 설정함
        [FindPackageShare('rtabmap_demos'), 'params', 'turtlebot3_rgbd_scan_nav2_params.yaml']
    )

    # Paths --> 실행할 런치 파일들의 경로
    gazebo_launch = PathJoinSubstitution( # turtlebot3_gazebo 패키지의 launch 폴더 안의 'turtlebot3_[world이름].launch.py'
        [pkg_turtlebot3_gazebo, 'launch', f'turtlebot3_{world}.launch.py'])

    nav2_launch = PathJoinSubstitution( # nav2_bringup 패키지의 navigation_launch.py
        [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])

    # rviz_launch = PathJoinSubstitution( # nav2_bringup 패키지의 rviz_launch.py
    #     [pkg_nav2_bringup, 'launch', 'rviz_launch.py'])

    rtabmap_launch = PathJoinSubstitution( # tabmap_demos 패키지 내의 turtlebot3_rgbd_scan.launch.py
        [pkg_rtabmap_demos, 'launch', 'turtlebot3', 'turtlebot3_rgbd_scan.launch.py'])

    # Includes --> 런치 파일 include 설정
    gazebo = IncludeLaunchDescription( # 로봇의 초기 위치(x, y)를 인자로 넘김
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments=[
            ('x_pose', LaunchConfiguration('x_pose')),
            ('y_pose', LaunchConfiguration('y_pose'))
        ]
    )
    nav2 = IncludeLaunchDescription( # 시뮬레이션 시간(use_sim_time)을 true로 설정하고, 파라미터 파일을 넘김
        PythonLaunchDescriptionSource([nav2_launch]),
        launch_arguments=[
            ('use_sim_time', 'true'),
            ('params_file', nav2_params_file)
        ]
    )
    # # rviz = IncludeLaunchDescription( # Rviz 실행
    # #     PythonLaunchDescriptionSource([rviz_launch]),
    # # )
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rtabmap_launch]),
        launch_arguments=[
            ('localization', LaunchConfiguration('localization')),
            ('use_sim_time', 'true'),
            
            # --- 여기부터 추가 ---
            ('approx_sync', 'true'), 
            ('qos', '1'),# 네트워크 부하 감소 (Best Effort)
            ('delete_db_on_start', 'false'),
            # 최적화 옵션 (메모리 절약, CPU 부하 감소)
            ('args', '--database_path /home/linux/map.db'
                     '--Grid/RangeMax 3.5 ' # 3.5m 이상은 지도 안 그림
                     '--Grid/CellSize 0.05 ' # 지도 격자 5cm
                     '--Cloud/VoxelSize 0.05 ' # POINTCLOUD 간격 5cm (다운샘플링)
            )
            # -------------------------------
        ]
    )
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # 라이다 & odom & TF (기본)
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            
            # 카메라 데이터 (RGB + Depth + Info)
            '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/depth/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        output='screen'
    )
    return [
        # Nodes to launch
        bridge,
        nav2,
        # rviz,
        rtabmap,
        gazebo
    ]


# 메인 함수
def generate_launch_description():
    return LaunchDescription([
        
        # Launch arguments --> 터미널에서 바꿀 수 있는 옵션들 정의
        DeclareLaunchArgument(
            'localization', default_value='false', # true면 지도 작성 안 하고 기존 지도에서 위치만 찾음
            description='Launch in localization mode.'),
        
        DeclareLaunchArgument(
            'world', default_value='house', # 가제보 맵 이름
            choices=['world', 'house', 'dqn_stage1', 'dqn_stage2', 'dqn_stage3', 'dqn_stage4'],
            description='Turtlebot3 gazebo world.'),
        
        DeclareLaunchArgument(
            'x_pose', default_value='-2.0',
            description='Initial position of the robot in the simulator.'),
        
        DeclareLaunchArgument(
            'y_pose', default_value='0.5',
            description='Initial position of the robot in the simulator.'),

        OpaqueFunction(function=launch_setup)
    ])
