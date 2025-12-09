from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    arm_ip = LaunchConfiguration('arm_ip')
    declare_arm_ip = DeclareLaunchArgument(
        'arm_ip', default_value='192.168.1.209',
        description='IP address for the xArm; printed on control box sticker'
    )
    arm_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('xarm_api'), 'launch', 'xarm7_driver.launch.py')
        ),
        launch_arguments={
            'robot_ip': arm_ip
        }.items()
    )

    cam_depth = LaunchConfiguration('cam_depth')
    declare_cam_depth = DeclareLaunchArgument(
        'cam_depth', default_value='false',
        description='Enable depth stream from RealSense camera'
    )
    cam_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'enable_depth': cam_depth
        }.items()
    )
    cam_rect = Node(
        package='image_proc', executable='image_proc',
        name='camera_rect_color',
        namespace='camera/camera/color',
        parameters=[{
            'camera_info_qos': 'transient_local'
        }],
        remappings=[
            ('image', 'image_raw')
        ]
    )

    return LaunchDescription([
        declare_arm_ip, arm_bringup,
        declare_cam_depth, cam_bringup, cam_rect
    ])
    