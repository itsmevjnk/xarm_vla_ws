from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

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
            os.path.join(get_package_share_directory('robot_bringup'), 'launch', 'arm_bringup.launch.py')
        ),
        launch_arguments={
            'ip': arm_ip
        }.items()
    )

    cam_depth = LaunchConfiguration('cam_depth')
    declare_cam_depth = DeclareLaunchArgument(
        'cam_depth', default_value='false',
        description='Enable depth stream from RealSense camera'
    )

    base_cam_sn = LaunchConfiguration('base_cam_sn')
    declare_base_cam_sn = DeclareLaunchArgument(
        'base_cam_sn', default_value='"242222070936"',
        description='Serial number of base (fixed) camera'
    )

    base_cam_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_bringup'), 'launch', 'camera_bringup.launch.py')
        ),
        launch_arguments={
            'name': 'base',
            'serial': base_cam_sn,
            'depth': cam_depth
        }.items()
    )

    wrist_cam_sn = LaunchConfiguration('wrist_cam_sn')
    declare_wrist_cam_sn = DeclareLaunchArgument(
        'wrist_cam_sn', default_value='"242322078188"',
        description='Serial number of wrist-mounted camera'
    )
    
    wrist_cam_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_bringup'), 'launch', 'camera_bringup.launch.py')
        ),
        launch_arguments={
            'name': 'wrist',
            'serial': wrist_cam_sn,
            'depth': cam_depth
        }.items()
    )

    cam_ns_bringup = GroupAction(
        actions=[
            PushRosNamespace(namespace='camera'),
            base_cam_bringup, wrist_cam_bringup
        ]
    )

    return LaunchDescription([
        declare_arm_ip, arm_bringup,
        declare_cam_depth,
        declare_base_cam_sn,
        declare_wrist_cam_sn,
        cam_ns_bringup
    ])
    