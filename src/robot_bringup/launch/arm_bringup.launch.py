from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    ip = LaunchConfiguration('ip')
    declare_ip = DeclareLaunchArgument(
        'ip', default_value='192.168.1.209',
        description='IP address for the xArm; printed on control box sticker'
    )
    arm_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('xarm_api'), 'launch', 'xarm7_driver.launch.py')
        ),
        launch_arguments={
            'robot_ip': ip,
            'add_gripper': 'true',
            'show_rviz': 'true' # will also publish link_base and link_tcp/link_eef TF
        }.items()
    )

    return LaunchDescription([
        declare_ip, arm_bringup
    ])
    