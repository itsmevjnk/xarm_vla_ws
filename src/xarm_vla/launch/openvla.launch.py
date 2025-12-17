from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    url = LaunchConfiguration('url')
    declare_url = DeclareLaunchArgument(
        'url', default_value='http://127.0.0.1:8000/act',
        description='OpenVLA API endpoint URL'
    )

    unnorm_key = LaunchConfiguration('unnorm_key')
    declare_unnorm_key = DeclareLaunchArgument(
        'unnorm_key', default_value='bridge_orig',
        description='Un-normalisation key to use'
    )
    
    vla_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('openvla'), 'launch', 'vla.launch.py')
        ),
        launch_arguments={
            'url': url,
            'unnorm_key': unnorm_key
        }.items()
    )

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

    srv_timeout = LaunchConfiguration('srv_timeout')
    declare_srv_timeout = DeclareLaunchArgument(
        'srv_timeout', default_value='0.25',
        description='Timeout for waiting for xArm move service to be available (in seconds)'
    )

    maxvel = LaunchConfiguration('maxvel')
    declare_maxvel = DeclareLaunchArgument(
        'maxvel', default_value='50.0',
        description='Maximum robot linear velocity (in mm/s)'
    )

    maxacc = LaunchConfiguration('maxacc')
    declare_maxacc = DeclareLaunchArgument(
        'maxacc', default_value='500.0',
        description='Maximum robot linear acceleration (in mm/s^2)'
    )

    maxtrans = LaunchConfiguration('maxtrans')
    declare_maxtrans = DeclareLaunchArgument(
        'maxtrans', default_value='5.0',
        description='Maximum translation along each axis (in mm)'
    )

    maxrot = LaunchConfiguration('maxrot')
    declare_maxrot = DeclareLaunchArgument(
        'maxrot', default_value='0.08',
        description='Maximum rotation about each axis (in rad)'
    )

    exec_node = Node(
        package='xarm_vla', executable='delta_node', name='vla_exec',
        parameters=[{
            'srv_timeout': srv_timeout,
            'maxvel': maxvel,
            'maxacc': maxacc,
            'maxtrans': maxtrans,
            'maxrot': maxrot
        }]
    )

    return LaunchDescription([
        declare_url, declare_unnorm_key, vla_launch,
        declare_arm_ip, arm_bringup,
        declare_srv_timeout, declare_maxvel, declare_maxacc, declare_maxtrans, declare_maxrot, exec_node
    ])
    