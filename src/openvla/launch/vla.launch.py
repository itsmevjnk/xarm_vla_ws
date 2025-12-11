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

    cam_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_bringup'), 'launch', 'camera_bringup.launch.py')
        )
    )

    vla_node = Node(
        package='openvla', executable='vla_node', name='vla_node',
        parameters=[{
            'url': url,
            'unnorm_key': unnorm_key
        }],
        remappings=[
            ('image', '/camera/camera/color/image_rect'),
            ('instruction', '/vla/instruction'),
            ('output', '/vla/output')
        ]
    )

    return LaunchDescription([
        declare_url, declare_unnorm_key,
        cam_bringup, vla_node
    ])
    