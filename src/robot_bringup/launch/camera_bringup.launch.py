from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    depth = LaunchConfiguration('depth')
    declare_depth = DeclareLaunchArgument(
        'depth', default_value='false',
        description='Enable depth stream from RealSense camera'
    )
    cam_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'enable_depth': depth
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
        declare_depth, cam_bringup, cam_rect
    ])
    