from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    name = LaunchConfiguration('name')
    declare_name = DeclareLaunchArgument(
        'name', default_value='camera',
        description='Camera name'
    )

    serial = LaunchConfiguration('serial')
    declare_serial = DeclareLaunchArgument(
        'serial', default_value='',
        description='Camera USB serial number (leave empty for automatic selection, does not work reliably with multi-camera setup)'
    )

    depth = LaunchConfiguration('depth')
    declare_depth = DeclareLaunchArgument(
        'depth', default_value='false',
        description='Enable depth stream from RealSense camera'
    )

    tf = LaunchConfiguration('tf')
    declare_tf = DeclareLaunchArgument(
        'tf', default_value='true',
        description='Publish camera TF'
    )

    cam_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'enable_depth': depth,
            'camera_name': name,
            'camera_namespace': '',
            'serial_no': serial,
            'publish_tf': tf,
            'pointcloud.enable': depth,
            'align_depth.enable': depth
        }.items()
    )
    cam_rect = Node(
        package='image_proc', executable='rectify_node',
        name='color_rectify',
        namespace=name,
        parameters=[{
            'camera_info_qos': 'transient_local'
        }],
        remappings=[
            ('image', 'color/image_raw'),
            ('camera_info', 'color/camera_info'),
            ('image_rect', 'color/image_rect'),
            ('image_rect/compressed', 'color/image_rect/compressed'),
            ('image_rect/compressedDepth', 'color/image_rect/compressedDepth'),
            ('image_rect/theora', 'color/image_rect/theora')
        ]
    )

    return LaunchDescription([
        declare_name, declare_serial, declare_depth, declare_tf,
        cam_bringup, cam_rect
    ])
    