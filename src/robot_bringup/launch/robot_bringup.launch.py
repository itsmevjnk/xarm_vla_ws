from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

import os
from ament_index_python import get_package_share_directory

import yaml

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
            'robot_ip': arm_ip,
            'add_gripper': 'true'
        }.items()
    )
    arm_tf_pub = Node(
        package='xarm_tf_pub', executable='pub_node',
        parameters=[{
            'base_frame': 'base_link',
            'tcp_frame': 'ee_link'
        }]
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

    # robot calibration stuff
    calibrate = LaunchConfiguration('calibrate')
    declare_calibrate = DeclareLaunchArgument(
        'calibrate', default_value='false',
        description='Launch AprilTag localisation and hand-eye calibration nodes'
    )

    calib_tag = LaunchConfiguration('calib_tag')
    declare_calib_tag = DeclareLaunchArgument(
        'calib_tag', default_value='1',
        description='End effector-mounted tag ID for hand-eye calibration'
    )

    calib_tag_size = LaunchConfiguration('calib_tag_size')
    declare_calib_tag_size = DeclareLaunchArgument(
        'calib_tag_size', default_value='0.080',
        description='End effector-mounted tag edge size (in metres) for hand-eye calibration'
    )

    calib_apriltag = Node(
        package='apriltag_ros', executable='apriltag_node',
        remappings=[
            ('image_rect', '/camera/camera/color/image_rect'),
            ('camera_info', '/camera/camera/color/camera_info')
        ],
        parameters=[{
            'image_transport': 'raw',
            'family': '36h11',
            'size': calib_tag_size,
            'profile': False,
            'max_hamming': 0,
            'detector': {
                'threads': 1,
                'decimate': 2.0,
                'blur': 0.0,
                'refine': True,
                'sharpening': 0.25,
                'debug': False
            },
            'pose_estimation_method': 'pnp',
            'tag': {
                'ids': PythonExpression(['[int(', calib_tag, ')]']),
                'frames': ['calib_tag_link']
            }
        }],
        condition=IfCondition(calibrate)   
    )

    # calib_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('hand_eye_calibration'), 'calibration.launch.py')
    #     ),
    #     launch_arguments={
    #         'tracking_base_frame': 'camera_color_optical_frame',
    #         'tracking_marker_frame': 'calib_tag_link',
    #         'robot_base_frame': 'base_link',
    #         'robot_effector_frame': 'ee_link',
    #         'calibration_type': 'eye-on-base'
    #     }.items(),
    #     condition=IfCondition(calibrate)   
    # )
    
    calib_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('easy_handeye2'), 'launch', 'calibrate.launch.py')
        ),
        launch_arguments={
            'tracking_base_frame': 'camera_color_optical_frame',
            'tracking_marker_frame': 'calib_tag_link',
            'robot_base_frame': 'base_link',
            'robot_effector_frame': 'ee_link',
            'calibration_type': 'eye_on_base',
            'name': 'vla_robot_calib',

            'freehand_robot_movement': 'true' # manual calibration
        }.items(),
        condition=IfCondition(calibrate)   
    )

    # load hand-eye calibration result
    # with open(os.path.join(get_package_share_directory('robot_bringup'), 'config', 'calibration.yaml')) as f:
    #     calibration = yaml.safe_load(f)
    
    # base_link_pub = Node(
    #     package='tf2_ros', executable='static_transform_publisher',
    #     name='base_link_pub',
    #     arguments=[
    #         str(x) for x in (
    #             calibration['hand_eye'] + ['camera_color_optical_frame', 'base_link']
    #         )
    #     ],
    #     condition=UnlessCondition(calibrate)
    # )

    base_link_pub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('easy_handeye2'), 'launch', 'publish.launch.py')
        ),
        launch_arguments={
            'name': 'vla_robot_calib'
        }.items(),
        condition=UnlessCondition(calibrate)
    )

    return LaunchDescription([
        declare_arm_ip, arm_bringup, arm_tf_pub,
        declare_cam_depth, cam_bringup, cam_rect,
        declare_calibrate, declare_calib_tag, declare_calib_tag_size,
        calib_apriltag, calib_launch,
        base_link_pub
    ])
    