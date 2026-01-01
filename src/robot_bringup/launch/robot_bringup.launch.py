from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch_ros.actions import PushRosNamespace, Node
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    moveit = LaunchConfiguration('moveit')
    declare_moveit = DeclareLaunchArgument(
        'moveit', default_value='false',
        description='Use MoveIt! for arm control'
    )

    arm_ip = LaunchConfiguration('arm_ip')
    declare_arm_ip = DeclareLaunchArgument(
        'arm_ip', default_value='192.168.1.209',
        description='IP address for the xArm; printed on control box sticker'
    )

    arm_api_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_bringup'), 'launch', 'arm_bringup.launch.py')
        ),
        launch_arguments={
            'ip': arm_ip
        }.items(),
        condition=UnlessCondition(moveit)
    )

    arm_moveit_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_bringup'), 'launch', 'moveit_bringup.launch.py')
        ),
        launch_arguments={
            'ip': arm_ip
        }.items(),
        condition=IfCondition(moveit)
    )

    # eye_on_hand_pub = Node(
    #     package='easy_handeye2',
    #     executable='handeye_publisher',
    #     parameters=[{
    #         'name': os.path.join(get_package_share_directory('robot_bringup'), 'config', 'xarm_rs_on_hand_calibration')
    #     }]
    # )

    eye_on_hand_pub = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=[
            "0.06789119937551764", "-0.030063300304697668", "0.02098551459424205",
            "-0.0003797703452797856", "-0.004984595856131313", "0.6933043531158767", "0.720627562288046",
            "link_eef", "wrist_color_optical_frame"
        ]
    )

    eye_on_base_pub = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=[
            "0.004199609723041731", "-0.6370350508292455", "0.9684640464267686",
            "-0.977570635392054", "0.16722200927121716", "-0.028232636865786184", "0.12488142636147731",
            "link_base", "base_color_optical_frame"
        ]
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
            'depth': cam_depth,
            'tf': 'false'
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
            'depth': cam_depth,
            'tf': 'false'
        }.items()
    )

    cam_ns_bringup = GroupAction(
        actions=[
            PushRosNamespace(namespace='camera'),
            base_cam_bringup, wrist_cam_bringup
        ]
    )

    eye_on_hand_depth_pub = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=[
            "0.015", "-0.000", "0.000",
            "0.000", "0.000", "0.000", "1.000",
            "wrist_color_optical_frame", "wrist_depth_optical_frame"
        ],
        condition=IfCondition(cam_depth)
    )

    eye_on_base_depth_pub = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=[
            "0.015", "-0.000", "0.000",
            "0.000", "0.000", "0.000", "1.000",
            "base_color_optical_frame", "base_depth_optical_frame"
        ],
        condition=IfCondition(cam_depth)
    )

    return LaunchDescription([
        declare_moveit,
        declare_arm_ip, arm_api_bringup, arm_moveit_bringup,
        eye_on_hand_pub, eye_on_base_pub,
        declare_cam_depth,
        declare_base_cam_sn,
        declare_wrist_cam_sn,
        cam_ns_bringup,
        eye_on_hand_depth_pub, eye_on_base_depth_pub
    ])
    