#!/usr/bin/env python3

import os
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file

def generate_launch_description():
    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'), 'config', 'xarm7_controllers.yaml'),
        prefix='', 
        add_gripper=True,
        add_bio_gripper=False,
        ros_namespace='',
        robot_type='xarm'
    )

    moveit_config = MoveItConfigsBuilder(
        # context=context,
        controllers_name='fake_controllers',
        dof=7,
        robot_type='xarm',
        prefix='',
        hw_ns='xarm',
        limited=True,
        effort_control=False,
        velocity_control=False,
        model1300=False,
        robot_sn='',
        attach_to='world',
        attach_xyz='0 0 0',
        attach_rpy='0 0 0',
        mesh_suffix='stl',
        kinematics_suffix='',
        ros2_control_plugin='uf_robot_hardware/UFRobotFakeSystemHardware',
        ros2_control_params=ros2_control_params,
        add_gripper=True,
        add_vacuum_gripper=False,
        add_bio_gripper=False,
        add_realsense_d435i=False,
        add_d435i_links=True,
        add_other_geometry=False,
        geometry_type='box',
        geometry_mass=0.1,
        geometry_height=0.1,
        geometry_radius=0.1,
        geometry_length=0.1,
        geometry_width=0.1,
        geometry_mesh_filename='',
        geometry_mesh_origin_xyz='0 0 0',
        geometry_mesh_origin_rpy='0 0 0',
        geometry_mesh_tcp_xyz='0 0 0',
        geometry_mesh_tcp_rpy='0 0 0',
    ).to_moveit_configs()

    target = LaunchConfiguration('target')
    declare_target = DeclareLaunchArgument(
        'target',
        description='Target classifier index'
    )

    cgn_moveit = Node(
        package='cgn_moveit', executable='cgn_moveit',
        parameters=[
            moveit_config.to_dict(),
            {
                'move_group': 'xarm7',
                'ee_link': 'link_tcp',
                'root_frame': 'link_base',
                'mask_cls': target
            }
        ],
        remappings=[
            ('grasp_target', '/cgn/target'),
            ('camera_info', '/camera/base/depth/camera_info'),
            ('mask', '/camera/base/color/seg_mask'),
            ('depth', '/camera/base/aligned_depth_to_color/image_raw'),
            ('infer', '/cgn/infer')
        ]
    )

    return LaunchDescription([
        declare_target, cgn_moveit
    ])
