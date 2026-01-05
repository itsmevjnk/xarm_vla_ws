from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    model_path = LaunchConfiguration('model_path')
    declare_model_path = DeclareLaunchArgument(
        'model_path', default_value=os.path.join(get_package_share_directory('cgn_ros2'), 'checkpoints'),
        description='Path to Contact-GraspNet model'
    )

    threshold = LaunchConfiguration('threshold')
    declare_threshold = DeclareLaunchArgument(
        'threshold', default_value='0.5',
        description='Confidence threshold for valid grasp selection'
    )

    cgn_node = Node(
        package='cgn_ros2', executable='cgn_node',
        parameters=[
            {
                'model_path': model_path,
                'threshold': threshold
            }
        ],
        remappings=[
            ('infer', '/cgn/infer')
        ]
    )


    return LaunchDescription([
        declare_model_path, declare_threshold, cgn_node
    ])
    