"""Launch HEROMainControl with optional parameter overrides.

Usage:
  ros2 launch pkrc_robot_control pkrc_main.launch.py
  ros2 launch pkrc_robot_control pkrc_main.launch.py params_file:=/path/to/tuned.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('pkrc_robot_control'),
        'config',
        'pkrc.yaml',
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to a parameter YAML file (defaults to bundled pkrc.yaml).',
    )

    main_node = Node(
        package='pkrc_robot_control',
        executable='main_control',
        name='hero_main_control',
        parameters=[LaunchConfiguration('params_file')],
        output='screen',
    )

    return LaunchDescription([params_file_arg, main_node])
