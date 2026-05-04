#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package path
    pkg_dir = get_package_share_directory('pkrc_sonar_tilt')
    config_file = os.path.join(pkg_dir, 'config', 'tilt_controller.yaml')

    # Launch arguments
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/ttyUSB0',
        description='U2D2 serial port'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='57600',
        description='Serial baudrate'
    )

    motor_id_arg = DeclareLaunchArgument(
        'motor_id',
        default_value='1',
        description='Dynamixel motor ID'
    )

    auto_home_arg = DeclareLaunchArgument(
        'auto_home',
        default_value='false',
        description='Auto-move to 45° on startup (default: false)'
    )

    # Node
    tilt_controller_node = Node(
        package='pkrc_sonar_tilt',
        executable='tilt_controller_node.py',
        name='sonar_tilt',
        output='screen',
        parameters=[
            config_file,
            {
                'device': LaunchConfiguration('device'),
                'baudrate': LaunchConfiguration('baudrate'),
                'motor_id': LaunchConfiguration('motor_id'),
                'auto_home': LaunchConfiguration('auto_home'),
            }
        ]
    )

    return LaunchDescription([
        device_arg,
        baudrate_arg,
        motor_id_arg,
        auto_home_arg,
        tilt_controller_node,
    ])
