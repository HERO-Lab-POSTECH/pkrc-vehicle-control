#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package path
    pkg_dir = get_package_share_directory('sonar_tilt_controller')
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

    # Node
    tilt_controller_node = Node(
        package='sonar_tilt_controller',
        executable='tilt_controller_node.py',
        name='sonar_tilt_controller',
        output='screen',
        parameters=[
            config_file,
            {
                'device': LaunchConfiguration('device'),
                'baudrate': LaunchConfiguration('baudrate'),
                'motor_id': LaunchConfiguration('motor_id'),
            }
        ]
    )

    return LaunchDescription([
        device_arg,
        baudrate_arg,
        motor_id_arg,
        tilt_controller_node,
    ])
