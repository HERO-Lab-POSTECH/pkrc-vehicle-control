#!/usr/bin/env python3
"""
IMU Roll/Pitch/Yaw Real-time Monitor

Subscribes to IMU topics from Microstrain GV7 and displays
roll, pitch, yaw in degrees with real-time terminal updates.

Usage:
  ros2 run pkrc_imu_utils imu_rpy_monitor
  ros2 run pkrc_imu_utils imu_rpy_monitor --ros-args -p use_filter:=true
"""

import math
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def quaternion_to_euler(x, y, z, w):
    """Convert quaternion to Euler angles (roll, pitch, yaw) in radians."""
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class ImuRpyMonitor(Node):
    def __init__(self):
        super().__init__('imu_rpy_monitor')

        self.declare_parameter('use_filter', False)
        use_filter = self.get_parameter('use_filter').value

        # Raw IMU data
        self.raw_rpy = None
        self.raw_count = 0

        # Filtered IMU data
        self.filter_rpy = None
        self.filter_count = 0

        # Subscribe to raw IMU
        self.create_subscription(Imu, '/imu/data', self.raw_imu_cb, 10)
        self.get_logger().info('Subscribed to /imu/data')

        if use_filter:
            self.create_subscription(Imu, '/ekf/imu/data', self.filter_imu_cb, 10)
            self.get_logger().info('Subscribed to /ekf/imu/data')

        self.use_filter = use_filter

        # Display timer at 10 Hz
        self.create_timer(0.1, self.display_cb)

        self.get_logger().info('IMU RPY Monitor started. Tilt the robot to see changes.')

    def raw_imu_cb(self, msg: Imu):
        q = msg.orientation
        roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        self.raw_rpy = (roll, pitch, yaw)
        self.raw_count += 1

    def filter_imu_cb(self, msg: Imu):
        q = msg.orientation
        roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        self.filter_rpy = (roll, pitch, yaw)
        self.filter_count += 1

    def display_cb(self):
        # Clear terminal
        os.system('clear')

        print('=' * 60)
        print('       IMU Roll / Pitch / Yaw Monitor (GV7)')
        print('=' * 60)
        print()

        if self.raw_rpy is not None:
            r, p, y = self.raw_rpy
            rd, pd, yd = math.degrees(r), math.degrees(p), math.degrees(y)
            print('  [Raw IMU - /imu/data]')
            print(f'    Roll  : {rd:+8.2f} deg  ({r:+7.4f} rad)')
            print(f'    Pitch : {pd:+8.2f} deg  ({p:+7.4f} rad)')
            print(f'    Yaw   : {yd:+8.2f} deg  ({y:+7.4f} rad)')
            print(f'    Messages received: {self.raw_count}')
        else:
            print('  [Raw IMU - /imu/data]')
            print('    Waiting for data...')

        print()

        if self.use_filter:
            if self.filter_rpy is not None:
                r, p, y = self.filter_rpy
                rd, pd, yd = math.degrees(r), math.degrees(p), math.degrees(y)
                print('  [Filtered IMU - /ekf/imu/data]')
                print(f'    Roll  : {rd:+8.2f} deg  ({r:+7.4f} rad)')
                print(f'    Pitch : {pd:+8.2f} deg  ({p:+7.4f} rad)')
                print(f'    Yaw   : {yd:+8.2f} deg  ({y:+7.4f} rad)')
                print(f'    Messages received: {self.filter_count}')
            else:
                print('  [Filtered IMU - /ekf/imu/data]')
                print('    Waiting for data...')

        print()
        print('-' * 60)
        print('  Frame: NED (use_enu_frame: false)')
        print('  Yaw 0 = North, +90 = East, -90 = West')
        print('  Roll + = Right side down, Pitch + = Nose down')
        print('-' * 60)
        print()
        print('  Ctrl+C to exit')


def main(args=None):
    rclpy.init(args=args)
    node = ImuRpyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
