#!/usr/bin/env python3
"""
IMU Calibration Helper for Microstrain GV7

Provides gyro bias capture and device settings management
through ROS2 service calls.

Usage:
  ros2 run pkrc_imu_utils imu_calibrate
"""

import sys
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, Empty


class ImuCalibrate(Node):
    def __init__(self):
        super().__init__('imu_calibrate')

        # Service clients
        self.gyro_bias_client = self.create_client(
            Trigger,
            '/microstrain_inertial_driver/mip/three_dm/capture_gyro_bias'
        )
        self.save_settings_client = self.create_client(
            Empty,
            '/microstrain_inertial_driver/mip/three_dm/device_settings/save'
        )
        self.load_settings_client = self.create_client(
            Empty,
            '/microstrain_inertial_driver/mip/three_dm/device_settings/load'
        )

    def wait_for_service(self, client, name, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f'Service {name} not available. Is the IMU driver running?')
            return False
        return True

    def capture_gyro_bias(self):
        """Capture gyro bias. Device must be stationary."""
        name = 'capture_gyro_bias'
        if not self.wait_for_service(self.gyro_bias_client, name):
            return False

        self.get_logger().info('Capturing gyro bias... Keep the device COMPLETELY STILL!')
        self.get_logger().info('This takes about 5-10 seconds...')

        req = Trigger.Request()
        future = self.gyro_bias_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if future.result() is not None:
            result = future.result()
            self.get_logger().info(f'Gyro bias capture: success={result.success}, message="{result.message}"')
            return result.success
        else:
            self.get_logger().error('Gyro bias capture timed out or failed')
            return False

    def save_settings(self):
        """Save current settings to device non-volatile memory."""
        name = 'device_settings/save'
        if not self.wait_for_service(self.save_settings_client, name):
            return False

        self.get_logger().info('Saving settings to device...')
        req = Empty.Request()
        future = self.save_settings_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is not None:
            self.get_logger().info('Settings saved to device non-volatile memory.')
            return True
        else:
            self.get_logger().error('Save settings failed')
            return False

    def load_settings(self):
        """Load settings from device non-volatile memory."""
        name = 'device_settings/load'
        if not self.wait_for_service(self.load_settings_client, name):
            return False

        self.get_logger().info('Loading settings from device...')
        req = Empty.Request()
        future = self.load_settings_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is not None:
            self.get_logger().info('Settings loaded from device.')
            return True
        else:
            self.get_logger().error('Load settings failed')
            return False

    def run_calibration(self):
        """Run the full calibration procedure."""
        print()
        print('=' * 60)
        print('     Microstrain GV7 IMU Calibration Helper')
        print('=' * 60)
        print()
        print('  Available actions:')
        print('    1. Capture Gyro Bias (device must be still)')
        print('    2. Save settings to device')
        print('    3. Load settings from device')
        print('    4. Full calibration (capture + save)')
        print('    5. Exit')
        print()

        while True:
            try:
                choice = input('  Select action [1-5]: ').strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break

            if choice == '1':
                print()
                input('  Place the device on a flat, stable surface and press Enter...')
                self.capture_gyro_bias()
                print()

            elif choice == '2':
                self.save_settings()
                print()

            elif choice == '3':
                self.load_settings()
                print()

            elif choice == '4':
                print()
                input('  Place the device on a flat, stable surface and press Enter...')
                print()
                if self.capture_gyro_bias():
                    time.sleep(1)
                    self.save_settings()
                    self.get_logger().info('Full calibration complete.')
                else:
                    self.get_logger().error('Gyro bias capture failed. Settings not saved.')
                print()

            elif choice == '5':
                break

            else:
                print('  Invalid choice. Try again.')
                print()


def main(args=None):
    rclpy.init(args=args)
    node = ImuCalibrate()
    try:
        node.run_calibration()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
