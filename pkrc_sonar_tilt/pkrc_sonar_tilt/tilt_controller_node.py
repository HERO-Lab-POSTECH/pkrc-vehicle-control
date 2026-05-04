#!/usr/bin/env python3
"""
Sonar Tilt Controller Node
- 소나 틸트 각도 제어
- 현재 각도 퍼블리시
- 목표 각도 서브스크라이브
- 베벨 기어비 2:1 적용 (Z1=15T, Z2=30T)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from std_srvs.srv import SetBool
from dynamixel_sdk import *
import threading
import time


class XW540Controller:
    """Dynamixel XW540 모터 컨트롤러"""

    # Control Table Addresses
    ADDR_DRIVE_MODE = 10
    ADDR_OPERATING_MODE = 11
    ADDR_TORQUE_ENABLE = 64
    ADDR_POSITION_D_GAIN = 80
    ADDR_POSITION_I_GAIN = 82
    ADDR_POSITION_P_GAIN = 84
    ADDR_PROFILE_ACCELERATION = 108
    ADDR_PROFILE_VELOCITY = 112
    ADDR_GOAL_POSITION = 116
    ADDR_MOVING = 122
    ADDR_MOVING_STATUS = 123
    ADDR_PRESENT_VELOCITY = 128
    ADDR_PRESENT_POSITION = 132

    # Protocol version
    PROTOCOL_VERSION = 2.0

    # Position range: 0~4095 = 0~360 degrees
    POSITION_MAX = 4095
    DEGREE_MAX = 360.0

    # 베벨 기어 설정 (Z1=15T, Z2=30T)
    GEAR_RATIO = 2.0  # 기어비 2:1

    # 센서 각도 제한 (소프트웨어 리밋)
    SENSOR_ANGLE_MIN = 0.0    # 최소 센서 각도
    SENSOR_ANGLE_MAX = 92.0   # 최대 센서 각도 (모터 184도)

    # Operating range guard: nominal 0~92° plus ±3° margin for hand-positioning
    OPERATING_MIN = -3.0
    OPERATING_MAX = 95.0

    # Sanity guard: detects catastrophic miswiring or motor slip
    SANITY_MIN = -100.0
    SANITY_MAX = 200.0

    def __init__(self, device_name='/dev/ttyUSB0', baudrate=57600, motor_id=1):
        self.device_name = device_name
        self.baudrate = baudrate
        self.motor_id = motor_id
        self.port_handler = None
        self.packet_handler = None
        self.connected = False
        self.lock = threading.Lock()

    def connect(self) -> bool:
        """모터 연결"""
        try:
            self.port_handler = PortHandler(self.device_name)
            self.packet_handler = PacketHandler(self.PROTOCOL_VERSION)

            if not self.port_handler.openPort():
                return False

            if not self.port_handler.setBaudRate(self.baudrate):
                return False

            # Ping test
            _, result, _ = self.packet_handler.ping(self.port_handler, self.motor_id)
            if result != COMM_SUCCESS:
                return False

            self.connected = True
            return True
        except Exception:
            return False

    def disconnect(self):
        """연결 해제"""
        if self.port_handler:
            self.port_handler.closePort()
        self.connected = False

    def setup_motor(self, profile_velocity=200, profile_acceleration=100):
        """모터 초기 설정 (Drive Mode, Profile 등)"""
        with self.lock:
            # Torque Disable (설정 변경을 위해)
            self.packet_handler.write1ByteTxRx(
                self.port_handler, self.motor_id, self.ADDR_TORQUE_ENABLE, 0
            )
            time.sleep(0.1)

            # Drive Mode를 Reverse로 설정 (Bit 0 = 1) - 역방향 회전
            current_drive_mode, _, _ = self.packet_handler.read1ByteTxRx(
                self.port_handler, self.motor_id, self.ADDR_DRIVE_MODE
            )
            reverse_drive_mode = current_drive_mode | 0x01
            self.packet_handler.write1ByteTxRx(
                self.port_handler, self.motor_id, self.ADDR_DRIVE_MODE, reverse_drive_mode
            )
            time.sleep(0.1)

            # Operating Mode 4 = Extended Position Control (multi-turn, signed)
            # Eliminates single-turn wrap-around at sensor limits (0° / 92°)
            self.packet_handler.write1ByteTxRx(
                self.port_handler, self.motor_id, self.ADDR_OPERATING_MODE, 4
            )
            time.sleep(0.1)

            # Profile 설정 (부드러운 이동)
            self.packet_handler.write4ByteTxRx(
                self.port_handler, self.motor_id, self.ADDR_PROFILE_VELOCITY, profile_velocity
            )
            self.packet_handler.write4ByteTxRx(
                self.port_handler, self.motor_id, self.ADDR_PROFILE_ACCELERATION, profile_acceleration
            )
            time.sleep(0.1)

            # Torque Enable
            self.packet_handler.write1ByteTxRx(
                self.port_handler, self.motor_id, self.ADDR_TORQUE_ENABLE, 1
            )
            time.sleep(0.1)

        return True

    def _sensor_angle_to_motor_angle(self, sensor_angle: float) -> float:
        """센서 각도를 모터 각도로 변환 (기어비 2:1 적용)"""
        return sensor_angle * self.GEAR_RATIO

    def _motor_angle_to_sensor_angle(self, motor_angle: float) -> float:
        """모터 각도를 센서 각도로 변환"""
        return motor_angle / self.GEAR_RATIO

    def _angle_to_position(self, angle: float) -> int:
        """각도를 다이나믹셀 위치값으로 변환"""
        return int(angle * 4096 / 360)

    def _position_to_angle(self, position: int) -> float:
        """다이나믹셀 위치값을 각도로 변환"""
        return position * 360 / 4096

    def set_torque(self, enable: bool) -> bool:
        """토크 ON/OFF"""
        with self.lock:
            result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler, self.motor_id,
                self.ADDR_TORQUE_ENABLE, 1 if enable else 0
            )
            return result == COMM_SUCCESS

    def get_torque_status(self) -> bool:
        """토크 상태 확인"""
        with self.lock:
            value, _, _ = self.packet_handler.read1ByteTxRx(
                self.port_handler, self.motor_id, self.ADDR_TORQUE_ENABLE
            )
            return bool(value)

    def set_profile_velocity(self, velocity: int) -> bool:
        """프로파일 속도 설정 (단위: 0.229 rev/min)"""
        with self.lock:
            result, _ = self.packet_handler.write4ByteTxRx(
                self.port_handler, self.motor_id,
                self.ADDR_PROFILE_VELOCITY, velocity
            )
            return result == COMM_SUCCESS

    def clamp_sensor_angle(self, sensor_angle: float) -> float:
        """센서 각도를 허용 범위 내로 제한"""
        return max(self.SENSOR_ANGLE_MIN, min(self.SENSOR_ANGLE_MAX, sensor_angle))

    def is_angle_valid(self, sensor_angle: float) -> bool:
        """센서 각도가 허용 범위 내인지 확인"""
        return self.SENSOR_ANGLE_MIN <= sensor_angle <= self.SENSOR_ANGLE_MAX

    def _get_raw_motor_position(self) -> int:
        """모터 raw position 읽기 (Extended Position Mode → signed 32-bit)."""
        position, _, _ = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.motor_id, self.ADDR_PRESENT_POSITION
        )
        # dynamixel_sdk returns unsigned; sign-extend for Extended mode
        if position > 0x7FFFFFFF:
            position -= 0x100000000
        return position

    def is_current_position_safe(self) -> bool:
        """Interim guard during the Extended-Mode migration.

        Replaced by the sensor-angle-based guard in the next refactor step
        (see set_goal_position_degree). Symmetric raw window so signed
        positions near the limits do not falsely fail.
        """
        position = self._get_raw_motor_position()
        max_safe_position = int(200.0 / 360.0 * 4096)  # ~2276
        return -max_safe_position <= position <= max_safe_position

    def set_goal_position_degree(self, sensor_angle: float) -> tuple[bool, float, str]:
        """목표 센서 각도 설정 (도 단위, 기어비 적용)

        - 목표 각도를 0~92도 범위로 제한
        - 현재 위치가 범위 밖이면 이동 거부 (수동 리셋 필요)

        Returns:
            tuple: (성공 여부, 실제 적용된 센서 각도, 에러 메시지)
        """
        # 각도 제한 적용 (0~92도 범위로 클램핑)
        clamped_angle = self.clamp_sensor_angle(sensor_angle)

        with self.lock:
            # 현재 위치가 범위 밖이면 이동 거부
            # (한 바퀴 돌아서 가는 것 방지)
            raw_pos = self._get_raw_motor_position()
            max_safe_position = int(200.0 / 360.0 * 4096)  # ~2276

            if not (-max_safe_position <= raw_pos <= max_safe_position):
                current_motor_deg = raw_pos * 360.0 / 4096.0
                return False, clamped_angle, f"Position unsafe (interim): motor={current_motor_deg:.1f}° (raw={raw_pos})"

            motor_angle = self._sensor_angle_to_motor_angle(clamped_angle)
            position = self._angle_to_position(motor_angle)

            result, _ = self.packet_handler.write4ByteTxRx(
                self.port_handler, self.motor_id,
                self.ADDR_GOAL_POSITION, position
            )
            return result == COMM_SUCCESS, clamped_angle, ""

    def get_present_position_degree(self) -> float:
        """현재 센서 각도 읽기 (도 단위, 기어비 적용, signed)."""
        with self.lock:
            position, _, _ = self.packet_handler.read4ByteTxRx(
                self.port_handler, self.motor_id, self.ADDR_PRESENT_POSITION
            )
            # Sign-extend for Extended Position Mode
            if position > 0x7FFFFFFF:
                position -= 0x100000000
            motor_angle = self._position_to_angle(position)
            return self._motor_angle_to_sensor_angle(motor_angle)

    def is_moving(self) -> bool:
        """이동 중인지 확인"""
        with self.lock:
            moving, _, _ = self.packet_handler.read1ByteTxRx(
                self.port_handler, self.motor_id, self.ADDR_MOVING
            )
            return bool(moving)


class SonarTiltControllerNode(Node):
    """소나 틸트 컨트롤러 ROS2 노드"""

    def __init__(self):
        super().__init__('sonar_tilt')

        # Parameters
        self.declare_parameter('device', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 57600)
        self.declare_parameter('motor_id', 1)
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('profile_velocity', 200)  # 이동 속도
        self.declare_parameter('profile_acceleration', 100)  # 가속도
        self.declare_parameter('auto_home', False)

        device = self.get_parameter('device').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        motor_id = self.get_parameter('motor_id').get_parameter_value().integer_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        profile_velocity = self.get_parameter('profile_velocity').get_parameter_value().integer_value
        profile_acceleration = self.get_parameter('profile_acceleration').get_parameter_value().integer_value
        self.auto_home = self.get_parameter('auto_home').get_parameter_value().bool_value

        # Motor controller
        self.controller = XW540Controller(device, baudrate, motor_id)

        # Publishers
        self.pub_current_angle = self.create_publisher(
            Float32, '/sonar/tilt/current_angle', 10
        )
        self.pub_goal_angle = self.create_publisher(
            Float32, '/sonar/tilt/goal_angle', 10
        )
        self.pub_is_moving = self.create_publisher(
            Bool, '/sonar/tilt/is_moving', 10
        )
        self.pub_torque_status = self.create_publisher(
            Bool, '/sonar/tilt/torque_enabled', 10
        )

        # Subscribers
        self.sub_set_angle = self.create_subscription(
            Float32, '/sonar/tilt/set_angle', self.set_angle_callback, 10
        )

        # Services
        self.srv_torque = self.create_service(
            SetBool, '/sonar/tilt/set_torque', self.set_torque_callback
        )

        # 목표 각도 저장
        self.goal_angle = 0.0

        # Connect to motor
        self.get_logger().info(f'Connecting to Dynamixel on {device}...')
        if self.controller.connect():
            self.get_logger().info('Connected to Dynamixel XW540!')

            # 모터 초기 설정 (Drive Mode Reverse, Profile 등)
            self.get_logger().info('Setting up motor (Reverse Mode, Gear Ratio 2:1)...')
            self.controller.setup_motor(profile_velocity, profile_acceleration)
            self.get_logger().info('Motor setup complete!')

            # 현재 위치 읽어서 목표 각도 초기화
            self.goal_angle = self.controller.get_present_position_degree()
            self.get_logger().info(f'Current sensor angle: {self.goal_angle:.2f}°')
        else:
            self.get_logger().error('Failed to connect to Dynamixel!')

        # Timer for publishing status
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_status)

        self.get_logger().info('Sonar Tilt Controller Node started')
        self.get_logger().info('Gear Ratio: 2:1 (Z1=15T, Z2=30T)')
        self.get_logger().info(
            f'Angle Limit: {self.controller.SENSOR_ANGLE_MIN}° ~ {self.controller.SENSOR_ANGLE_MAX}° '
            f'(motor: {self.controller.SENSOR_ANGLE_MIN * self.controller.GEAR_RATIO}° ~ '
            f'{self.controller.SENSOR_ANGLE_MAX * self.controller.GEAR_RATIO}°)'
        )
        self.get_logger().info('Topics:')
        self.get_logger().info('  - /sonar/tilt/current_angle (Float32, pub) - sensor angle')
        self.get_logger().info('  - /sonar/tilt/goal_angle (Float32, pub) - sensor angle')
        self.get_logger().info('  - /sonar/tilt/is_moving (Bool, pub)')
        self.get_logger().info('  - /sonar/tilt/torque_enabled (Bool, pub)')
        self.get_logger().info('  - /sonar/tilt/set_angle (Float32, sub) - sensor angle')
        self.get_logger().info('Services:')
        self.get_logger().info('  - /sonar/tilt/set_torque (SetBool)')

    def publish_status(self):
        """현재 상태 퍼블리시"""
        if not self.controller.connected:
            return

        try:
            # 현재 각도 (센서 각도)
            current = self.controller.get_present_position_degree()
            msg_current = Float32()
            msg_current.data = current
            self.pub_current_angle.publish(msg_current)

            # 목표 각도
            msg_goal = Float32()
            msg_goal.data = self.goal_angle
            self.pub_goal_angle.publish(msg_goal)

            # 이동 중 여부
            msg_moving = Bool()
            msg_moving.data = self.controller.is_moving()
            self.pub_is_moving.publish(msg_moving)

            # 토크 상태
            msg_torque = Bool()
            msg_torque.data = self.controller.get_torque_status()
            self.pub_torque_status.publish(msg_torque)

        except Exception as e:
            self.get_logger().warn(f'Failed to read motor status: {e}')

    def set_angle_callback(self, msg: Float32):
        """목표 각도 설정 콜백 (센서 각도 기준)"""
        if not self.controller.connected:
            self.get_logger().warn('Motor not connected!')
            return

        requested_angle = msg.data

        # 범위 체크 및 경고
        if not self.controller.is_angle_valid(requested_angle):
            self.get_logger().warn(
                f'Requested angle {requested_angle:.2f}° is out of range! '
                f'Valid range: {self.controller.SENSOR_ANGLE_MIN}° ~ {self.controller.SENSOR_ANGLE_MAX}°. '
                f'Clamping to valid range.'
            )

        # 토크 자동 활성화
        if not self.controller.get_torque_status():
            self.controller.set_torque(True)
            self.get_logger().info('Torque enabled automatically')

        success, actual_angle, error_msg = self.controller.set_goal_position_degree(requested_angle)
        self.goal_angle = actual_angle
        motor_angle = actual_angle * self.controller.GEAR_RATIO

        if success:
            if requested_angle != actual_angle:
                self.get_logger().info(
                    f'Moving to sensor: {actual_angle:.2f}° (clamped from {requested_angle:.2f}°) '
                    f'(motor: {motor_angle:.2f}°)'
                )
            else:
                self.get_logger().info(f'Moving to sensor: {actual_angle:.2f}° (motor: {motor_angle:.2f}°)')
        else:
            if error_msg:
                self.get_logger().error(f'BLOCKED! {error_msg}. Manual reset required!')
            else:
                self.get_logger().error(f'Failed to set goal position: {actual_angle}°')

    def set_torque_callback(self, request, response):
        """토크 설정 서비스 콜백"""
        if not self.controller.connected:
            response.success = False
            response.message = 'Motor not connected'
            return response

        if self.controller.set_torque(request.data):
            response.success = True
            response.message = f'Torque {"enabled" if request.data else "disabled"}'
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = 'Failed to set torque'

        return response

    def destroy_node(self):
        """노드 종료"""
        self.get_logger().info('Shutting down...')
        if self.controller.connected:
            self.controller.set_torque(False)
            self.controller.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SonarTiltControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
