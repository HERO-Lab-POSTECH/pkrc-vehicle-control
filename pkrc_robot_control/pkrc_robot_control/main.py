#!/usr/bin/env python3
"""
HERO Robot 메인 제어 프로그램 (CAN 버전 - 4개 쓰러스터 홀로노믹)
- 간단한 초기화 및 실행
- 모든 제어 로직은 모듈화되어 있음
- VESC CAN 통신 쓰러스터 제어 (4개 모터 홀로노믹)
- 제어 모드: PKRC 수동 제어 (추후 다른 모드 추가 예정)
"""

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from .actuators.can_control import VESCControlNode
from .actuators.relay_control import RelayControlModule
from .actuators.lumen import LumenController
from .sensors.battery import BatteryMonitor
from .actuators.rgb_led import BlueRoboticsLED
from .input.pkrc_joy import PKRCJoystickController
from .control.hovering import HoveringController
from .control.pid_control import PIDModeController
from .sensors.sonar_tilt import SonarTiltModule
from .gui.null_gui import NullGUI
from .sensors.camera import CameraManager
from .control.odom_router import OdometryRouter
from . import _params
import time


class HEROMainControl(VESCControlNode):
    """HERO Robot 메인 제어 노드 (CAN 버전 - 4개 쓰러스터 홀로노믹)"""
    
    def __init__(self):
        # VESCControlNode 초기화 (20Hz 업데이트)
        super().__init__(node_name='hero_main_control', update_rate=20.0)

        # Declare all parameters first; subsequent module construction reads them.
        _params.declare_all(self)

        # 조이스틱 토픽 구독
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # === GUI placeholder (NullGUI: no-op, 미래 통합 Qt GUI로 교체 예정) ===
        self.gui = NullGUI()

        # === 하드웨어 모듈 초기화 ===
        self.relay_controller = RelayControlModule(
            auto_init=True, gui=self.gui, logger=self.get_logger(), node=self,
        )
        try:
            self.lumen_controller = LumenController(
                pin=32, frequency=50, auto_init=True,
                logger=self.get_logger(),
            )  # Pin 32 (hero_ws/control 핀 매핑)
            self.get_logger().info('✅ Lumen 라이트 초기화 완료')
        except Exception as e:
            self.get_logger().warn(f'⚠️  Lumen 라이트 초기화 실패: {e}')
            self.lumen_controller = None
        self.battery_monitor = BatteryMonitor(
            can_channel='can0',
            low_voltage_threshold=_params.load_scalar(self, 'battery.low_voltage_threshold'),
            critical_voltage_threshold=_params.load_scalar(self, 'battery.critical_voltage_threshold'),
            auto_init=True,
            gui=self.gui,
            logger=self.get_logger(),
        )
        
        try:
            self.rgb_led = BlueRoboticsLED(
                    spi_bus=0, spi_device=0,
                    logger=self.get_logger(),
                    node=self,
                )
            self.rgb_led.set_orange()  # 초기: 주황색 (시동 OFF)
            self.get_logger().info('✅ RGB LED 초기화 완료')
        except Exception as e:
            self.get_logger().warn(f'⚠️  RGB LED 초기화 실패: {e}')
            self.rgb_led = None
        
        # === 배터리 모니터링 시작 ===
        self.battery_monitor.start_monitoring(node=self, update_interval=5.0)

        # === USB 카메라 (CameraManager: 자체 ROS2 timer 사용) ===
        self.camera_mgr = CameraManager(node=self)

        # === 소나 틸트 모듈 초기화 ===
        try:
            self.sonar_tilt = SonarTiltModule(
                ros_node=self,
                logger=self.get_logger()
            )
            self.get_logger().info('소나 틸트 모듈 초기화 완료')
        except Exception as e:
            self.get_logger().warn(f'소나 틸트 모듈 초기화 실패: {e}')
            self.sonar_tilt = None

        # === 조이스틱 컨트롤러 초기화 ===
        self.joystick = PKRCJoystickController(
            vesc_controller=self.controller,  # VESCControlNode의 controller 사용
            relay_controller=self.relay_controller,
            lumen_controller=self.lumen_controller,
            rgb_led=self.rgb_led,
            gui=self.gui,
            logger=self.get_logger(),
            main_node=self,  # 녹화 제어를 위한 메인 노드
            sonar_tilt=self.sonar_tilt,  # 소나 틸트 모듈
            deadzone=_params.load_scalar(self, 'joystick.deadzone'),
            sensitivity_scale=0.5,
            max_current=_params.load_scalar(self, 'joystick.max_current'),
            joy_timeout=_params.load_scalar(self, 'joystick.joy_timeout'),
        )
        
        # === 호버링 컨트롤러 초기화 ===
        self.hovering_controller = HoveringController(
            vesc_controller=self.controller,
            gui=self.gui,
            logger=self.get_logger(),
            max_current=_params.load_scalar(self, 'joystick.max_current'),
            odom_timeout_sec=_params.load_scalar(self, 'odom_timeout_sec'),
            enable_yaw_control=_params.load_scalar(self, 'enable_yaw_control'),
            invert_yaw=_params.load_scalar(self, 'invert_yaw'),
            fastlio_params=_params.load_pid_dict(self, 'hovering.fastlio'),
            cartographer_params=_params.load_pid_dict(self, 'hovering.cartographer'),
        )
        # PKRC 조이스틱에 호버링 컨트롤러 연결
        self.joystick.hovering = self.hovering_controller

        # === PID 모드 컨트롤러 초기화 ===
        self.pid_controller = PIDModeController(
            vesc_controller=self.controller,
            gui=self.gui,
            logger=self.get_logger(),
            max_current=_params.load_scalar(self, 'joystick.max_current'),
            odom_timeout_sec=_params.load_scalar(self, 'odom_timeout_sec'),
            enable_yaw_control=_params.load_scalar(self, 'enable_yaw_control'),
            invert_yaw=_params.load_scalar(self, 'invert_yaw'),
            fastlio_params=_params.load_pid_dict(self, 'pid.fastlio'),
            cartographer_params=_params.load_pid_dict(self, 'pid.cartographer'),
        )
        # PKRC 조이스틱에 PID 컨트롤러 연결
        self.joystick.pid_ctrl = self.pid_controller

        # === Odometry router (Fast-LIO + Cartographer → hovering, pid 양쪽 fan-out) ===
        self.odom_router = OdometryRouter(
            node=self,
            hovering_ctrl=self.hovering_controller,
            pid_ctrl=self.pid_controller,
        )

        # === VESC 전류 퍼블리셔 (rosbag 기록용) ===
        self.vesc_cmd_pub = self.create_publisher(
            Float32MultiArray, '/hero/vesc_currents', 10
        )

        # === 외부 모니터링용 publisher (BEST_EFFORT, D6) ===
        # /pkrc/system/state: [is_armed, sensitivity, lumen_brightness], 1Hz
        # /pkrc/motors/cmd_current: 4개 VESC actual current, 2Hz (제어주기 20Hz는 GUI엔 과함)
        monitoring_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub_pkrc_system = self.create_publisher(
            Float32MultiArray, '/pkrc/system/state', monitoring_qos
        )
        self.pub_pkrc_motors = self.create_publisher(
            Float32MultiArray, '/pkrc/motors/cmd_current', monitoring_qos
        )
        self._monitoring_system_timer = self.create_timer(1.0, self._publish_pkrc_system)
        self._monitoring_motors_timer = self.create_timer(0.5, self._publish_pkrc_motors)

        # === 호버링 제어 타이머 (20Hz) ===
        self.hovering_timer = self.create_timer(0.05, self.hovering_update_loop)

        # === PID 모드 제어 타이머 (20Hz) ===
        self.pid_timer = self.create_timer(0.05, self.pid_update_loop)

        # === 오도메트리 타임아웃 체크 타이머 (10Hz) ===
        self.odom_timeout_timer = self.create_timer(0.1, self.odom_timeout_check_loop)

        # === 조이스틱 타임아웃 체크 타이머 ===
        # 50ms마다 체크 (20Hz)
        self.timeout_timer = self.create_timer(0.05, self.timeout_check_loop)

        # === 시작 메시지 ===
        self.print_startup_info()
    
    def _publish_vesc_currents(self) -> None:
        """VESC 실제 출력 전류를 토픽으로 발행 (rosbag 기록 및 모니터링용)"""
        status = self.controller.get_current_status()
        msg = Float32MultiArray()
        msg.data = [
            float(status['vesc_1']['actual']),
            float(status['vesc_2']['actual']),
            float(status['vesc_3']['actual']),
            float(status['vesc_4']['actual']),
        ]
        self.vesc_cmd_pub.publish(msg)

    def _publish_pkrc_system(self) -> None:
        """1Hz heartbeat: [is_armed, sensitivity, lumen_brightness]."""
        lumen_brightness = (
            self.lumen_controller.get_brightness()
            if self.lumen_controller is not None else 0.0
        )
        msg = Float32MultiArray()
        msg.data = [
            float(self.joystick.is_armed),
            float(self.joystick.sensitivity_scale),
            float(lumen_brightness),
        ]
        self.pub_pkrc_system.publish(msg)

    def _publish_pkrc_motors(self) -> None:
        """2Hz 다운샘플 발행: 4개 VESC actual current."""
        status = self.controller.get_current_status()
        msg = Float32MultiArray()
        msg.data = [
            float(status['vesc_1']['actual']),
            float(status['vesc_2']['actual']),
            float(status['vesc_3']['actual']),
            float(status['vesc_4']['actual']),
        ]
        self.pub_pkrc_motors.publish(msg)

    def hovering_update_loop(self) -> None:
        """호버링 제어 루프 (20Hz 타이머)"""
        if (self.joystick.control_mode == PKRCJoystickController.MODE_HOVERING
                and self.joystick.is_armed):
            self.hovering_controller.compute_and_send_commands(
                sensitivity_scale=self.joystick.sensitivity_scale
            )
        self._publish_vesc_currents()

    def pid_update_loop(self) -> None:
        """PID 모드 제어 루프 (20Hz 타이머)"""
        if (self.joystick.control_mode == PKRCJoystickController.MODE_PID
                and self.joystick.is_armed):
            self.pid_controller.compute_and_send_commands(
                sensitivity_scale=self.joystick.sensitivity_scale
            )

    def odom_timeout_check_loop(self) -> None:
        """오도메트리 타임아웃 체크 루프 (10Hz) - 호버링/PID 중 토픽 끊김 감지"""
        if not self.joystick.is_armed:
            return

        mode = self.joystick.control_mode

        # 호버링 모드 타임아웃
        if mode == PKRCJoystickController.MODE_HOVERING:
            if self.hovering_controller.check_odom_timeout():
                source = self.hovering_controller.odom_source
                self.get_logger().warn(
                    f'[호버링] {source} 오도메트리 끊김 -> 노말 모드로 복귀'
                )
                self.hovering_controller.deactivate(reason=f'{source} 토픽 끊김')
                self.joystick.control_mode = PKRCJoystickController.MODE_NORMAL
                if self.rgb_led:
                    self.rgb_led.set_green()
                self.joystick.gui.update_system(
                    is_armed=self.joystick.is_armed,
                    sensitivity=self.joystick.sensitivity_scale,
                    lumen_brightness=(self.joystick.lumen.get_brightness()
                                      if self.joystick.lumen is not None else 0.0),
                    control_mode=PKRCJoystickController.MODE_NORMAL
                )

        # PID 모드 타임아웃
        elif mode == PKRCJoystickController.MODE_PID:
            if self.pid_controller.check_odom_timeout():
                source = self.pid_controller.odom_source
                self.get_logger().warn(
                    f'[PID 모드] {source} 오도메트리 끊김 -> 노말 모드로 복귀'
                )
                self.pid_controller.deactivate(reason=f'{source} 토픽 끊김')
                self.joystick.control_mode = PKRCJoystickController.MODE_NORMAL
                if self.rgb_led:
                    self.rgb_led.set_green()
                self.joystick.gui.update_system(
                    is_armed=self.joystick.is_armed,
                    sensitivity=self.joystick.sensitivity_scale,
                    lumen_brightness=(self.joystick.lumen.get_brightness()
                                      if self.joystick.lumen is not None else 0.0),
                    control_mode=PKRCJoystickController.MODE_NORMAL
                )

    def joy_callback(self, msg: Joy) -> None:
        """조이스틱 콜백 (모든 처리는 joystick 모듈에서)"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.joystick.handle_joy_message(msg, current_time)
    
    def timeout_check_loop(self) -> None:
        """조이스틱 타임아웃 체크 루프"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.joystick.check_timeout(current_time)
    
    def print_startup_info(self):
        """시작 정보 출력"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('HERO Robot 제어 시스템 시작 (CAN - 4개 쓰러스터 홀로노믹)')
        self.get_logger().info('=' * 60)
        self.get_logger().info('')
        self.get_logger().info('VESC CAN 통신 (4개 쓰러스터 홀로노믹):')
        self.get_logger().info('  최대 전류: 8.0A')
        self.get_logger().info('  VESC 1 (오른쪽): 0x101 - 전후이동')
        self.get_logger().info('  VESC 2 (뒤쪽):   0x102 - 좌우이동')
        self.get_logger().info('  VESC 3 (왼쪽):   0x103 - 전후이동')
        self.get_logger().info('  VESC 4 (앞쪽):   0x104 - 좌우이동')
        self.get_logger().info('  업데이트 주기: 20Hz')
        self.get_logger().info('')
        self.get_logger().info('조이스틱 제어 (PKRC 모드):')
        self.get_logger().info('  왼쪽 X축: 좌우 스트레이프')
        self.get_logger().info('  왼쪽 Y축: 전진/후진')
        self.get_logger().info('  오른쪽 X축: 좌우 회전')
        self.get_logger().info('  Menu: 시동 ON | Options: 시동 OFF')
        self.get_logger().info('  D-Pad 상하: 감도 조절 | D-Pad 좌우: 라이트 밝기')
        self.get_logger().info('  LB/RB + X/Y/B: 릴레이 제어')
        self.get_logger().info('')
        self.get_logger().info('제어 모드 전환 (RT 홀드 + 버튼):')
        self.get_logger().info('  RT + Y: 노말 모드 (수동 쓰러스터 제어)')
        self.get_logger().info('  RT + A: 호버링 모드 (현재 위치 고정)')
        self.get_logger().info('    -> 오도메트리 소스 자동 선택: Fast-LIO 우선, 없으면 Cartographer')
        self.get_logger().info('    -> 토픽 끊김 시 자동으로 노말 모드 복귀')
        self.get_logger().info('')
        self.get_logger().info('소나 틸트 제어 (LT 홀드 + 버튼):')
        self.get_logger().info('  Y 버튼: 0도')
        self.get_logger().info('  A 버튼: 90도')
        self.get_logger().info('  X 버튼: 단계 증가 (0->30->45->60->90)')
        self.get_logger().info('  B 버튼: 단계 감소')
        self.get_logger().info('')
        self.get_logger().info('상태: 시동 OFF (주황색)')
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    
    node = HEROMainControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자에 의한 종료')
    finally:
        node.get_logger().info('프로그램 종료 중...')

        # 시동 OFF
        try:
            node.joystick.disarm_system()
        except Exception as e:
            node.get_logger().error(f'disarm_system 실패: {e}')

        # 카메라 정리
        try:
            node.camera_mgr.shutdown()
        except Exception as e:
            node.get_logger().error(f'camera shutdown 실패: {e}')

        # 릴레이 정리
        try:
            node.relay_controller.cleanup()
        except Exception as e:
            node.get_logger().error(f'relay_controller cleanup 실패: {e}')

        # Lumen 정리
        try:
            if node.lumen_controller is not None:
                node.lumen_controller.cleanup()
        except Exception as e:
            node.get_logger().error(f'lumen_controller cleanup 실패: {e}')

        # 배터리 모니터 정리
        try:
            node.battery_monitor.cleanup()
        except Exception as e:
            node.get_logger().error(f'battery_monitor cleanup 실패: {e}')

        # RGB LED 파란색으로 (종료 표시)
        try:
            if node.rgb_led:
                node.rgb_led.set_blue()
                time.sleep(0.2)
                node.rgb_led.spi.close()
                node.get_logger().info('🔵 RGB LED: 파란색 (종료 상태)')
        except Exception as e:
            node.get_logger().error(f'rgb_led cleanup 실패: {e}')

        # ROS2 종료 (VESCControlNode의 shutdown_node 사용)
        try:
            node.shutdown_node()
        except Exception as e:
            node.get_logger().error(f'shutdown_node 실패: {e}')

        try:
            rclpy.shutdown()
        except Exception as e:
            # logger도 이미 종료된 시점이라 print fallback
            print(f'rclpy.shutdown 실패: {e}')

        # logger context torn down at this point; print is the only safe option
        print('✅ 프로그램 종료 완료\n')


if __name__ == '__main__':
    main()

