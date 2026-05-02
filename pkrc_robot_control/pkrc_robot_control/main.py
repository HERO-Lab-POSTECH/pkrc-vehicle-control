#!/usr/bin/env python3
"""
HERO Robot 메인 제어 프로그램 (CAN 버전 - 4개 쓰러스터 홀로노믹)
- 간단한 초기화 및 실행
- 모든 제어 로직은 모듈화되어 있음
- VESC CAN 통신 쓰러스터 제어 (4개 모터 홀로노믹)
- 제어 모드: PKRC 수동 제어 (추후 다른 모드 추가 예정)
"""

import rclpy
from sensor_msgs.msg import Joy, Image, CompressedImage
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from can_control_module import VESCControlNode
from relay_control_module import RelayControlModule
from lumen_module import LumenController
from GUI_module import WebGUIModule
from battery_module import BatteryMonitor
from rgb_led_module import BlueRoboticsLED
from PKRC_joy_module import PKRCJoystickController
from hovering_module import HoveringController
from PID_control_module import PIDModeController
from sonar_tilt_module import SonarTiltModule
import threading
import cv2
import time
import os
from datetime import datetime

DEFAULT_CAMERA_DEVICE = '/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._exploreHD_USB_Camera_SN00009-video-index0'


class HEROMainControl(VESCControlNode):
    """HERO Robot 메인 제어 노드 (CAN 버전 - 4개 쓰러스터 홀로노믹)"""
    
    def __init__(self):
        # VESCControlNode 초기화 (20Hz 업데이트)
        super().__init__(node_name='hero_main_control', update_rate=20.0)
        
        # 조이스틱 토픽 구독
        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # 카메라 장치 경로 (고정 경로 사용)
        self.camera_device = DEFAULT_CAMERA_DEVICE
        
        # === 웹 GUI 초기화 (가장 먼저) ===
        self.web_gui = WebGUIModule(
            host='0.0.0.0',
            port=5000,
            enable_camera=False,
            camera_device=self.camera_device,
            ros_node=self  # Foxglove 토픽 발행용 ROS2 노드 전달
        )
        
        # === 하드웨어 모듈 초기화 ===
        self.relay_controller = RelayControlModule(auto_init=True, web_gui=self.web_gui)
        self.lumen_controller = LumenController(pin=32, frequency=50, auto_init=True)  # Pin 32 (hero_ws/control 핀 매핑)
        self.battery_monitor = BatteryMonitor(
            can_channel='can0',
            low_voltage_threshold=13.0,
            critical_voltage_threshold=12.5,
            auto_init=True,
            web_gui=self.web_gui
        )
        
        try:
            self.rgb_led = BlueRoboticsLED(spi_bus=0, spi_device=0, web_gui=self.web_gui)
            self.rgb_led.set_orange()  # 초기: 주황색 (시동 OFF)
            self.get_logger().info('✅ RGB LED 초기화 완료')
        except Exception as e:
            self.get_logger().warn(f'⚠️  RGB LED 초기화 실패: {e}')
            self.rgb_led = None
        
        # === 배터리 모니터링 시작 ===
        self.battery_monitor.start_monitoring(update_interval=5.0)
        
        # === USB 카메라 초기화 ===
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.video_lock = threading.Lock()
        self.camera = None
        self.camera_reconnect_interval = 3.0  # 재연결 시도 간격 (초)
        self.last_frame_time = time.time()
        self.frame_timeout = 2.0  # 프레임 타임아웃 (초)

        # === ROS2 카메라 토픽 퍼블리셔 ===
        self.cv_bridge = CvBridge()
        self.pub_camera_compressed = self.create_publisher(
            CompressedImage, '/camera/image/compressed', 10
        )
        self.pub_camera_raw = self.create_publisher(
            Image, '/camera/image_raw', 1  # Raw는 큰 데이터라 QoS 1로
        )
        self.camera_publish_rate = 15  # Hz (Foxglove용 발행 속도)
        self.last_camera_publish_time = 0.0

        # 카메라 초기화
        self._init_camera()

        # === 녹화 관련 변수 ===
        self.recording_just_stopped = False  # 녹화 완료 플래시용
        self.is_recording = False
        self.video_writer = None
        self.recording_start_time = None
        self.recording_frame_count = 0
        self.current_video_filename = None
        self.video_dir = os.path.join(os.path.dirname(__file__), 'video')
        os.makedirs(self.video_dir, exist_ok=True)
        
        # 카메라 스레드 시작
        self.camera_thread = threading.Thread(target=self.camera_loop, daemon=True)
        self.camera_thread.start()

        # === 소나 틸트 모듈 초기화 ===
        try:
            self.sonar_tilt = SonarTiltModule(
                ros_node=self,
                web_gui=self.web_gui,
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
            web_gui=self.web_gui,
            logger=self.get_logger(),
            main_node=self,  # 녹화 제어를 위한 메인 노드
            sonar_tilt=self.sonar_tilt,  # 소나 틸트 모듈
            deadzone=0.05,  # 조이스틱 데드존 증가 (20%)
            sensitivity_scale=0.5,
            max_current=8.0  # 최대 전류 8A
        )
        
        # === 호버링 컨트롤러 초기화 ===
        self.hovering_controller = HoveringController(
            vesc_controller=self.controller,
            web_gui=self.web_gui,
            logger=self.get_logger(),
            max_current=8.0,
            enable_yaw_control=True,  # bag 분석 결과: yaw_cmd 방향 반전 필요
            invert_yaw=True
        )
        # PKRC 조이스틱에 호버링 컨트롤러 연결
        self.joystick.hovering = self.hovering_controller

        # === PID 모드 컨트롤러 초기화 ===
        self.pid_controller = PIDModeController(
            vesc_controller=self.controller,
            web_gui=self.web_gui,
            logger=self.get_logger(),
            max_current=8.0,
            enable_yaw_control=True,
            invert_yaw=True
        )
        # PKRC 조이스틱에 PID 컨트롤러 연결
        self.joystick.pid_ctrl = self.pid_controller

        # === Fast-LIO 오도메트리 구독 (호버링/PID 위치 피드백) ===
        self.fastlio_odom_sub = self.create_subscription(
            Odometry,
            '/fast_lio/odometry',
            self.fastlio_odom_callback,
            10
        )

        # === Cartographer 오도메트리 구독 (호버링/PID 위치 피드백 - 대체 소스) ===
        self.carto_odom_sub = self.create_subscription(
            Odometry,
            '/cartographer_2d/odometry',
            self.carto_odom_callback,
            10
        )

        # === VESC 전류 퍼블리셔 (rosbag 기록용) ===
        self.vesc_cmd_pub = self.create_publisher(
            Float32MultiArray, '/hero/vesc_currents', 10
        )

        # === 호버링 제어 타이머 (20Hz) ===
        self.hovering_timer = self.create_timer(0.05, self.hovering_update_loop)

        # === PID 모드 제어 타이머 (20Hz) ===
        self.pid_timer = self.create_timer(0.05, self.pid_update_loop)

        # === 오도메트리 타임아웃 체크 타이머 (10Hz) ===
        self.odom_timeout_timer = self.create_timer(0.1, self.odom_timeout_check_loop)

        # === 조이스틱 타임아웃 체크 타이머 ===
        # 50ms마다 체크 (20Hz)
        self.timeout_timer = self.create_timer(0.05, self.timeout_check_loop)

        # === 웹 GUI 시작 ===
        self.start_web_gui()
        
        # === 시작 메시지 ===
        self.print_startup_info()
    
    def _init_camera(self):
        """카메라 초기화 (재연결 지원)"""
        if self.camera is not None:
            try:
                self.camera.release()
            except:
                pass
            self.camera = None

        try:
            self.camera = cv2.VideoCapture(self.camera_device, cv2.CAP_V4L2)

            if not self.camera.isOpened() and self.camera_device != '/dev/video0':
                self.get_logger().warn(f'⚠️  {self.camera_device} 장치를 열 수 없어 /dev/video0로 재시도합니다')
                self.camera.release()
                self.camera = cv2.VideoCapture(0, cv2.CAP_V4L2)
                if self.camera.isOpened():
                    self.camera_device = '/dev/video0'

            if self.camera and self.camera.isOpened():
                # 고화질 설정 (1280x720)
                self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                self.camera.set(cv2.CAP_PROP_FPS, 30)
                self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 버퍼 최소화로 지연 감소

                # 실제 설정된 값 확인
                actual_w = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_h = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
                self.get_logger().info(f'✅ USB 카메라 초기화 완료 ({self.camera_device}, {actual_w}x{actual_h})')
                self.last_frame_time = time.time()
                return True
            else:
                self.get_logger().warn('⚠️  USB 카메라를 찾을 수 없습니다')
                return False
        except Exception as e:
            self.get_logger().error(f'❌ 카메라 초기화 실패: {e}')
            return False

    def camera_loop(self):
        """카메라 프레임 읽기 루프 (+ 녹화 + 자동 재연결)"""
        reconnect_wait_until = 0

        while rclpy.ok():
            current_time = time.time()

            # 카메라 연결 상태 체크 및 재연결
            if self.camera is None or not self.camera.isOpened():
                if current_time >= reconnect_wait_until:
                    self.get_logger().warn('🔄 카메라 재연결 시도...')
                    if self._init_camera():
                        reconnect_wait_until = 0
                    else:
                        reconnect_wait_until = current_time + self.camera_reconnect_interval
                time.sleep(0.1)
                continue

            # 프레임 읽기
            try:
                ret, frame = self.camera.read()
            except Exception as e:
                self.get_logger().error(f'❌ 카메라 읽기 오류: {e}')
                ret = False

            if ret and frame is not None:
                self.last_frame_time = current_time
                with self.frame_lock:
                    self.latest_frame = frame.copy()
                self.web_gui.latest_frame = frame.copy()

                # 녹화 중이면 프레임 저장
                if self.is_recording:
                    with self.video_lock:
                        if self.video_writer is not None:
                            try:
                                self.video_writer.write(frame)
                                self.recording_frame_count += 1

                                # GUI 상태 업데이트
                                elapsed = current_time - self.recording_start_time
                                self.web_gui.state['recording'] = {
                                    'status': True,
                                    'duration': elapsed,
                                    'filename': os.path.basename(self.current_video_filename)
                                }
                            except Exception as e:
                                self.get_logger().error(f'❌ 프레임 저장 실패: {e}')

                # === ROS2 토픽으로 카메라 이미지 발행 (Foxglove용) ===
                if current_time - self.last_camera_publish_time >= 1.0 / self.camera_publish_rate:
                    self.last_camera_publish_time = current_time
                    try:
                        # Compressed Image (JPEG) - 대역폭 효율적
                        compressed_msg = CompressedImage()
                        compressed_msg.header.stamp = self.get_clock().now().to_msg()
                        compressed_msg.header.frame_id = 'camera'
                        compressed_msg.format = 'jpeg'
                        _, jpeg_data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                        compressed_msg.data = jpeg_data.tobytes()
                        self.pub_camera_compressed.publish(compressed_msg)
                    except Exception as e:
                        pass  # 발행 실패 무시
            else:
                # 프레임 타임아웃 체크
                if current_time - self.last_frame_time > self.frame_timeout:
                    self.get_logger().warn(f'⚠️  카메라 프레임 타임아웃 ({self.frame_timeout}초)')
                    try:
                        self.camera.release()
                    except:
                        pass
                    self.camera = None
                    reconnect_wait_until = current_time + self.camera_reconnect_interval

            time.sleep(0.025)  # ~40 FPS 시도
    
    def fastlio_odom_callback(self, msg: Odometry):
        """Fast-LIO 오도메트리 콜백"""
        self.hovering_controller.update_fastlio_odometry(msg)
        self.pid_controller.update_fastlio_odometry(msg)

    def carto_odom_callback(self, msg: Odometry):
        """Cartographer 오도메트리 콜백"""
        self.hovering_controller.update_carto_odometry(msg)
        self.pid_controller.update_carto_odometry(msg)

    def _publish_vesc_currents(self):
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

    def hovering_update_loop(self):
        """호버링 제어 루프 (20Hz 타이머)"""
        if (self.joystick.control_mode == PKRCJoystickController.MODE_HOVERING
                and self.joystick.is_armed):
            self.hovering_controller.compute_and_send_commands(
                sensitivity_scale=self.joystick.sensitivity_scale
            )
        self._publish_vesc_currents()

    def pid_update_loop(self):
        """PID 모드 제어 루프 (20Hz 타이머)"""
        if (self.joystick.control_mode == PKRCJoystickController.MODE_PID
                and self.joystick.is_armed):
            self.pid_controller.compute_and_send_commands(
                sensitivity_scale=self.joystick.sensitivity_scale
            )

    def odom_timeout_check_loop(self):
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
                if self.joystick.gui:
                    self.joystick.gui.update_system(
                        is_armed=self.joystick.is_armed,
                        sensitivity=self.joystick.sensitivity_scale,
                        lumen_brightness=self.joystick.lumen.get_brightness(),
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
                if self.joystick.gui:
                    self.joystick.gui.update_system(
                        is_armed=self.joystick.is_armed,
                        sensitivity=self.joystick.sensitivity_scale,
                        lumen_brightness=self.joystick.lumen.get_brightness(),
                        control_mode=PKRCJoystickController.MODE_NORMAL
                    )

    def start_web_gui(self):
        """웹 GUI 시작"""
        try:
            self.web_thread = threading.Thread(target=self.web_gui.start, daemon=True)
            self.web_thread.start()
            self.get_logger().info('✅ 웹 GUI 서버 시작')
        except Exception as e:
            self.get_logger().error(f'❌ 웹 GUI 시작 실패: {e}')
    
    def start_recording(self):
        """녹화 시작"""
        if self.is_recording:
            self.get_logger().warn('⚠️  이미 녹화 중입니다')
            return
        
        # 현재 프레임에서 실제 해상도 가져오기
        if self.latest_frame is None:
            self.get_logger().error('❌ 카메라 프레임이 없습니다')
            return
        
        with self.frame_lock:
            height, width = self.latest_frame.shape[:2]
        
        # 파일명 생성
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.current_video_filename = os.path.join(self.video_dir, f"video_{timestamp}.avi")
        
        # VideoWriter 초기화 (MJPG 코덱, 실제 해상도 사용)
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        with self.video_lock:
            self.video_writer = cv2.VideoWriter(
                self.current_video_filename,
                fourcc,
                30.0,  # FPS
                (width, height)  # 실제 프레임 해상도
            )
            
            if not self.video_writer.isOpened():
                self.get_logger().error('❌ 비디오 파일 생성 실패')
                self.video_writer.release()
                self.video_writer = None
                return
        
        self.is_recording = True
        self.recording_start_time = time.time()
        self.recording_frame_count = 0  # 프레임 카운터 추가
        self.get_logger().info(f'🔴 녹화 시작: {os.path.basename(self.current_video_filename)} ({width}x{height})')
    
    def stop_recording(self):
        """녹화 중지"""
        if not self.is_recording:
            self.get_logger().warn('⚠️  녹화 중이 아닙니다')
            return
        
        self.is_recording = False
        
        with self.video_lock:
            if self.video_writer is not None:
                self.video_writer.release()
                self.video_writer = None
        
        duration = time.time() - self.recording_start_time
        self.get_logger().info(f'⏹️  녹화 중지 (길이: {duration:.1f}초, 프레임: {self.recording_frame_count}개)')
        self.get_logger().info(f'💾 저장 완료: {os.path.basename(self.current_video_filename)}')
        
        # 파일 크기 확인
        if os.path.exists(self.current_video_filename):
            file_size = os.path.getsize(self.current_video_filename) / (1024 * 1024)  # MB
            self.get_logger().info(f'📦 파일 크기: {file_size:.2f} MB')
        else:
            self.get_logger().error(f'❌ 파일이 생성되지 않았습니다!')
        
        # GUI 상태 업데이트 (완료 플래시)
        self.web_gui.state['recording'] = {
            'status': False,
            'flash': True,  # 파란색 플래시
            'duration': 0,
            'filename': ''
        }

        # 1초 후 flash 끄기
        def clear_flash():
            time.sleep(1.0)
            self.web_gui.state['recording']['flash'] = False
        threading.Thread(target=clear_flash, daemon=True).start()

        self.current_video_filename = None
        self.recording_frame_count = 0
    
    def toggle_recording(self):
        """녹화 토글"""
        if self.is_recording:
            self.stop_recording()
        else:
            self.start_recording()
    
    def joy_callback(self, msg: Joy):
        """조이스틱 콜백 (모든 처리는 joystick 모듈에서)"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.joystick.handle_joy_message(msg, current_time)
    
    def timeout_check_loop(self):
        """조이스틱 타임아웃 체크 루프"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        self.joystick.check_timeout(current_time)
    
    def print_startup_info(self):
        """시작 정보 출력"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('HERO Robot 제어 시스템 시작 (CAN - 4개 쓰러스터 홀로노믹)')
        self.get_logger().info('=' * 60)
        self.get_logger().info('')
        self.get_logger().info('웹 인터페이스:')
        self.get_logger().info(f'  URL: http://localhost:{self.web_gui.port}')
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
        print('\n🔄 프로그램 종료 중...')
        
        # 시동 OFF
        try:
            node.joystick.disarm_system()
        except:
            pass
        
        # 카메라 정리
        try:
            if node.camera and node.camera.isOpened():
                node.camera.release()
        except:
            pass
        
        # 릴레이 정리
        try:
            node.relay_controller.cleanup()
        except:
            pass
        
        # Lumen 정리
        try:
            node.lumen_controller.cleanup()
        except:
            pass
        
        # 배터리 모니터 정리
        try:
            node.battery_monitor.cleanup()
        except:
            pass
        
        # RGB LED 파란색으로 (종료 표시)
        try:
            if node.rgb_led:
                node.rgb_led.set_blue()
                time.sleep(0.2)
                node.rgb_led.spi.close()
                print('🔵 RGB LED: 파란색 (종료 상태)')
        except:
            pass
        
        # 웹 GUI 정리
        try:
            node.web_gui.stop()
        except:
            pass
        
        # ROS2 종료 (VESCControlNode의 shutdown_node 사용)
        try:
            node.shutdown_node()
        except:
            pass
        
        try:
            rclpy.shutdown()
        except:
            pass
        
        print('✅ 프로그램 종료 완료\n')


if __name__ == '__main__':
    main()

