#!/usr/bin/env python3
"""
웹 GUI 모듈
- Flask 웹 서버를 모듈로 제공
- 다른 ROS2 노드에서 쉽게 사용 가능
- 실시간 데이터 업데이트
- ROS2 토픽 발행 (Foxglove 연동용)
"""

from flask import Flask, render_template, Response
from flask_socketio import SocketIO
import threading
import cv2
import time
import os
import json

# ROS2 토픽 발행을 위한 Optional import
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import Float32, Float32MultiArray, String
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("ROS2 not available - Foxglove topic publishing disabled")


class WebGUIModule:
    """웹 GUI 모듈 클래스"""
    
    def __init__(self, host='0.0.0.0', port=5000, enable_camera=True, camera_device=None, ros_node=None):
        """
        초기화

        Args:
            host: 서버 호스트 (기본: 모든 인터페이스)
            port: 서버 포트 (기본: 5000)
            enable_camera: 카메라 스트리밍 활성화 여부
            camera_device: 고정 카메라 경로 (예: /dev/v4l/by-id/...), None이면 기본값 사용
            ros_node: ROS2 노드 (Foxglove 토픽 발행용)
        """
        self.host = host
        self.port = port
        self.enable_camera = enable_camera
        self.running = False  # 카메라 루프 제어용
        self.camera_device = camera_device or '/dev/video0'
        self.ros_node = ros_node

        # ROS2 Publishers (Foxglove용)
        self.ros_publishers = {}
        if ROS2_AVAILABLE and ros_node is not None:
            self._init_ros_publishers()
        
        # 현재 상태 저장
        self.state = {
            'joystick': {
                'left_x': 0.0,
                'left_y': 0.0,
                'right_x': 0.0,
                'right_y': 0.0,
                'buttons': [],
                'axes': []
            },
            'system': {
                'is_armed': False,
                'sensitivity': 0.5,
                'lumen_brightness': 0.0,
                'control_mode': 'NORMAL'
            },
            'battery': {
                'voltage': 0.0,
                'percentage': 0.0,
                'status': 'unknown'
            },
            'relays': {
                'relay_1': False,
                'relay_2': False,
                'relay_3': False
            },
            'motor_commands': {
                'vesc_1': 0.0,
                'vesc_2': 0.0,
                'vesc_3': 0.0,
                'vesc_4': 0.0
            },
            'led': {
                'color': 'orange',  # 'green', 'orange', 'blue'
                'color_name': '주황색'
            },
            'recording': {
                'status': False,
                'duration': 0,
                'filename': ''
            },
            'sonar_tilt': {
                'current_angle': 0.0,
                'goal_angle': 0.0
            },
            'timestamp': time.time()
        }
        
        # Flask 앱 생성
        template_dir = os.path.join(os.path.dirname(__file__), 'templates')
        self.app = Flask(__name__, template_folder=template_dir)
        self.app.config['SECRET_KEY'] = 'hero_robot_secret_key'
        
        # SocketIO 설정
        self.socketio = SocketIO(
            self.app,
            cors_allowed_origins="*",
            async_mode='threading',
            ping_timeout=60,
            ping_interval=25
        )
        
        # 카메라
        self.camera = None
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
        if self.enable_camera:
            self._init_camera()
        
        # 라우트 설정
        self._setup_routes()
        
        # 서버 스레드
        self.server_thread = None
        self.running = True  # 카메라 루프를 위해 미리 True로 설정
        self.server_running = False
        
        print(f"✅ 웹 GUI 모듈 초기화 완료 (포트: {self.port})")

    def _init_ros_publishers(self):
        """ROS2 퍼블리셔 초기화 (Foxglove 연동용)"""
        if not ROS2_AVAILABLE or self.ros_node is None:
            return

        try:
            # Motor current publisher
            self.ros_publishers['motor_current'] = self.ros_node.create_publisher(
                Float32MultiArray, '/robot/motor_current', 10
            )

            # Relay status publisher
            self.ros_publishers['relay_status'] = self.ros_node.create_publisher(
                String, '/relay/status', 10
            )

            # Battery state publisher
            self.ros_publishers['battery'] = self.ros_node.create_publisher(
                BatteryState, '/robot/battery', 10
            )

            # System state publisher
            self.ros_publishers['system_state'] = self.ros_node.create_publisher(
                String, '/robot/system_state', 10
            )

            # Sonar tilt goal angle publisher
            self.ros_publishers['sonar_goal'] = self.ros_node.create_publisher(
                Float32, '/sonar/tilt/goal_angle', 10
            )

            # LED status publisher
            self.ros_publishers['led_status'] = self.ros_node.create_publisher(
                String, '/robot/led_status', 10
            )

            print("✅ ROS2 Foxglove 퍼블리셔 초기화 완료")
        except Exception as e:
            print(f"⚠️ ROS2 퍼블리셔 초기화 실패: {e}")
    
    def _init_camera(self):
        """USB 카메라 초기화"""
        try:
            self.camera = cv2.VideoCapture(self.camera_device, cv2.CAP_V4L2)
            
            if not self.camera.isOpened() and self.camera_device != '/dev/video0':
                print(f'⚠️  {self.camera_device} 장치를 열 수 없어 /dev/video0로 재시도합니다')
                self.camera.release()
                self.camera = cv2.VideoCapture(0, cv2.CAP_V4L2)
            
            if self.camera.isOpened():
                self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self.camera.set(cv2.CAP_PROP_FPS, 30)
                print(f'✅ USB 카메라 초기화 완료 ({self.camera_device})')
            else:
                print('⚠️  USB 카메라를 찾을 수 없습니다')
                self.camera = None
        except Exception as e:
            print(f'⚠️  카메라 초기화 실패: {e}')
            self.camera = None
        
        # 카메라 스레드 시작
        camera_thread = threading.Thread(target=self._camera_loop, daemon=True)
        camera_thread.start()
    
    def _camera_loop(self):
        """카메라 프레임 읽기 루프"""
        while self.running:
            if self.camera and self.camera.isOpened():
                ret, frame = self.camera.read()
                if ret and frame is not None:
                    with self.frame_lock:
                        self.latest_frame = frame.copy()
            time.sleep(0.033)  # ~30 FPS
    
    def _setup_routes(self):
        """Flask 라우트 설정"""
        
        @self.app.route('/')
        def index():
            return render_template('index.html')
        
        @self.app.route('/video_feed')
        def video_feed():
            return Response(
                self._generate_frames(),
                mimetype='multipart/x-mixed-replace; boundary=frame'
            )
        
        @self.socketio.on('connect')
        def handle_connect():
            print('✅ 클라이언트 연결됨')
            self.socketio.emit('state_update', self.state)
        
        @self.socketio.on('disconnect')
        def handle_disconnect():
            print('❌ 클라이언트 연결 해제됨')
    
    def _generate_frames(self):
        """카메라 프레임 생성기 (web_gui.py 방식)"""
        while True:
            if self.latest_frame is not None:
                with self.frame_lock:
                    frame = self.latest_frame.copy()
                
                # JPEG 인코딩
                ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if ret:
                    frame_bytes = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            time.sleep(0.033)  # ~30 FPS
    
    def start(self):
        """웹 서버 시작 (별도 스레드)"""
        if self.server_running:
            print("⚠️  서버가 이미 실행 중입니다")
            return
        
        self.server_running = True
        
        def run_server():
            print("=" * 60)
            print("🌐 웹 GUI 서버 시작")
            print("=" * 60)
            print(f"📡 접속 주소: http://{self.host}:{self.port}")
            print(f"   로컬:      http://localhost:{self.port}")
            print("=" * 60)
            
            self.socketio.run(
                self.app,
                host=self.host,
                port=self.port,
                debug=False,
                allow_unsafe_werkzeug=True,
                use_reloader=False
            )
        
        self.server_thread = threading.Thread(target=run_server, daemon=True)
        self.server_thread.start()
        
        # 서버 시작 대기
        time.sleep(1)
        print("✅ 웹 GUI 서버 준비 완료")
    
    def stop(self):
        """웹 서버 정지"""
        self.running = False

        if self.camera and self.camera.isOpened():
            self.camera.release()

        print("웹 GUI 서버 정지")
    
    # === 상태 업데이트 메서드 ===
    
    def update_joystick(self, left_x, left_y, right_x, right_y=0.0, axes=None, buttons=None):
        """
        조이스틱 상태 업데이트
        
        Args:
            left_x: 왼쪽 스틱 X축
            left_y: 왼쪽 스틱 Y축
            right_x: 오른쪽 스틱 X축
            right_y: 오른쪽 스틱 Y축
            axes: 전체 축 리스트
            buttons: 전체 버튼 리스트
        """
        self.state['joystick']['left_x'] = left_x
        self.state['joystick']['left_y'] = left_y
        self.state['joystick']['right_x'] = right_x
        self.state['joystick']['right_y'] = right_y
        
        if axes is not None:
            self.state['joystick']['axes'] = axes
        if buttons is not None:
            self.state['joystick']['buttons'] = buttons
        
        self._broadcast_state()
    
    def update_system(self, is_armed=None, sensitivity=None, lumen_brightness=None, control_mode=None):
        """
        시스템 상태 업데이트

        Args:
            is_armed: 시동 상태
            sensitivity: 감도 (0.0 ~ 1.0)
            lumen_brightness: 라이트 밝기 (0.0 ~ 1.0)
            control_mode: 제어 모드 ('NORMAL', 'HOVERING')
        """
        if is_armed is not None:
            self.state['system']['is_armed'] = is_armed
        if sensitivity is not None:
            self.state['system']['sensitivity'] = sensitivity
        if lumen_brightness is not None:
            self.state['system']['lumen_brightness'] = lumen_brightness
        if control_mode is not None:
            self.state['system']['control_mode'] = control_mode

        # ROS2 토픽 발행 (Foxglove용)
        if ROS2_AVAILABLE and 'system_state' in self.ros_publishers:
            try:
                msg = String()
                msg.data = json.dumps({
                    'is_armed': self.state['system']['is_armed'],
                    'sensitivity': self.state['system']['sensitivity'],
                    'lumen_brightness': self.state['system']['lumen_brightness'],
                    'control_mode': self.state['system']['control_mode']
                })
                self.ros_publishers['system_state'].publish(msg)
            except Exception:
                pass

        self._broadcast_state()
    
    def update_relays(self, relay_1=None, relay_2=None, relay_3=None):
        """
        릴레이 상태 업데이트

        Args:
            relay_1: CH1 상태
            relay_2: CH2 상태
            relay_3: CH3 상태
        """
        if relay_1 is not None:
            self.state['relays']['relay_1'] = relay_1
        if relay_2 is not None:
            self.state['relays']['relay_2'] = relay_2
        if relay_3 is not None:
            self.state['relays']['relay_3'] = relay_3

        # ROS2 토픽 발행 (Foxglove용)
        if ROS2_AVAILABLE and 'relay_status' in self.ros_publishers:
            try:
                msg = String()
                msg.data = json.dumps({
                    'relay_1': self.state['relays']['relay_1'],
                    'relay_2': self.state['relays']['relay_2'],
                    'relay_3': self.state['relays']['relay_3']
                })
                self.ros_publishers['relay_status'].publish(msg)
            except Exception:
                pass

        self._broadcast_state()
    
    def update_motors(self, vesc_1=None, vesc_2=None, vesc_3=None, vesc_4=None):
        """
        모터 명령 업데이트 (4개 VESC 홀로노믹)

        Args:
            vesc_1: VESC 1 명령값 (A) - 오른쪽 모터
            vesc_2: VESC 2 명령값 (A) - 뒷쪽 모터
            vesc_3: VESC 3 명령값 (A) - 왼쪽 모터
            vesc_4: VESC 4 명령값 (A) - 앞쪽 모터
        """
        if vesc_1 is not None:
            self.state['motor_commands']['vesc_1'] = vesc_1
        if vesc_2 is not None:
            self.state['motor_commands']['vesc_2'] = vesc_2
        if vesc_3 is not None:
            self.state['motor_commands']['vesc_3'] = vesc_3
        if vesc_4 is not None:
            self.state['motor_commands']['vesc_4'] = vesc_4

        # ROS2 토픽 발행 (Foxglove용)
        if ROS2_AVAILABLE and 'motor_current' in self.ros_publishers:
            try:
                msg = Float32MultiArray()
                msg.data = [
                    self.state['motor_commands']['vesc_1'],
                    self.state['motor_commands']['vesc_2'],
                    self.state['motor_commands']['vesc_3'],
                    self.state['motor_commands']['vesc_4']
                ]
                self.ros_publishers['motor_current'].publish(msg)
            except Exception:
                pass

        self._broadcast_state()
    
    def update_battery(self, voltage=None, percentage=None, status=None):
        """
        배터리 상태 업데이트

        Args:
            voltage: 배터리 전압 (V)
            percentage: 배터리 잔량 (%)
            status: 배터리 상태 ('good', 'low', 'critical', 'unknown')
        """
        if voltage is not None:
            self.state['battery']['voltage'] = voltage
        if percentage is not None:
            self.state['battery']['percentage'] = percentage
        if status is not None:
            self.state['battery']['status'] = status

        # ROS2 토픽 발행 (Foxglove용)
        if ROS2_AVAILABLE and 'battery' in self.ros_publishers:
            try:
                msg = BatteryState()
                msg.voltage = float(self.state['battery']['voltage'])
                msg.percentage = float(self.state['battery']['percentage']) / 100.0
                msg.present = True
                self.ros_publishers['battery'].publish(msg)
            except Exception:
                pass

        self._broadcast_state()
    
    def update_led(self, color, color_name):
        """
        LED 색상 상태 업데이트

        Args:
            color: LED 색상 ('green', 'orange', 'blue')
            color_name: 색상 이름 (한글)
        """
        self.state['led']['color'] = color
        self.state['led']['color_name'] = color_name

        # ROS2 토픽 발행 (Foxglove용)
        if ROS2_AVAILABLE and 'led_status' in self.ros_publishers:
            try:
                msg = String()
                msg.data = color
                self.ros_publishers['led_status'].publish(msg)
            except Exception:
                pass

        self._broadcast_state()

    def update_sonar_tilt(self, current_angle=None, goal_angle=None):
        """
        소나 틸트 상태 업데이트

        Args:
            current_angle: 현재 각도 (도)
            goal_angle: 목표 각도 (도)
        """
        if current_angle is not None:
            self.state['sonar_tilt']['current_angle'] = current_angle
        if goal_angle is not None:
            self.state['sonar_tilt']['goal_angle'] = goal_angle

        # ROS2 토픽 발행 (Foxglove용)
        if ROS2_AVAILABLE and 'sonar_goal' in self.ros_publishers:
            try:
                msg = Float32()
                msg.data = float(self.state['sonar_tilt']['goal_angle'])
                self.ros_publishers['sonar_goal'].publish(msg)
            except Exception:
                pass

        self._broadcast_state()

    def update_state(self, state_dict):
        """전체 상태 업데이트"""
        self.state.update(state_dict)
        self._broadcast_state()
    
    def _broadcast_state(self):
        """현재 상태를 모든 클라이언트에게 브로드캐스트"""
        self.state['timestamp'] = time.time()
        if self.running:
            self.socketio.emit('state_update', self.state)
    
    def get_state(self):
        """현재 상태 반환"""
        return self.state.copy()


# 편의 함수
def create_web_gui(host='0.0.0.0', port=5000, enable_camera=True, camera_device=None):
    """
    웹 GUI 모듈 생성 및 시작
    
    Args:
        host: 서버 호스트
        port: 서버 포트
        enable_camera: 카메라 활성화
        camera_device: 고정 카메라 경로
        
    Returns:
        WebGUIModule: 초기화되고 시작된 GUI 모듈
    """
    gui = WebGUIModule(
        host=host,
        port=port,
        enable_camera=enable_camera,
        camera_device=camera_device
    )
    gui.start()
    return gui


if __name__ == '__main__':
    """테스트 코드"""
    print("=" * 60)
    print("🧪 웹 GUI 모듈 테스트")
    print("=" * 60)
    
    # GUI 생성 및 시작
    gui = create_web_gui(port=5000)
    
    try:
        # 테스트 데이터 업데이트
        import random
        
        while True:
            # 랜덤 조이스틱 값
            gui.update_joystick(
                left_x=random.uniform(-1, 1),
                left_y=random.uniform(-1, 1),
                right_x=random.uniform(-1, 1)
            )
            
            # 랜덤 모터 값 (4개 모터)
            gui.update_motors(
                vesc_1=random.uniform(-10, 10),
                vesc_2=random.uniform(-10, 10),
                vesc_3=random.uniform(-10, 10),
                vesc_4=random.uniform(-10, 10)
            )
            
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        print("\n⚠️  Ctrl+C 감지")
    finally:
        gui.stop()
        print("✅ 테스트 종료")
