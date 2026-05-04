import can
import rclpy
from rclpy.node import Node
from .._log import make_logger


class VESCController:
    """
    VESC CAN 통신 제어를 위한 기본 클래스
    여러 VESC를 CAN 통신으로 제어합니다.
    """
    
    def __init__(self, channel='can0', interface='socketcan', *, logger=None):
        """
        Args:
            channel: CAN 채널 이름 (default: 'can0')
            interface: CAN 인터페이스 타입 (default: 'socketcan')
            logger: rclpy logger (None 이면 print fallback). Keyword-only.
        """
        self._log = make_logger(logger)
        try:
            self.bus = can.interface.Bus(channel=channel, interface=interface)
            self._log('info', f'CAN bus 초기화 완료 ({channel})')
        except (OSError, can.CanError) as e:
            self.bus = None
            self._log('error',
                      f'CAN bus 초기화 실패 ({channel}): {e}. '
                      '디그레이드 모드로 시작 (CAN 의존 동작 비활성화)')
        
        # VESC CAN ID 매핑 (Decimal ID -> Extended CAN ID)
        self.vesc_can_ids = {
            "vesc_1": 0x101,  # ID 1
            "vesc_2": 0x102,  # ID 2
            "vesc_3": 0x103,  # ID 3
            "vesc_4": 0x104   # ID 4
        }
        
        # 각 VESC의 현재 목표 전류값
        self.target_current = {vesc: 0.0 for vesc in self.vesc_can_ids}
        
        # 각 VESC의 실제 출력 전류값 (램프 적용)
        self.actual_current = {vesc: 0.0 for vesc in self.vesc_can_ids}
        
        # 램프 제어 파라미터
        self.max_ramp_step = 0.5  # 한 루프당 최대 전류 변화량 (A)
        self.min_update_threshold = 0.2  # 최소 업데이트 임계값 (A)
        
        # 안전 제한
        self.max_current_limit = 2.0  # 최대 전류 제한 (A)

    def send_current(self, can_id, current):
        """
        특정 VESC에 전류 명령 전송

        Args:
            can_id: VESC의 CAN ID (extended format)
            current: 전류값 (A), 음수는 역방향
        """
        if self.bus is None:
            return False  # degraded mode: silent skip (init이 이미 에러 로그 1회 출력)
        # 전류를 밀리암페어로 변환 (Big Endian)
        scaled_current = int(current * 1000)
        
        # 32bit 정수를 4바이트로 분할 (Big Endian)
        data = [
            (scaled_current >> 24) & 0xFF,
            (scaled_current >> 16) & 0xFF,
            (scaled_current >> 8) & 0xFF,
            scaled_current & 0xFF
        ]
        
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=True,
            dlc=4
        )
        
        try:
            self.bus.send(msg)
            return True
        except can.CanError as e:
            self._log('warn', f'CAN 전송 실패 (ID {hex(can_id)}): {e}')
            return False
    
    def set_current(self, vesc_name, current):
        """
        특정 VESC의 목표 전류 설정
        
        Args:
            vesc_name: VESC 이름 (예: "vesc_1")
            current: 목표 전류값 (A)
        """
        if vesc_name in self.target_current:
            # 최대 전류 제한 적용
            limited_current = max(-self.max_current_limit, min(self.max_current_limit, current))
            
            # 경고 메시지 출력
            if abs(current) > self.max_current_limit:
                self._log('warn',
                          f'{vesc_name}: 전류 제한 적용 ({current:.2f}A → {limited_current:.2f}A)')

            self.target_current[vesc_name] = limited_current
        else:
            self._log('warn', f'알 수 없는 VESC 이름: {vesc_name}')
    
    def set_group_current(self, vesc_names, current):
        """
        여러 VESC의 목표 전류를 동시에 설정
        
        Args:
            vesc_names: VESC 이름 리스트 (예: ["vesc_1", "vesc_2"])
            current: 목표 전류값 (A)
        """
        for vesc_name in vesc_names:
            self.set_current(vesc_name, current)
    
    def set_all_current(self, current):
        """
        모든 VESC의 목표 전류를 동시에 설정
        
        Args:
            current: 목표 전류값 (A)
        """
        for vesc_name in self.vesc_can_ids.keys():
            self.target_current[vesc_name] = current
    
    def update(self):
        """
        램프 제어를 적용하여 실제 전류를 목표 전류로 점진적으로 변경
        이 함수를 주기적으로 호출해야 합니다.
        """
        if self.bus is None:
            return  # degraded mode
        for vesc, can_id in self.vesc_can_ids.items():
            target = self.target_current[vesc]
            actual = self.actual_current[vesc]
            
            # 목표값과 현재값의 차이 계산
            diff = target - actual
            
            # 램프 적용
            if abs(diff) > self.max_ramp_step:
                if diff > 0:
                    self.actual_current[vesc] += self.max_ramp_step
                else:
                    self.actual_current[vesc] -= self.max_ramp_step
            else:
                self.actual_current[vesc] = target
            
            # CAN 메시지 전송
            self.send_current(can_id, self.actual_current[vesc])
    
    def stop_all(self):
        """모든 VESC를 정지"""
        self.set_all_current(0.0)
    
    def get_current_status(self):
        """
        현재 전류 상태 반환
        
        Returns:
            dict: 각 VESC의 목표 전류와 실제 전류
        """
        status = {}
        for vesc in self.vesc_can_ids.keys():
            status[vesc] = {
                'target': self.target_current[vesc],
                'actual': self.actual_current[vesc]
            }
        return status
    
    def set_max_current_limit(self, limit):
        """
        최대 전류 제한값 변경
        
        Args:
            limit: 새로운 최대 전류 제한값 (A)
        """
        if limit > 0:
            self.max_current_limit = limit
            self._log('info', f'최대 전류 제한: {limit}A')
        else:
            self._log('warn', f'잘못된 전류 제한값: {limit}A (양수여야 함)')
    
    def get_max_current_limit(self):
        """
        현재 최대 전류 제한값 반환
        
        Returns:
            float: 최대 전류 제한값 (A)
        """
        return self.max_current_limit
    
    def shutdown(self):
        """CAN 버스 종료"""
        if self.bus is None:
            return
        self.stop_all()
        self.update()  # 정지 명령 전송
        self.bus.shutdown()


class VESCControlNode(Node):
    """
    ROS2 노드로 VESC 제어를 래핑한 클래스
    타이머를 이용한 주기적 업데이트 제공
    """
    
    def __init__(self, node_name='vesc_control_node', update_rate=10.0):
        """
        Args:
            node_name: ROS2 노드 이름
            update_rate: 제어 루프 주기 (Hz)
        """
        super().__init__(node_name)
        
        # VESC 컨트롤러 초기화 (로거 주입)
        self.controller = VESCController(logger=self.get_logger())
        
        # 타이머 생성 (주기적 업데이트)
        timer_period = 1.0 / update_rate
        self.timer = self.create_timer(timer_period, self.control_loop)
        
        self.get_logger().info(f'{node_name} 초기화 완료 (업데이트 주기: {update_rate}Hz)')
    
    def control_loop(self):
        """제어 루프 콜백 - 타이머에 의해 주기적으로 호출됨"""
        self.controller.update()
    
    def get_controller(self):
        """VESCController 인스턴스 반환"""
        return self.controller
    
    def shutdown_node(self):
        """노드 종료"""
        self.get_logger().info('노드 종료 중...')
        self.controller.shutdown()
        self.destroy_node()