#!/usr/bin/env python3
"""
호버링 모듈 - 현재 위치 유지 (Station Keeping)
- Fast-LIO 또는 Cartographer 오도메트리를 이용한 위치 피드백
- PID 제어로 현재 위치 고정
- 4개 쓰러스터 홀로노믹 드라이브
- [완전 개편] 4-모터 풀액티브 Yaw 믹싱 및 스마트 데드존 부스트 적용 (핑퐁/진동 방지)
- 오도메트리 토픽 끊김 감지 및 자동 모드 복귀
"""

import math
import time

from .pid_utils import SimplePID, quaternion_to_yaw, normalize_angle

# 오도메트리 소스 상수
ODOM_SOURCE_FASTLIO = 'Fast-LIO'
ODOM_SOURCE_CARTOGRAPHER = 'Cartographer'
ODOM_SOURCE_NONE = 'None'


class HoveringController:
    """호버링 (위치 고정) 제어기"""

    def __init__(self, vesc_controller, gui=None, logger=None, max_current=8.0,
                 odom_timeout_sec=0.5, enable_yaw_control=True, invert_yaw=False,
                 fastlio_params=None, cartographer_params=None):
        """초기화"""
        self.vesc = vesc_controller
        self.gui = gui
        self.logger = logger
        self.max_current = max_current
        self.odom_timeout_sec = odom_timeout_sec
        self.enable_yaw_control = enable_yaw_control
        self.invert_yaw = invert_yaw
        self._fastlio_params = fastlio_params or {}
        self._cartographer_params = cartographer_params or {}

        self.stabilize_duration = 1.5
        self.activate_time = 0.0
        self.is_active = False

        # 오도메트리 소스 추적
        self.odom_source = ODOM_SOURCE_NONE
        self.last_fastlio_time = 0.0
        self.last_carto_time = 0.0
        self.has_fastlio_odom = False
        self.has_carto_odom = False

        # 소스별 최신 위치 독립 저장
        self._fastlio_x, self._fastlio_y, self._fastlio_yaw = 0.0, 0.0, 0.0
        self._carto_x, self._carto_y, self._carto_yaw = 0.0, 0.0, 0.0

        # 현재/목표 위치
        self.current_x, self.current_y, self.current_yaw = 0.0, 0.0, 0.0
        self.target_x, self.target_y, self.target_yaw = 0.0, 0.0, 0.0

        # PID 제어기
        self.pid_forward = SimplePID()
        self.pid_strafe = SimplePID()
        self.pid_yaw = SimplePID()
        self._yaw_scale = 0.3
        self._yaw_deadband = 2.0

    def activate(self):
        """호버링 활성화 - 현재 위치를 목표로 설정"""
        current_time = time.time()
        fastlio_fresh = self.has_fastlio_odom and (current_time - self.last_fastlio_time) < self.odom_timeout_sec
        carto_fresh = self.has_carto_odom and (current_time - self.last_carto_time) < self.odom_timeout_sec

        # 소스 자동 선택: Fast-LIO 우선
        if fastlio_fresh:
            selected_source = ODOM_SOURCE_FASTLIO
        elif carto_fresh:
            selected_source = ODOM_SOURCE_CARTOGRAPHER
        else:
            if self.logger:
                self.logger.warn('호버링 활성화 실패: 유효한 오도메트리 소스 없음')
            return False

        self.odom_source = selected_source

        # activate 시점에 선택된 소스의 최신 위치로 current/target 설정
        if selected_source == ODOM_SOURCE_FASTLIO:
            self.current_x, self.current_y, self.current_yaw = self._fastlio_x, self._fastlio_y, self._fastlio_yaw
        elif selected_source == ODOM_SOURCE_CARTOGRAPHER:
            self.current_x, self.current_y, self.current_yaw = self._carto_x, self._carto_y, self._carto_yaw

        self.target_x, self.target_y, self.target_yaw = self.current_x, self.current_y, self.current_yaw

        # 소스별 파라미터 적용 후 PID 재생성
        params = self._fastlio_params if selected_source == ODOM_SOURCE_FASTLIO else self._cartographer_params
        self.pid_forward = SimplePID(kp=params['kp'], ki=params['ki'], kd=params['kd'], output_limit=1.0)
        self.pid_strafe  = SimplePID(kp=params['kp'], ki=params['ki'], kd=params['kd'], output_limit=1.0)
        self.pid_yaw     = SimplePID(kp=params['yaw_kp'], ki=params['yaw_ki'], kd=params['yaw_kd'], output_limit=params['yaw_limit'])
        
        self._yaw_scale  = params['yaw_scale']
        self._yaw_deadband = params['yaw_deadband']
        self.stabilize_duration = params['stabilize_duration']

        self.is_active = True
        self.activate_time = time.time()

        if self.logger:
            self.logger.info(f'[호버링] 활성화 - 소스: [{selected_source}]')

        return True

    def deactivate(self, reason=''):
        """호버링 비활성화 - 모터 정지"""
        self.is_active = False
        self.odom_source = ODOM_SOURCE_NONE
        self.vesc.stop_all()
        if self.logger:
            self.logger.info(f'호버링 비활성화 ({reason})')

    def update_fastlio_odometry(self, msg):
        self.last_fastlio_time = time.time()
        self.has_fastlio_odom = True
        self._fastlio_x = msg.pose.pose.position.x
        self._fastlio_y = msg.pose.pose.position.y
        self._fastlio_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        if self.odom_source == ODOM_SOURCE_FASTLIO:
            self.current_x, self.current_y, self.current_yaw = self._fastlio_x, self._fastlio_y, self._fastlio_yaw

    def update_carto_odometry(self, msg):
        self.last_carto_time = time.time()
        self.has_carto_odom = True
        self._carto_x = msg.pose.pose.position.x
        self._carto_y = msg.pose.pose.position.y
        self._carto_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        if self.odom_source == ODOM_SOURCE_CARTOGRAPHER:
            self.current_x, self.current_y, self.current_yaw = self._carto_x, self._carto_y, self._carto_yaw

    def check_odom_timeout(self):
        if not self.is_active or self.odom_source == ODOM_SOURCE_NONE:
            return False
        current_time = time.time()
        elapsed = current_time - (self.last_fastlio_time if self.odom_source == ODOM_SOURCE_FASTLIO else self.last_carto_time)
        return elapsed > self.odom_timeout_sec

    def compute_and_send_commands(self, sensitivity_scale=0.5):
        if not self.is_active or self.odom_source == ODOM_SOURCE_NONE:
            return

        current_time = time.time()
        elapsed = current_time - self.activate_time

        # 초기 안정화 대기
        if elapsed < self.stabilize_duration:
            self.target_x, self.target_y, self.target_yaw = self.current_x, self.current_y, self.current_yaw
            self.pid_forward.reset()
            self.pid_strafe.reset()
            self.pid_yaw.reset()
            return

        # 위치 오차 계산 (월드 -> 로봇 프레임 변환)
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        cos_yaw, sin_yaw = math.cos(self.current_yaw), math.sin(self.current_yaw)
        
        forward_error = dx * cos_yaw + dy * sin_yaw
        strafe_error = -dx * sin_yaw + dy * cos_yaw

        # Yaw 오차 계산 및 데드밴드 처리
        yaw_error = normalize_angle(self.target_yaw - self.current_yaw)
        if self.invert_yaw:
            yaw_error = -yaw_error
            
        if abs(math.degrees(yaw_error)) < self._yaw_deadband:
            yaw_error = 0.0
            self.pid_yaw.integral = 0.0 # 데드밴드 내에서는 적분 리셋 (Windup 방지)

        # PID 계산 (-1.0 ~ 1.0)
        forward_cmd = self.pid_forward.compute(forward_error, current_time)
        strafe_cmd = self.pid_strafe.compute(strafe_error, current_time)
        yaw_cmd = self.pid_yaw.compute(yaw_error, current_time) if self.enable_yaw_control else 0.0

        # 감도 적용된 최대 전류
        current_max = self.max_current * sensitivity_scale

        # 1. 제어 명령을 전류값(A)으로 스케일링
        base_fwd = forward_cmd * current_max
        base_str = strafe_cmd * current_max
        base_yaw = yaw_cmd * self._yaw_scale * current_max

        # 2. [완전 개편] 4-모터 풀액티브 Yaw 믹싱 (텐션 완전 삭제)
        # V1(우), V3(좌)는 전후진 추력으로 회전에 기여
        # V2(뒤), V4(앞)는 좌우 추력으로 회전에 기여 -> 회전 토크 극대화!
        v1_cmd = base_fwd - base_yaw
        v2_cmd = -base_str - base_yaw  
        v3_cmd = base_fwd + base_yaw
        v4_cmd = base_str - base_yaw   

        # 3. 스마트 데드존 부스트 함수 (핑퐁 진동을 막기 위해 0.8 -> 0.3으로 대폭 하향)
        def apply_boost(cmd, deadzone_a=0.3):
            """
            PID 명령이 들어올 때만 데드존을 건너뛰게 만들어 즉각 반응하게 함.
            (회전이 아직도 너무 강하다면 0.2로, 안 돈다면 0.4로 미세 조절하세요)
            """
            if abs(cmd) < 0.05: # 힘이 거의 필요 없을 땐 깔끔하게 완전 정지
                return 0.0
            # 명령 방향에 맞춰 데드존만큼의 기본 전류를 더해줌
            return cmd + math.copysign(deadzone_a, cmd)

        # 각 모터에 부스트 적용
        vesc_1 = apply_boost(v1_cmd, deadzone_a=0.3)
        vesc_2 = apply_boost(v2_cmd, deadzone_a=0.3)
        vesc_3 = apply_boost(v3_cmd, deadzone_a=0.3)
        vesc_4 = apply_boost(v4_cmd, deadzone_a=0.3)

        # 4. 최대 전류 제한 (Clamp)
        vesc_1 = max(-self.max_current, min(self.max_current, vesc_1))
        vesc_2 = max(-self.max_current, min(self.max_current, vesc_2))
        vesc_3 = max(-self.max_current, min(self.max_current, vesc_3))
        vesc_4 = max(-self.max_current, min(self.max_current, vesc_4))

        # 5. VESC 명령 전송 (V4 배선 반전 유지 - 수식에서 완벽히 고려됨)
        self.vesc.set_current('vesc_1', vesc_1)
        self.vesc.set_current('vesc_2', vesc_2)
        self.vesc.set_current('vesc_3', vesc_3)
        self.vesc.set_current('vesc_4', -vesc_4) 

        # GUI 업데이트 및 디버그
        self.gui.update_motors(vesc_1=vesc_1, vesc_2=vesc_2, vesc_3=vesc_3, vesc_4=vesc_4)

        if self.logger:
            dist = math.sqrt(dx*dx + dy*dy)
            self.logger.info(
                f'[{self.odom_source}] D:{dist:.2f}m Y_err:{math.degrees(yaw_error):.1f}° | '
                f'V1:{vesc_1:+.1f}A V2:{vesc_2:+.1f}A V3:{vesc_3:+.1f}A V4:{vesc_4:+.1f}A',
                throttle_duration_sec=1.0
            )