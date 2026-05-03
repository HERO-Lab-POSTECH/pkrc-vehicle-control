#!/usr/bin/env python3
"""
PID 제어 모드 - 조이스틱으로 목표 위치/yaw를 이동하면서 PID로 추종
- 왼쪽 조이스틱: 현재 yaw 방향 기준 전후좌우 이동 (목표 위치 연속 업데이트)
- 오른쪽 조이스틱 X: yaw만 변경 (목표 yaw 연속 업데이트)
- 조이스틱 중립: 현재 위치+yaw 고정 (hovering과 동일)
- Fast-LIO 또는 Cartographer 오도메트리 피드백
"""

import math
import time

from .pid_utils import (
    SimplePID, quaternion_to_yaw,
)
from .hovering import (
    ODOM_SOURCE_FASTLIO, ODOM_SOURCE_CARTOGRAPHER, ODOM_SOURCE_NONE
)

# ── 소스별 파라미터 ─────────────────────────────────────────────────
PARAMS_FASTLIO_PID = {
    'kp': 1.2,  'ki': 0.08, 'kd': 0.6,            # XY PID
    'yaw_kp': 0.4, 'yaw_ki': 0.01, 'yaw_kd': 0.5,
    'yaw_limit': 0.7,           # 이동 중 yaw 보정 여유 확보
    'yaw_scale': 1.0,
    'yaw_deadband': 4.0,        # deg: 정지 시 이 이내 오차는 yaw 제어 안함 (windup 방지)
    'stabilize_duration': 1.0,  # sec
    'joystick_speed': 0.3,      # m/s (왼쪽 스틱 풀입력 시 목표 이동 속도)
    'joystick_yaw_speed': 25.0, # deg/s (오른쪽 스틱 풀입력 시 yaw 변경 속도)
}

PARAMS_CARTOGRAPHER_PID = {
    'kp': 1.5,  'ki': 0.10, 'kd': 0.4,
    'yaw_kp': 0.3, 'yaw_ki': 0.05, 'yaw_kd': 0.5,
    'yaw_limit': 0.4,
    'yaw_scale': 0.3,
    'yaw_deadband': 4.0,
    'stabilize_duration': 1.5,
    'joystick_speed': 0.3,
    'joystick_yaw_speed': 25.0,
}
# ───────────────────────────────────────────────────────────────────


class PIDModeController:
    """
    PID 모드 제어기

    조이스틱으로 목표 위치/yaw를 실시간으로 조정하면서 PID로 추종.
    - 왼쪽 스틱 중립: 현재 위치 고정 (station keeping)
    - 왼쪽 스틱 입력: 로봇이 바라보는 방향 기준으로 목표 위치 이동
    - 오른쪽 스틱 입력: 목표 yaw 변경
    """

    def __init__(self, vesc_controller, web_gui=None, logger=None, max_current=8.0,
                 odom_timeout_sec=0.5, enable_yaw_control=True, invert_yaw=False):
        self.vesc = vesc_controller
        self.gui = web_gui
        self.logger = logger
        self.max_current = max_current
        self.odom_timeout_sec = odom_timeout_sec
        self.enable_yaw_control = enable_yaw_control
        self.invert_yaw = invert_yaw

        # VESC 최소 동작 전류 (hovering과 동일 값)
        self.min_thrust_current = 1.0
        self.dead_threshold = 0.1

        # 상태
        self.is_active = False
        self.activate_time = 0.0
        self.stabilize_duration = 1.0
        self._last_compute_time = 0.0

        # 오도메트리 소스
        self.odom_source = ODOM_SOURCE_NONE
        self.last_fastlio_time = 0.0
        self.last_carto_time = 0.0
        self.has_fastlio_odom = False
        self.has_carto_odom = False

        # 소스별 최신 위치 버퍼
        self._fastlio_x = 0.0
        self._fastlio_y = 0.0
        self._fastlio_yaw = 0.0
        self._carto_x = 0.0
        self._carto_y = 0.0
        self._carto_yaw = 0.0

        # 현재/목표 위치
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0

        # 조이스틱 최신 입력 (PKRC에서 set_joystick으로 업데이트)
        self._joy_forward = 0.0
        self._joy_strafe = 0.0
        self._joy_rotation = 0.0

        # PID (activate 시 소스별 파라미터로 재생성)
        self.pid_forward = SimplePID(kp=1.2, ki=0.08, kd=0.6, output_limit=1.0)
        self.pid_strafe  = SimplePID(kp=1.2, ki=0.08, kd=0.6, output_limit=1.0)
        self.pid_yaw     = SimplePID(kp=0.3, ki=0.01, kd=0.5, output_limit=0.5)
        self._yaw_scale = 1.0
        self._yaw_deadband_rad = math.radians(4.0)
        self._joystick_speed = 0.5
        self._joystick_yaw_speed_rad = math.radians(45.0)

    # ── 활성화/비활성화 ─────────────────────────────────────────────

    def activate(self):
        """
        PID 모드 활성화 - 소스 자동 선택(Fast-LIO 우선), 현재 위치를 목표로 설정

        Returns:
            bool: 성공 여부
        """
        current_time = time.time()
        fastlio_fresh = self.has_fastlio_odom and (current_time - self.last_fastlio_time) < self.odom_timeout_sec
        carto_fresh   = self.has_carto_odom   and (current_time - self.last_carto_time)   < self.odom_timeout_sec

        if fastlio_fresh:
            selected_source = ODOM_SOURCE_FASTLIO
            params = PARAMS_FASTLIO_PID
        elif carto_fresh:
            selected_source = ODOM_SOURCE_CARTOGRAPHER
            params = PARAMS_CARTOGRAPHER_PID
        else:
            if self.logger:
                self.logger.warn(
                    'PID 모드 활성화 실패: 유효한 오도메트리 소스 없음 '
                    f'(Fast-LIO: {current_time - self.last_fastlio_time:.1f}초 전, '
                    f'Cartographer: {current_time - self.last_carto_time:.1f}초 전)'
                )
            return False

        self.odom_source = selected_source

        # activate 시점 위치로 current/target 동시 설정
        if selected_source == ODOM_SOURCE_FASTLIO:
            self.current_x   = self._fastlio_x
            self.current_y   = self._fastlio_y
            self.current_yaw = self._fastlio_yaw
        else:
            self.current_x   = self._carto_x
            self.current_y   = self._carto_y
            self.current_yaw = self._carto_yaw

        self.target_x   = self.current_x
        self.target_y   = self.current_y
        self.target_yaw = self.current_yaw

        # 소스별 파라미터로 PID 재생성
        self.pid_forward = SimplePID(kp=params['kp'], ki=params['ki'], kd=params['kd'], output_limit=1.0)
        self.pid_strafe  = SimplePID(kp=params['kp'], ki=params['ki'], kd=params['kd'], output_limit=1.0)
        self.pid_yaw     = SimplePID(kp=params['yaw_kp'], ki=params['yaw_ki'],
                                     kd=params['yaw_kd'], output_limit=params['yaw_limit'])
        self._yaw_scale             = params['yaw_scale']
        self._yaw_deadband_rad      = math.radians(params['yaw_deadband'])
        self.stabilize_duration     = params['stabilize_duration']
        self._joystick_speed        = params['joystick_speed']
        self._joystick_yaw_speed_rad = math.radians(params['joystick_yaw_speed'])

        # 조이스틱 리셋
        self._joy_forward  = 0.0
        self._joy_strafe   = 0.0
        self._joy_rotation = 0.0

        self.is_active = True
        self.activate_time = current_time
        self._last_compute_time = current_time

        if self.logger:
            self.logger.info(
                f'[PID 모드] 활성화 - 소스: [{selected_source}] | '
                f'안정화 대기 {self.stabilize_duration}s | '
                f'속도: {self._joystick_speed}m/s yaw: {params["joystick_yaw_speed"]}deg/s'
            )
        return True

    def deactivate(self, reason=''):
        """PID 모드 비활성화 - 모터 정지"""
        self.is_active = False
        self.odom_source = ODOM_SOURCE_NONE
        self.vesc.stop_all()
        if self.logger:
            msg = 'PID 모드 비활성화'
            if reason:
                msg += f' ({reason})'
            self.logger.info(msg)

    # ── 조이스틱 입력 ────────────────────────────────────────────────

    def set_joystick(self, forward, strafe, rotation):
        """
        PKRC_joy_module에서 호출 - PID 모드 중 조이스틱 최신값 저장

        Args:
            forward:  왼쪽 스틱 Y (-1.0 후진 ~ +1.0 전진)
            strafe:   왼쪽 스틱 X (-1.0 좌 ~ +1.0 우)
            rotation: 오른쪽 스틱 X (-1.0 ~ +1.0, yaw 변경용)
        """
        self._joy_forward  = forward
        self._joy_strafe   = strafe
        self._joy_rotation = rotation

    # ── 오도메트리 업데이트 ──────────────────────────────────────────

    def update_fastlio_odometry(self, msg):
        """Fast-LIO 오도메트리 업데이트 (main.py 콜백에서 호출)"""
        self.last_fastlio_time = time.time()
        self.has_fastlio_odom = True
        self._fastlio_x   = msg.pose.pose.position.x
        self._fastlio_y   = msg.pose.pose.position.y
        self._fastlio_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        if self.odom_source == ODOM_SOURCE_FASTLIO:
            self.current_x   = self._fastlio_x
            self.current_y   = self._fastlio_y
            self.current_yaw = self._fastlio_yaw

    def update_carto_odometry(self, msg):
        """Cartographer 오도메트리 업데이트 (main.py 콜백에서 호출)"""
        self.last_carto_time = time.time()
        self.has_carto_odom = True
        self._carto_x   = msg.pose.pose.position.x
        self._carto_y   = msg.pose.pose.position.y
        self._carto_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        if self.odom_source == ODOM_SOURCE_CARTOGRAPHER:
            self.current_x   = self._carto_x
            self.current_y   = self._carto_y
            self.current_yaw = self._carto_yaw

    def check_odom_timeout(self):
        """오도메트리 타임아웃 체크"""
        if not self.is_active or self.odom_source == ODOM_SOURCE_NONE:
            return False
        current_time = time.time()
        if self.odom_source == ODOM_SOURCE_FASTLIO:
            elapsed = current_time - self.last_fastlio_time
        elif self.odom_source == ODOM_SOURCE_CARTOGRAPHER:
            elapsed = current_time - self.last_carto_time
        else:
            return False
        return elapsed > self.odom_timeout_sec

    # ── 내부 유틸 ────────────────────────────────────────────────────

    def _apply_min_thrust(self, current_a):
        if abs(current_a) < self.dead_threshold:
            return 0.0
        return math.copysign(max(abs(current_a), self.min_thrust_current), current_a)

    @staticmethod
    def _normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    # ── 메인 제어 루프 ───────────────────────────────────────────────

    def compute_and_send_commands(self, sensitivity_scale=0.5):
        """
        20Hz 타이머에서 호출
        1. 조이스틱 입력으로 목표 위치/yaw 업데이트
        2. PID로 현재 위치를 목표로 추종해 VESC 명령 전송
        """
        if not self.is_active or self.odom_source == ODOM_SOURCE_NONE:
            return

        current_time = time.time()
        dt = current_time - self._last_compute_time
        if dt <= 0 or dt > 0.5:
            dt = 0.05  # 기본 20Hz
        self._last_compute_time = current_time

        # ── 안정화 대기 ────────────────────────────────────────────
        elapsed = current_time - self.activate_time
        if elapsed < self.stabilize_duration:
            self.target_x   = self.current_x
            self.target_y   = self.current_y
            self.target_yaw = self.current_yaw
            self.pid_forward.reset()
            self.pid_strafe.reset()
            self.pid_yaw.reset()
            if self.logger:
                self.logger.info(
                    f'[PID 모드] 안정화 대기 {self.stabilize_duration - elapsed:.1f}s ...',
                    throttle_duration_sec=0.5
                )
            return

        # ── 1. 조이스틱 → 목표 위치/yaw 업데이트 ─────────────────
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)

        xy_moving = abs(self._joy_forward) > 0.01 or abs(self._joy_strafe) > 0.01

        # 왼쪽 스틱: 로봇 기준 방향을 월드 프레임으로 변환해 목표 위치 이동
        if xy_moving:
            speed = self._joystick_speed * dt
            # 로봇 프레임 → 월드 프레임 변환 (strafe 부호 반전 적용)
            self.target_x += (self._joy_forward * cos_yaw - self._joy_strafe * sin_yaw) * speed
            self.target_y += (self._joy_forward * sin_yaw + self._joy_strafe * cos_yaw) * speed

        # 오른쪽 스틱: 목표 yaw 변경 (부호 반전 적용)
        if abs(self._joy_rotation) > 0.01:
            self.target_yaw -= self._joy_rotation * self._joystick_yaw_speed_rad * dt
            self.target_yaw = self._normalize_angle(self.target_yaw)
            # yaw 변경 시 적분 리셋
            self.pid_yaw.reset()

        # ── 2. PID 계산 ────────────────────────────────────────────
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y

        # 월드 오차 → 로봇 프레임 오차
        forward_error = dx * cos_yaw + dy * sin_yaw
        strafe_error  = -dx * sin_yaw + dy * cos_yaw

        yaw_error = self._normalize_angle(self.target_yaw - self.current_yaw)
        if self.invert_yaw:
            yaw_error = -yaw_error

        forward_cmd = self.pid_forward.compute(forward_error, current_time)
        strafe_cmd  = self.pid_strafe.compute(strafe_error, current_time)

        if self.enable_yaw_control:
            if xy_moving:
                # XY 이동 중: deadband 무시하고 항상 yaw 보정 (이동 시 yaw 흔들림 억제)
                yaw_cmd = self.pid_yaw.compute(yaw_error, current_time)
            elif abs(self._joy_rotation) < 0.01 and abs(yaw_error) < self._yaw_deadband_rad:
                # 정지 + 오차 작음: yaw 제어 안함 + 적분 리셋 (windup 방지)
                yaw_cmd = 0.0
                self.pid_yaw.reset()
            else:
                yaw_cmd = self.pid_yaw.compute(yaw_error, current_time)
        else:
            yaw_cmd = 0.0

        # ── 3. 홀로노믹 모터 믹싱 ──────────────────────────────────
        current_max = self.max_current * sensitivity_scale
        rotation_scaled = yaw_cmd * self._yaw_scale

        motor1 = forward_cmd - rotation_scaled   # V1 오른쪽 (전후 + yaw)
        motor2 = -strafe_cmd                     # V2 뒤쪽  (스트레이프)
        motor3 = forward_cmd + rotation_scaled   # V3 왼쪽  (전후 + yaw)
        motor4 = strafe_cmd                      # V4 앞쪽  (스트레이프)

        max_val = max(abs(motor1), abs(motor2), abs(motor3), abs(motor4))
        if max_val > 1.0:
            motor1 /= max_val
            motor2 /= max_val
            motor3 /= max_val
            motor4 /= max_val

        vesc_1 = self._apply_min_thrust(motor1 * current_max)
        vesc_2 = self._apply_min_thrust(motor2 * current_max)
        vesc_3 = self._apply_min_thrust(motor3 * current_max)
        vesc_4 = self._apply_min_thrust(motor4 * current_max)

        self.vesc.set_current('vesc_1', vesc_1)
        self.vesc.set_current('vesc_2', vesc_2)
        self.vesc.set_current('vesc_3', vesc_3)
        self.vesc.set_current('vesc_4', -vesc_4)  # 앞쪽 모터 배선 반대

        # 로그
        dist = math.sqrt(dx ** 2 + dy ** 2)
        if self.logger:
            self.logger.info(
                f'[PID/{self.odom_source}] 거리:{dist:.3f}m yaw오차:{math.degrees(yaw_error):.1f}° '
                f'joy:({self._joy_forward:+.2f},{self._joy_strafe:+.2f},{self._joy_rotation:+.2f}) | '
                f'V1:{vesc_1:+.2f}A V2:{vesc_2:+.2f}A V3:{vesc_3:+.2f}A V4:{vesc_4:+.2f}A',
                throttle_duration_sec=0.3
            )
