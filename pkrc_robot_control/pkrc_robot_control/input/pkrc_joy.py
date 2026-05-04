#!/usr/bin/env python3
"""
PKRC 조이스틱 제어 모듈 (CAN 버전 - 4모터 홀로노믹)
- 조이스틱 매핑 (Xbox 컨트롤러 기준)
- 버튼/축 입력 처리
- VESC CAN 통신 쓰러스터 명령 계산 (4개 모터)
- 릴레이/LED/감도 제어
- 모듈화된 설계로 다른 제어 모드와 쉽게 교체 가능
"""

from sensor_msgs.msg import Joy
from .common_controls import CommonControls


class PKRCJoystickController:
    """PKRC 조이스틱 제어 클래스 (CAN 통신 - 4모터 홀로노믹)"""

    # Xbox 컨트롤러 버튼 매핑
    BTN_A = 0
    BTN_B = 1
    BTN_X = 2
    BTN_Y = 3
    BTN_LB = 4
    BTN_RB = 5
    BTN_OPTIONS = 6  # 시동 OFF
    BTN_MENU = 7     # 시동 ON
    BTN_SPECIAL_8 = 8  # 녹화 토글

    # D-Pad 축 매핑
    DPAD_HORIZONTAL = 6
    DPAD_VERTICAL = 7

    # 트리거 축 매핑
    AXIS_RT = 5  # RT: 1.0=released, -1.0=fully pressed

    # 제어 모드
    MODE_NORMAL = 'NORMAL'
    MODE_HOVERING = 'HOVERING'
    MODE_PID = 'PID'

    def __init__(self,
                 vesc_controller,
                 relay_controller,
                 lumen_controller,
                 rgb_led,
                 gui,
                 logger,
                 main_node=None,
                 sonar_tilt=None,
                 deadzone=0.15,
                 sensitivity_scale=0.5,
                 sensitivity_step=0.1,
                 dpad_debounce_time=0.3,
                 max_current=4.0):
        """
        초기화

        Args:
            vesc_controller: VESC CAN 컨트롤러
            relay_controller: 릴레이 제어기
            lumen_controller: Lumen 라이트 제어기
            rgb_led: RGB LED 제어기
            gui: GUI 인터페이스 (NullGUI 또는 미래 통합 Qt GUI)
            logger: ROS2 로거
            main_node: 메인 노드 (녹화 제어용)
            sonar_tilt: 소나 틸트 모듈
            deadzone: 조이스틱 데드존
            sensitivity_scale: 초기 감도
            sensitivity_step: 감도 조절 단계
            dpad_debounce_time: D-Pad 디바운스 시간
            max_current: 최대 전류 (A)
        """
        self.vesc = vesc_controller
        self.relay = relay_controller
        self.lumen = lumen_controller
        self.led = rgb_led
        self.gui = gui
        self.logger = logger
        self.main_node = main_node
        self.sonar_tilt = sonar_tilt

        # 조이스틱 설정
        self.deadzone = deadzone

        # 조이스틱 타임아웃 (명령이 안 오면 자동 정지)
        self.last_joy_time = 0.0
        self.joy_timeout = 0.2  # 0.2초 동안 명령이 없으면 정지
        self.sensitivity_scale = sensitivity_scale
        self.sensitivity_step = sensitivity_step
        self.dpad_debounce_time = dpad_debounce_time

        # 전류 제어 파라미터
        self.max_current = max_current  # 최대 전류 (A)

        # VESC Controller에 최대 전류 제한 설정
        self.vesc.set_max_current_limit(max_current)

        # 시스템 상태
        self.is_armed = False

        # === 제어 모드 ===
        self.control_mode = self.MODE_NORMAL
        self.hovering = None      # HoveringController (main.py에서 설정)
        self.pid_ctrl = None      # PIDModeController (main.py에서 설정)

        # RT 모드 전환용 이전 버튼 상태
        self._prev_rt_y = False
        self._prev_rt_a = False
        self._prev_rt_x = False

        # 버튼 상태 추적
        self.prev_buttons = []
        self.last_dpad_time = {
            'up': 0.0,
            'down': 0.0,
            'left': 0.0,
            'right': 0.0
        }

        # 공통 제어 모듈 (릴레이, D-Pad 감도/루먼)
        self.common = CommonControls(
            relay_controller=relay_controller,
            lumen_controller=lumen_controller,
            gui=gui,
            logger=logger,
            main_node=main_node,
            sensitivity_step=sensitivity_step,
            dpad_debounce_time=dpad_debounce_time
        )

    def apply_deadzone(self, value):
        """데드존 적용"""
        if abs(value) < self.deadzone:
            return 0.0
        sign = 1 if value > 0 else -1
        scaled = (abs(value) - self.deadzone) / (1.0 - self.deadzone)
        return sign * scaled

    def calculate_thruster_commands(self, forward, strafe, rotation):
        """
        조이스틱 입력으로부터 쓰러스터 명령 계산 (4개 VESC - 홀로노믹)

        Args:
            forward: 전진/후진 (-1.0 ~ 1.0)
            strafe: 좌우 스트레이프 (-1.0 왼쪽 ~ 1.0 오른쪽)
            rotation: 회전 (-1.0 왼쪽 ~ 1.0 오른쪽)

        Returns:
            dict: 각 VESC의 전류 명령값 (A)

        모터 배치:
                  VESC 4 (앞)
                  좌우이동용
                      |
        VESC 3 (왼쪽) - 🚤 - VESC 1 (오른쪽)
        전후이동             전후이동
                      |
                  VESC 2 (뒤)
                  좌우이동용
        """
        # 최대 전류 가져오기
        max_current = self.vesc.get_max_current_limit()

        # 감도 적용된 최대 전류
        current_max = max_current * self.sensitivity_scale

        # 홀로노믹 드라이브 계산
        # 회전은 좌/우 쓰러스터(1, 3)만 담당하고, 앞/뒤(2, 4)는 스트레이프만 담당.
        # 회전 출력은 전후/좌우 대비 1/4로 감소 (너무 빠른 회전 방지)
        rotation_scaled = rotation * 0.25
        motor1 = forward - rotation_scaled   # 오른쪽 모터
        motor2 = -strafe                     # 뒷쪽 모터 (회전 성분 제거)
        motor3 = forward + rotation_scaled   # 왼쪽 모터
        motor4 = strafe                      # 앞쪽 모터 (회전 성분 제거)

        # 정규화 (최대값이 1을 초과하면 모든 값을 비율에 맞게 축소)
        max_val = max(abs(motor1), abs(motor2), abs(motor3), abs(motor4))
        if max_val > 1.0:
            motor1 /= max_val
            motor2 /= max_val
            motor3 /= max_val
            motor4 /= max_val

        # 전류 값으로 변환 (암페어)
        vesc_1 = motor1 * current_max
        vesc_2 = motor2 * current_max
        vesc_3 = motor3 * current_max
        vesc_4 = motor4 * current_max

        return {
            'vesc_1': vesc_1,    # 오른쪽 모터
            'vesc_2': vesc_2,    # 뒷쪽 모터
            'vesc_3': vesc_3,    # 왼쪽 모터
            'vesc_4': vesc_4,    # 앞쪽 모터
            'vesc_1_normalized': vesc_1 / max_current if max_current > 0 else 0.0,  # GUI용
            'vesc_2_normalized': vesc_2 / max_current if max_current > 0 else 0.0,
            'vesc_3_normalized': vesc_3 / max_current if max_current > 0 else 0.0,
            'vesc_4_normalized': vesc_4 / max_current if max_current > 0 else 0.0
        }

    def adjust_sensitivity(self, increase=True):
        """감도 조절"""
        if increase:
            self.sensitivity_scale = min(1.0, self.sensitivity_scale + self.sensitivity_step)
        else:
            self.sensitivity_scale = max(0.1, self.sensitivity_scale - self.sensitivity_step)

        self.logger.info(f'감도: {self.sensitivity_scale:.1f}')

        # 웹 GUI 업데이트
        self.gui.update_system(
            is_armed=self.is_armed,
            sensitivity=self.sensitivity_scale,
            lumen_brightness=(self.lumen.get_brightness() if self.lumen is not None else 0.0)
        )

    def arm_system(self):
        """시동 ON"""
        self.is_armed = True
        self.logger.info('시동 ON - 제어 가능')

        # RGB LED 초록색 (웹 GUI 자동 업데이트)
        if self.led:
            self.led.set_green()

        # 웹 GUI 시스템 상태 업데이트
        self.gui.update_system(
            is_armed=self.is_armed,
            sensitivity=self.sensitivity_scale,
            lumen_brightness=(self.lumen.get_brightness() if self.lumen is not None else 0.0)
        )

    def disarm_system(self):
        """시동 OFF (긴급정지)"""
        self.is_armed = False
        self.vesc.stop_all()
        self.logger.warn('시동 OFF - 모든 VESC 정지')

        # 모드 해제
        if self.control_mode == self.MODE_HOVERING and self.hovering:
            self.hovering.deactivate()
        if self.control_mode == self.MODE_PID and self.pid_ctrl:
            self.pid_ctrl.deactivate()
        self.control_mode = self.MODE_NORMAL

        # RGB LED 주황색 (웹 GUI 자동 업데이트)
        if self.led:
            self.led.set_orange()

        # 웹 GUI 시스템 상태 업데이트
        self.gui.update_system(
            is_armed=self.is_armed,
            sensitivity=self.sensitivity_scale,
            lumen_brightness=(self.lumen.get_brightness() if self.lumen is not None else 0.0),
            control_mode=self.MODE_NORMAL
        )

    def _handle_mode_switch(self, msg, buttons):
        """
        RT + 버튼 조합으로 제어 모드 전환

        RT + Y: 노말 모드 (수동 쓰러스터 제어)
        RT + A: 호버링 모드 (현재 위치 고정)

        Args:
            msg: Joy 메시지
            buttons: 버튼 리스트
        """
        if len(msg.axes) <= self.AXIS_RT:
            return

        rt_pressed = msg.axes[self.AXIS_RT] < -0.5
        y_pressed = len(buttons) > self.BTN_Y and buttons[self.BTN_Y]
        a_pressed = len(buttons) > self.BTN_A and buttons[self.BTN_A]
        x_pressed = len(buttons) > self.BTN_X and buttons[self.BTN_X]

        if not rt_pressed:
            self._prev_rt_y = False
            self._prev_rt_a = False
            self._prev_rt_x = False
            return

        # ── RT + Y: 노말 모드 ────────────────────────────────────────
        if y_pressed and not self._prev_rt_y:
            if self.control_mode != self.MODE_NORMAL:
                if self.hovering:
                    self.hovering.deactivate()
                if self.pid_ctrl:
                    self.pid_ctrl.deactivate()
                self.control_mode = self.MODE_NORMAL
                self.logger.info('제어 모드: NORMAL (수동 제어)')
                if self.led:
                    self.led.stop_pattern()
                    self.led.set_green()
                self.gui.update_system(
                    is_armed=self.is_armed,
                    sensitivity=self.sensitivity_scale,
                    lumen_brightness=(self.lumen.get_brightness() if self.lumen is not None else 0.0),
                    control_mode=self.MODE_NORMAL
                )

        # ── RT + A: 호버링 모드 ──────────────────────────────────────
        if a_pressed and not self._prev_rt_a:
            if self.control_mode != self.MODE_HOVERING:
                if self.pid_ctrl:
                    self.pid_ctrl.deactivate()
                if self.hovering:
                    if self.hovering.activate():
                        self.control_mode = self.MODE_HOVERING
                        self.logger.info('제어 모드: HOVERING (위치 고정)')
                        if self.led:
                            self.led.turn_off()
                        self.gui.update_system(
                            is_armed=self.is_armed,
                            sensitivity=self.sensitivity_scale,
                            lumen_brightness=(self.lumen.get_brightness() if self.lumen is not None else 0.0),
                            control_mode=self.MODE_HOVERING
                        )
                    else:
                        self.logger.warn('호버링 전환 실패 (오도메트리 없음) -> 노말 모드 유지')
                        self.control_mode = self.MODE_NORMAL
                        self.gui.update_system(
                            is_armed=self.is_armed,
                            sensitivity=self.sensitivity_scale,
                            lumen_brightness=(self.lumen.get_brightness() if self.lumen is not None else 0.0),
                            control_mode=self.MODE_NORMAL
                        )
                else:
                    self.logger.warn('호버링 컨트롤러 미초기화 -> 노말 모드 유지')
                    self.control_mode = self.MODE_NORMAL

        # ── RT + X: PID 모드 ─────────────────────────────────────────
        if x_pressed and not self._prev_rt_x:
            if self.control_mode != self.MODE_PID:
                if self.hovering:
                    self.hovering.deactivate()
                if self.pid_ctrl:
                    if self.pid_ctrl.activate():
                        self.control_mode = self.MODE_PID
                        self.logger.info('제어 모드: PID (조이스틱 위치 추종)')
                        if self.led:
                            self.led.turn_off()
                        self.gui.update_system(
                            is_armed=self.is_armed,
                            sensitivity=self.sensitivity_scale,
                            lumen_brightness=(self.lumen.get_brightness() if self.lumen is not None else 0.0),
                            control_mode=self.MODE_PID
                        )
                    else:
                        self.logger.warn('PID 모드 전환 실패 (오도메트리 없음) -> 노말 모드 유지')
                        self.control_mode = self.MODE_NORMAL
                        self.gui.update_system(
                            is_armed=self.is_armed,
                            sensitivity=self.sensitivity_scale,
                            lumen_brightness=(self.lumen.get_brightness() if self.lumen is not None else 0.0),
                            control_mode=self.MODE_NORMAL
                        )
                else:
                    self.logger.warn('PID 컨트롤러 미초기화 -> 노말 모드 유지')
                    self.control_mode = self.MODE_NORMAL

        self._prev_rt_y = y_pressed
        self._prev_rt_a = a_pressed
        self._prev_rt_x = x_pressed

    def handle_joy_message(self, msg: Joy, current_time: float):
        """
        조이스틱 메시지 처리

        Args:
            msg: Joy 메시지
            current_time: 현재 시간 (초)
        """
        # 조이스틱 메시지 수신 시간 기록 (타임아웃 체크용)
        self.last_joy_time = current_time

        buttons = msg.buttons if msg.buttons else []

        # 버튼 배열 초기화
        if len(buttons) > len(self.prev_buttons):
            self.prev_buttons = [0] * len(buttons)

        # === 시동 제어 (Menu/Options 버튼) ===
        if len(buttons) > self.BTN_MENU and buttons[self.BTN_MENU] and not self.prev_buttons[self.BTN_MENU]:
            self.arm_system()

        if len(buttons) > self.BTN_OPTIONS and buttons[self.BTN_OPTIONS] and not self.prev_buttons[self.BTN_OPTIONS]:
            self.disarm_system()

        # === 제어 모드 전환 (RT + Y/A) ===
        self._handle_mode_switch(msg, buttons)

        # === 소나 틸트 제어 (LT + 버튼 조합) ===
        if self.sonar_tilt is not None:
            self.sonar_tilt.handle_joystick(list(msg.axes), list(buttons))

        # === 공통 제어 처리 (릴레이, D-Pad 감도/루먼, 녹화) ===
        # 모든 모드에서 동작 (감도, 라이트, 릴레이 등)
        self.common.process_common_controls(msg, current_time, self)

        # 이전 버튼 상태 저장
        self.prev_buttons = list(buttons)

        # === 호버링 모드: 조이스틱 쓰러스터 제어 비활성화 ===
        if self.control_mode == self.MODE_HOVERING:
            strafe = self.apply_deadzone(msg.axes[0]) if len(msg.axes) > 0 else 0.0
            forward = self.apply_deadzone(msg.axes[1]) if len(msg.axes) > 1 else 0.0
            rotation = self.apply_deadzone(-msg.axes[3]) if len(msg.axes) > 3 else 0.0
            self.gui.update_joystick(
                left_x=-strafe, left_y=forward, right_x=rotation, right_y=0.0
            )
            return

        # === PID 모드: 조이스틱 입력을 PID 컨트롤러에 전달 ===
        # (실제 모터 명령은 타이머에서 compute_and_send_commands로 처리)
        if self.control_mode == self.MODE_PID:
            strafe   = self.apply_deadzone(msg.axes[0]) if len(msg.axes) > 0 else 0.0
            forward  = self.apply_deadzone(msg.axes[1]) if len(msg.axes) > 1 else 0.0
            rotation = self.apply_deadzone(-msg.axes[3]) if len(msg.axes) > 3 else 0.0
            if self.pid_ctrl:
                self.pid_ctrl.set_joystick(forward, strafe, rotation)
            self.gui.update_joystick(
                left_x=-strafe, left_y=forward, right_x=rotation, right_y=0.0
            )
            return

        # === 노말 모드: 조이스틱 축 입력 처리 (4모터 홀로노믹) ===
        # 왼쪽 스틱 X축: 좌우 스트레이프
        strafe = self.apply_deadzone(msg.axes[0]) if len(msg.axes) > 0 else 0.0

        # 왼쪽 스틱 Y축: 전진/후진
        forward = self.apply_deadzone(msg.axes[1]) if len(msg.axes) > 1 else 0.0

        # 오른쪽 스틱 X축: 회전
        rotation = self.apply_deadzone(-msg.axes[3]) if len(msg.axes) > 3 else 0.0

        # 좌우/전후 움직임 우선순위 로직
        if abs(strafe) > abs(forward):
            forward = 0.0
        elif abs(forward) > abs(strafe):
            strafe = 0.0

        # 웹 GUI에 조이스틱 상태 전송 (GUI 표시용 부호 보정)
        self.gui.update_joystick(
            left_x=-strafe,  # GUI 표시 방향 맞춤
            left_y=forward,
            right_x=rotation,
            right_y=0.0
        )

        # 시동이 꺼져있으면 모터 제어 불가
        if not self.is_armed:
            return

        # 조이스틱이 데드존 내에 있으면 명시적으로 정지
        if abs(forward) < 0.01 and abs(strafe) < 0.01 and abs(rotation) < 0.01:
            self.vesc.stop_all()
            commands = {
                'vesc_1': 0.0,
                'vesc_2': 0.0,
                'vesc_3': 0.0,
                'vesc_4': 0.0,
                'vesc_1_normalized': 0.0,
                'vesc_2_normalized': 0.0,
                'vesc_3_normalized': 0.0,
                'vesc_4_normalized': 0.0
            }
        else:
            # 쓰러스터 명령 계산 (4모터 홀로노믹)
            commands = self.calculate_thruster_commands(forward, strafe, rotation)

            # VESC 전류 명령 전송
            self.vesc.set_current('vesc_1', commands['vesc_1'])   # 오른쪽
            self.vesc.set_current('vesc_2', commands['vesc_2'])   # 뒷쪽
            self.vesc.set_current('vesc_3', commands['vesc_3'])   # 왼쪽
            self.vesc.set_current('vesc_4', -commands['vesc_4'])  # 앞쪽 (모터 배선 반대)

        # 웹 GUI에 모터 명령 전송 (4개 모터)
        self.gui.update_motors(
            vesc_1=commands['vesc_1'],  # 오른쪽 모터 실제 전류 (A)
            vesc_2=commands['vesc_2'],  # 뒷쪽 모터 실제 전류 (A)
            vesc_3=commands['vesc_3'],  # 왼쪽 모터 실제 전류 (A)
            vesc_4=commands['vesc_4']   # 앞쪽 모터 실제 전류 (A)
        )

        # 디버그 출력 (값이 0이 아닐 때만)
        if abs(forward) > 0.01 or abs(strafe) > 0.01 or abs(rotation) > 0.01:
            self.logger.info(
                f'[노말] 전진:{forward:+.2f} 스트레이프:{strafe:+.2f} 회전:{rotation*0.25:+.2f}(x0.25) | '
                f'VESC: 1(우):{commands["vesc_1"]:+.2f}A 2(뒤):{commands["vesc_2"]:+.2f}A '
                f'3(좌):{commands["vesc_3"]:+.2f}A 4(앞):{commands["vesc_4"]:+.2f}A',
                throttle_duration_sec=0.5
            )

    def check_timeout(self, current_time: float):
        """
        조이스틱 타임아웃 체크
        일정 시간 동안 조이스틱 메시지가 없으면 자동으로 모터 정지

        Args:
            current_time: 현재 시간 (초)
        """
        # 시동이 꺼져있으면 체크 불필요
        if not self.is_armed:
            return

        # 마지막 조이스틱 메시지 이후 경과 시간
        elapsed = current_time - self.last_joy_time

        # 타임아웃 발생 시 모터 정지 (노말 모드에서만)
        # 호버링 모드에서는 타이머가 독립적으로 제어하므로 타임아웃 미적용
        if elapsed > self.joy_timeout and self.control_mode == self.MODE_NORMAL:
            self.vesc.stop_all()
