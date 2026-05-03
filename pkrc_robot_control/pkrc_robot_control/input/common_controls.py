#!/usr/bin/env python3
"""
공통 조이스틱 제어 모듈
- 릴레이 제어 (LB/RB + 버튼)
- D-Pad 감도/루먼 조절
- 녹화 제어
- 모든 컨트롤러에서 공유
"""

from sensor_msgs.msg import Joy


class CommonControls:
    """공통 조이스틱 제어 클래스"""

    # Xbox 컨트롤러 버튼 매핑
    BTN_A = 0
    BTN_B = 1
    BTN_X = 2
    BTN_Y = 3
    BTN_LB = 4
    BTN_RB = 5
    BTN_OPTIONS = 6
    BTN_MENU = 7
    BTN_SPECIAL_8 = 8

    # D-Pad 축 매핑
    DPAD_HORIZONTAL = 6
    DPAD_VERTICAL = 7

    def __init__(self,
                 relay_controller,
                 lumen_controller,
                 web_gui,
                 logger,
                 main_node=None,
                 sensitivity_step=0.1,
                 dpad_debounce_time=0.3):
        """
        공통 제어 초기화

        Args:
            relay_controller: 릴레이 제어기
            lumen_controller: Lumen 라이트 제어기
            web_gui: 웹 GUI 모듈
            logger: ROS2 로거
            main_node: 메인 노드 (녹화 제어용)
            sensitivity_step: 감도 조절 단계
            dpad_debounce_time: D-Pad 디바운스 시간
        """
        self.relay = relay_controller
        self.lumen = lumen_controller
        self.gui = web_gui
        self.logger = logger
        self.main_node = main_node

        self.sensitivity_step = sensitivity_step
        self.dpad_debounce_time = dpad_debounce_time

        # D-Pad 디바운스 타이머
        self.last_dpad_time = {
            'up': 0.0,
            'down': 0.0,
            'left': 0.0,
            'right': 0.0
        }

        # 이전 버튼 상태
        self.prev_buttons = []

    def handle_buttons(self, msg: Joy, current_time: float, controller):
        """
        공통 버튼 처리 (릴레이, 녹화)

        Args:
            msg: Joy 메시지
            current_time: 현재 시간
            controller: 호출한 컨트롤러 (is_armed, sensitivity_scale 접근용)

        Returns:
            list: 업데이트된 prev_buttons
        """
        buttons = msg.buttons if msg.buttons else []

        # 버튼 배열 초기화
        if len(buttons) > len(self.prev_buttons):
            self.prev_buttons = [0] * len(buttons)

        # LB/RB 상태
        lb_pressed = len(buttons) > self.BTN_LB and buttons[self.BTN_LB]
        rb_pressed = len(buttons) > self.BTN_RB and buttons[self.BTN_RB]

        # === 릴레이 제어 (LB/RB + 버튼) ===
        if len(buttons) > self.BTN_X and buttons[self.BTN_X] and not self.prev_buttons[self.BTN_X]:
            if lb_pressed:
                self.relay.set_relay('CH1', False)  # LB + X: CH1 OFF
            elif rb_pressed:
                self.relay.set_relay('CH1', True)   # RB + X: CH1 ON

        if len(buttons) > self.BTN_Y and buttons[self.BTN_Y] and not self.prev_buttons[self.BTN_Y]:
            if lb_pressed:
                self.relay.set_relay('CH2', False)  # LB + Y: CH2 OFF
            elif rb_pressed:
                self.relay.set_relay('CH2', True)   # RB + Y: CH2 ON

        if len(buttons) > self.BTN_B and buttons[self.BTN_B] and not self.prev_buttons[self.BTN_B]:
            if lb_pressed:
                self.relay.set_relay('CH3', False)  # LB + B: CH3 OFF
            elif rb_pressed:
                self.relay.set_relay('CH3', True)   # RB + B: CH3 ON

        if len(buttons) > self.BTN_A and buttons[self.BTN_A] and not self.prev_buttons[self.BTN_A]:
            if lb_pressed:
                self.relay.set_all_relays(False)    # LB + A: 모두 OFF
            elif rb_pressed:
                self.relay.set_all_relays(True)     # RB + A: 모두 ON

        # === 녹화 제어 (버튼 8번) ===
        if len(buttons) > self.BTN_SPECIAL_8 and buttons[self.BTN_SPECIAL_8] and not self.prev_buttons[self.BTN_SPECIAL_8]:
            if self.main_node is not None:
                self.main_node.camera_mgr.toggle_recording()

        # 이전 버튼 상태 저장
        self.prev_buttons = list(buttons)

        return self.prev_buttons

    def handle_dpad(self, msg: Joy, current_time: float, controller):
        """
        D-Pad 처리 (감도, 루먼 밝기)

        Args:
            msg: Joy 메시지
            current_time: 현재 시간
            controller: 호출한 컨트롤러 (is_armed, sensitivity_scale 접근용)
        """
        if len(msg.axes) <= max(self.DPAD_VERTICAL, self.DPAD_HORIZONTAL):
            return

        # 감도 조절 (상하)
        if msg.axes[self.DPAD_VERTICAL] > 0.5:  # 위
            if (current_time - self.last_dpad_time['up']) > self.dpad_debounce_time:
                self._adjust_sensitivity(controller, increase=True)
                self.last_dpad_time['up'] = current_time
        elif msg.axes[self.DPAD_VERTICAL] < -0.5:  # 아래
            if (current_time - self.last_dpad_time['down']) > self.dpad_debounce_time:
                self._adjust_sensitivity(controller, increase=False)
                self.last_dpad_time['down'] = current_time

        # 라이트 밝기 조절 (좌우) — lumen 없으면 무시 (degraded mode)
        if self.lumen is not None:
            if msg.axes[self.DPAD_HORIZONTAL] > 0.5:  # 오른쪽 (D-Pad)
                if (current_time - self.last_dpad_time['left']) > self.dpad_debounce_time:
                    self.lumen.decrease_brightness(0.1)
                    self._update_lumen_gui(controller)
                    self.last_dpad_time['left'] = current_time
            elif msg.axes[self.DPAD_HORIZONTAL] < -0.5:  # 왼쪽 (D-Pad)
                if (current_time - self.last_dpad_time['right']) > self.dpad_debounce_time:
                    self.lumen.increase_brightness(0.1)
                    self._update_lumen_gui(controller)
                    self.last_dpad_time['right'] = current_time

    def _adjust_sensitivity(self, controller, increase=True):
        """감도 조절"""
        if increase:
            controller.sensitivity_scale = min(1.0, controller.sensitivity_scale + self.sensitivity_step)
        else:
            controller.sensitivity_scale = max(0.1, controller.sensitivity_scale - self.sensitivity_step)

        self.logger.info(f'감도: {controller.sensitivity_scale:.1f}')

        # 웹 GUI 업데이트
        self.gui.update_system(
            is_armed=controller.is_armed,
            sensitivity=controller.sensitivity_scale,
            lumen_brightness=(self.lumen.get_brightness() if self.lumen is not None else 0.0)
        )

    def _update_lumen_gui(self, controller):
        """루먼 밝기 GUI 업데이트"""
        brightness = self.lumen.get_brightness()
        self.logger.info(f'라이트 밝기: {brightness*100:.0f}%')
        self.gui.update_system(
            is_armed=controller.is_armed,
            sensitivity=controller.sensitivity_scale,
            lumen_brightness=brightness
        )

    def process_common_controls(self, msg: Joy, current_time: float, controller):
        """
        모든 공통 제어 처리 (버튼 + D-Pad)

        Args:
            msg: Joy 메시지
            current_time: 현재 시간
            controller: 호출한 컨트롤러
        """
        self.handle_buttons(msg, current_time, controller)
        self.handle_dpad(msg, current_time, controller)
