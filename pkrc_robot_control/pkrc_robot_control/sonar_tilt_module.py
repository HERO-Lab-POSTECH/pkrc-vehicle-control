#!/usr/bin/env python3
"""
소나 틸트 제어 모듈
- ROS2 토픽을 통한 소나 각도 제어
- 조이스틱 입력 처리 (LT + 버튼 조합)
- 현재 각도 구독 및 GUI 업데이트
"""

from std_msgs.msg import Float32


class SonarTiltModule:
    """소나 틸트 제어 모듈"""

    # 각도 단계 (0, 30, 45, 60, 90)
    ANGLE_STEPS = [0.0, 30.0, 45.0, 60.0, 90.0]

    def __init__(self, ros_node, web_gui=None, logger=None):
        """
        초기화

        Args:
            ros_node: ROS2 노드 (publisher/subscriber 생성용)
            web_gui: WebGUIModule 인스턴스
            logger: ROS2 로거
        """
        self.node = ros_node
        self.gui = web_gui
        self.logger = logger

        # 현재 상태
        self.current_angle = 0.0
        self.goal_angle = 0.0
        self.current_step_index = 0

        # 버튼 이전 상태 (에지 감지용)
        self.prev_x_pressed = False
        self.prev_b_pressed = False
        self.prev_a_pressed = False
        self.prev_y_pressed = False

        # Publisher: 목표 각도 설정
        self.pub_set_angle = self.node.create_publisher(
            Float32, '/sonar/tilt/set_angle', 10
        )

        # Subscriber: 현재 각도 수신
        self.sub_current_angle = self.node.create_subscription(
            Float32, '/sonar/tilt/current_angle',
            self._current_angle_callback, 10
        )

        self._log_info('소나 틸트 모듈 초기화 완료')
        self._log_info('조작법 (LT 홀드 + 버튼):')
        self._log_info('  Y 버튼: 0도')
        self._log_info('  A 버튼: 90도')
        self._log_info('  X 버튼: 단계 감소 (90->60->45->30->0)')
        self._log_info('  B 버튼: 단계 증가 (0->30->45->60->90)')

    def _log_info(self, msg):
        """로그 출력"""
        if self.logger:
            self.logger.info(f'🎯 [Tilt] {msg}')
        else:
            print(f'🎯 [Tilt] {msg}')

    def _current_angle_callback(self, msg: Float32):
        """현재 각도 수신 콜백"""
        self.current_angle = msg.data

        # GUI 업데이트
        if self.gui:
            self.gui.update_sonar_tilt(
                current_angle=self.current_angle,
                goal_angle=self.goal_angle
            )

    def set_angle(self, angle: float):
        """
        목표 각도 설정

        Args:
            angle: 목표 각도 (0 ~ 90도)
        """
        # 범위 제한
        angle = max(0.0, min(90.0, angle))
        self.goal_angle = angle

        # 목표 각도에 맞춰 step_index 동기화
        closest_idx = 0
        min_diff = abs(angle - self.ANGLE_STEPS[0])
        for i, step_angle in enumerate(self.ANGLE_STEPS):
            diff = abs(angle - step_angle)
            if diff < min_diff:
                min_diff = diff
                closest_idx = i
        self.current_step_index = closest_idx

        # ROS2 토픽 발행
        msg = Float32()
        msg.data = angle
        self.pub_set_angle.publish(msg)

        self._log_info(f'목표 각도 설정: {angle:.1f}도 (단계: {self.current_step_index})')

        # GUI 업데이트
        if self.gui:
            self.gui.update_sonar_tilt(
                current_angle=self.current_angle,
                goal_angle=self.goal_angle
            )

    def step_up(self):
        """각도 한 단계 증가"""
        if self.current_step_index < len(self.ANGLE_STEPS) - 1:
            self.current_step_index += 1
            new_angle = self.ANGLE_STEPS[self.current_step_index]
            self.set_angle(new_angle)
            self._log_info(f'단계 증가: {new_angle}도')
        else:
            self._log_info('이미 최대 각도 (90도)')

    def step_down(self):
        """각도 한 단계 감소"""
        if self.current_step_index > 0:
            self.current_step_index -= 1
            new_angle = self.ANGLE_STEPS[self.current_step_index]
            self.set_angle(new_angle)
            self._log_info(f'단계 감소: {new_angle}도')
        else:
            self._log_info('이미 최소 각도 (0도)')

    def handle_joystick(self, axes, buttons):
        """
        조이스틱 입력 처리 (PKRC_joy_module에서 호출)

        Args:
            axes: 조이스틱 축 값 리스트
            buttons: 조이스틱 버튼 값 리스트

        조작법:
        - LT (axes[2] < -0.5) 홀드 상태에서:
          - Y 버튼 (buttons[3]): 0도
          - A 버튼 (buttons[0]): 90도
          - X 버튼 (buttons[2]): 단계 감소
          - B 버튼 (buttons[1]): 단계 증가
        """
        if len(axes) < 5 or len(buttons) < 3:
            return

        # LT (왼쪽 트리거) 체크 - 1이 기본, -1이 완전히 눌린 상태
        lt_pressed = axes[2] < -0.5

        if not lt_pressed:
            # 트리거가 눌리지 않으면 이전 상태만 리셋
            self.prev_x_pressed = False
            self.prev_b_pressed = False
            self.prev_a_pressed = False
            self.prev_y_pressed = False
            return

        # 현재 버튼/축 상태
        a_pressed = buttons[0] == 1
        b_pressed = buttons[1] == 1
        x_pressed = buttons[2] == 1
        y_pressed = buttons[3] == 1

        # Y 버튼 -> 0도 (에지 감지)
        if y_pressed and not self.prev_y_pressed:
            self.current_step_index = 0
            self.set_angle(0.0)
            self._log_info('Y 버튼 -> 0도')

        # A 버튼 -> 90도 (에지 감지)
        elif a_pressed and not self.prev_a_pressed:
            self.current_step_index = len(self.ANGLE_STEPS) - 1
            self.set_angle(90.0)
            self._log_info('A 버튼 -> 90도')

        # X 버튼 -> 단계 감소 (에지 감지)
        elif x_pressed and not self.prev_x_pressed:
            self.step_down()

        # B 버튼 -> 단계 증가 (에지 감지)
        elif b_pressed and not self.prev_b_pressed:
            self.step_up()

        # 이전 상태 업데이트
        self.prev_x_pressed = x_pressed
        self.prev_b_pressed = b_pressed
        self.prev_a_pressed = a_pressed
        self.prev_y_pressed = y_pressed

    def get_current_angle(self) -> float:
        """현재 각도 반환"""
        return self.current_angle

    def get_goal_angle(self) -> float:
        """목표 각도 반환"""
        return self.goal_angle

    def get_current_step(self) -> int:
        """현재 단계 인덱스 반환"""
        return self.current_step_index
