#!/usr/bin/env python3
"""
PWM 쓰러스터 제어 모듈
- BlueBoat M200 모터 2개 제어 (Pin 32, 33)
- PWM 신호 생성 (50Hz, 1100~1900us)
- 1500us = 중립, <1500us = 전진, >1500us = 후진
"""

import Jetson.GPIO as GPIO


class PWMThrusterController:
    """PWM 쓰러스터 제어 클래스"""
    
    # PWM 상수
    PWM_FREQUENCY = 50  # 50Hz (ESC 표준)
    
    # ========================================
    # ⚙️ NEUTRAL_DUTY 캘리브레이션
    # ========================================
    # ESC마다 정확한 중립점이 다를 수 있습니다!
    # 
    # 🔍 캘리브레이션 방법:
    #   1. 조이스틱을 건드리지 않고 코드 실행
    #   2. 모터 동작 관찰:
    #      - 완전히 멈춤 → 캘리브레이션 완료! ✅
    #      - 약하게 전진 → NEUTRAL_DUTY를 올리세요 (예: 7.55)
    #      - 약하게 후진 → NEUTRAL_DUTY를 내리세요 (예: 7.45)
    #   3. 모터가 완전히 멈출 때까지 반복
    # 
    # 💡 1% = 200us (50Hz 기준)
    # 
    # 🔍 문제별 조정 방법:
    #    - 전진 후 멈추면 계속 돔 → 값을 올리세요! ⬆️
    #    - 후진 후 멈추면 계속 돔 → 값을 내리세요! ⬇️
    # 
    #    7.55 = 1510us
    #    7.52 = 1504us
    #    7.50 = 1500us (표준 중립) ← 전진 후 멈춤 문제면 여기부터 시작
    #    7.47 = 1494us
    #    7.45 = 1490us
    # 
    # ⚠️ 경고: 7.25(1450us) 이하로 내리지 마세요!
    #    ESC가 "전진 모드"로 인식합니다!
    # 
    # 🔥 중요: stop_all()은 항상 이 값을 출력합니다!
    #    전진 후 멈추면 계속 돈다 = 이 값을 올리세요! ⬆️
    #    후진 후 멈추면 계속 돈다 = 이 값을 내리세요! ⬇️
    # ========================================
    NEUTRAL_DUTY = 7.55  # ← 여기를 조정하세요! (전진 후 멈춤 문제 = 올리기!)
    
    # PWM 범위 (duty cycle %)
    MIN_DUTY = 5.5   # 1100us (최대 전진)
    MAX_DUTY = 9.5   # 1900us (최대 후진)
    
    # ========================================
    # 🛡️ DEADZONE (데드존 - 떨림 방지)
    # ========================================
    # 중립 근처의 값을 강제로 중립으로 만듭니다
    # 이 값이 클수록 더 넓은 범위를 중립으로 처리합니다
    # 
    # 💡 권장값:
    #    0.30 = ±60us (1460~1580us) - 매우 확실! ✅✅
    #    0.25 = ±50us (1470~1570us) - 확실
    #    0.15 = ±30us (1490~1550us) - 안전
    #    0.10 = ±20us (1500~1540us) - 보통
    # ========================================
    DEADZONE_DUTY = 0.30  # ← 매우 크게! 확실히 멈추게!
    
    # ========================================
    # 🎚️ TRIM 설정 (좌우 균형 조정)
    # ========================================
    # 보트가 한쪽으로 치우치면 여기서 조정하세요!
    # -1.0 ~ 1.0 범위 (음수: 약화, 양수: 강화)
    #
    # 예시:
    #   - 오른쪽으로 치우침 → TRIM_LEFT = -0.05 (왼쪽 약화)
    #   - 왼쪽으로 치우침 → TRIM_RIGHT = -0.05 (오른쪽 약화)
    # ========================================
    TRIM_LEFT = 0.0  # 왼쪽 모터 트림 (-1.0 ~ 1.0)
    TRIM_RIGHT = 0.0  # 오른쪽 모터 트림 (-1.0 ~ 1.0)
    
    def __init__(self, left_pin=32, right_pin=33, logger=None):
        """
        초기화
        
        Args:
            left_pin: 왼쪽 모터 핀 (기본: 32)
            right_pin: 오른쪽 모터 핀 (기본: 33)
            logger: ROS2 로거 (선택)
        """
        self.left_pin = left_pin
        self.right_pin = right_pin
        self.logger = logger
        
        self.left_pwm = None
        self.right_pwm = None
        self.is_initialized = False
        
        # 현재 duty cycle 추적
        self.left_duty = self.NEUTRAL_DUTY
        self.right_duty = self.NEUTRAL_DUTY
    
    def initialize(self):
        """GPIO 및 PWM 초기화"""
        try:
            GPIO.setmode(GPIO.BOARD)
            GPIO.setwarnings(False)
            
            # 왼쪽 모터 (Pin 32)
            GPIO.setup(self.left_pin, GPIO.OUT)
            self.left_pwm = GPIO.PWM(self.left_pin, self.PWM_FREQUENCY)
            self.left_pwm.start(self.NEUTRAL_DUTY)
            
            # 오른쪽 모터 (Pin 33)
            GPIO.setup(self.right_pin, GPIO.OUT)
            self.right_pwm = GPIO.PWM(self.right_pin, self.PWM_FREQUENCY)
            self.right_pwm.start(self.NEUTRAL_DUTY)
            
            self.is_initialized = True
            
            if self.logger:
                self.logger.info('✅ PWM 쓰러스터 초기화 완료')
                self.logger.info(f'   왼쪽 모터: Pin {self.left_pin}')
                self.logger.info(f'   오른쪽 모터: Pin {self.right_pin}')
                
                # TRIM 값이 설정되어 있으면 표시
                if self.TRIM_LEFT != 0.0 or self.TRIM_RIGHT != 0.0:
                    self.logger.info(f'🎚️  트림 설정: 왼쪽={self.TRIM_LEFT:+.2f}, 오른쪽={self.TRIM_RIGHT:+.2f}')
            
            return True
            
        except Exception as e:
            if self.logger:
                self.logger.error(f'❌ PWM 초기화 실패: {e}')
            self.is_initialized = False
            return False
    
    def _clamp_duty(self, duty):
        """Duty cycle을 안전 범위로 제한"""
        return max(self.MIN_DUTY, min(self.MAX_DUTY, duty))
    
    def _normalized_to_duty(self, normalized_value):
        """
        정규화된 값 (-1.0 ~ 1.0)을 duty cycle로 변환 (데드존 포함)
        
        Args:
            normalized_value: -1.0 (최대 후진) ~ 0.0 (중립) ~ 1.0 (최대 전진)
            
        Returns:
            float: duty cycle (%)
        """
        # -1.0 ~ 1.0 → 1100us ~ 1900us
        # 전진 (양수): duty가 작아짐 (< NEUTRAL)
        # 후진 (음수): duty가 커짐 (> NEUTRAL)
        
        # NEUTRAL_DUTY를 중심으로 ±400us 범위
        # +1.0 → NEUTRAL - 2.0 = 5.47 → 1094us (최대 전진)
        #  0.0 → NEUTRAL = 7.47 → 1494us (중립)
        # -1.0 → NEUTRAL + 2.0 = 9.47 → 1894us (최대 후진)
        
        duty = self.NEUTRAL_DUTY - (normalized_value * 2.0)
        duty = self._clamp_duty(duty)
        
        # 데드존 적용: 중립 근처 값은 중립으로 강제
        # DEADZONE = 0.15 → 7.32~7.62 (1464us~1524us) 범위는 NEUTRAL_DUTY로
        if abs(duty - self.NEUTRAL_DUTY) < self.DEADZONE_DUTY:
            duty = self.NEUTRAL_DUTY
        
        return duty
    
    def set_left_motor(self, normalized_value):
        """
        왼쪽 모터 설정 (TRIM 적용)
        
        Args:
            normalized_value: -1.0 (최대 후진) ~ 0.0 (중립) ~ 1.0 (최대 전진)
        """
        if not self.is_initialized or self.left_pwm is None:
            return
        
        # TRIM 적용
        trimmed_value = normalized_value + self.TRIM_LEFT
        trimmed_value = max(-1.0, min(1.0, trimmed_value))
        
        duty = self._normalized_to_duty(trimmed_value)
        self.left_duty = duty
        self.left_pwm.ChangeDutyCycle(duty)
    
    def set_right_motor(self, normalized_value):
        """
        오른쪽 모터 설정 (TRIM 적용)
        
        Args:
            normalized_value: -1.0 (최대 후진) ~ 0.0 (중립) ~ 1.0 (최대 전진)
        """
        if not self.is_initialized or self.right_pwm is None:
            return
        
        # TRIM 적용
        trimmed_value = normalized_value + self.TRIM_RIGHT
        trimmed_value = max(-1.0, min(1.0, trimmed_value))
        
        duty = self._normalized_to_duty(trimmed_value)
        self.right_duty = duty
        self.right_pwm.ChangeDutyCycle(duty)
    
    def set_both_motors(self, left_value, right_value):
        """
        양쪽 모터 동시 설정
        
        Args:
            left_value: 왼쪽 모터 값 (-1.0 ~ 1.0)
            right_value: 오른쪽 모터 값 (-1.0 ~ 1.0)
        """
        self.set_left_motor(left_value)
        self.set_right_motor(right_value)
    
    def stop_all(self):
        """모든 모터 중립 (정지)"""
        self.set_both_motors(0.0, 0.0)
    
    def get_status(self):
        """
        현재 상태 반환
        
        Returns:
            dict: 현재 duty cycle 상태
        """
        return {
            'left_duty': self.left_duty,
            'right_duty': self.right_duty,
            'left_us': int(self.left_duty / 100 * 20000),
            'right_us': int(self.right_duty / 100 * 20000)
        }
    
    def cleanup(self):
        """
        정리 (신호 유지 - 비프음 방지)
        
        중요: PWM 신호를 끊지 않고 중립 신호를 유지한 채로 종료합니다.
        이렇게 하면 ESC가 "신호 손실" 경고음을 내지 않습니다.
        """
        if self.logger:
            self.logger.info('🛑 PWM 쓰러스터 정리 중...')
        
        # 중립 신호로 설정
        self.stop_all()
        
        if self.logger:
            self.logger.info('✅ 중립 신호 유지 중 (비프음 방지)')
        
        # PWM stop()과 GPIO.cleanup()을 호출하지 않음!
        # 신호를 계속 보내는 상태로 두어 ESC가 "신호 손실"로 인식하지 않도록 함


class DifferentialDrive:
    """
    2륜 차동 구동 계산 헬퍼 클래스
    """
    
    @staticmethod
    def calculate(forward, rotation):
        """
        전진/후진과 회전 입력으로부터 좌우 모터 값 계산
        
        Args:
            forward: 전진/후진 (-1.0 ~ 1.0)
            rotation: 회전 (-1.0 왼쪽 ~ 1.0 오른쪽)
            
        Returns:
            tuple: (left_motor, right_motor) 각각 -1.0 ~ 1.0
        """
        # 차동 구동 공식
        left = forward + rotation
        right = forward - rotation
        
        # 값 제한 (-1.0 ~ 1.0)
        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))
        
        return left, right

