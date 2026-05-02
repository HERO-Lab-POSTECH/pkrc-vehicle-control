#!/usr/bin/env python3
"""
Blue Robotics Lumen 라이트 제어 모듈
- PWM 신호로 밝기 조절 (1100μs ~ 1900μs)
- 다양한 시각적 피드백 기능 제공
"""

import Jetson.GPIO as GPIO
import time
import threading


class LumenController:
    """Lumen 라이트 PWM 제어 클래스"""
    
    def __init__(self, pin=32, frequency=50, auto_init=True):
        """
        초기화
        
        Args:
            pin: PWM 출력 핀 번호 (BOARD 모드)
            frequency: PWM 주파수 (Hz) - 서보 신호는 보통 50Hz
            auto_init: 자동으로 GPIO 초기화 여부
        """
        self.pin = pin
        self.frequency = frequency
        
        # PWM 펄스 폭 범위 (마이크로초)
        self.min_pulse = 1100  # 최소 밝기 (꺼짐)
        self.max_pulse = 1900  # 최대 밝기
        
        # 현재 밝기 (0.0 ~ 1.0)
        self._brightness = 0.0
        
        # 애니메이션 제어
        self._animation_thread = None
        self._animation_running = False
        self._stop_animation = False
        
        # GPIO 초기화
        self.pwm = None
        if auto_init:
            self.initialize()
    
    def initialize(self):
        """GPIO 및 PWM 초기화"""
        try:
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self.pin, GPIO.OUT)
            
            # PWM 시작 (50Hz)
            self.pwm = GPIO.PWM(self.pin, self.frequency)
            self.pwm.start(0)
            
            print(f"✅ Lumen 라이트 초기화 완료 (핀: {self.pin})")
            return True
        except Exception as e:
            print(f"❌ Lumen 라이트 초기화 실패: {e}")
            return False
    
    def pulse_to_duty_cycle(self, pulse_us):
        """
        펄스 폭(마이크로초)을 듀티 사이클(%)로 변환
        
        Args:
            pulse_us: 펄스 폭 (마이크로초)
            
        Returns:
            float: 듀티 사이클 (%)
        """
        # 주기 = 1/주파수 (초)
        period_us = (1.0 / self.frequency) * 1_000_000
        # 듀티 사이클 = (펄스 폭 / 주기) * 100
        duty_cycle = (pulse_us / period_us) * 100.0
        return duty_cycle
    
    def set_brightness(self, brightness, smooth=False, duration=0.5):
        """
        밝기 설정
        
        Args:
            brightness: 밝기 (0.0 ~ 1.0)
            smooth: 부드럽게 전환 여부
            duration: 부드러운 전환 시간 (초)
        """
        if self.pwm is None:
            print("⚠️  PWM이 초기화되지 않았습니다.")
            return False
        
        # 애니메이션 중지
        self.stop_animation()
        
        # 범위 제한
        brightness = max(0.0, min(1.0, brightness))
        
        if smooth and abs(brightness - self._brightness) > 0.01:
            # 부드러운 전환
            steps = 20
            step_delay = duration / steps
            brightness_step = (brightness - self._brightness) / steps
            
            for i in range(steps):
                self._brightness += brightness_step
                self._set_brightness_immediate(self._brightness)
                time.sleep(step_delay)
            
            # 최종 값으로 정확히 설정
            self._brightness = brightness
            self._set_brightness_immediate(brightness)
        else:
            # 즉시 전환
            self._brightness = brightness
            self._set_brightness_immediate(brightness)
        
        return True
    
    def _set_brightness_immediate(self, brightness):
        """밝기를 즉시 설정 (내부 사용)"""
        if self.pwm is None:
            return
        
        # 밝기를 펄스 폭으로 변환
        pulse_us = self.min_pulse + (self.max_pulse - self.min_pulse) * brightness
        
        # 듀티 사이클 계산 및 설정
        duty_cycle = self.pulse_to_duty_cycle(pulse_us)
        self.pwm.ChangeDutyCycle(duty_cycle)
    
    def get_brightness(self):
        """현재 밝기 반환"""
        return self._brightness
    
    def increase_brightness(self, step=0.1):
        """
        밝기 증가
        
        Args:
            step: 증가량 (0.0 ~ 1.0)
        """
        new_brightness = min(1.0, self._brightness + step)
        return self.set_brightness(new_brightness)
    
    def decrease_brightness(self, step=0.1):
        """
        밝기 감소
        
        Args:
            step: 감소량 (0.0 ~ 1.0)
        """
        new_brightness = max(0.0, self._brightness - step)
        return self.set_brightness(new_brightness)
    
    def turn_off(self):
        """라이트 끄기"""
        return self.set_brightness(0.0)
    
    def turn_on(self, brightness=1.0):
        """
        라이트 켜기
        
        Args:
            brightness: 밝기 (0.0 ~ 1.0), 기본값은 최대 밝기
        """
        return self.set_brightness(brightness)
    
    def stop_animation(self):
        """현재 실행 중인 애니메이션 중지"""
        if self._animation_running:
            self._stop_animation = True
            if self._animation_thread:
                self._animation_thread.join(timeout=2.0)
            self._animation_running = False
            self._stop_animation = False
    
    def _run_animation(self, animation_func, *args, **kwargs):
        """애니메이션 실행 (내부 사용)"""
        self.stop_animation()
        self._animation_running = True
        self._animation_thread = threading.Thread(
            target=animation_func, 
            args=args, 
            kwargs=kwargs,
            daemon=True
        )
        self._animation_thread.start()
    
    def boot_signal(self, cycles=3, duration=1.5):
        """
        부팅 완료 신호 (밝기가 서서히 증가했다 감소하는 패턴 반복)
        
        Args:
            cycles: 반복 횟수
            duration: 한 사이클 시간 (초)
        """
        def _boot_animation():
            original_brightness = self._brightness
            
            for i in range(cycles):
                if self._stop_animation:
                    break
                
                # 서서히 증가
                steps = 20
                step_delay = (duration / 2) / steps
                for j in range(steps):
                    if self._stop_animation:
                        break
                    brightness = j / steps
                    self._set_brightness_immediate(brightness)
                    time.sleep(step_delay)
                
                # 서서히 감소
                for j in range(steps, -1, -1):
                    if self._stop_animation:
                        break
                    brightness = j / steps
                    self._set_brightness_immediate(brightness)
                    time.sleep(step_delay)
            
            # 원래 밝기로 복원
            self._brightness = original_brightness
            self._set_brightness_immediate(original_brightness)
            self._animation_running = False
        
        self._run_animation(_boot_animation)
        print("🚀 부팅 신호 시작")
    
    def error_signal(self, error_level='warning', duration=3.0):
        """
        오류 신호 (깜빡임 패턴)
        
        Args:
            error_level: 오류 레벨 ('warning', 'error', 'critical')
            duration: 신호 지속 시간 (초)
        """
        def _error_animation():
            original_brightness = self._brightness
            
            # 오류 레벨에 따른 깜빡임 패턴
            if error_level == 'warning':
                # 경고: 느린 깜빡임 (1초 간격)
                blink_interval = 1.0
                blink_count = int(duration / blink_interval)
            elif error_level == 'error':
                # 에러: 중간 속도 깜빡임 (0.5초 간격)
                blink_interval = 0.5
                blink_count = int(duration / blink_interval)
            else:  # critical
                # 치명적: 빠른 깜빡임 (0.2초 간격)
                blink_interval = 0.2
                blink_count = int(duration / blink_interval)
            
            for i in range(blink_count):
                if self._stop_animation:
                    break
                
                # 켜기
                self._set_brightness_immediate(1.0)
                time.sleep(blink_interval / 2)
                
                if self._stop_animation:
                    break
                
                # 끄기
                self._set_brightness_immediate(0.0)
                time.sleep(blink_interval / 2)
            
            # 원래 밝기로 복원
            self._brightness = original_brightness
            self._set_brightness_immediate(original_brightness)
            self._animation_running = False
        
        self._run_animation(_error_animation)
        print(f"⚠️  오류 신호 시작 (레벨: {error_level})")
    
    def pulse_pattern(self, min_brightness=0.2, max_brightness=1.0, period=2.0, duration=None):
        """
        펄스 패턴 (호흡등 효과)
        
        Args:
            min_brightness: 최소 밝기 (0.0 ~ 1.0)
            max_brightness: 최대 밝기 (0.0 ~ 1.0)
            period: 한 주기 시간 (초)
            duration: 지속 시간 (초), None이면 무한 반복
        """
        def _pulse_animation():
            start_time = time.time()
            
            while True:
                if self._stop_animation:
                    break
                
                # 지속 시간 체크
                if duration is not None and (time.time() - start_time) >= duration:
                    break
                
                # 사인파 패턴으로 밝기 변화
                elapsed = time.time() - start_time
                phase = (elapsed % period) / period  # 0.0 ~ 1.0
                
                # 사인 함수를 사용한 부드러운 변화
                import math
                brightness = min_brightness + (max_brightness - min_brightness) * \
                            (0.5 + 0.5 * math.sin(2 * math.pi * phase - math.pi / 2))
                
                self._set_brightness_immediate(brightness)
                time.sleep(0.05)  # 20Hz 업데이트
            
            self._animation_running = False
        
        self._run_animation(_pulse_animation)
        print("💓 펄스 패턴 시작")
    
    def strobe(self, frequency=5.0, duration=2.0):
        """
        스트로브 효과 (빠른 깜빡임)
        
        Args:
            frequency: 깜빡임 주파수 (Hz)
            duration: 지속 시간 (초)
        """
        def _strobe_animation():
            interval = 1.0 / frequency / 2  # 켜기/끄기 각각의 시간
            blink_count = int(duration * frequency)
            
            for i in range(blink_count):
                if self._stop_animation:
                    break
                
                self._set_brightness_immediate(1.0)
                time.sleep(interval)
                
                if self._stop_animation:
                    break
                
                self._set_brightness_immediate(0.0)
                time.sleep(interval)
            
            self._animation_running = False
        
        self._run_animation(_strobe_animation)
        print(f"⚡ 스트로브 시작 ({frequency}Hz)")
    
    def cleanup(self):
        """정리"""
        self.stop_animation()
        
        if self.pwm:
            self.pwm.stop()
        
        try:
            GPIO.cleanup(self.pin)
            print("✅ Lumen 라이트 정리 완료")
        except:
            pass


# 편의 함수들
def create_lumen_controller(pin=32, frequency=50):
    """
    Lumen 컨트롤러 생성
    
    Args:
        pin: PWM 핀 번호
        frequency: PWM 주파수
        
    Returns:
        LumenController: 초기화된 컨트롤러
    """
    return LumenController(pin=pin, frequency=frequency, auto_init=True)


if __name__ == '__main__':
    """테스트 코드"""
    print("=" * 60)
    print("🔦 Lumen 모듈 테스트")
    print("=" * 60)
    
    # 컨트롤러 생성
    lumen = create_lumen_controller(pin=32)
    
    try:
        # 부팅 신호
        print("\n1️⃣  부팅 신호 테스트...")
        lumen.boot_signal(cycles=2, duration=1.0)
        time.sleep(3)
        
        # 밝기 설정
        print("\n2️⃣  밝기 설정 테스트...")
        lumen.set_brightness(0.5, smooth=True)
        time.sleep(2)
        
        # 밝기 증가/감소
        print("\n3️⃣  밝기 증가/감소 테스트...")
        lumen.increase_brightness(0.3)
        time.sleep(1)
        lumen.decrease_brightness(0.5)
        time.sleep(1)
        
        # 경고 신호
        print("\n4️⃣  경고 신호 테스트...")
        lumen.error_signal(error_level='warning', duration=3.0)
        time.sleep(4)
        
        # 에러 신호
        print("\n5️⃣  에러 신호 테스트...")
        lumen.error_signal(error_level='error', duration=3.0)
        time.sleep(4)
        
        # 펄스 패턴
        print("\n6️⃣  펄스 패턴 테스트...")
        lumen.pulse_pattern(min_brightness=0.2, max_brightness=1.0, period=2.0, duration=5.0)
        time.sleep(6)
        
        # 끄기
        print("\n7️⃣  라이트 끄기...")
        lumen.turn_off()
        
        print("\n✅ 모든 테스트 완료!")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Ctrl+C 감지")
    
    finally:
        lumen.cleanup()
        print("👋 프로그램 종료")
