#!/usr/bin/env python3
"""
릴레이 제어 모듈
- CH1: GPIO7번 (Jetson.GPIO 라이브러리 사용)
- CH2, CH3: 복잡한 GPIO 라인 방식 (gpioset/gpioget 사용)
- ROS2와 독립적으로 사용 가능한 모듈
"""

import subprocess
import sys
import time
import signal

try:
    import Jetson.GPIO as GPIO
except ImportError:
    print("Jetson.GPIO 라이브러리가 설치되지 않았습니다.")
    print("설치 명령: sudo pip3 install Jetson.GPIO")
    # ImportError를 발생시키지 않고 경고만 출력
    GPIO = None


class RelayControlModule:
    """릴레이 제어 모듈 클래스"""
    
    def __init__(self, auto_init=True, web_gui=None):
        """
        릴레이 제어 모듈 초기화
        
        Args:
            auto_init (bool): 자동 초기화 여부 (기본값: True)
            web_gui (WebGUIModule): 웹 GUI 모듈 인스턴스 (기본값: None)
        """
        self.web_gui = web_gui
        self._cleanup_done = False  # cleanup 중복 호출 방지
        
        # CH1: GPIO7번 (Jetson.GPIO 방식)
        self.CH1_PIN = 7
        self.ch1_initialized = False
        
        # CH2, CH3: 복잡한 GPIO 라인 방식
        # 상태 추적 변수 추가 (gpioget 호출하면 상태가 깨지므로)
        self.relay_states = {
            'CH1': False,
            'CH2': False,
            'CH3': False
        }
        
        self.channels = {
            'CH2': {
                'name': 'GPIO01',
                'pinmux_addr': '0x02430068',
                'gpio_line': 105,
                'pinmux_value': '0x040',
                'initialized': False
            },
            'CH3': {
                'name': 'GPIO11',
                'pinmux_addr': '0x02430070',
                'gpio_line': 106,
                'pinmux_value': '0x040',
                'initialized': False
            }
        }
        
        # 시그널 핸들러 설정
        signal.signal(signal.SIGINT, self._signal_handler)
        
        if auto_init:
            self.initialize()
    
    def _update_web_gui(self):
        """웹 GUI에 릴레이 상태 업데이트 (내부 상태 사용)"""
        if self.web_gui is not None:
            try:
                # 내부 상태 변수 사용 (gpioget 호출하면 CH3 상태가 깨짐)
                self.web_gui.update_relays(
                    relay_1=self.relay_states['CH1'],
                    relay_2=self.relay_states['CH2'],
                    relay_3=self.relay_states['CH3']
                )
            except Exception as e:
                print(f"웹 GUI 업데이트 실패: {e}")
    
    def initialize(self):
        """모든 릴레이 채널 초기화"""
        print("=== 릴레이 제어 모듈 초기화 중... ===")
        
        # CH1 초기화 (Jetson.GPIO 방식)
        self._init_ch1()
        
        # CH2, CH3 초기화 (복잡한 방식)
        self._init_ch2_ch3()
        
        print("=== 릴레이 제어 모듈 초기화 완료 ===")
        print(f"CH1: GPIO7번 (Jetson.GPIO) - {'✓' if self.ch1_initialized else '✗'}")
        print(f"CH2: GPIO01 (복잡한 방식) - {'✓' if self.channels['CH2']['initialized'] else '✗'}")
        print(f"CH3: GPIO11 (복잡한 방식) - {'✓' if self.channels['CH3']['initialized'] else '✗'}")
        print("모든 릴레이가 OFF 상태로 초기화되었습니다.")
        
        # 초기화 완료 후 웹 GUI에 상태 전송
        self._update_web_gui()
    
    def _init_ch1(self):
        """CH1 초기화 (Jetson.GPIO 방식)"""
        if GPIO is None:
            print("✗ CH1: Jetson.GPIO 라이브러리가 없습니다.")
            return
            
        try:
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self.CH1_PIN, GPIO.OUT)
            GPIO.output(self.CH1_PIN, GPIO.LOW)
            print("✓ CH1 (GPIO7) initialized successfully")
            self.ch1_initialized = True
        except Exception as e:
            print(f"✗ CH1 initialization failed: {e}")
            self.ch1_initialized = False
    
    def _init_ch2_ch3(self):
        """CH2, CH3 초기화 (복잡한 방식)"""
        for ch_id, ch_info in self.channels.items():
            if self._init_complex_channel(ch_id):
                ch_info['initialized'] = True
    
    def _init_complex_channel(self, channel_id):
        """복잡한 방식 채널 초기화"""
        ch = self.channels[channel_id]
        
        if ch['pinmux_addr'] is None or ch['gpio_line'] is None:
            print(f"{channel_id} ({ch['name']}): 주소/라인 정보 없음 - 건너뜀")
            return False
            
        print(f"Initializing {channel_id} ({ch['name']})...")
        
        # pinmux 설정
        cmd = f"sudo busybox devmem {ch['pinmux_addr']} w {ch['pinmux_value']}"
        result = self.run_command(cmd)
        if result is None:
            print(f"  Failed to set pinmux for {channel_id}")
            return False
            
        # GPIO 초기 LOW 설정
        self.run_command(f"gpioset gpiochip0 {ch['gpio_line']}=0")
        
        # 테스트
        test_val = self.run_command(f"gpioget gpiochip0 {ch['gpio_line']}")
        if test_val == "0":
            print(f"  ✓ {channel_id} initialized successfully")
            return True
        else:
            print(f"  ✗ {channel_id} initialization failed")
            return False
    
    def run_command(self, cmd, show_error=False):
        """명령어 실행"""
        try:
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True, check=True)
            return result.stdout.strip()
        except subprocess.CalledProcessError as e:
            if show_error:
                print(f"Error: {e}")
            return None
    
    # CH1 제어 (Jetson.GPIO 방식)
    def ch1_on(self):
        """CH1 릴레이 ON"""
        if not self.ch1_initialized:
            print("CH1 not initialized")
            return False
        if GPIO is None:
            print("Jetson.GPIO 라이브러리가 없습니다.")
            return False
            
        GPIO.output(self.CH1_PIN, GPIO.HIGH)
        self.relay_states['CH1'] = True
        print("✅ CH1 릴레이 ON (GPIO7)")
        self._update_web_gui()
        return True
    
    def ch1_off(self):
        """CH1 릴레이 OFF"""
        if not self.ch1_initialized:
            print("CH1 not initialized")
            return False
        if GPIO is None:
            print("Jetson.GPIO 라이브러리가 없습니다.")
            return False
            
        GPIO.output(self.CH1_PIN, GPIO.LOW)
        self.relay_states['CH1'] = False
        print("❌ CH1 릴레이 OFF (GPIO7)")
        self._update_web_gui()
        return True
    
    def ch1_status(self):
        """CH1 상태 확인"""
        if not self.ch1_initialized or GPIO is None:
            return None
        return GPIO.input(self.CH1_PIN)
    
    # CH2, CH3 제어 (복잡한 방식)
    def ch2_on(self):
        """CH2 릴레이 ON"""
        return self._set_complex_channel('CH2', 1)
    
    def ch2_off(self):
        """CH2 릴레이 OFF"""
        return self._set_complex_channel('CH2', 0)
    
    def ch3_on(self):
        """CH3 릴레이 ON"""
        return self._set_complex_channel('CH3', 1)
    
    def ch3_off(self):
        """CH3 릴레이 OFF"""
        return self._set_complex_channel('CH3', 0)
    
    def _set_complex_channel(self, channel_id, value):
        """복잡한 방식 채널 설정"""
        ch = self.channels[channel_id]
        
        if not ch['initialized']:
            print(f"{channel_id} not initialized")
            return False
            
        if value not in [0, 1]:
            print("Value must be 0 or 1")
            return False
            
        result = self.run_command(f"gpioset gpiochip0 {ch['gpio_line']}={value}")
        if result is not None:
            state = "HIGH" if value == 1 else "LOW"
            self.relay_states[channel_id] = bool(value)  # 상태 저장
            print(f"{channel_id} ({ch['name']}): {state}")
            self._update_web_gui()
            return True
        else:
            print(f"Failed to set {channel_id}")
            return False
    
    def _get_complex_channel(self, channel_id):
        """복잡한 방식 채널 상태 읽기"""
        ch = self.channels[channel_id]
        
        if not ch['initialized']:
            return None
            
        value = self.run_command(f"gpioget gpiochip0 {ch['gpio_line']}")
        if value is not None:
            return int(value)
        return None
    
    # 통합 제어 함수들
    def set_relay(self, channel, state):
        """
        통합 릴레이 제어 함수
        
        Args:
            channel (str): 'CH1', 'CH2', 'CH3'
            state (bool): True=ON, False=OFF
            
        Returns:
            bool: 성공 여부
        """
        channel_map = {
            'CH1': (self.ch1_on, self.ch1_off),
            'CH2': (self.ch2_on, self.ch2_off),
            'CH3': (self.ch3_on, self.ch3_off)
        }
        
        if channel not in channel_map:
            print(f"Unknown channel: {channel}")
            return False
        
        on_func, off_func = channel_map[channel]
        return on_func() if state else off_func()
    
    def set_all_relays(self, state):
        """
        모든 릴레이를 동일한 상태로 설정
        
        Args:
            state (bool): True=ON, False=OFF
        """
        state_text = "ON" if state else "OFF"
        print(f"모든 릴레이를 {state_text}으로 설정합니다...")
        
        for channel in ['CH1', 'CH2', 'CH3']:
            self.set_relay(channel, state)
        
        print(f"✅ 모든 릴레이 {state_text} (CH1, CH2, CH3)")
    
    def all_on(self):
        """모든 릴레이 ON (기존 호환성 유지)"""
        self.set_all_relays(True)
    
    def all_off(self):
        """모든 릴레이 OFF (기존 호환성 유지)"""
        self.set_all_relays(False)
    
    def get_status(self):
        """현재 릴레이 상태 확인"""
        print("=== 현재 릴레이 상태 ===")
        
        # CH1 상태
        if self.ch1_initialized:
            ch1_val = self.ch1_status()
            ch1_status = "ON" if ch1_val else "OFF"
            print(f"CH1 (GPIO7): {ch1_status}")
        else:
            print("CH1 (GPIO7): NOT INITIALIZED")
        
        # CH2, CH3 상태
        for ch_id, ch_info in self.channels.items():
            if ch_info['initialized']:
                val = self._get_complex_channel(ch_id)
                state = "ON" if val == 1 else "OFF" if val == 0 else "ERROR"
                print(f"{ch_id} ({ch_info['name']}): {state}")
            else:
                print(f"{ch_id} ({ch_info['name']}): NOT INITIALIZED")
    
    def cleanup(self):
        """GPIO 정리 (중복 호출 방지)"""
        if self._cleanup_done:
            return  # 이미 정리됨
        
        print("GPIO 정리 중...")
        self._cleanup_done = True
        
        # 안전하게 모든 릴레이 OFF
        try:
            self.all_off()
        except Exception as e:
            print(f"릴레이 OFF 중 오류: {e}")
        
        # CH1 정리 (Jetson.GPIO) - 안전하게 처리
        if self.ch1_initialized and GPIO is not None:
            try:
                GPIO.cleanup()
                self.ch1_initialized = False
            except Exception as e:
                print(f"GPIO cleanup 중 오류 (무시 가능): {e}")
        
        print("GPIO 정리 완료")
    
    def _signal_handler(self, signum, frame):
        """시그널 핸들러 (Ctrl+C)"""
        print("\n프로그램 종료 중...")
        self.cleanup()
        sys.exit(0)


# 사용 예제
if __name__ == "__main__":
    # 릴레이 제어 모듈 생성
    relay = RelayControlModule()
    
    try:
        print("\n=== 릴레이 제어 테스트 ===")
        
        # 상태 확인
        relay.get_status()
        
        # 1. 개별 함수 테스트
        print("\n1. 개별 함수 테스트...")
        relay.ch1_on()
        time.sleep(1)
        relay.ch1_off()
        
        # 2. 통합 함수 테스트
        print("\n2. 통합 함수 테스트...")
        relay.set_relay('CH2', True)   # CH2 ON
        time.sleep(1)
        relay.set_relay('CH2', False)  # CH2 OFF
        
        relay.set_relay('CH3', True)   # CH3 ON
        time.sleep(1)
        relay.set_relay('CH3', False)  # CH3 OFF
        
        # 3. 모든 릴레이 통합 제어 테스트
        print("\n3. 모든 릴레이 통합 제어 테스트...")
        relay.set_all_relays(True)     # 모든 릴레이 ON
        time.sleep(2)
        relay.set_all_relays(False)    # 모든 릴레이 OFF
        
        # 4. 기존 함수 호환성 테스트
        print("\n4. 기존 함수 호환성 테스트...")
        relay.all_on()
        time.sleep(2)
        relay.all_off()
        
        print("\n테스트 완료!")
        
    except KeyboardInterrupt:
        print("\n사용자에 의해 중단되었습니다.")
    finally:
        relay.cleanup()