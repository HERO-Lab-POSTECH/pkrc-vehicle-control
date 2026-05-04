#!/usr/bin/env python3
"""
Blue Robotics Subsea RGB LED Indicator 제어 모듈 (SPI0 전용)
- 실제 테스트로 검증된 방식 사용
- WS2812B RGB LED 제어
- GRB 색상 순서
"""

import spidev
import time
import threading

from .._log import make_logger


class BlueRoboticsLED:
    """Blue Robotics RGB LED 제어 클래스 (SPI0 전용)"""
    
    def __init__(self, spi_bus=0, spi_device=0, web_gui=None, *, logger=None):
        """
        초기화
        
        Args:
            spi_bus: SPI 버스 (기본: 0)
            spi_device: SPI 디바이스 (기본: 0)
            web_gui: 웹 GUI 모듈 (선택적, 없으면 GUI 업데이트 안 함)
            logger: rclpy logger (None이면 print fallback). Keyword-only.
        
        연결:
            Red (Vin)       → 5V
            Black (GND)     → GND
            Green (Data in) → Pin 19 (SPI0 MOSI)
        """
        self.spi = spidev.SpiDev()
        self.spi.open(spi_bus, spi_device)
        self.spi.max_speed_hz = 8000000  # 8MHz (검증됨!)
        self.spi.mode = 0
        self.web_gui = web_gui  # 웹 GUI 참조 저장
        self._log = make_logger(logger)

        # 현재 색상
        self.current_r = 0
        self.current_g = 0
        self.current_b = 0
        
        # 패턴 제어
        self.pattern_thread = None
        self.pattern_running = False
        
        self._log('info', f"✅ RGB LED 초기화 완료 (SPI{spi_bus}.{spi_device})")
    
    def _send_color(self, r, g, b):
        """
        색상 전송 (검증된 방식)
        
        Args:
            r: 빨강 (0-255)
            g: 초록 (0-255)
            b: 파랑 (0-255)
        """
        # 리셋 신호
        self.spi.xfer2([0x00] * 10)
        
        # RGB 순서로 데이터 생성 (실제 테스트 결과)
        data = []
        
        for byte in [r, g, b]:  # RGB 순서!
            for i in range(7, -1, -1):
                if byte & (1 << i):
                    data.append(0b11110000)  # 1 비트
                else:
                    data.append(0b10000000)  # 0 비트
        
        # 데이터 전송
        self.spi.xfer2(data)
        
        # 리셋 신호
        self.spi.xfer2([0x00] * 10)
        
        time.sleep(0.001)  # 안정화
    
    def set_color(self, r, g, b, color_name=None, web_color=None):
        """
        LED 색상 설정
        
        Args:
            r: 빨강 (0-255)
            g: 초록 (0-255)
            b: 파랑 (0-255)
            color_name: 색상 이름 (웹 GUI용, 한글)
            web_color: 웹 GUI 색상 코드 ('green', 'orange', 'blue')
        """
        self.current_r = r
        self.current_g = g
        self.current_b = b
        self._send_color(r, g, b)
        
        # 웹 GUI 자동 업데이트
        if self.web_gui and color_name and web_color:
            try:
                self.web_gui.update_led(web_color, color_name)
            except:
                pass  # GUI 업데이트 실패해도 LED는 작동
    
    def turn_off(self):
        """LED 끄기"""
        self.set_color(0, 0, 0)
    
    def set_red(self, brightness=255):
        """빨간색 🔴"""
        self.set_color(brightness, 0, 0, color_name='빨간색', web_color='red')
    
    def set_green(self, brightness=255):
        """초록색 🟢"""
        self.set_color(0, brightness, 0, color_name='초록색', web_color='green')
    
    def set_blue(self, brightness=255):
        """파란색 🔵"""
        self.set_color(0, 0, brightness, color_name='파란색', web_color='blue')
    
    def set_yellow(self, brightness=255):
        """노란색 🟡"""
        self.set_color(brightness, brightness, 0, color_name='노란색', web_color='yellow')
    
    def set_cyan(self, brightness=255):
        """청록색 🔵"""
        self.set_color(0, brightness, brightness, color_name='청록색', web_color='cyan')
    
    def set_magenta(self, brightness=255):
        """자홍색 🟣"""
        self.set_color(brightness, 0, brightness, color_name='자홍색', web_color='magenta')
    
    def set_white(self, brightness=255):
        """흰색 ⚪"""
        self.set_color(brightness, brightness, brightness, color_name='흰색', web_color='white')
    
    def set_orange(self, brightness=255):
        """주황색 🟠"""
        # 주황색: 빨강 + 약간의 초록
        self.set_color(brightness, int(brightness * 0.5), 0, color_name='주황색', web_color='orange')

    def set_purple(self, brightness=255):
        """보라색 🟣"""
        self.set_color(int(brightness * 0.5), 0, brightness, color_name='보라색', web_color='purple')
    
    def blink(self, r, g, b, times=3, interval=0.5):
        """
        깜빡임
        
        Args:
            r, g, b: 색상
            times: 반복 횟수
            interval: 간격 (초)
        """
        for _ in range(times):
            self.set_color(r, g, b)
            time.sleep(interval)
            self.turn_off()
            time.sleep(interval)
    
    def fade(self, r, g, b, duration=2.0):
        """
        페이드 인/아웃
        
        Args:
            r, g, b: 목표 색상
            duration: 지속 시간 (초)
        """
        steps = 50
        
        # 페이드 인
        for i in range(steps + 1):
            factor = i / steps
            self.set_color(
                int(r * factor),
                int(g * factor),
                int(b * factor)
            )
            time.sleep(duration / (steps * 2))
        
        # 페이드 아웃
        for i in range(steps, -1, -1):
            factor = i / steps
            self.set_color(
                int(r * factor),
                int(g * factor),
                int(b * factor)
            )
            time.sleep(duration / (steps * 2))
    
    def start_blink_pattern(self, r, g, b, interval=0.5):
        """깜빡임 패턴 시작 (별도 스레드)"""
        self.stop_pattern()
        self.pattern_running = True
        
        def blink_loop():
            while self.pattern_running:
                self.set_color(r, g, b)
                time.sleep(interval)
                self.turn_off()
                time.sleep(interval)
        
        self.pattern_thread = threading.Thread(target=blink_loop, daemon=True)
        self.pattern_thread.start()
    
    def start_fade_pattern(self, r, g, b, duration=2.0):
        """페이드 패턴 시작 (별도 스레드)"""
        self.stop_pattern()
        self.pattern_running = True
        
        def fade_loop():
            while self.pattern_running:
                steps = 50
                # 페이드 인
                for i in range(steps + 1):
                    if not self.pattern_running:
                        break
                    factor = i / steps
                    self.set_color(
                        int(r * factor),
                        int(g * factor),
                        int(b * factor)
                    )
                    time.sleep(duration / (steps * 2))
                
                # 페이드 아웃
                for i in range(steps, -1, -1):
                    if not self.pattern_running:
                        break
                    factor = i / steps
                    self.set_color(
                        int(r * factor),
                        int(g * factor),
                        int(b * factor)
                    )
                    time.sleep(duration / (steps * 2))
        
        self.pattern_thread = threading.Thread(target=fade_loop, daemon=True)
        self.pattern_thread.start()
    
    def stop_pattern(self):
        """패턴 중지"""
        if self.pattern_running:
            self.pattern_running = False
            if self.pattern_thread:
                self.pattern_thread.join(timeout=1.0)
    
    def cleanup(self):
        """정리"""
        self.stop_pattern()
        self.turn_off()
        self.spi.close()
        self._log('info', "✅ RGB LED 정리 완료")


# 테스트 코드
if __name__ == '__main__':
    print("=" * 60)
    print("🔵 Blue Robotics RGB LED 테스트 (SPI0)")
    print("=" * 60)
    print()
    
    led = BlueRoboticsLED(spi_bus=0, spi_device=0)
    
    try:
        print("1️⃣  기본 색상 테스트")
        
        print("   🔴 빨간색")
        led.set_red()
        time.sleep(1.5)
        
        print("   🟢 초록색")
        led.set_green()
        time.sleep(1.5)
        
        print("   🔵 파란색")
        led.set_blue()
        time.sleep(1.5)
        
        print("   🟡 노란색")
        led.set_yellow()
        time.sleep(1.5)
        
        print("   ⚪ 흰색")
        led.set_white()
        time.sleep(1.5)
        
        print("\n2️⃣  밝기 테스트")
        for brightness in [255, 200, 150, 100, 50]:
            print(f"   밝기: {brightness}/255")
            led.set_white(brightness)
            time.sleep(0.8)
        
        print("\n3️⃣  깜빡임 테스트")
        print("   빨간색 깜빡임")
        led.blink(255, 0, 0, times=3, interval=0.3)
        
        print("\n4️⃣  페이드 테스트")
        print("   파란색 페이드")
        led.fade(0, 0, 255, duration=2.0)
        
        print("\n5️⃣  연속 패턴 테스트 (5초)")
        print("   노란색 깜빡임 패턴")
        led.start_blink_pattern(255, 255, 0, interval=0.3)
        time.sleep(5)
        led.stop_pattern()
        
        print("\n6️⃣  상태 표시 시뮬레이션")
        
        print("   ✅ 정상 (초록색)")
        led.set_green()
        time.sleep(2)
        
        print("   ⚠️  경고 (노란색 깜빡임)")
        led.start_blink_pattern(255, 255, 0, interval=0.5)
        time.sleep(3)
        led.stop_pattern()
        
        print("   🚨 위험 (빨간색 빠른 깜빡임)")
        led.start_blink_pattern(255, 0, 0, interval=0.15)
        time.sleep(3)
        led.stop_pattern()
        
        print("   💙 대기 (파란색 페이드)")
        led.start_fade_pattern(0, 0, 255, duration=1.5)
        time.sleep(5)
        led.stop_pattern()
        
        print("\n⚫ LED 끄기")
        led.turn_off()
        
        print("\n✅ 테스트 완료!")
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Ctrl+C 감지")
    except Exception as e:
        print(f"\n❌ 오류: {e}")
        import traceback
        traceback.print_exc()
    finally:
        led.cleanup()
        print("\n" + "=" * 60)
