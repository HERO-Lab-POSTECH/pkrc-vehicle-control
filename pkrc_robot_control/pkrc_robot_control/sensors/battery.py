#!/usr/bin/env python3
"""
배터리 전압 모니터링 모듈
- CAN 통신을 통한 VESC 배터리 전압 읽기
- 실시간 전압 모니터링
- 저전압 경고
"""

import can
import struct
import threading
import time
from typing import Optional, Dict, Callable

from .._log import make_logger


class BatteryMonitor:
    """배터리 전압 모니터링 클래스"""
    
    def __init__(self,
                 can_channel='can0',
                 bustype='socketcan',
                 low_voltage_threshold=20.0,
                 critical_voltage_threshold=18.0,
                 auto_init=True,
                 gui=None,
                 *,
                 logger=None):
        """
        배터리 모니터 초기화

        Args:
            can_channel: CAN 채널 이름 (기본: 'can0')
            bustype: CAN 버스 타입 (기본: 'socketcan')
            low_voltage_threshold: 저전압 경고 임계값 (V)
            critical_voltage_threshold: 위험 전압 임계값 (V)
            auto_init: 자동 초기화 여부
            gui: GUI 인터페이스 (NullGUI 또는 미래 통합 Qt GUI)
            logger: rclpy logger (None 이면 print fallback). Keyword-only.
        """
        self._log = make_logger(logger)
        self.can_channel = can_channel
        self.bustype = bustype
        self.low_voltage_threshold = low_voltage_threshold
        self.critical_voltage_threshold = critical_voltage_threshold
        self.gui = gui  # GUI 참조 저장
        
        # 전압 데이터 저장
        self.voltages: Dict[int, float] = {}  # {vesc_id: voltage}
        self.last_update_time: Dict[int, float] = {}  # {vesc_id: timestamp}
        
        # 상태
        self.bus: Optional[can.interface.Bus] = None
        self.is_monitoring = False
        self.monitor_thread: Optional[threading.Thread] = None
        
        # 콜백 함수
        self.voltage_callback: Optional[Callable] = None
        self.low_voltage_callback: Optional[Callable] = None
        self.critical_voltage_callback: Optional[Callable] = None
        
        # 락
        self.lock = threading.Lock()
        
        # 웹 GUI 업데이트 제한 (1분에 한 번)
        self.last_gui_update_time = 0.0
        self.gui_update_interval = 60.0  # 60초
        
        if auto_init:
            self.initialize()

    def initialize(self):
        """CAN 버스 초기화"""
        try:
            self.bus = can.interface.Bus(channel=self.can_channel, bustype=self.bustype)
            self._log('info', f'✅ 배터리 모니터 초기화 완료 (CAN: {self.can_channel})')
            return True
        except Exception as e:
            self._log('error', f'❌ 배터리 모니터 초기화 실패: {e}')
            self.bus = None
            return False
    
    def read_voltage_once(self, timeout=2.0) -> Optional[Dict[int, float]]:
        """
        배터리 전압 한 번 읽기
        
        Args:
            timeout: 타임아웃 (초)
            
        Returns:
            {vesc_id: voltage} 딕셔너리 또는 None
        """
        if not self.bus:
            self._log('error', '❌ CAN 버스가 초기화되지 않았습니다')
            return None

        try:
            start_time = time.time()
            voltages_read = {}
            
            while time.time() - start_time < timeout:
                msg = self.bus.recv(timeout=0.1)
                if msg:
                    voltage_data = self._parse_voltage_message(msg)
                    if voltage_data:
                        vesc_id, voltage = voltage_data
                        voltages_read[vesc_id] = voltage
                        
                        # 4개 VESC 모두 읽었으면 종료
                        if len(voltages_read) >= 4:
                            break
            
            if voltages_read:
                with self.lock:
                    self.voltages.update(voltages_read)
                    current_time = time.time()
                    for vesc_id in voltages_read.keys():
                        self.last_update_time[vesc_id] = current_time
                
                return voltages_read
            else:
                self._log('warn', '⚠️  타임아웃: 전압 데이터를 받지 못했습니다')
                return None

        except Exception as e:
            self._log('error', f'❌ 전압 읽기 오류: {e}')
            return None
    
    def _parse_voltage_message(self, msg: can.Message) -> Optional[tuple]:
        """
        CAN 메시지에서 전압 정보 파싱
        
        Args:
            msg: CAN 메시지
            
        Returns:
            (vesc_id, voltage) 튜플 또는 None
        """
        try:
            can_id = msg.arbitration_id
            cmd = (can_id >> 8) & 0xFF
            vesc_id = can_id & 0xFF
            
            # Status 5 (0x1B = 27) - 전압 정보 포함
            if cmd == 0x1B and len(msg.data) >= 6:
                # Bytes 4-5: Input voltage * 10 (Big Endian)
                voltage_raw = struct.unpack('>H', msg.data[4:6])[0]
                voltage = voltage_raw / 10.0
                
                return (vesc_id, voltage)
            
            return None
            
        except Exception as e:
            # 파싱 오류는 조용히 무시
            return None
    
    def start_monitoring(self, update_interval=1.0):
        """
        실시간 전압 모니터링 시작
        
        Args:
            update_interval: 업데이트 간격 (초)
        """
        if self.is_monitoring:
            self._log('warn', '⚠️  이미 모니터링 중입니다')
            return

        if not self.bus:
            self._log('error', '❌ CAN 버스가 초기화되지 않았습니다')
            return
        
        self.is_monitoring = True
        self.monitor_thread = threading.Thread(
            target=self._monitoring_loop,
            args=(update_interval,),
            daemon=True
        )
        self.monitor_thread.start()
        self._log('info', '✅ 배터리 전압 모니터링 시작')
    
    def _monitoring_loop(self, update_interval):
        """모니터링 루프"""
        while self.is_monitoring:
            try:
                # CAN 메시지 수신
                msg = self.bus.recv(timeout=0.1)
                if msg:
                    voltage_data = self._parse_voltage_message(msg)
                    if voltage_data:
                        vesc_id, voltage = voltage_data
                        
                        # 전압 업데이트
                        with self.lock:
                            self.voltages[vesc_id] = voltage
                            self.last_update_time[vesc_id] = time.time()
                        
                        # 콜백 호출
                        if self.voltage_callback:
                            self.voltage_callback(vesc_id, voltage)
                        
                        # 저전압 체크
                        self._check_voltage_warnings(vesc_id, voltage)
                    
            except Exception as e:
                self._log('warn', f'⚠️  모니터링 오류: {e}')
                time.sleep(0.1)
    
    def _check_voltage_warnings(self, vesc_id, voltage):
        """전압 경고 체크 및 웹 GUI 자동 업데이트 (1분에 한 번)"""
        # 평균 전압 및 퍼센테이지 계산
        avg_voltage = self.get_voltage()
        percentage = self.get_battery_percentage()
        
        # 배터리 상태 결정
        if voltage <= self.critical_voltage_threshold:
            status = 'critical'
            if self.critical_voltage_callback:
                self.critical_voltage_callback(vesc_id, voltage)
        elif voltage <= self.low_voltage_threshold:
            status = 'low'
            if self.low_voltage_callback:
                self.low_voltage_callback(vesc_id, voltage)
        else:
            status = 'good'
        
        # GUI 자동 업데이트 (1분에 한 번만)
        current_time = time.time()
        if self.gui and avg_voltage and percentage:
            # 1분이 지났거나 처음 업데이트거나 상태가 변경된 경우에만 업데이트
            time_since_last_update = current_time - self.last_gui_update_time
            should_update = (
                time_since_last_update >= self.gui_update_interval or  # 1분 경과
                self.last_gui_update_time == 0.0 or  # 첫 업데이트
                status in ['critical', 'low']  # 경고 상태는 즉시 업데이트
            )
            
            if should_update:
                try:
                    self.gui.update_battery(
                        voltage=avg_voltage,
                        percentage=percentage,
                        status=status
                    )
                    self.last_gui_update_time = current_time
                except:
                    pass  # GUI 업데이트 실패해도 모니터링은 계속
    
    def _print_voltage_status(self):
        """현재 전압 상태 출력"""
        with self.lock:
            if self.voltages:
                voltage_str = " | ".join([f"VESC{id}: {v:.2f}V" for id, v in sorted(self.voltages.items())])
                self._log('info', f'🔋 배터리: {voltage_str}')
    
    def stop_monitoring(self):
        """모니터링 중지"""
        if not self.is_monitoring:
            return
        
        self.is_monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=2.0)
        self._log('info', '✅ 배터리 전압 모니터링 중지')
    
    def get_voltage(self, vesc_id: Optional[int] = None) -> Optional[float]:
        """
        특정 VESC의 전압 가져오기
        
        Args:
            vesc_id: VESC ID (None이면 평균 전압 반환)
            
        Returns:
            전압 (V) 또는 None
        """
        with self.lock:
            if vesc_id is not None:
                return self.voltages.get(vesc_id)
            else:
                # 평균 전압 반환
                if self.voltages:
                    return sum(self.voltages.values()) / len(self.voltages)
                return None
    
    def get_all_voltages(self) -> Dict[int, float]:
        """모든 VESC의 전압 가져오기"""
        with self.lock:
            return self.voltages.copy()
    
    def get_min_voltage(self) -> Optional[float]:
        """최소 전압 가져오기"""
        with self.lock:
            if self.voltages:
                return min(self.voltages.values())
            return None
    
    def get_max_voltage(self) -> Optional[float]:
        """최대 전압 가져오기"""
        with self.lock:
            if self.voltages:
                return max(self.voltages.values())
            return None
    
    def is_voltage_low(self) -> bool:
        """저전압 상태 확인"""
        min_voltage = self.get_min_voltage()
        if min_voltage:
            return min_voltage <= self.low_voltage_threshold
        return False
    
    def is_voltage_critical(self) -> bool:
        """위험 전압 상태 확인"""
        min_voltage = self.get_min_voltage()
        if min_voltage:
            return min_voltage <= self.critical_voltage_threshold
        return False
    
    def voltage_to_percentage(self, voltage: float, v_max=16.8, v_min=12.0) -> float:
        """
        전압을 퍼센테이지로 변환
        
        Args:
            voltage: 전압 (V)
            v_max: 최대 전압 (100%, 기본: 16.8V)
            v_min: 최소 전압 (0%, 기본: 12.0V)
            
        Returns:
            배터리 잔량 (0.0 ~ 100.0%)
        """
        if voltage >= v_max:
            return 100.0
        elif voltage <= v_min:
            return 0.0
        else:
            percentage = ((voltage - v_min) / (v_max - v_min)) * 100.0
            return round(percentage, 1)
    
    def get_battery_percentage(self, vesc_id: Optional[int] = None, v_max=16.8, v_min=12.0) -> Optional[float]:
        """
        배터리 잔량 퍼센테이지 가져오기
        
        Args:
            vesc_id: VESC ID (None이면 평균 전압 기준)
            v_max: 최대 전압 (100%)
            v_min: 최소 전압 (0%)
            
        Returns:
            배터리 잔량 (%) 또는 None
        """
        voltage = self.get_voltage(vesc_id)
        if voltage:
            return self.voltage_to_percentage(voltage, v_max, v_min)
        return None
    
    def set_voltage_callback(self, callback: Callable):
        """전압 업데이트 콜백 설정"""
        self.voltage_callback = callback
    
    def set_low_voltage_callback(self, callback: Callable):
        """저전압 경고 콜백 설정"""
        self.low_voltage_callback = callback
    
    def set_critical_voltage_callback(self, callback: Callable):
        """위험 전압 경고 콜백 설정"""
        self.critical_voltage_callback = callback
    
    def cleanup(self):
        """정리"""
        self.stop_monitoring()
        if self.bus:
            self.bus.shutdown()
            self.bus = None
        self._log('info', '✅ 배터리 모니터 정리 완료')


# 편의 함수
def read_battery_voltage_simple(can_channel='can0', timeout=2.0) -> Optional[float]:
    """
    간단한 배터리 전압 읽기 (일회성)
    
    Args:
        can_channel: CAN 채널
        timeout: 타임아웃 (초)
        
    Returns:
        평균 전압 (V) 또는 None
    """
    monitor = BatteryMonitor(can_channel=can_channel, auto_init=True)
    voltages = monitor.read_voltage_once(timeout=timeout)
    monitor.cleanup()
    
    if voltages:
        avg_voltage = sum(voltages.values()) / len(voltages)
        return avg_voltage
    return None


# 테스트 코드
if __name__ == '__main__':
    print("=" * 60)
    print("🔋 배터리 전압 모니터 테스트")
    print("=" * 60)
    print()
    
    # 방법 1: 한 번만 읽기
    print("📊 방법 1: 한 번만 읽기")
    voltage = read_battery_voltage_simple()
    if voltage:
        print(f"✓ 평균 배터리 전압: {voltage:.2f}V")
    print()
    
    # 방법 2: 실시간 모니터링
    print("📊 방법 2: 실시간 모니터링 (10초)")
    monitor = BatteryMonitor(
        low_voltage_threshold=22.0,
        critical_voltage_threshold=20.0,
        auto_init=True
    )
    
    # 콜백 함수 설정
    def on_voltage_update(vesc_id, voltage):
        pass  # 조용히 업데이트
    
    def on_low_voltage(vesc_id, voltage):
        print(f"⚠️  저전압 감지: VESC {vesc_id} = {voltage:.2f}V")
    
    def on_critical_voltage(vesc_id, voltage):
        print(f"🚨 위험 전압 감지: VESC {vesc_id} = {voltage:.2f}V")
    
    monitor.set_voltage_callback(on_voltage_update)
    monitor.set_low_voltage_callback(on_low_voltage)
    monitor.set_critical_voltage_callback(on_critical_voltage)
    
    # 모니터링 시작
    monitor.start_monitoring(update_interval=2.0)
    
    try:
        time.sleep(10)
    except KeyboardInterrupt:
        print("\n⚠️  Ctrl+C 감지")
    finally:
        # 최종 상태 출력
        print()
        print("📊 최종 전압 상태:")
        all_voltages = monitor.get_all_voltages()
        for vesc_id, voltage in sorted(all_voltages.items()):
            print(f"  VESC {vesc_id}: {voltage:.2f}V")
        
        avg_voltage = monitor.get_voltage()
        min_voltage = monitor.get_min_voltage()
        max_voltage = monitor.get_max_voltage()
        
        if avg_voltage:
            print(f"\n평균: {avg_voltage:.2f}V | 최소: {min_voltage:.2f}V | 최대: {max_voltage:.2f}V")
        
        monitor.cleanup()
        print("\n✅ 테스트 완료")
