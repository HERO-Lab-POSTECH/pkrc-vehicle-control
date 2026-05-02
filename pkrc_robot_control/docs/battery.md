# 🔋 배터리 전압 모니터링 모듈

`battery_module.py`는 CAN 통신을 통해 VESC의 배터리 전압을 모니터링하는 모듈입니다.

## 📋 주요 기능

- ✅ CAN 통신을 통한 VESC 배터리 전압 읽기
- ✅ 실시간 전압 모니터링
- ✅ 저전압/위험 전압 경고
- ✅ 콜백 함수 지원
- ✅ 스레드 안전
- ✅ 간단한 API

## 🚀 사용 방법

### 방법 1: 한 번만 전압 읽기 (간단)

```python
from battery_module import read_battery_voltage_simple

# 배터리 전압 읽기
voltage = read_battery_voltage_simple()
if voltage:
    print(f"배터리 전압: {voltage:.2f}V")
```

### 방법 2: BatteryMonitor 클래스 사용 (고급)

```python
from battery_module import BatteryMonitor

# 모니터 초기화
monitor = BatteryMonitor(
    can_channel='can0',
    low_voltage_threshold=22.0,      # 저전압 경고 임계값
    critical_voltage_threshold=20.0,  # 위험 전압 임계값
    auto_init=True
)

# 한 번만 읽기
voltages = monitor.read_voltage_once(timeout=2.0)
if voltages:
    for vesc_id, voltage in voltages.items():
        print(f"VESC {vesc_id}: {voltage:.2f}V")

# 정리
monitor.cleanup()
```

### 방법 3: 실시간 모니터링

```python
from battery_module import BatteryMonitor
import time

# 모니터 초기화
monitor = BatteryMonitor(
    low_voltage_threshold=22.0,
    critical_voltage_threshold=20.0,
    auto_init=True
)

# 콜백 함수 정의
def on_voltage_update(vesc_id, voltage):
    print(f"VESC {vesc_id}: {voltage:.2f}V")

def on_low_voltage(vesc_id, voltage):
    print(f"⚠️  저전압: VESC {vesc_id} = {voltage:.2f}V")

def on_critical_voltage(vesc_id, voltage):
    print(f"🚨 위험: VESC {vesc_id} = {voltage:.2f}V")

# 콜백 설정
monitor.set_voltage_callback(on_voltage_update)
monitor.set_low_voltage_callback(on_low_voltage)
monitor.set_critical_voltage_callback(on_critical_voltage)

# 모니터링 시작 (2초마다 상태 출력)
monitor.start_monitoring(update_interval=2.0)

try:
    # 계속 실행
    while True:
        time.sleep(1)
        
        # 현재 전압 확인
        avg_voltage = monitor.get_voltage()
        min_voltage = monitor.get_min_voltage()
        max_voltage = monitor.get_max_voltage()
        
        if avg_voltage:
            print(f"평균: {avg_voltage:.2f}V | 최소: {min_voltage:.2f}V | 최대: {max_voltage:.2f}V")
        
        # 저전압 체크
        if monitor.is_voltage_low():
            print("⚠️  저전압 상태!")
        if monitor.is_voltage_critical():
            print("🚨 위험 전압 상태!")

except KeyboardInterrupt:
    print("종료 중...")
finally:
    monitor.cleanup()
```

### 방법 4: ROS2 노드에서 사용

```python
import rclpy
from rclpy.node import Node
from battery_module import BatteryMonitor

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot_node')
        
        # 배터리 모니터 초기화
        self.battery_monitor = BatteryMonitor(
            low_voltage_threshold=22.0,
            critical_voltage_threshold=20.0,
            auto_init=True
        )
        
        # 콜백 설정
        self.battery_monitor.set_low_voltage_callback(self.on_low_voltage)
        self.battery_monitor.set_critical_voltage_callback(self.on_critical_voltage)
        
        # 모니터링 시작
        self.battery_monitor.start_monitoring(update_interval=5.0)
        
        # 주기적으로 전압 체크 (10초마다)
        self.create_timer(10.0, self.check_battery)
        
        self.get_logger().info('로봇 노드 시작 (배터리 모니터링 활성화)')
    
    def check_battery(self):
        """배터리 상태 체크"""
        avg_voltage = self.battery_monitor.get_voltage()
        if avg_voltage:
            self.get_logger().info(f'배터리 전압: {avg_voltage:.2f}V')
    
    def on_low_voltage(self, vesc_id, voltage):
        """저전압 경고"""
        self.get_logger().warn(f'저전압 경고: VESC {vesc_id} = {voltage:.2f}V')
    
    def on_critical_voltage(self, vesc_id, voltage):
        """위험 전압 경고"""
        self.get_logger().error(f'위험 전압: VESC {vesc_id} = {voltage:.2f}V')
        # 긴급 정지 등의 조치 수행
    
    def destroy_node(self):
        """노드 정리"""
        if hasattr(self, 'battery_monitor'):
            self.battery_monitor.cleanup()
        super().destroy_node()
```

## 📚 API 레퍼런스

### BatteryMonitor 클래스

#### 초기화
```python
BatteryMonitor(
    can_channel='can0',              # CAN 채널
    bustype='socketcan',             # CAN 버스 타입
    low_voltage_threshold=20.0,      # 저전압 임계값 (V)
    critical_voltage_threshold=18.0, # 위험 전압 임계값 (V)
    auto_init=True                   # 자동 초기화
)
```

#### 주요 메서드

| 메서드 | 설명 | 반환값 |
|--------|------|--------|
| `initialize()` | CAN 버스 초기화 | `bool` |
| `read_voltage_once(timeout=2.0)` | 전압 한 번 읽기 | `Dict[int, float]` |
| `start_monitoring(update_interval=1.0)` | 실시간 모니터링 시작 | `None` |
| `stop_monitoring()` | 모니터링 중지 | `None` |
| `get_voltage(vesc_id=None)` | 전압 가져오기 (None이면 평균) | `float` |
| `get_all_voltages()` | 모든 전압 가져오기 | `Dict[int, float]` |
| `get_min_voltage()` | 최소 전압 | `float` |
| `get_max_voltage()` | 최대 전압 | `float` |
| `is_voltage_low()` | 저전압 상태 확인 | `bool` |
| `is_voltage_critical()` | 위험 전압 상태 확인 | `bool` |
| `set_voltage_callback(callback)` | 전압 업데이트 콜백 설정 | `None` |
| `set_low_voltage_callback(callback)` | 저전압 경고 콜백 설정 | `None` |
| `set_critical_voltage_callback(callback)` | 위험 전압 경고 콜백 설정 | `None` |
| `cleanup()` | 정리 | `None` |

#### 콜백 함수 시그니처

```python
def voltage_callback(vesc_id: int, voltage: float):
    """전압 업데이트 콜백"""
    pass

def low_voltage_callback(vesc_id: int, voltage: float):
    """저전압 경고 콜백"""
    pass

def critical_voltage_callback(vesc_id: int, voltage: float):
    """위험 전압 경고 콜백"""
    pass
```

## 🧪 테스트

```bash
# 모듈 테스트
cd /home/hero/ros2_ws/hero_ws/control
python3 battery_module.py
```

## ⚙️ 설정

### 전압 임계값 권장 사항

| 배터리 타입 | 저전압 임계값 | 위험 전압 임계값 |
|------------|--------------|-----------------|
| 6S LiPo (22.2V) | 21.0V | 19.8V |
| 5S LiPo (18.5V) | 17.5V | 16.5V |
| 4S LiPo (14.8V) | 14.0V | 13.2V |

**참고**: LiPo 배터리는 셀당 3.5V 이하로 방전하면 손상될 수 있습니다.

## 🔧 문제 해결

### CAN 버스를 찾을 수 없음
```bash
# CAN 인터페이스 확인
ip link show can0

# CAN 인터페이스 활성화
sudo ip link set can0 up type can bitrate 500000
```

### 전압 데이터를 받지 못함
- VESC가 켜져 있는지 확인
- CAN 케이블 연결 확인
- VESC CAN ID 설정 확인 (1, 2, 3, 4)

## 📝 예제

전체 예제는 `battery_module.py`의 `__main__` 섹션을 참고하세요.

## 🔗 관련 모듈

- `can_control_module.py` - VESC CAN 제어
- `relay_control_module.py` - 릴레이 제어
- `lumen_module.py` - Lumen 라이트 제어
