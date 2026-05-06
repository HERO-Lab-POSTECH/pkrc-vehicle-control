# pkrc-vehicle-control

HERO Lab POSTECH PKRC 로봇 운용 코드. ROS2 Humble (Ubuntu 22.04) on NVIDIA Jetson Orin NX.

## Packages

- `pkrc_robot_control` — 로봇 메인 컨트롤러 (HEROMainControl, CAN 4-thruster holonomic)
- `pkrc_imu_utils` — IMU 보정/모니터링 유틸리티
- `pkrc_sonar_tilt` — Dynamixel 기반 sonar tilt 제어

## Bootstrap (새 머신에서)

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/HERO-Lab-POSTECH/pkrc-vehicle-control.git
vcs import . < pkrc-vehicle-control/deploy/pkrc-vehicle.repos
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 run pkrc_robot_control main_control
ros2 launch pkrc_sonar_tilt tilt_controller.launch.py
```

## Dependencies

`deploy/pkrc-vehicle.repos`로 명시 — `sensor_packages` (HERO-Lab-POSTECH), `microstrain_inertial` (LORD-MicroStrain).

## 외부 모니터링 (Remote Qt dashboard)

같은 서브넷에 있는 외부 PC에서 ROS 2 sub만으로 로봇 상태 시각화 가능 (D6, 2026-05-06).

### 토픽 카탈로그

| 토픽 | 메시지 | 빈도 | 비고 |
|---|---|---|---|
| `/pkrc/system/state` | `std_msgs/Float32MultiArray` `[is_armed, sensitivity, lumen_brightness]` | 1 Hz heartbeat | `is_armed`은 0/1로 캐스팅된 bool |
| `/pkrc/battery/state` | `sensor_msgs/BatteryState` | 0.2 Hz (5s 측정 주기) | `voltage` (V), `percentage` (0~1 fraction) |
| `/pkrc/relays/state` | `std_msgs/UInt8` | 변경 즉시 + 1 Hz heartbeat | bit0=CH1, bit1=CH2, bit2=CH3 |
| `/pkrc/led/color` | `std_msgs/String` | 변경 즉시 + 1 Hz heartbeat | "green"/"orange"/"blue"/"red"/"off" |
| `/pkrc/motors/cmd_current` | `std_msgs/Float32MultiArray` (4) | 2 Hz | 4 VESC actual current (A) |
| `/camera/image/compressed` | `sensor_msgs/CompressedImage` | 15 Hz | 1280×720 JPEG |
| `/joy` | `sensor_msgs/Joy` | 50 Hz | 입력 |
| `/sonar/tilt/{current_angle,goal_angle,is_moving,torque_enabled}` | various | 10 Hz | 기존 |

### QoS

**모든 `/pkrc/*` 토픽은 `BEST_EFFORT` + `KEEP_LAST(10)`.** 외부 PC sub도 BEST_EFFORT로 걸어야 매칭됨:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy
qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
self.create_subscription(BatteryState, '/pkrc/battery/state', cb, qos)
```

DDS QoS 호환 함정 — Pub `BEST_EFFORT` × Sub `RELIABLE` 조합은 **조용히 매칭 실패** (에러 메시지 없음). `ros2 topic info <topic> -v`로 양쪽 reliability 일치 확인.

### 외부 PC 셋업

같은 서브넷 + ROS 2 Humble 가정. 양쪽 셸에서 동일 환경변수:

```bash
export ROS_DOMAIN_ID=123                    # Jetson과 동일 (~/.bashrc에 이미 설정됨)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # 또는 rmw_fastrtps_cpp — 양쪽 동일하게
```

검증:

```bash
# 외부 PC에서
ros2 topic list | grep pkrc
# 5개 보여야 정상: /pkrc/system/state, /pkrc/battery/state, /pkrc/relays/state, /pkrc/led/color, /pkrc/motors/cmd_current

ros2 topic hz /pkrc/system/state            # ~1.0
ros2 topic echo /pkrc/battery/state         # 메시지 흐름
ros2 topic info /pkrc/system/state -v       # QoS compatibility 확인
```

### 트러블슈팅

| 증상 | 원인 | 조치 |
|---|---|---|
| 토픽이 외부 PC에서 안 보임 | `ROS_DOMAIN_ID` 또는 `RMW_IMPLEMENTATION` 불일치 | 양쪽 환경변수 동일하게 export 후 셸 재시작 |
| Discovery는 되는데 echo가 멈춤 | sub의 QoS가 RELIABLE이라 매칭 실패 | sub에서 `ReliabilityPolicy.BEST_EFFORT` 사용 |
| 같은 서브넷인데도 안 보임 | 라우터/스위치가 multicast 차단 | `ROS_DISCOVERY_SERVER` fallback 또는 unicast peer list 설정 |
| 메시지가 가끔 끊김 | BEST_EFFORT 특성 (drop 허용) | 정상. heartbeat (1Hz)에서 자동 회복 |

## License

MIT
