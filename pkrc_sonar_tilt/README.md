# pkrc_sonar_tilt

Dynamixel XW540 모터를 이용한 소나 틸트 각도 제어 ROS2 패키지

## 실행

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch pkrc_sonar_tilt tilt_controller.launch.py
```

## 사용법 (새 터미널에서)

### 각도 이동
```bash
# 45도로 이동
ros2 topic pub /sonar/tilt/set_angle std_msgs/msg/Float32 "{data: 45.0}" --once

# 90도로 이동
ros2 topic pub /sonar/tilt/set_angle std_msgs/msg/Float32 "{data: 90.0}" --once

# 0도로 이동
ros2 topic pub /sonar/tilt/set_angle std_msgs/msg/Float32 "{data: 0.0}" --once
```

### 현재 각도 확인
```bash
ros2 topic echo /sonar/tilt/current_angle
```

### 토크 제어
```bash
# 토크 끄기 (손으로 돌릴 수 있음)
ros2 service call /sonar/tilt/set_torque std_srvs/srv/SetBool "{data: false}"

# 토크 켜기
ros2 service call /sonar/tilt/set_torque std_srvs/srv/SetBool "{data: true}"
```

## 토픽 목록

| 토픽 | 타입 | 방향 | 설명 |
|------|------|------|------|
| `/sonar/tilt/current_angle` | Float32 | pub | 현재 각도 (도) |
| `/sonar/tilt/goal_angle` | Float32 | pub | 목표 각도 (도) |
| `/sonar/tilt/is_moving` | Bool | pub | 이동 중 여부 |
| `/sonar/tilt/torque_enabled` | Bool | pub | 토크 상태 |
| `/sonar/tilt/set_angle` | Float32 | sub | 각도 설정 |

## 서비스

| 서비스 | 타입 | 설명 |
|--------|------|------|
| `/sonar/tilt/set_torque` | SetBool | 토크 ON/OFF |

## 설정 파라미터

`config/tilt_controller.yaml`:
- `device`: 시리얼 포트 (기본: `/dev/ttyUSB0`)
- `baudrate`: 통신 속도 (기본: `57600`)
- `motor_id`: 모터 ID (기본: `1`)
- `publish_rate`: 상태 퍼블리시 주기 (기본: `10.0` Hz)
- `profile_velocity`: 모터 속도 (기본: `50`)
- `profile_acceleration`: 모터 가속도 (기본: `100`)
- `auto_home`: 시작 시 센서 45°로 자동 이동 (기본: `false`)

## Launch 인자

```bash
# 기본 실행
ros2 launch pkrc_sonar_tilt tilt_controller.launch.py

# 시작 시 자동 홈잉 (센서 45°로 이동)
ros2 launch pkrc_sonar_tilt tilt_controller.launch.py auto_home:=true

# 다른 시리얼 포트 사용
ros2 launch pkrc_sonar_tilt tilt_controller.launch.py device:=/dev/u2d2
```

## udev 설정 (권장)

U2D2를 항상 `/dev/u2d2`로 인식하게 설정:
```bash
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014", SYMLINK+="u2d2", MODE="0666"' | sudo tee /etc/udev/rules.d/99-u2d2.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

설정 후 launch 파일에서 `device:=/dev/u2d2` 사용 가능.

## 하드웨어

- **모터**: Dynamixel XW540-T260
- **인터페이스**: U2D2 (USB-Serial)
- **프로토콜**: Dynamixel Protocol 2.0
- **Operating Mode**: 4 (Extended Position Control) — 한 바퀴 wrap 영향 없음
- **Drive Mode**: Reverse (물리적 회전 방향 보존)
- **베벨 기어비**: 2:1 (Z1=15T, Z2=30T)

## 진단

시작 시 INFO 로그에 현재 위치(sensor / motor / raw)와 허용 범위가 출력됩니다.
범위 밖에서 실행 시 `Position out of operating range: ...` 에러가 발생하며,
손으로 0~92° 범위 내로 돌리면 회복됩니다. 다중 회전 미끄러짐 같은 catastrophic
상황에서는 `Severe out-of-range: ...` 메시지가 표시되니 모터 전원을 재투입하세요.
