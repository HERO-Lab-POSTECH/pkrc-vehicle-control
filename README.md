# pkrc-vehicle-control

HERO Lab POSTECH PKRC 로봇 운용 코드. ROS2 Humble (Ubuntu 22.04) on NVIDIA Jetson Orin NX.

## Packages

- `pkrc_robot_control` — 로봇 메인 컨트롤러 (HEROMainControl, CAN 4-thruster holonomic)
- `pkrc_imu_utils` — IMU 보정/모니터링 유틸리티
- `sonar_tilt_controller` — Dynamixel 기반 sonar tilt 제어

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
ros2 launch sonar_tilt_controller tilt_controller.launch.py
```

## Dependencies

`deploy/pkrc-vehicle.repos`로 명시 — `sensor_packages` (HERO-Lab-POSTECH), `microstrain_inertial` (LORD-MicroStrain).

## License

MIT
