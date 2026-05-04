# pkrc_robot_control

HERO PKRC vehicle main controller. 4-thruster CAN holonomic with joystick + IMU + odometry + camera + battery monitor.

## Entry points

```bash
ros2 run pkrc_robot_control main_control     # or alias `maincon`
```

The node is `HEROMainControl(VESCControlNode)`, defined in `pkrc_robot_control/main.py`. Update rate is 20 Hz.

## Topics

**Subscribed:**
- `/joy` — Xbox-mapped joystick (`sensor_msgs/Joy`)
- `/fast_lio/odometry` — Fast-LIO odometry (preferred for hovering / PID modes)
- `/cartographer_2d/odometry` — Cartographer odometry (fallback when Fast-LIO times out)

**Published:**
- `/camera/image/compressed` — USB camera compressed stream
- `/camera/image_raw` — USB camera raw image
- `/hero/vesc_currents` — VESC actual output currents (4 motors, `Float32MultiArray`)

(Sonar tilt — `/sonar/tilt/set_angle` etc. — lives in the sibling `pkrc_sonar_tilt` package.)

## Hardware

| Component | Interface | Notes |
|---|---|---|
| 4× VESC thrusters | CAN (`can0`) | IDs `0x101`–`0x104`. Init failure → degraded mode (no commands sent). |
| 3× relays | Jetson.GPIO (CH1) + sysfs/devmem (CH2, CH3) | CH2/CH3 require sudo NOPASSWD (busybox devmem + gpioset). |
| Lumen light | PWM pin 32 | Brightness 0.0–1.0; pulse range 1100–1900 µs. |
| RGB LED | SPI0 (MOSI = pin 19) | WS2812B; orange = disarmed, green = armed, blue = shutdown. |
| Battery | CAN (read from VESC status messages) | Polled every 5 s; warns at 13.0 V, critical at 12.5 V. |

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select pkrc_robot_control --symlink-install
source install/setup.bash
```

For first-time setup (vendored sub-repos), see top-level `README.md`.

## Parameters & launch

The node has 52 declarable ROS 2 parameters covering battery thresholds, joystick scalars, mode-shared toggles, and PID tuning gains for hovering and PID modes (split per odometry source). Defaults are baked into the source via `_params.PARAM_DEFAULTS`; `config/pkrc.yaml` ships the same values for operator visibility.

**Default operation (no tuning):**

```bash
ros2 run pkrc_robot_control main_control     # or `maincon`
```

**Tuned operation:**

```bash
# Use bundled yaml as a starting point:
cp ~/ros2_ws/install/pkrc_robot_control/share/pkrc_robot_control/config/pkrc.yaml ~/my_tuned.yaml
# Edit ~/my_tuned.yaml, then:
ros2 launch pkrc_robot_control pkrc_main.launch.py params_file:=~/my_tuned.yaml
```

**Parameter groups (counts in parentheses):**

| Prefix | Count | Examples |
|---|--:|---|
| `battery.*` | 2 | `low_voltage_threshold`, `critical_voltage_threshold` |
| `joystick.*` | 3 | `deadzone`, `max_current`, `joy_timeout` |
| (mode-shared) | 3 | `odom_timeout_sec`, `enable_yaw_control`, `invert_yaw` |
| `hovering.fastlio.*` | 10 | `kp`, `ki`, `kd`, `yaw_kp`, … |
| `hovering.cartographer.*` | 10 | (same keys, Cartographer-tuned values) |
| `pid.fastlio.*` | 12 | hovering keys + `joystick_speed`, `joystick_yaw_speed_deg` |
| `pid.cartographer.*` | 12 | (same keys, Cartographer-tuned values) |

The `test_params_default_match.py` test enforces that `config/pkrc.yaml` and `_params.PARAM_DEFAULTS` agree bit-exact.

## Quick troubleshooting

- **`CAN bus 초기화 실패`** — `can0` interface is down. Bring up with `sudo ip link set can0 up type can bitrate 500000`. The node tolerates this and starts in degraded mode.
- **Lumen init fails** — `pigpio` daemon not running, or pin 32 already exported. The node tolerates this and continues without lights.
- **Relay CH2/CH3 init fails** — sudo NOPASSWD not configured. The node tolerates this; CH1 still works.
- **`ROS_DOMAIN_ID` mismatch** — workstation is set to `123` in `~/.bashrc`. Don't override casually; the running robot uses it.

## Related packages

- `pkrc_imu_utils` — IMU calibration / RPY monitor scripts
- `pkrc_sonar_tilt` — Dynamixel sonar tilt (separate ROS 2 package, separate launch)
