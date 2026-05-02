# QoS 현황 (2026-03-28)

## Oculus Sonar

| 토픽 | Reliability | Depth |
|------|-------------|-------|
| `/sensor/sonar/oculus/[model]/sonar` | 🟢 BEST_EFFORT | 10 |
| `/sensor/sonar/oculus/[model]/metadata` | 🟢 BEST_EFFORT | 10 |
| `/sensor/sonar/oculus/[model]/raw_data` | 🟢 BEST_EFFORT | 100 |
| `/sensor/sonar/oculus/[model]/image` | 🟢 BEST_EFFORT | 10 |
| `/sensor/sonar/oculus/[model]/fan_image` | 🟢 BEST_EFFORT | 10 |
| `/sensor/sonar/oculus/[model]/param/*` (9개) | 🟢 BEST_EFFORT | 10 |

---

## Livox LiDAR

| 토픽 | Reliability | Depth |
|------|-------------|-------|
| `/sensor/lidar/[name]/points` | 🟢 BEST_EFFORT | 32~256 |
| `/sensor/ins/[name]/imu` | 🟢 BEST_EFFORT | 32~256 |

> depth는 `multi_topic` 설정에 따라 동적 계산 (`kMinEthPacketQueueSize(32)` × 2 or × 8)
> FAST-LIO는 별도 컴퓨터에서 구독 (구독자도 BEST_EFFORT로 설정됨)
