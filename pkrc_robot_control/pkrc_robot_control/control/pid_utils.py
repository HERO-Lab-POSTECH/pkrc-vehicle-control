"""Shared PID utilities for control package.

`SimplePID`: PI(D) controller with anti-windup integral clamp.
`quaternion_to_yaw`: ROS geometry_msgs Quaternion → yaw radians.
`normalize_angle`: wrap angle into [-pi, pi].

Single source of truth — both `hovering` and `pid_control` import from here.
"""

import math
import time


class SimplePID:
    """간단한 PID 제어기"""

    def __init__(self, kp=1.0, ki=0.0, kd=0.5, output_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def reset(self):
        """PID 상태 초기화"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def compute(self, error, current_time=None):
        """PID 출력 계산"""
        if current_time is None:
            current_time = time.time()

        if self.prev_time is None:
            dt = 0.05  # 기본 20Hz
        else:
            dt = current_time - self.prev_time
            if dt <= 0:
                dt = 0.05

        # Proportional
        p = self.kp * error

        # Integral (anti-windup): 출력 포화 시 적분 정지
        if abs(p) < self.output_limit:
            self.integral += error * dt

        if self.ki > 0.001:
            max_integral = self.output_limit / self.ki
            self.integral = max(-max_integral, min(max_integral, self.integral))
        i = self.ki * self.integral

        # Derivative
        d = self.kd * (error - self.prev_error) / dt if dt > 0 else 0.0

        # Output (clamped)
        output = p + i + d
        output = max(-self.output_limit, min(self.output_limit, output))

        self.prev_error = error
        self.prev_time = current_time

        return output


def quaternion_to_yaw(q):
    """ROS Quaternion (geometry_msgs) → yaw 각도(rad)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """각도를 [-pi, pi]로 wrap."""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle
