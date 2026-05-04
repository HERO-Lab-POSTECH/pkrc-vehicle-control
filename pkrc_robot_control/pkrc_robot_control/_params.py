"""Centralized parameter declaration and access for HEROMainControl.

Single source of truth: ``PARAM_DEFAULTS`` lists every node parameter and
its default value. ``config/pkrc.yaml`` must mirror these values exactly
— ``test_params_default_match.py`` enforces the invariant.
"""
from typing import Any, Dict


# 52 parameters total. Each value is the bit-exact pre-D4 hardcoded value
# (audited from main.py + control/{hovering,pid_control}.py call sites).
PARAM_DEFAULTS: Dict[str, Any] = {
    # Battery thresholds (V) — 2
    'battery.low_voltage_threshold': 13.0,
    'battery.critical_voltage_threshold': 12.5,

    # Joystick scalars — 3
    'joystick.deadzone': 0.05,
    'joystick.max_current': 8.0,
    'joystick.joy_timeout': 0.2,

    # Mode-shared settings — 3
    'odom_timeout_sec': 0.5,
    'enable_yaw_control': True,
    'invert_yaw': True,

    # Hovering — Fast-LIO source (10)
    'hovering.fastlio.kp': 1.2,
    'hovering.fastlio.ki': 0.08,
    'hovering.fastlio.kd': 0.6,
    'hovering.fastlio.yaw_kp': 1.5,
    'hovering.fastlio.yaw_ki': 0.05,
    'hovering.fastlio.yaw_kd': 0.6,
    'hovering.fastlio.yaw_limit': 1.0,
    'hovering.fastlio.yaw_scale': 1.0,
    'hovering.fastlio.yaw_deadband_deg': 2.0,
    'hovering.fastlio.stabilize_duration': 1.5,

    # Hovering — Cartographer source (10)
    'hovering.cartographer.kp': 1.5,
    'hovering.cartographer.ki': 0.10,
    'hovering.cartographer.kd': 0.4,
    'hovering.cartographer.yaw_kp': 0.8,
    'hovering.cartographer.yaw_ki': 0.05,
    'hovering.cartographer.yaw_kd': 1.0,
    'hovering.cartographer.yaw_limit': 1.0,
    'hovering.cartographer.yaw_scale': 0.5,
    'hovering.cartographer.yaw_deadband_deg': 4.0,
    'hovering.cartographer.stabilize_duration': 1.5,

    # PID mode — Fast-LIO source (12)
    'pid.fastlio.kp': 1.2,
    'pid.fastlio.ki': 0.08,
    'pid.fastlio.kd': 0.6,
    'pid.fastlio.yaw_kp': 0.4,
    'pid.fastlio.yaw_ki': 0.01,
    'pid.fastlio.yaw_kd': 0.5,
    'pid.fastlio.yaw_limit': 0.7,
    'pid.fastlio.yaw_scale': 1.0,
    'pid.fastlio.yaw_deadband_deg': 4.0,
    'pid.fastlio.stabilize_duration': 1.0,
    'pid.fastlio.joystick_speed': 0.3,
    'pid.fastlio.joystick_yaw_speed_deg': 25.0,

    # PID mode — Cartographer source (12)
    'pid.cartographer.kp': 1.5,
    'pid.cartographer.ki': 0.10,
    'pid.cartographer.kd': 0.4,
    'pid.cartographer.yaw_kp': 0.3,
    'pid.cartographer.yaw_ki': 0.05,
    'pid.cartographer.yaw_kd': 0.5,
    'pid.cartographer.yaw_limit': 0.4,
    'pid.cartographer.yaw_scale': 0.3,
    'pid.cartographer.yaw_deadband_deg': 4.0,
    'pid.cartographer.stabilize_duration': 1.5,
    'pid.cartographer.joystick_speed': 0.3,
    'pid.cartographer.joystick_yaw_speed_deg': 25.0,
}


def declare_all(node) -> None:
    """Declare every PARAM_DEFAULTS entry on ``node``.

    Call once during node ``__init__``, before any ``load_*`` access.
    """
    for name, default in PARAM_DEFAULTS.items():
        node.declare_parameter(name, default)


def load_scalar(node, name: str) -> Any:
    """Read a single declared parameter."""
    return node.get_parameter(name).value


def load_pid_dict(node, prefix: str) -> Dict[str, Any]:
    """Load all parameters under ``prefix`` (e.g. 'hovering.fastlio') into
    a flat dict keyed by the suffix only — matches the legacy
    ``PARAMS_FASTLIO`` dict shape that hovering/pid_control consume.

    Maps ``yaw_deadband_deg`` -> ``yaw_deadband`` for backward compat with
    existing dict consumers.
    """
    prefix = prefix.rstrip('.')  # defensive: trailing-dot input would yield '.kp'-style keys
    raw = node.get_parameters_by_prefix(prefix)
    result: Dict[str, Any] = {}
    for full_key, param in raw.items():
        suffix = full_key  # get_parameters_by_prefix strips the prefix already
        if suffix == 'yaw_deadband_deg':
            result['yaw_deadband'] = param.value
        elif suffix == 'joystick_yaw_speed_deg':
            result['joystick_yaw_speed'] = param.value
        else:
            result[suffix] = param.value
    return result
