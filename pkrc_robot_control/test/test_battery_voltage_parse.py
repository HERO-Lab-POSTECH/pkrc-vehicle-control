"""Unit tests for `BatteryMonitor._parse_voltage_message`.

Covers the three branches of the parser:
1. Status 5 message (cmd byte 0x1B, ≥ 6 data bytes) → returns (vesc_id, voltage).
2. Other status messages (any cmd != 0x1B) → returns None.
3. Truncated data (length < 6) → returns None without raising.

No CAN bus needed — the parser operates on a constructed `can.Message`.
"""
import can

from pkrc_robot_control.sensors.battery import BatteryMonitor


def _msg(arbitration_id: int, data: bytes) -> can.Message:
    """Build a python-can Message for the parser to chew on."""
    return can.Message(
        arbitration_id=arbitration_id,
        data=data,
        is_extended_id=True,
    )


def _bm() -> BatteryMonitor:
    """Construct a BatteryMonitor without opening a CAN bus."""
    return BatteryMonitor(auto_init=False, logger=None)


def test_parse_status_5_returns_vesc_id_and_voltage():
    """Voltage = raw / 10.0; raw is big-endian uint16 at data[4:6]."""
    bm = _bm()
    # cmd 0x1B (status 5), vesc_id 0x01, voltage 16.5V → raw = 165 = 0x00A5
    arbitration_id = (0x1B << 8) | 0x01
    data = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0xA5, 0x00, 0x00])
    result = bm._parse_voltage_message(_msg(arbitration_id, data))
    assert result == (0x01, 16.5)


def test_parse_non_status_5_returns_none():
    """cmd ≠ 0x1B is not a voltage message — return None."""
    bm = _bm()
    # cmd 0x09 (status 1), valid 8-byte data
    arbitration_id = (0x09 << 8) | 0x01
    result = bm._parse_voltage_message(_msg(arbitration_id, b'\x00' * 8))
    assert result is None


def test_parse_truncated_data_returns_none():
    """data length < 6 → cannot decode — return None without raising."""
    bm = _bm()
    arbitration_id = (0x1B << 8) | 0x01
    result = bm._parse_voltage_message(_msg(arbitration_id, b'\x00' * 5))
    assert result is None
