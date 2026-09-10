"""STS3215 control-table addresses and helpers.

Addresses come from the Feetech SMS/STS memory table. Only the registers this
project actually touches are listed; sizes are in bytes.
"""

from __future__ import annotations

from typing import NamedTuple

COUNTS_PER_REV = 4096
"""Encoder counts for a full turn (0..4095 spans 360 deg)."""


class Register(NamedTuple):
    address: int
    size: int
    name: str
    sign_bit: int | None = None
    """Bit carrying the sign, for sign-magnitude values.

    Feetech does not use two's complement, and the sign bit is not in the same
    place for every register: position and speed put it at bit 15, but load
    puts it at bit 10 and the homing offset at bit 11. Decoding load with bit 15
    silently turns a small negative load into a value near 1024.
    """


def decode_signed(raw: int, register: Register) -> int:
    """Decode a sign-magnitude register value."""
    if register.sign_bit is None:
        return raw
    flag = 1 << register.sign_bit
    return -(raw & (flag - 1)) if raw & flag else raw


def encode_signed(value: int, register: Register) -> int:
    """Encode an integer into a register's sign-magnitude representation."""
    if register.sign_bit is None:
        if value < 0:
            raise ValueError(f"{register.name} cannot hold a negative value")
        return value
    flag = 1 << register.sign_bit
    magnitude = abs(value)
    if magnitude >= flag:
        raise ValueError(
            f"{value} does not fit in {register.name} "
            f"(magnitude must be under {flag})"
        )
    return magnitude | flag if value < 0 else magnitude


CENTER_POSITION = COUNTS_PER_REV // 2
"""Mid-scale of the encoder (2048), i.e. the centre of a full 360 deg turn."""

FACTORY_MIN_ANGLE_LIMIT = 0
FACTORY_MAX_ANGLE_LIMIT = COUNTS_PER_REV - 1
FACTORY_OFFSET = 0


# -- EPROM (read-only) -------------------------------------------------------
MODEL = Register(3, 2, "model")
FIRMWARE_MAJOR = Register(0, 1, "firmware_major")

# -- EPROM (read/write) ------------------------------------------------------
ID = Register(5, 1, "id")
BAUD_RATE = Register(6, 1, "baud_rate")
MAX_VOLTAGE_LIMIT = Register(14, 1, "max_voltage_limit")
MIN_VOLTAGE_LIMIT = Register(15, 1, "min_voltage_limit")
MIN_ANGLE_LIMIT = Register(9, 2, "min_angle_limit")
MAX_ANGLE_LIMIT = Register(11, 2, "max_angle_limit")
OFFSET = Register(31, 2, "offset", sign_bit=11)
MODE = Register(33, 1, "mode")
MAX_TORQUE_LIMIT = Register(16, 2, "max_torque_limit")
PROTECTION_CURRENT = Register(28, 2, "protection_current")
OVERLOAD_TORQUE = Register(36, 1, "overload_torque")
P_COEFFICIENT = Register(21, 1, "p_coefficient")
D_COEFFICIENT = Register(22, 1, "d_coefficient")
I_COEFFICIENT = Register(23, 1, "i_coefficient")
LOCK = Register(55, 1, "lock")

# -- SRAM (read/write) -------------------------------------------------------
TORQUE_ENABLE = Register(40, 1, "torque_enable")
ACCELERATION = Register(41, 1, "acceleration")
GOAL_POSITION = Register(42, 2, "goal_position")
GOAL_TIME = Register(44, 2, "goal_time")
GOAL_SPEED = Register(46, 2, "goal_speed")

# -- SRAM (read-only) --------------------------------------------------------
PRESENT_POSITION = Register(56, 2, "present_position", sign_bit=15)
PRESENT_SPEED = Register(58, 2, "present_speed", sign_bit=15)
PRESENT_LOAD = Register(60, 2, "present_load", sign_bit=10)
PRESENT_VOLTAGE = Register(62, 1, "present_voltage")
PRESENT_TEMPERATURE = Register(63, 1, "present_temperature")
STATUS = Register(65, 1, "status")
MOVING = Register(66, 1, "moving")
PRESENT_CURRENT = Register(69, 2, "present_current")

MODEL_NUMBERS = {777: "sts3215", 2825: "sts3250"}

POSITION_MODE = 0
WHEEL_MODE = 1

# Bits of the STATUS register (address 65).
STATUS_BITS = (
    (0x01, "voltage"),
    (0x02, "angle sensor"),
    (0x04, "overheat"),
    (0x08, "overcurrent"),
    (0x20, "overload"),
)


def describe_status(flags: int) -> str:
    """Render the STATUS register as human-readable text."""
    if not flags:
        return "ok"
    names = [name for bit, name in STATUS_BITS if flags & bit]
    unknown = flags & ~sum(bit for bit, _ in STATUS_BITS)
    if unknown:
        names.append(f"unknown bits {unknown:#04x}")
    return ", ".join(names)


def counts_to_degrees(counts: int) -> float:
    return counts * 360.0 / COUNTS_PER_REV


def degrees_to_counts(degrees: float) -> int:
    return round(degrees * COUNTS_PER_REV / 360.0)


FACTORY_TORQUE_SETTINGS = (
    (MAX_TORQUE_LIMIT, 1000),
    (PROTECTION_CURRENT, 500),
    (OVERLOAD_TORQUE, 80),
)
"""Full-power settings, read off servos with unchanged factory configuration.

Previous applications may deliberately store lower torque, current and overload
limits -- for example 500 / 250 / 25 for a servo that must clamp gently. Those
values remain in EPROM when the servo is reused, potentially capping its torque
at 50% and, with it, how hard it can accelerate.
"""


FACTORY_GAINS = (
    (P_COEFFICIENT, 32),
    (D_COEFFICIENT, 32),
    (I_COEFFICIENT, 0),
)
"""Feetech's default position-loop gains.

Previous applications may lower P to 16 to reduce shakiness in a jointed
mechanism carrying a payload at the end of a long lever. Measured here over a
2000-count move, that costs 15%: 1.19s at P=16 against 1.01s at P=32, and the
lower gain also settles 2 counts short. Raising it further (48, 64) buys almost
nothing and only risks oscillation once a real load is on the platform.
"""

FACTORY_SETTINGS = FACTORY_TORQUE_SETTINGS + FACTORY_GAINS
"""Everything factory-reset restores besides travel: torque caps and gains."""
