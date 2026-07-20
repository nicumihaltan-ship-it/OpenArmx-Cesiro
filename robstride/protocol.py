"""RobStride private CAN protocol (CAN 2.0B extended frames).

29-bit ID layout, per RS04 User Manual section 4:

    bit28..24   communication type   (5 bits)
    bit23..8    "data area 2"        (16 bits)
    bit7..0     destination address  (8 bits)

Payload is always 8 bytes.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum

# --------------------------------------------------------------------------
# CAN id packing
# --------------------------------------------------------------------------

ID_MASK = 0x1FFFFFFF


def pack_id(comm_type: int, data2: int, dest: int) -> int:
    """Build a 29-bit extended CAN id."""
    return ((comm_type & 0x1F) << 24) | ((data2 & 0xFFFF) << 8) | (dest & 0xFF)


def unpack_id(can_id: int) -> tuple[int, int, int]:
    """Split a 29-bit extended CAN id into (comm_type, data2, dest)."""
    can_id &= ID_MASK
    return (can_id >> 24) & 0x1F, (can_id >> 8) & 0xFFFF, can_id & 0xFF


class CommType(IntEnum):
    GET_ID = 0
    MOTION_CONTROL = 1
    FEEDBACK = 2
    ENABLE = 3
    STOP = 4
    SET_ZERO = 6
    SET_CAN_ID = 7
    PARAM_READ = 17
    PARAM_WRITE = 18
    FAULT_FEEDBACK = 21
    SAVE = 22
    SET_BAUD = 23
    SET_ACTIVE_REPORT = 24
    SET_PROTOCOL = 25


class RunMode(IntEnum):
    """Values for parameter 0x7005 (run_mode)."""

    OPERATION = 0     # MIT-style 5-parameter motion control
    POSITION_PP = 1   # profile position
    VELOCITY = 2      # speed mode
    CURRENT = 3       # Iq current mode
    POSITION_CSP = 5  # cyclic synchronous position


class MotorMode(IntEnum):
    """Mode-status field carried in every feedback frame (bits 22..23)."""

    RESET = 0
    CALI = 1
    RUN = 2


DEFAULT_HOST_ID = 0xFD  # 253, the host id the RobStride tooling uses


# --------------------------------------------------------------------------
# Fixed-point helpers
# --------------------------------------------------------------------------


def float_to_uint(x: float, x_min: float, x_max: float, bits: int = 16) -> int:
    """Scale a float into an unsigned integer, matching the vendor's C helper."""
    span = x_max - x_min
    x = min(max(x, x_min), x_max)
    return int((x - x_min) * ((1 << bits) - 1) / span)


def uint_to_float(v: int, x_min: float, x_max: float, bits: int = 16) -> float:
    """Inverse of :func:`float_to_uint`."""
    span = x_max - x_min
    return v * span / ((1 << bits) - 1) + x_min


# --------------------------------------------------------------------------
# Frames
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Frame:
    """A CAN frame ready to hand to the bus layer."""

    can_id: int
    data: bytes

    @property
    def parts(self) -> tuple[int, int, int]:
        return unpack_id(self.can_id)


def _frame(comm_type: int, data2: int, dest: int, payload: bytes = b"") -> Frame:
    data = bytes(payload).ljust(8, b"\x00")[:8]
    return Frame(pack_id(comm_type, data2, dest), data)


# --- outgoing ------------------------------------------------------------


def get_device_id(motor_id: int, host_id: int = DEFAULT_HOST_ID) -> Frame:
    """Type 0 - probe a motor; reply carries its 64-bit MCU UID."""
    return _frame(CommType.GET_ID, host_id, motor_id)


def enable(motor_id: int, host_id: int = DEFAULT_HOST_ID) -> Frame:
    """Type 3 - enable the motor (enters Motor mode)."""
    return _frame(CommType.ENABLE, host_id, motor_id)


def stop(motor_id: int, clear_fault: bool = False,
         host_id: int = DEFAULT_HOST_ID) -> Frame:
    """Type 4 - stop the motor. ``clear_fault`` also clears latched faults."""
    payload = bytes([1 if clear_fault else 0])
    return _frame(CommType.STOP, host_id, motor_id, payload)


def read_version(motor_id: int, host_id: int = DEFAULT_HOST_ID) -> Frame:
    """Type 4 with the 0x00C4 magic - firmware version read."""
    return _frame(CommType.STOP, host_id, motor_id, bytes([0x00, 0xC4]))


def set_zero(motor_id: int, host_id: int = DEFAULT_HOST_ID) -> Frame:
    """Type 6 - set the current position as mechanical zero."""
    return _frame(CommType.SET_ZERO, host_id, motor_id, bytes([1]))


def set_can_id(motor_id: int, new_id: int,
               host_id: int = DEFAULT_HOST_ID) -> Frame:
    """Type 7 - change the motor's CAN id. Takes effect immediately."""
    data2 = (host_id & 0xFF) | ((new_id & 0xFF) << 8)
    return _frame(CommType.SET_CAN_ID, data2, motor_id)


def motion_control(motor_id: int, torque: float, position: float, velocity: float,
                   kp: float, kd: float, limits: "MotorLimits") -> Frame:
    """Type 1 - the 5-parameter operation-control command.

    Control law: ``t_ref = kd*(v_set - v_act) + kp*(p_set - p_act) + t_ff``
    """
    torque_raw = float_to_uint(torque, limits.t_min, limits.t_max)
    payload = struct.pack(
        ">HHHH",
        float_to_uint(position, limits.p_min, limits.p_max),
        float_to_uint(velocity, limits.v_min, limits.v_max),
        float_to_uint(kp, limits.kp_min, limits.kp_max),
        float_to_uint(kd, limits.kd_min, limits.kd_max),
    )
    return _frame(CommType.MOTION_CONTROL, torque_raw, motor_id, payload)


def param_read(motor_id: int, index: int, host_id: int = DEFAULT_HOST_ID) -> Frame:
    """Type 17 - read one parameter by index."""
    return _frame(CommType.PARAM_READ, host_id, motor_id,
                  struct.pack("<H", index) + b"\x00" * 6)


def param_write(motor_id: int, index: int, raw: bytes,
                host_id: int = DEFAULT_HOST_ID) -> Frame:
    """Type 18 - write one parameter. Volatile until a type-22 save."""
    payload = struct.pack("<H", index) + b"\x00\x00" + bytes(raw).ljust(4, b"\x00")[:4]
    return _frame(CommType.PARAM_WRITE, host_id, motor_id, payload)


def save_params(motor_id: int, host_id: int = DEFAULT_HOST_ID) -> Frame:
    """Type 22 - persist 0x20xx parameters to flash."""
    return _frame(CommType.SAVE, host_id, motor_id, bytes([1, 2, 3, 4, 5, 6, 7, 8]))


def set_baud(motor_id: int, code: int, host_id: int = DEFAULT_HOST_ID) -> Frame:
    """Type 23 - change CAN bitrate. 1=1M 2=500K 3=250K 4=125K. Needs a power cycle."""
    return _frame(CommType.SET_BAUD, host_id, motor_id,
                  bytes([1, 2, 3, 4, 5, 6, code & 0xFF, 0]))


def set_active_report(motor_id: int, enabled: bool,
                      host_id: int = DEFAULT_HOST_ID) -> Frame:
    """Type 24 - toggle unsolicited type-2 feedback (interval via 0x7026)."""
    return _frame(CommType.SET_ACTIVE_REPORT, host_id, motor_id,
                  bytes([1, 2, 3, 4, 5, 6, 1 if enabled else 0, 0]))


def set_protocol(motor_id: int, code: int, host_id: int = DEFAULT_HOST_ID) -> Frame:
    """Type 25 - 0=private 1=CANopen 2=MIT. Needs a power cycle."""
    return _frame(CommType.SET_PROTOCOL, host_id, motor_id,
                  bytes([1, 2, 3, 4, 5, 6, code & 0xFF, 0]))


# --------------------------------------------------------------------------
# Motor limits (per model, used for the fixed-point scaling above)
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class MotorLimits:
    name: str
    p_max: float
    v_max: float
    t_max: float
    kp_max: float = 5000.0
    kd_max: float = 100.0
    kp_min: float = 0.0
    kd_min: float = 0.0
    i_max: float = 90.0
    gear_ratio: float = 9.0
    verified: bool = False   # True only where a primary source confirms the numbers

    @property
    def p_min(self) -> float:
        return -self.p_max

    @property
    def v_min(self) -> float:
        return -self.v_max

    @property
    def t_min(self) -> float:
        return -self.t_max


# --------------------------------------------------------------------------
# Incoming frame decoding
# --------------------------------------------------------------------------


FAULT_BITS = {
    16: "undervoltage",
    17: "three-phase overcurrent",
    18: "overtemperature",
    19: "magnetic encoder fault",
    20: "stall / overload",
    21: "uncalibrated",
}

# Bits of the 0x3023 faultSta word (also the type-21 fault frame).
FAULT_STA_BITS = {
    0: "motor overtemperature (>145 C)",
    1: "driver chip fault",
    2: "undervoltage (<12 V)",
    3: "overvoltage (>60 V)",
    4: "B-phase current sampling overcurrent",
    5: "C-phase current sampling overcurrent",
    7: "encoder uncalibrated",
    8: "hardware identification fault",
    9: "position initialisation fault",
    14: "stall overload algorithm protection",
    16: "A-phase current sampling overcurrent",
}

WARN_STA_BITS = {
    0: "motor overtemperature warning (>135 C)",
}


@dataclass
class Feedback:
    """Decoded type-2 (or type-24 report) motor feedback frame."""

    motor_id: int
    host_id: int
    position: float          # rad, load side
    velocity: float          # rad/s, load side
    torque: float            # Nm
    temperature: float       # deg C
    mode: MotorMode
    faults: list[str]
    fault_bits: int


def decode_feedback(can_id: int, data: bytes, limits: MotorLimits) -> Feedback:
    _, data2, dest = unpack_id(can_id)
    motor_id = data2 & 0xFF
    fault_bits = (data2 >> 8) & 0x3F
    mode = MotorMode((data2 >> 14) & 0x03)

    pos_raw, vel_raw, tor_raw, temp_raw = struct.unpack(">HHHH", data[:8])
    faults = [name for bit, name in FAULT_BITS.items()
              if fault_bits & (1 << (bit - 16))]

    return Feedback(
        motor_id=motor_id,
        host_id=dest,
        position=uint_to_float(pos_raw, limits.p_min, limits.p_max),
        velocity=uint_to_float(vel_raw, limits.v_min, limits.v_max),
        torque=uint_to_float(tor_raw, limits.t_min, limits.t_max),
        temperature=temp_raw / 10.0,
        mode=mode,
        faults=faults,
        fault_bits=fault_bits,
    )


@dataclass
class ParamReply:
    motor_id: int
    index: int
    raw: bytes      # the 4 value bytes, little-endian
    ok: bool


def decode_param_reply(can_id: int, data: bytes) -> ParamReply:
    _, data2, _ = unpack_id(can_id)
    index = struct.unpack("<H", data[0:2])[0]
    return ParamReply(
        motor_id=data2 & 0xFF,
        index=index,
        raw=bytes(data[4:8]),
        ok=((data2 >> 8) & 0xFF) == 0,
    )


@dataclass
class DeviceInfo:
    motor_id: int
    uid: bytes      # 64-bit MCU unique identifier


def decode_device_id(can_id: int, data: bytes) -> DeviceInfo:
    _, data2, _ = unpack_id(can_id)
    return DeviceInfo(motor_id=data2 & 0xFF, uid=bytes(data[:8]))


@dataclass
class FaultReport:
    motor_id: int
    fault_value: int
    warn_value: int

    @property
    def faults(self) -> list[str]:
        return [n for b, n in FAULT_STA_BITS.items() if self.fault_value & (1 << b)]

    @property
    def warnings(self) -> list[str]:
        return [n for b, n in WARN_STA_BITS.items() if self.warn_value & (1 << b)]


def decode_fault_frame(can_id: int, data: bytes) -> FaultReport:
    _, data2, _ = unpack_id(can_id)
    fault, warn = struct.unpack("<II", data[:8])
    return FaultReport(motor_id=data2 & 0xFF, fault_value=fault, warn_value=warn)
