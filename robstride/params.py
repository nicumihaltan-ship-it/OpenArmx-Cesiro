"""The RobStride parameter table.

Two index spaces are reachable through the same type-17/type-18 read/write
frames:

* ``0x0000``-``0x30xx`` - the "function code" table shown by RobStride Studio.
  ``0x20xx`` entries are stored in flash (a type-22 save makes writes stick),
  ``0x30xx`` entries are read-only observation values.
* ``0x70xx`` - the runtime control parameters documented in manual 4.1.13.

Source: RS04 User Manual 260713, sections 3.3.4 and 4.1.13.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import Enum


class Access(str, Enum):
    RO = "R"
    RW = "R/W"
    CFG = "config"


class Group(str, Enum):
    IDENTITY = "Identity"
    VERSION = "Version"
    CONFIG = "Config (flash)"
    OBSERVE = "Observation (live)"
    CONTROL = "Control (runtime)"


_STRUCT = {
    "uint8": "<B",
    "int8": "<b",
    "uint16": "<H",
    "int16": "<h",
    "uint32": "<I",
    "int32": "<i",
    "float": "<f",
}


@dataclass(frozen=True)
class Param:
    index: int
    name: str
    dtype: str
    access: Access
    group: Group
    unit: str = ""
    scale: float = 1.0        # engineering value = raw * scale
    minimum: float | None = None
    maximum: float | None = None
    note: str = ""

    @property
    def is_string(self) -> bool:
        return self.dtype == "string"

    @property
    def writable(self) -> bool:
        return self.access is not Access.RO

    def decode(self, raw: bytes):
        """Decode the 4 payload bytes of a type-17 reply."""
        if self.is_string:
            return raw.split(b"\x00")[0].decode("ascii", "replace")
        fmt = _STRUCT[self.dtype]
        width = struct.calcsize(fmt)
        value = struct.unpack(fmt, bytes(raw[:width]).ljust(width, b"\x00"))[0]
        return value * self.scale if self.scale != 1.0 else value

    def encode(self, value) -> bytes:
        """Encode an engineering value into the 4 payload bytes of a type-18 write."""
        if self.is_string:
            return str(value).encode("ascii", "replace").ljust(4, b"\x00")[:4]
        fmt = _STRUCT[self.dtype]
        if self.dtype == "float":
            native = float(value) / self.scale
        else:
            native = int(round(float(value) / self.scale))
        return struct.pack(fmt, native).ljust(4, b"\x00")[:4]


def _p(index, name, dtype, access, group, unit="", scale=1.0,
       minimum=None, maximum=None, note=""):
    return Param(index, name, dtype, access, group, unit, scale,
                 minimum, maximum, note)


RO, RW, CFG = Access.RO, Access.RW, Access.CFG
IDENT, VER, CONF, OBS, CTRL = (Group.IDENTITY, Group.VERSION, Group.CONFIG,
                               Group.OBSERVE, Group.CONTROL)

PARAMS: list[Param] = [
    # -- identity ---------------------------------------------------------
    _p(0x0000, "Name", "string", RW, IDENT, note="User label"),
    _p(0x0001, "BarCode", "string", RW, IDENT, note="Serial / barcode"),

    # -- firmware version -------------------------------------------------
    _p(0x1000, "BootCodeVersion", "string", RO, VER),
    _p(0x1001, "BootBuildDate", "string", RO, VER),
    _p(0x1002, "BootBuildTime", "string", RO, VER),
    _p(0x1003, "AppCodeVersion", "string", RO, VER, note="Motor firmware version"),
    _p(0x1004, "AppGitVersion", "string", RO, VER),
    _p(0x1005, "AppBuildDate", "string", RO, VER),
    _p(0x1006, "AppBuildTime", "string", RO, VER),
    _p(0x1007, "AppCodeName", "string", RO, VER),

    # -- stored configuration (0x20xx, needs type-22 save) ----------------
    _p(0x2000, "echoPara1", "uint16", CFG, CONF, minimum=5, maximum=115),
    _p(0x2001, "echoPara2", "uint16", CFG, CONF, minimum=5, maximum=115),
    _p(0x2002, "echoPara3", "uint16", CFG, CONF, minimum=5, maximum=115),
    _p(0x2003, "echoPara4", "uint16", CFG, CONF, minimum=5, maximum=115),
    _p(0x2004, "echoFreHz", "uint32", RW, CONF, "Hz", minimum=1, maximum=10000),
    _p(0x2005, "MechOffset", "float", RW, CONF, "rad", minimum=-50, maximum=50,
       note="Magnetic encoder angle offset"),
    _p(0x2006, "chasu_offset", "float", RW, CONF, "rad", minimum=-50, maximum=50,
       note="Reserved"),
    _p(0x2007, "Status1", "float", CFG, CONF, "Nm", minimum=-10, maximum=10,
       note="Torque limitation - do not change"),
    _p(0x2008, "I_FW_MAX", "float", RW, CONF, "A", minimum=0, maximum=33,
       note="Field-weakening current, default 0"),
    _p(0x2009, "CAN_ID", "uint8", RW, CONF, minimum=0, maximum=127,
       note="This node's CAN id"),
    _p(0x200A, "CAN_MASTER", "uint8", RW, CONF, minimum=0, maximum=300,
       note="Host CAN id"),
    _p(0x200B, "CAN_TIMEOUT", "uint32", RW, CONF, minimum=0, maximum=100000,
       note="Watchdog; 20000 = 1 s, 0 disables"),
    _p(0x200C, "status2", "int16", CFG, CONF, minimum=-200, maximum=1500,
       note="Reserved"),
    _p(0x200D, "status3", "uint32", CFG, CONF, minimum=0, maximum=100000,
       note="Reserved"),
    _p(0x200E, "status4", "float", CFG, CONF, minimum=1, maximum=64, note="Reserved"),
    _p(0x200F, "status5", "float", CFG, CONF, minimum=0, maximum=20, note="Reserved"),
    _p(0x2010, "status6", "uint8", CFG, CONF, minimum=0, maximum=1, note="Reserved"),
    _p(0x2011, "cur_filt_gain", "float", RW, CONF, minimum=0, maximum=1,
       note="Current filter"),
    _p(0x2012, "cur_kp", "float", RW, CONF, minimum=0, maximum=200),
    _p(0x2013, "cur_ki", "float", RW, CONF, minimum=0, maximum=200),
    _p(0x2014, "spd_kp", "float", RW, CONF, minimum=0, maximum=200),
    _p(0x2015, "spd_ki", "float", RW, CONF, minimum=0, maximum=200),
    _p(0x2016, "loc_kp", "float", RW, CONF, minimum=0, maximum=200),
    _p(0x2017, "spd_filt_gain", "float", RW, CONF, minimum=0, maximum=1),
    _p(0x2018, "limit_spd", "float", RW, CONF, "rad/s", minimum=0, maximum=200,
       note="Position mode speed limit"),
    _p(0x2019, "limit_cur", "float", RW, CONF, "A", minimum=0, maximum=23,
       note="Position/velocity mode current limit"),
    _p(0x201A, "spd_step_value", "float", RW, CONF, minimum=0, maximum=100,
       note="Reserved"),
    _p(0x201B, "vel_max", "float", RW, CONF, "rad/s", minimum=0, maximum=100,
       note="Reserved"),
    _p(0x201C, "acc_set", "float", RW, CONF, "rad/s^2", minimum=0, maximum=27,
       note="High speed segment offset"),
    _p(0x201D, "cfg_fault1", "uint32", CFG, CONF, note="Reserved"),
    _p(0x201E, "cfg_fault2", "uint32", CFG, CONF, note="Reserved"),
    _p(0x201F, "cfg_fault3", "uint32", CFG, CONF, note="Reserved"),
    _p(0x2020, "cfg_fault4", "uint32", CFG, CONF, note="Reserved"),
    _p(0x2021, "cfg_fault5", "uint32", CFG, CONF, note="Reserved"),
    _p(0x2022, "cfg_fault6", "uint32", CFG, CONF, note="Reserved"),
    _p(0x2023, "cfg_fault7", "uint32", CFG, CONF, note="Reserved"),
    _p(0x2024, "baud", "uint8", RW, CONF, note="Baud rate flag"),
    _p(0x2025, "zero_sta", "uint8", RW, CONF,
       note="0 = 0..2pi range, 1 = -pi..pi range"),
    _p(0x2026, "position_offset", "float", RW, CONF, "rad"),
    _p(0x2027, "protocol_1", "uint8", RW, CONF,
       note="0 = private, 1 = CANopen, 2 = MIT"),
    _p(0x2028, "damper", "uint8", RW, CONF,
       note="1 disables post-power-off anti-backdrive damping"),
    _p(0x2029, "add_offset", "float", RW, CONF, "rad", note="Zero point offset"),

    # -- live observation values (0x30xx, read-only) ----------------------
    _p(0x3000, "timeUse0", "uint16", RO, OBS),
    _p(0x3001, "timeUse1", "uint16", RO, OBS),
    _p(0x3002, "timeUse2", "uint16", RO, OBS),
    _p(0x3003, "timeUse3", "uint16", RO, OBS),
    _p(0x3004, "encoderRaw", "uint16", RO, OBS, note="Magnetic encoder sample"),
    _p(0x3005, "mcuTemp", "int16", RO, OBS, "C", scale=0.1),
    _p(0x3006, "motorTemp", "int16", RO, OBS, "C", scale=0.1, note="Motor NTC"),
    _p(0x3007, "encoder2raw", "uint16", RO, OBS),
    _p(0x3008, "adc1Offset", "int32", RO, OBS),
    _p(0x3009, "adc2Offset", "int32", RO, OBS),
    _p(0x300A, "adc1Raw", "uint16", RO, OBS),
    _p(0x300B, "adc2Raw", "uint16", RO, OBS),
    _p(0x300C, "VBUS", "float", RO, OBS, "V", note="Bus voltage"),
    _p(0x300D, "cmdId", "float", RO, OBS, "A", note="Id loop command"),
    _p(0x300E, "cmdIq", "float", RO, OBS, "A", note="Iq loop command"),
    _p(0x300F, "cmdlocref", "float", RO, OBS, "rad", note="Position loop command"),
    _p(0x3010, "cmdspdref", "float", RO, OBS, "rad/s", note="Speed loop command"),
    _p(0x3012, "cmdTorque", "float", RO, OBS, "Nm"),
    _p(0x3013, "cmdPos", "float", RO, OBS, "rad", note="MIT protocol angle command"),
    _p(0x3014, "cmdVel", "float", RO, OBS, "rad/s", note="MIT protocol speed command"),
    _p(0x3015, "rotation", "int16", RO, OBS, note="Turn count"),
    _p(0x3016, "modPos", "float", RO, OBS, "rad", note="Uncounted mechanical angle"),
    _p(0x3017, "mechPos", "float", RO, OBS, "rad", note="Load-side mechanical angle"),
    _p(0x3018, "mechVel", "float", RO, OBS, "rad/s", note="Load-side speed"),
    _p(0x3019, "elecPos", "float", RO, OBS, "rad", note="Electrical angle"),
    _p(0x301A, "ia", "float", RO, OBS, "A", note="U phase current"),
    _p(0x301B, "ib", "float", RO, OBS, "A", note="V phase current"),
    _p(0x301C, "ic", "float", RO, OBS, "A", note="W phase current"),
    _p(0x301D, "timeout", "uint32", RO, OBS, note="Timeout counter"),
    _p(0x301E, "phaseOrder", "uint8", RO, OBS, note="Direction marking"),
    _p(0x301F, "iqf", "float", RO, OBS, "A", note="Filtered Iq"),
    _p(0x3020, "boardTemp", "int16", RO, OBS, "C", scale=0.1),
    _p(0x3021, "iq", "float", RO, OBS, "A", note="Raw Iq"),
    _p(0x3022, "id", "float", RO, OBS, "A", note="Raw Id"),
    _p(0x3023, "faultSta", "uint32", RO, OBS, note="Fault status word"),
    _p(0x3024, "warnSta", "uint32", RO, OBS, note="Warning status word"),
    _p(0x3025, "drv_fault", "uint16", RO, OBS, note="Driver chip fault word"),
    _p(0x3026, "drv_temp", "int16", RO, OBS, "C", note="Driver chip temperature"),
    _p(0x3027, "Uq", "float", RO, OBS, "V", note="Q-axis voltage"),
    _p(0x3028, "as_angle", "float", RO, OBS, "rad", note="Encoder initial angle"),
    _p(0x3029, "cs_angle", "float", RO, OBS, "rad",
       note="Differential encoder initial angle"),
    _p(0x302A, "chasu_angle", "float", RO, OBS, "rad", note="Differential angle"),
    _p(0x302B, "ibus", "float", RO, OBS, "A"),
    _p(0x302C, "torque_fdb", "float", RO, OBS, "Nm", note="Torque feedback"),
    _p(0x302D, "rated_i", "float", RO, OBS, "A", note="Rated current"),
    _p(0x302E, "MechPos_init", "float", RO, OBS, "rad"),
    _p(0x302F, "obs_vel_max", "float", RO, OBS, "rad/s", note="Speed setpoint"),
    _p(0x3030, "loc_reff", "float", RO, OBS, "rad"),
    _p(0x3031, "instep", "float", RO, OBS),
    _p(0x3032, "position", "float", RO, OBS, "rad", note="Motor position"),
    _p(0x3033, "chasu_angle_init", "float", RO, OBS, "rad"),
    _p(0x3034, "chasu_angle_out", "float", RO, OBS, "rad"),
    _p(0x3035, "motormechinit", "float", RO, OBS, "rad"),
    _p(0x3036, "mech_angle_init2", "float", RO, OBS, "rad"),
    _p(0x3037, "mech_angle_rotat", "int16", RO, OBS),
    _p(0x3038, "log_fault1", "uint32", RO, OBS, note="Fault log"),
    _p(0x3039, "log_fault2", "uint32", RO, OBS, note="Fault log"),
    _p(0x303A, "log_fault3", "uint32", RO, OBS, note="Fault log"),
    _p(0x303B, "log_fault4", "uint32", RO, OBS, note="Fault log"),
    _p(0x303C, "log_fault5", "uint32", RO, OBS, note="Fault log"),
    _p(0x303D, "log_fault6", "uint32", RO, OBS, note="Fault log"),
    _p(0x303E, "log_fault7", "uint32", RO, OBS, note="Fault log"),
    _p(0x303F, "log_fault8", "uint32", RO, OBS, note="Fault log"),
    _p(0x3040, "ElecOffset", "float", RO, OBS, "rad", note="Electrical angle offset"),
    _p(0x3041, "mcOverTemp", "int16", RO, OBS, "C", scale=0.1,
       note="Overtemperature threshold"),
    _p(0x3042, "Kt_Nm_per_Amp", "float", RO, OBS, "Nm/A", note="Torque constant"),
    _p(0x3043, "Tqcali_Type", "uint8", RO, OBS, note="Motor type"),
    _p(0x3044, "theta_mech_1", "float", RO, OBS, "rad"),
    _p(0x3045, "adcOffset_1", "int32", RO, OBS),
    _p(0x3046, "adcOffset_2", "int32", RO, OBS),
    _p(0x3047, "coder_reg", "uint16", RO, OBS),
    _p(0x3048, "pos_cnt1", "uint16", RO, OBS),

    # -- runtime control parameters (0x70xx) ------------------------------
    _p(0x7005, "run_mode", "uint8", RW, CTRL, minimum=0, maximum=5,
       note="0 operation, 1 position PP, 2 velocity, 3 current, 5 position CSP"),
    _p(0x7006, "iq_ref", "float", RW, CTRL, "A", minimum=-90, maximum=90,
       note="Current mode Iq command"),
    _p(0x700A, "spd_ref", "float", RW, CTRL, "rad/s", minimum=-20, maximum=20,
       note="Velocity mode speed command"),
    _p(0x700B, "limit_torque", "float", RW, CTRL, "Nm", minimum=0, maximum=120),
    _p(0x7010, "cur_kp", "float", RW, CTRL, note="Default 0.17"),
    _p(0x7011, "cur_ki", "float", RW, CTRL, note="Default 0.012"),
    _p(0x7014, "cur_filt_gain", "float", RW, CTRL, minimum=0, maximum=1,
       note="Default 0.1"),
    _p(0x7016, "loc_ref", "float", RW, CTRL, "rad",
       note="Position mode angle command"),
    _p(0x7017, "limit_spd", "float", RW, CTRL, "rad/s", minimum=0, maximum=20,
       note="Position mode (CSP) speed limit"),
    _p(0x7018, "limit_cur", "float", RW, CTRL, "A", minimum=0, maximum=90,
       note="Velocity/position mode current limit"),
    _p(0x7019, "mechPos", "float", RO, CTRL, "rad", note="Load coil mechanical angle"),
    _p(0x701A, "iqf", "float", RO, CTRL, "A", minimum=-90, maximum=90),
    _p(0x701B, "mechVel", "float", RO, CTRL, "rad/s", minimum=-15, maximum=15),
    _p(0x701C, "VBUS", "float", RO, CTRL, "V"),
    _p(0x701E, "loc_kp", "float", RW, CTRL, note="Default 60"),
    _p(0x701F, "spd_kp", "float", RW, CTRL, note="Default 6"),
    _p(0x7020, "spd_ki", "float", RW, CTRL, note="Default 0.02"),
    _p(0x7021, "spd_filt_gain", "float", RW, CTRL, note="Default 0.1"),
    _p(0x7022, "acc_rad", "float", RW, CTRL, "rad/s^2",
       note="Velocity mode acceleration, default 15"),
    _p(0x7024, "vel_max", "float", RW, CTRL, "rad/s",
       note="Position mode (PP) speed, default 10"),
    _p(0x7025, "acc_set", "float", RW, CTRL, "rad/s^2",
       note="Position mode (PP) acceleration, default 10"),
    _p(0x7026, "EPScan_time", "uint16", RW, CTRL,
       note="Active report interval: 1 = 10 ms, +1 adds 5 ms"),
    _p(0x7028, "canTimeout", "uint32", RW, CTRL,
       note="CAN watchdog, 20000 = 1 s, default 0"),
    _p(0x7029, "zero_sta", "uint8", RW, CTRL, minimum=0, maximum=1,
       note="0 = 0..2pi, 1 = -pi..pi"),
    _p(0x702A, "damper", "uint8", RW, CTRL, minimum=0, maximum=1,
       note="Damping switch"),
    _p(0x702B, "add_offset", "float", RW, CTRL, "rad", note="Zero offset"),
    _p(0x702C, "alveolous_open", "uint8", RW, CTRL, minimum=0, maximum=1,
       note="Cogging compensation switch"),
    _p(0x702D, "iq_test", "uint8", RW, CTRL, minimum=0, maximum=1,
       note="Initialisation calibration switch"),
    _p(0x702E, "dcc_set", "float", RW, CTRL, "rad/s^2",
       note="PP mode deceleration, default 10"),
]

BY_INDEX: dict[int, Param] = {p.index: p for p in PARAMS}
BY_NAME: dict[str, Param] = {}
for _param in PARAMS:
    # 0x20xx/0x70xx share several names; the 0x70xx entry is the runtime one.
    BY_NAME.setdefault(_param.name, _param)


def get(index: int) -> Param | None:
    return BY_INDEX.get(index)


#: Sensible default channel set for the oscilloscope.
SCOPE_DEFAULTS = [0x3017, 0x3018, 0x302C, 0x3021, 0x3022, 0x3006, 0x300C]
