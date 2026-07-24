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

# Identity and firmware version. Same layout on every model seen so far.
_COMMON_HEAD: list[Param] = [
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

]

# --------------------------------------------------------------------------
# Model-specific ranges.
#
# The 0x20xx and 0x30xx tables are NOT the same across models - RS00 and RS03
# disagree on 21 of 40 config indices and 33 observation indices. Applying one
# model's table to another silently misreads values, and worse, misdirects
# writes: on RS00 the CAN id lives at 0x200A, while 0x2009 is motor_baud. A
# user "changing the CAN id" from the wrong table would change the bitrate and
# drop the motor off the bus.
#
# So these tables are keyed by model and never shared.
# --------------------------------------------------------------------------

_RS04_SPECIFIC: list[Param] = [
    # -- stored configuration (0x20xx, needs type-22 save) ----------------
    _p(0x2000, "echoPara1", "uint16", CFG, CONF, minimum=5, maximum=115),
    _p(0x2001, "echoPara2", "uint16", CFG, CONF, minimum=5, maximum=115),
    _p(0x2002, "echoPara3", "uint16", CFG, CONF, minimum=5, maximum=115),
    _p(0x2003, "echoPara4", "uint16", CFG, CONF, minimum=5, maximum=115),
    _p(0x2004, "echoFreHz", "uint32", RW, CONF, "Hz", minimum=1, maximum=10000),
    # RS02 glosses this as the encoder angle offset, RS03 as the low-speed-end
    # position offset. The manuals disagree; treat with care.
    _p(0x2005, "MechOffset", "float", RW, CONF, "rad", minimum=-50, maximum=50,
       note="Encoder angle offset (calibration output - do not hand-edit)"),
    # Not reserved: 差速偏置值, the calibrated zero for the chasu encoder.
    _p(0x2006, "chasu_offset", "float", RW, CONF, "rad", minimum=-50, maximum=50,
       note="Chasu encoder zero offset (calibration output - do not hand-edit)"),
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
    _p(0x3004, "encoderRaw", "uint16", RO, OBS,
       note="Rotor-side magnetic encoder raw sample, 0-16383"),
    _p(0x3005, "mcuTemp", "int16", RO, OBS, "C", scale=0.1),
    _p(0x3006, "motorTemp", "int16", RO, OBS, "C", scale=0.1, note="Motor NTC"),
    # The English manual calls this "Bus voltage", which is a mistranslation:
    # the Chinese reads 差速磁编码器采样值, and RS00/RS05 name the register
    # chasu_coder_raw outright.
    _p(0x3007, "encoder2raw", "uint16", RO, OBS,
       note="Chasu (output-side) magnetic encoder raw sample"),
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
    _p(0x3016, "modPos", "float", RO, OBS, "rad",
       note="Rotor angle, NOT turn-counted (wraps every motor revolution)"),
    # English "load end loop mechanical Angle" mistranslates 计圈 - this is
    # turn-counted, i.e. the multi-turn output position.
    _p(0x3017, "mechPos", "float", RO, OBS, "rad",
       note="Output angle, turn-counted (multi-turn) - use this for control"),
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
    _p(0x3028, "as_angle", "float", RO, OBS, "rad",
       note="Main magnetic encoder angle at init (motor side)"),
    _p(0x3029, "cs_angle", "float", RO, OBS, "rad",
       note="Differential encoder angle at init - the vernier partner"),
    _p(0x302A, "chasu_angle", "float", RO, OBS, "rad",
       note="Raw difference between the two encoders; resolves the turn number"),
    _p(0x302B, "ibus", "float", RO, OBS, "A"),
    _p(0x302C, "torque_fdb", "float", RO, OBS, "Nm", note="Torque feedback"),
    _p(0x302D, "rated_i", "float", RO, OBS, "A", note="Rated current"),
    _p(0x302E, "MechPos_init", "float", RO, OBS, "rad"),
    _p(0x302F, "obs_vel_max", "float", RO, OBS, "rad/s", note="Speed setpoint"),
    _p(0x3030, "loc_reff", "float", RO, OBS, "rad",
       note="Position loop setpoint; seeded from mech_angle_init2 at boot"),
    _p(0x3031, "instep", "float", RO, OBS),
    _p(0x3032, "position", "float", RO, OBS, "rad",
       note="Likely unwrapped multi-turn output position - undocumented"),
    # See PARAMETERS.md. This group is the power-on absolute-position
    # bootstrap, not a control input. The relations below are derived from the
    # manual's own sample column and hold exactly.
    _p(0x3033, "chasu_angle_init", "float", RO, OBS, "rad",
       note="Encoder difference minus chasu_offset (0x2006), at init"),
    _p(0x3034, "chasu_angle_out", "float", RO, OBS, "rad",
       note="chasu_angle_init x gear ratio"),
    _p(0x3035, "motormechinit", "float", RO, OBS, "rad",
       note="mech_angle_init2 x gear ratio - init angle, motor side"),
    _p(0x3036, "mech_angle_init2", "float", RO, OBS, "rad",
       note="Resolved absolute output angle at init; seeds the position loop"),
    _p(0x3037, "mech_angle_rotat", "int16", RO, OBS,
       note="Turn number resolved at init; should repeat across power cycles"),
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

]

# Extracted from the official RS00 and RS03 user manuals (260713), each
# read twice - once from flattened text, once from cell-aligned table
# extraction - with both passes required to agree. Rows whose secondary fields
# (min/max or remark) printed inconsistently are marked UNVERIFIED in their
# note and carry no range.
#
# Note how far these diverge from RS04 and from each other: CAN_ID is 0x200A
# on RS00 but 0x2009 on RS03, where RS00 has motor_baud.

_RS00_SPECIFIC: list[Param] = [
    _p(0x2000, "echoPara1", "uint16", CFG, CONF, minimum=5, maximum=74),
    _p(0x2001, "echoPara2", "uint16", CFG, CONF, minimum=5, maximum=74),
    _p(0x2002, "echoPara3", "uint16", CFG, CONF, minimum=5, maximum=74),
    _p(0x2003, "echoPara4", "uint16", CFG, CONF, minimum=5, maximum=74),
    _p(0x2004, "echoFreHz", "uint32", RW, CONF, minimum=1, maximum=10000),
    _p(0x2005, "MechOffset", "float", CFG, CONF, minimum=-7, maximum=7, note="Motor magnetic encoder Angle offset"),
    _p(0x2006, "MechPos_init", "float", RW, CONF, minimum=-50, maximum=50, note="Reserved parameter"),
    _p(0x2007, "limit_torque", "float", RW, CONF, minimum=0, maximum=17, note="Torque limitation"),
    _p(0x2008, "I_FW_MAX", "float", RW, CONF, minimum=0, maximum=33, note="Weak magnetic current value, default 0"),
    _p(0x2009, "motor_baud", "uint8", CFG, CONF, minimum=0, maximum=20, note="Baud rate flag bit"),
    _p(0x200A, "CAN_ID", "uint8", CFG, CONF, minimum=0, maximum=127, note="id of this object"),
    _p(0x200B, "CAN_MASTER", "uint8", CFG, CONF, minimum=0, maximum=127, note="can host id"),
    _p(0x200C, "CAN_TIMEOUT", "uint32", RW, CONF, minimum=0, maximum=100000, note="can timeout threshold. The default value is 0"),
    _p(0x200D, "status2", "int16", RW, CONF, minimum=0, maximum=1500, note="Reserved parameter"),
    _p(0x200E, "status3", "uint32", RW, CONF, minimum=1000, maximum=1000000, note="Reserved parameter"),
    _p(0x200F, "status1", "float", RW, CONF, minimum=1, maximum=64, note="Reserved parameter"),
    _p(0x2010, "Status6", "uint8", RW, CONF, minimum=0, maximum=1, note="Reserved parameter"),
    _p(0x2011, "cur_filt_gain", "float", RW, CONF, minimum=0, maximum=1, note="Current filtering parameter"),
    _p(0x2012, "cur_kp", "float", RW, CONF, minimum=0, maximum=200, note="Current kp"),
    _p(0x2013, "cur_ki", "float", RW, CONF, minimum=0, maximum=200, note="Current ki"),
    _p(0x2014, "spd_kp", "float", RW, CONF, minimum=0, maximum=200, note="Velocity kp"),
    _p(0x2015, "spd_ki", "float", RW, CONF, minimum=0, maximum=200, note="Speed ki"),
    _p(0x2016, "loc_kp", "float", RW, CONF, minimum=0, maximum=200, note="Position kp"),
    _p(0x2017, "spd_filt_gain", "float", RW, CONF, minimum=0, maximum=1, note="Velocity filter parameter"),
    _p(0x2018, "limit_spd", "float", RW, CONF, minimum=0, maximum=200, note="Location mode speed limit"),
    _p(0x2019, "limit_cur", "float", RW, CONF, minimum=0, maximum=23, note="Position, Velocity mode current limit"),
    _p(0x201A, "loc_ref_filt_gain", "float", RW, CONF, minimum=0, maximum=100, note="Reserved parameter"),
    _p(0x201B, "limit_loc", "float", RW, CONF, minimum=0, maximum=100, note="Reserved parameter"),
    _p(0x201C, "position_offset", "float", RW, CONF, minimum=0, maximum=27, note="High speed segment offset"),
    _p(0x201D, "chasu_angle_offset", "float", RW, CONF, minimum=0, maximum=27, note="The low end is offset"),
    _p(0x201E, "spd_step_value", "float", RW, CONF, minimum=0, maximum=150, note="Velocity-mode acceleration"),
    _p(0x201F, "vel_max", "float", RW, CONF, minimum=0, maximum=20, note="PP mode speed"),
    _p(0x2020, "acc_set", "float", RW, CONF, minimum=0, maximum=1000, note="PP mode acceleration"),
    _p(0x2021, "zero_sta", "float", RW, CONF, note="Zero marker. CAVEAT: the manual's 3.3.4 table lists type=float, max=100, min=0, but zero_sta is  [UNVERIFIED: check against the manual]"),
    _p(0x2022, "protocol_1", "uint8", RW, CONF, note="Protocol flag. Max/min cells are blank in the manual. Per section 4.1.12 the values are 0=privat"),
    _p(0x2023, "damper", "uint8", RW, CONF, note="Damping switch. CAVEAT: the manual prints max=0 and min=20, which is inverted/nonsensical. Secti [UNVERIFIED: check against the manual]"),
    _p(0x2024, "add_offset", "float", RW, CONF, note="Position offset parameter. CAVEAT: the manual prints max=-7 and min=7, i.e. the two columns are  [UNVERIFIED: check against the manual]"),
    _p(0x3000, "timeUse0", "uint16", RO, OBS),
    _p(0x3001, "timeUse1", "uint16", RO, OBS),
    _p(0x3002, "timeUse2", "uint16", RO, OBS),
    _p(0x3003, "timeUse3", "uint16", RO, OBS),
    _p(0x3004, "encoderRaw", "int16", RO, OBS, note="Magnetic encoder sampling value"),
    _p(0x3005, "mcuTemp", "int16", RO, OBS, note="mcu internal temperature, *10"),
    _p(0x3006, "motorTemp", "int16", RO, OBS, note="Motor ntc temperature, *10"),
    _p(0x3007, "vBus(mv)", "uint16", RO, OBS, note="Bus voltage"),
    _p(0x3008, "adc1Offset", "int32", RO, OBS, note="adc sampling channel 1 Zero current bias"),
    _p(0x3009, "adc2Offset", "int32", RO, OBS, note="adc sampling channel 2 Zero current bias"),
    _p(0x300A, "adc1Raw", "uint16", RO, OBS, note="adc sampling value 1"),
    _p(0x300B, "adc2Raw", "uint16", RO, OBS, note="adc sampling value 2"),
    _p(0x300C, "VBUS", "float", RO, OBS, note="Bus voltage V"),
    _p(0x300D, "cmdId", "float", RO, OBS, note="id ring instruction, A"),
    _p(0x300E, "cmdIq", "float", RO, OBS, note="iq ring command, A"),
    _p(0x300F, "cmdlocref", "float", RO, OBS, note="Position loop command, rad"),
    _p(0x3010, "cmdspdref", "float", RO, OBS, note="Speed loop command, rad/s"),
    _p(0x3011, "cmdTorque", "float", RO, OBS, note="Torque instruction, nm"),
    _p(0x3012, "cmdPos", "float", RO, OBS, note="mit Protocol Angle instruction"),
    _p(0x3013, "cmdVel", "float", RO, OBS, note="mit Protocol Speed instruction"),
    _p(0x3014, "rotation", "int16", RO, OBS, note="Number of turns"),
    _p(0x3015, "modPos", "float", RO, OBS, note="Motor uncounted coil mechanical Angle, rad"),
    _p(0x3016, "mechPos", "float", RO, OBS, note="Load end loop mechanical Angle, rad"),
    _p(0x3017, "mechVel", "float", RO, OBS, note="Load speed: rad/s"),
    _p(0x3018, "elecPos", "float", RO, OBS, note="Electrical Angle"),
    _p(0x3019, "ia", "float", RO, OBS, note="U-wire current, A"),
    _p(0x301A, "ib", "float", RO, OBS, note="V-wire current, A"),
    _p(0x301B, "ic", "float", RO, OBS, note="W-wire current, A"),
    _p(0x301C, "timeout", "uint32", RO, OBS, note="Timeout counter value"),
    _p(0x301D, "phaseOrder", "uint8", RO, OBS, note="Directional marking"),
    _p(0x301E, "iqf", "float", RO, OBS, note="iq filter value, A"),
    _p(0x301F, "boardTemp", "int16", RO, OBS, note="Plate temperature, *10"),
    _p(0x3020, "iq", "float", RO, OBS, note="iq Original value, A"),
    _p(0x3021, "id", "float", RO, OBS, note="id Original value, A"),
    _p(0x3022, "faultSta", "uint32", RO, OBS, note="Fault status value"),
    _p(0x3023, "warnSta", "uint32", RO, OBS, note="Warning status value"),
    _p(0x3024, "drv_fault", "uint16", RO, OBS, note="The driver chip fault value is 1"),
    _p(0x3025, "drv_temp", "int16", RO, OBS, note="The driver chip fault value is 2"),
    _p(0x3026, "Uq", "float", RO, OBS, note="Q-axis voltage"),
    _p(0x3027, "Ud", "float", RO, OBS, note="D-axis voltage"),
    _p(0x3028, "dtc_u", "float", RO, OBS, note="The duty cycle of the U-phase output"),
    _p(0x3029, "dtc_v", "float", RO, OBS, note="The duty cycle of the V-phase output"),
    _p(0x302A, "dtc_w", "float", RO, OBS, note="The duty cycle of the W-phase output"),
    _p(0x302B, "v_bus", "float", RO, OBS, note="Vbus in the closed loop"),
    _p(0x302C, "torque_fdb", "float", RO, OBS, note="Torque feedback value, nm"),
    _p(0x302D, "rated_i", "float", RO, OBS, note="Rated current of motor"),
    _p(0x302E, "limit_i", "float", RO, OBS, note="The motor limits the maximum current"),
    _p(0x302F, "spd_ref", "float", RO, OBS, note="Motor speed expectation"),
    _p(0x3030, "spd_reff", "float", RO, OBS, note="Motor speed expectation 2"),
    _p(0x3031, "zero_fault", "float", RO, OBS, note="Motor position determination parameters"),
    _p(0x3032, "chasu_coder_raw", "float", RO, OBS, note="Motor position determination parameters"),
    _p(0x3033, "chasu_angle", "float", RO, OBS, note="Motor position determination parameters"),
    _p(0x3034, "as_angle", "float", RO, OBS, note="Motor position determination parameters"),
    _p(0x3035, "vel_max", "float", RO, OBS, note="Motor position determination parameters"),
    _p(0x3036, "judge", "float", RO, OBS, note="Motor position determination parameters"),
    _p(0x3037, "position", "float", RO, OBS, note="Position value"),
    _p(0x3038, "chasu_angle_init", "float", RO, OBS, note="Angle initialization"),
    _p(0x3039, "chasu_angle_out", "float", RO, OBS, note="Motor position determination parameters"),
    _p(0x303A, "motormechinit", "float", RO, OBS, note="Motor position determination parameters"),
    _p(0x303B, "mech_angle_init2", "float", RO, OBS, note="Motor position determination parameters"),
    _p(0x303C, "mech_angle_rotat", "float", RO, OBS, note="Motor position determination parameters"),
    _p(0x303D, "fault1", "uint32", RO, OBS, note="Log failure"),
    _p(0x303E, "fault2", "uint32", RO, OBS, note="Log failure"),
    _p(0x303F, "fault3", "uint32", RO, OBS, note="Log failure"),
    _p(0x3040, "fault4", "uint32", RO, OBS, note="Log failure"),
    _p(0x3041, "fault5", "uint32", RO, OBS, note="Log failure"),
    _p(0x3042, "fault6", "uint32", RO, OBS, note="Log failure"),
    _p(0x3043, "fault7", "uint32", RO, OBS, note="Log failure"),
    _p(0x3044, "fault8", "uint32", RO, OBS, note="Log failure"),
    _p(0x3045, "ElecOffset", "float", RO, OBS, note="electrical Angle offset"),
    _p(0x3046, "mcOverTemp", "int16", RO, OBS, note="Overtemperature threshold"),
    _p(0x3047, "Kt_Nm/Amp", "float", RO, OBS, note="Moment coefficient"),
    _p(0x3048, "Tqcali_Type", "uint8", RO, OBS, note="Motor type"),
    _p(0x3049, "low_position", "float", RO, OBS, note="Motor position determination parameters"),
    _p(0x304A, "theta_mech_1", "float", RO, OBS, note="Type 2 Low speed Angle"),
    _p(0x304B, "instep", "float", RO, OBS, note="Motor protection decision parameters"),
    _p(0x304C, "adc0ffset_1", "int32", RO, OBS, note="adc sampling channel 1 Zero current bias"),
    _p(0x304D, "adc0ffset_2", "int32", RO, OBS, note="adc sampling channel 2 Zero current bias"),
    _p(0x304E, "pos_cnt1", "uint16", RO, OBS, note="System parameters"),
    _p(0x304F, "H'", "uint8", RO, OBS, note="System parameters. Name is printed as the single character H followed by a right single quote (U [UNVERIFIED: check against the manual]"),
]

_RS03_SPECIFIC: list[Param] = [
    _p(0x2000, "echoPara1", "uint16", CFG, CONF, minimum=5, maximum=74),
    _p(0x2001, "echoPara2", "uint16", CFG, CONF, minimum=5, maximum=74),
    _p(0x2002, "echoPara3", "uint16", CFG, CONF, minimum=5, maximum=74),
    _p(0x2003, "echoPara4", "uint16", CFG, CONF, minimum=5, maximum=74),
    _p(0x2004, "echoFreHz", "uint32", RW, CONF, minimum=1, maximum=10000),
    _p(0x2005, "MechOffset", "float", CFG, CONF, minimum=-7, maximum=7, note="Motor magnetic encoder Angle offset"),
    _p(0x2006, "Chasu_offset", "float", RW, CONF, minimum=-50, maximum=50, note="Reserved parameter. NOTE: RS00 has MechPos_init at this index with identical type/min/max/remark"),
    _p(0x2007, "Status1", "float", RW, CONF, minimum=0, maximum=17, note="Torque limitation. NOTE: RS00 names this same row limit_torque; RS03 prints Status1. Max 17 also"),
    _p(0x2008, "I_FW_MAX", "float", RW, CONF, minimum=0, maximum=33, note="Weak magnetic current value, default 0"),
    _p(0x2009, "CAN_ID", "uint8", CFG, CONF, note="Remark column prints 'Motor index, marking the joint position of the motor'. NAME AND INDEX CONF [UNVERIFIED: check against the manual]"),
    _p(0x200A, "CAN_MASTER", "uint8", CFG, CONF, note="Remark column prints 'id of this object' (which describes CAN_ID, not CAN_MASTER). Name and inde [UNVERIFIED: check against the manual]"),
    _p(0x200B, "CAN_TIMEOUT", "uint8", CFG, CONF, note="Remark column prints 'can host id' (which describes CAN_MASTER). Name and index confirmed. CAVEA [UNVERIFIED: check against the manual]"),
    _p(0x200C, "status2", "uint32", RW, CONF, note="Remark column prints 'can timeout threshold. The default value is 0'. Name and index confirmed a [UNVERIFIED: check against the manual]"),
    _p(0x200D, "status3", "int16", RW, CONF, minimum=0, maximum=1500, note="Reserved parameter"),
    _p(0x200E, "Status4", "uint32", RW, CONF, minimum=1000, maximum=1000000, note="Reserved parameter"),
    _p(0x200F, "Status5", "float", RW, CONF, minimum=1, maximum=64, note="Reserved parameter"),
    _p(0x2010, "Status6", "uint8", RW, CONF, minimum=0, maximum=1, note="Reserved parameter"),
    _p(0x2011, "cur_filt_gain", "float", RW, CONF, minimum=0, maximum=1, note="Current filtering parameter"),
    _p(0x2012, "cur_kp", "float", RW, CONF, minimum=0, maximum=200, note="Current kp"),
    _p(0x2013, "cur_ki", "float", RW, CONF, minimum=0, maximum=200, note="Current ki"),
    _p(0x2014, "spd_kp", "float", RW, CONF, minimum=0, maximum=200, note="Velocity kp"),
    _p(0x2015, "spd_ki", "float", RW, CONF, minimum=0, maximum=200, note="Speed ki"),
    _p(0x2016, "loc_kp", "float", RW, CONF, minimum=0, maximum=200, note="Position kp"),
    _p(0x2017, "spd_filt_gain", "float", RW, CONF, minimum=0, maximum=1, note="Velocity filter parameter"),
    _p(0x2018, "limit_spd", "float", RW, CONF, minimum=0, maximum=200, note="Location mode speed limit"),
    _p(0x2019, "limit_cur", "float", RW, CONF, minimum=0, maximum=23, note="Position, Velocity mode current limit"),
    _p(0x201A, "limit_a", "float", RW, CONF, minimum=0, maximum=100, note="Reserved parameter"),
    _p(0x201B, "fault1", "float", RW, CONF, minimum=0, maximum=100, note="Reserved parameter"),
    _p(0x201C, "fault2", "float", RW, CONF, minimum=0, maximum=27, note="High speed segment offset"),
    _p(0x201D, "fault3", "float", RW, CONF, minimum=0, maximum=27, note="The low end is offset"),
    _p(0x201E, "fault4", "float", RW, CONF, minimum=0, maximum=150, note="Velocity-mode acceleration"),
    _p(0x201F, "fault5", "float", RW, CONF, minimum=0, maximum=20, note="PP mode speed"),
    _p(0x2020, "fault6", "float", RW, CONF, minimum=0, maximum=1000, note="PP mode acceleration"),
    _p(0x2021, "fault7", "float", RW, CONF, minimum=0, maximum=100, note="Zero marker"),
    _p(0x2022, "baud", "uint8", RW, CONF, minimum=0, maximum=10, note="Baud rate flag. This is RS03's baud-rate parameter; note it lives at 0x2022 here, NOT at 0x2009 "),
    _p(0x2023, "zero_sta", "uint8", RW, CONF, note="Zero point flag. Max/min cells are blank in the manual. Per section 4.2.2, 0 => power-on positio"),
    _p(0x2024, "position_offset", "uint8", RW, CONF, note="Position offset. CAVEAT: dtype printed as uint8, but max is 27 with a positional/radian meaning  [UNVERIFIED: check against the manual]"),
    _p(0x2025, "protocol_1", "uint8", RW, CONF, note="Protocol flag. Max/min cells blank in the manual. Per section 4.1.12: 0=private (default), 1=CAN"),
    _p(0x2026, "damper", "uint8", RW, CONF, note="Damping switch. CAVEAT: the manual prints max=0 and min=20, which is inverted/nonsensical (same  [UNVERIFIED: check against the manual]"),
    _p(0x2027, "add_offset", "float", RW, CONF, note="Position offset parameter. CAVEAT: the manual prints max=-7 and min=7, i.e. the columns are tran [UNVERIFIED: check against the manual]"),
    _p(0x3000, "timeUse0", "uint16", RO, OBS),
    _p(0x3001, "timeUse1", "uint16", RO, OBS),
    _p(0x3002, "timeUse2", "uint16", RO, OBS),
    _p(0x3003, "timeUse3", "uint16", RO, OBS),
    _p(0x3004, "encoderRaw", "int16", RO, OBS, note="Magnetic encoder sampling value"),
    _p(0x3005, "mcuTemp", "int16", RO, OBS, note="mcu internal temperature, *10"),
    _p(0x3006, "motorTemp", "int16", RO, OBS, note="Motor ntc temperature, *10"),
    _p(0x3007, "vBus(mv)", "uint16", RO, OBS, note="Bus voltage"),
    _p(0x3008, "adc1Offset", "int32", RO, OBS, note="adc sampling channel 1 Zero current bias"),
    _p(0x3009, "adc2Offset", "int32", RO, OBS, note="adc sampling channel 2 Zero current bias"),
    _p(0x300A, "adc1Raw", "uint16", RO, OBS, note="adc sampling value 1"),
    _p(0x300B, "adc2Raw", "uint16", RO, OBS, note="adc sampling value 2"),
    _p(0x300C, "VBUS", "float", RO, OBS, note="Bus voltage V"),
    _p(0x300D, "cmdId", "float", RO, OBS, note="id ring instruction, A"),
    _p(0x300E, "cmdIq", "float", RO, OBS, note="iq ring command, A"),
    _p(0x300F, "cmdlocref", "float", RO, OBS, note="Position loop command, rad"),
    _p(0x3010, "cmdspdref", "float", RO, OBS, note="Speed loop command, rad/s"),
    _p(0x3011, "cmdTorque", "float", RO, OBS, note="Torque instruction, nm"),
    _p(0x3012, "cmdPos", "float", RO, OBS, note="mit Protocol Angle instruction"),
    _p(0x3013, "cmdVel", "float", RO, OBS, note="mit Protocol Speed instruction"),
    _p(0x3014, "rotation", "int16", RO, OBS, note="Number of turns"),
    _p(0x3015, "modPos", "float", RO, OBS, note="Motor uncounted coil mechanical Angle, rad"),
    _p(0x3016, "mechPos", "float", RO, OBS, note="Load end loop mechanical Angle, rad"),
    _p(0x3017, "mechVel", "float", RO, OBS, note="Load speed: rad/s"),
    _p(0x3018, "elecPos", "float", RO, OBS, note="Electrical Angle"),
    _p(0x3019, "ia", "float", RO, OBS, note="U-wire current, A"),
    _p(0x301A, "ib", "float", RO, OBS, note="V-wire current, A"),
    _p(0x301B, "ic", "float", RO, OBS, note="W-wire current, A"),
    _p(0x301C, "timeout", "uint32", RO, OBS, note="Timeout counter value"),
    _p(0x301D, "phaseOrder", "uint8", RO, OBS, note="Directional marking"),
    _p(0x301E, "iqf", "float", RO, OBS, note="iq filter value, A"),
    _p(0x301F, "boardTemp", "int16", RO, OBS, note="Plate temperature, *10"),
    _p(0x3020, "iq", "float", RO, OBS, note="iq Original value, A"),
    _p(0x3021, "id", "float", RO, OBS, note="id Original value, A"),
    _p(0x3022, "faultSta", "uint32", RO, OBS, note="Fault status value"),
    _p(0x3023, "warnSta", "uint32", RO, OBS, note="Warning status value"),
    _p(0x3024, "drv_fault", "uint16", RO, OBS, note="The driver chip fault value is 1"),
    _p(0x3025, "drv_temp", "int16", RO, OBS, note="The driver chip fault value is 2"),
    _p(0x3026, "Uq", "float", RO, OBS, note="Q-axis voltage"),
    _p(0x3027, "as_angle", "float", RO, OBS, note="Magnetic encoder initial angle"),
    _p(0x3028, "cs_angle", "float", RO, OBS, note="Differential magnetic encoder initial angle"),
    _p(0x3029, "chasu_angle", "float", RO, OBS, note="Differential angle"),
    _p(0x302A, "v_bus", "float", RO, OBS, note="Motor voltage"),
    _p(0x302B, "ElecOffset", "float", RO, OBS, note="Electrical angle offset"),
    _p(0x302C, "torque_fdb", "float", RO, OBS, note="Torque feedback value, nm"),
    _p(0x302D, "rated_i", "float", RO, OBS, note="Rated current of motor"),
    _p(0x302E, "MechPos_init", "float", RO, OBS, note="Motor retention parameters. NOTE: on RS00, MechPos_init is a writable 0x2006 config parameter; o"),
    _p(0x302F, "instep", "float", RO, OBS, note="Motor protection parameters"),
    _p(0x3030, "status", "uint8", RO, OBS, note="Reserved parameters"),
    _p(0x3031, "cmdlocref", "float", RO, OBS, note="Position setpoint"),
    _p(0x3032, "vel_max", "float", RO, OBS, note="Motor speed setpoint"),
    _p(0x3033, "fault1", "float", RO, OBS, note="Motor position determination parameters"),
    _p(0x3034, "fault2", "float", RO, OBS, note="Motor position determination parameters"),
    _p(0x3035, "fault3", "float", RO, OBS, note="Motor position determination parameters"),
    _p(0x3036, "fault4", "float", RO, OBS, note="Motor position determination parameters"),
    _p(0x3037, "fault5", "float", RO, OBS, note="Log failure"),
    _p(0x3038, "fault6", "uint32", RO, OBS, note="Log failure"),
    _p(0x3039, "fault7", "uint32", RO, OBS, note="Log failure"),
    _p(0x303A, "fault8", "uint32", RO, OBS, note="Log failure"),
    _p(0x303B, "mcOverTemp", "int16", RO, OBS, note="Over-temperature threshold"),
    _p(0x303C, "Kt_Nm/Amp", "float", RO, OBS, note="Torque coefficient"),
    _p(0x303D, "Tqcali_Type", "uint8", RO, OBS, note="Motor type"),
    _p(0x303E, "theta_mech_1", "float", RO, OBS, note="Type 2 low-speed angle"),
    _p(0x303F, "adc0ffset_1", "uint32", RO, OBS, note="ADC sampling channel 1 zero current offset"),
    _p(0x3040, "adc0ffset_2", "uint32", RO, OBS, note="ADC sampling channel 2 zero current offset"),
    _p(0x3041, "can_status", "uint8", RO, OBS, note="CAN status"),
    _p(0x3042, "position", "float", RO, OBS, note="Initialization position"),
    _p(0x3043, "chasu_angle_init", "float", RO, OBS, note="Motor position determination parameters"),
    _p(0x3044, "chasu_angle_out", "float", RO, OBS, note="Motor position determination parameters"),
    _p(0x3045, "motormechinit", "float", RO, OBS, note="Motor position determination parameters"),
    _p(0x3046, "mech_angle_init2", "float", RO, OBS, note="Motor position determination parameters"),
    _p(0x3047, "mech_angle_rotat", "int16", RO, OBS, note="Motor position determination parameters"),
    _p(0x3048, "coder_reg", "uint16", RO, OBS, note="System parameters"),
    _p(0x3049, "pos_cnt1", "uint16", RO, OBS, note="System parameters"),
]

# --------------------------------------------------------------------------
# Runtime control (0x70xx).
#
# Verified identical between RS00, RS03 and RS04 - all 29 indices agree. This
# is the range that matters for actual motion control, so the good news is
# that control code is model-portable; only configuration and diagnostics are
# not.
# --------------------------------------------------------------------------

_CONTROL: list[Param] = [
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

# --------------------------------------------------------------------------
# Public API - everything is keyed by model
# --------------------------------------------------------------------------

#: Model-specific 0x20xx / 0x30xx tables. A model absent from here has no
#: confirmed table, and only the common and control ranges are exposed for it.
MODEL_SPECIFIC: dict[str, list[Param]] = {
    "RS00": _RS00_SPECIFIC,
    "RS03": _RS03_SPECIFIC,
    "RS04": _RS04_SPECIFIC,
}

FALLBACK_MODEL = "RS04"


def has_table(model: str) -> bool:
    """True when this model's config/observation table has been confirmed."""
    return model in MODEL_SPECIFIC


def params_for(model: str) -> list[Param]:
    """The full parameter list for one model.

    Models without a confirmed table get only the ranges known to be
    universal, rather than another model's table pretending to fit.
    """
    return _COMMON_HEAD + MODEL_SPECIFIC.get(model, []) + _CONTROL


def index_map(model: str) -> dict[int, Param]:
    return {p.index: p for p in params_for(model)}


_CACHE: dict[str, dict[int, Param]] = {}


def get(index: int, model: str = FALLBACK_MODEL) -> Param | None:
    """Look up one parameter for a given model."""
    table = _CACHE.get(model)
    if table is None:
        table = _CACHE[model] = index_map(model)
    return table.get(index)


_BY_NAME: dict[str, dict[str, Param]] = {}


def by_name(name: str, model: str = FALLBACK_MODEL) -> Param | None:
    """Look one up by name instead of index.

    The only safe way to reach a model-specific register generically: the
    *names* are stable across models where the indices are not - mechPos is
    0x3017 on an RS04 and 0x3016 on an RS00 - so code that wants "this
    motor's mechPos" has to ask for it by meaning. Returns ``None`` when the
    model has no confirmed table, which is the honest answer rather than
    another model's index.

    Six names appear in both the stored and the runtime range (limit_spd,
    limit_cur, cur_kp, zero_sta, damper, add_offset, and mechPos as well on
    some models). The stored entry wins here, because that is the one whose
    index a caller cannot work out for itself. The runtime copy is identical
    on every model, so code that wants *that* one should say 0x7017 or
    0x7019 outright rather than asking by name.
    """
    table = _BY_NAME.get(model)
    if table is None:
        table = _BY_NAME[model] = {}
        for param in params_for(model):
            table.setdefault(param.name, param)
    return table.get(name)


def index_of(name: str, model: str = FALLBACK_MODEL) -> int | None:
    """This model's index for a named parameter, or ``None``."""
    param = by_name(name, model)
    return param.index if param else None


def is_model_specific(index: int) -> bool:
    """True for indices whose meaning depends on the motor model."""
    return 0x2000 <= index < 0x4000


#: Sensible default channel set for the oscilloscope, in common/control ranges
#: plus RS04 observation indices. Callers should filter to params_for(model).
SCOPE_DEFAULTS = [0x3017, 0x3018, 0x302C, 0x3021, 0x3022, 0x3006, 0x300C]
