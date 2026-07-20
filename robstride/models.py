"""Per-model fixed-point scaling constants.

Every value in a type-1 command and a type-2 feedback frame is a uint16 scaled
between a model-specific min and max. Get these wrong and position, velocity and
torque all decode to plausible-looking but incorrect numbers - so each entry
carries a ``verified`` flag saying whether a primary source backs it.

All five models below are confirmed against the official RobStride user
manuals, published by the vendor at
https://github.com/RobStride/Product_Information, and independently
cross-checked against the ``kscalelabs/actuator`` Rust driver, which agrees on
every constant.

Note that Kp and Kd scale differently on the small motors: RS00/01/02 use
0..500 and 0..5, while RS03/04 use 0..5000 and 0..100. Using the RS04 values
everywhere silently misscales gains by 10x on the small joints.

Override any of it without touching code by dropping a ``models.json`` next to
this file:

    {"RS02": {"p_max": 12.57, "v_max": 44.0, "t_max": 17.0, "verified": true}}
"""

from __future__ import annotations

import json
import logging
from pathlib import Path

from .protocol import MotorLimits

log = logging.getLogger(__name__)

_CONFIG = Path(__file__).with_name("models.json")

#: The firmware constant is the literal 12.57f, not 4*pi (12.566). Firmware
#: older than 0.0.2.6 used 12.5 - check AppCodeVersion (0x1003) on very old
#: units, since the difference silently rescales every position readout.
P_LIMIT = 12.57

_BUILTIN: dict[str, MotorLimits] = {
    # In use on the OpenArmX arms, verified against the RS00 manual.
    "RS00": MotorLimits("RS00", p_max=P_LIMIT, v_max=33.0, t_max=14.0,
                        kp_max=500.0, kd_max=5.0,
                        i_max=16.0, gear_ratio=10.0, verified=True),
    # Not fitted to the OpenArmX arms; kept for completeness. Note these have
    # no confirmed 0x20xx/0x30xx parameter table in params.py, so the tool
    # blocks config writes for them until someone extracts their manuals.
    #
    # RS01 and RS02 genuinely share protocol constants despite differing
    # physically (315 vs 410 rpm no-load). RS01's protocol V_MAX of 44 rad/s
    # over-provisions its real capability - do not "fix" it down to 33.
    "RS01": MotorLimits("RS01", p_max=P_LIMIT, v_max=44.0, t_max=17.0,
                        kp_max=500.0, kd_max=5.0,
                        i_max=23.0, gear_ratio=7.75, verified=True),
    "RS02": MotorLimits("RS02", p_max=P_LIMIT, v_max=44.0, t_max=17.0,
                        kp_max=500.0, kd_max=5.0,
                        i_max=23.0, gear_ratio=7.75, verified=True),
    # In use on the OpenArmX arms, verified against the RS03 manual.
    "RS03": MotorLimits("RS03", p_max=P_LIMIT, v_max=20.0, t_max=60.0,
                        kp_max=5000.0, kd_max=100.0,
                        i_max=43.0, gear_ratio=9.0, verified=True),
    "RS04": MotorLimits("RS04", p_max=P_LIMIT, v_max=15.0, t_max=120.0,
                        kp_max=5000.0, kd_max=100.0,
                        i_max=90.0, gear_ratio=9.0, verified=True),
}

DEFAULT_MODEL = "RS04"


def _load_overrides() -> dict[str, MotorLimits]:
    merged = dict(_BUILTIN)
    if not _CONFIG.exists():
        return merged
    try:
        # utf-8-sig, because Notepad and PowerShell both write a BOM and a
        # plain utf-8 read would reject the file.
        raw = json.loads(_CONFIG.read_text(encoding="utf-8-sig"))
    except Exception as exc:
        log.warning("Ignoring malformed %s: %s", _CONFIG, exc)
        return merged

    for name, spec in raw.items():
        base = merged.get(name)
        fields = {
            "p_max": P_LIMIT, "v_max": 15.0, "t_max": 120.0,
            "kp_max": 5000.0, "kd_max": 100.0, "kp_min": 0.0, "kd_min": 0.0,
            "i_max": 90.0, "gear_ratio": 9.0, "verified": False,
        }
        if base is not None:
            fields = {k: getattr(base, k) for k in fields}
        fields.update({k: v for k, v in spec.items() if k in fields})
        merged[name] = MotorLimits(name=name, **fields)
        log.info("Loaded model override for %s", name)
    return merged


MODELS: dict[str, MotorLimits] = _load_overrides()


def model_names() -> list[str]:
    return sorted(MODELS)


def unverified() -> list[str]:
    """Models whose scaling constants nobody has confirmed yet."""
    return sorted(n for n, m in MODELS.items() if not m.verified)
