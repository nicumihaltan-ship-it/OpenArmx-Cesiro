"""Protocol checks against the worked examples in the RS04 manual."""

import struct

import pytest

from robstride import params as P
from robstride import protocol as proto
from robstride.models import MODELS


def test_id_packing_matches_manual_example():
    """Manual 4.1.14: type 17, host 0xFD, motor 0x7F -> 0x11 00FD 7F."""
    can_id = proto.pack_id(0x11, 0x00FD, 0x7F)
    assert can_id == 0x1100FD7F
    assert proto.unpack_id(can_id) == (0x11, 0x00FD, 0x7F)


def test_read_frame_matches_manual_example():
    """Reading loc_kp (0x701E) from motor 0x7F."""
    frame = proto.param_read(0x7F, 0x701E, host_id=0xFD)
    assert frame.can_id == 0x1100FD7F
    assert frame.data[:2] == bytes([0x1E, 0x70])


def test_param_reply_decodes_ieee754_float():
    """Manual: reply payload 1E 70 00 00 00 00 F0 41 means loc_kp == 30.0."""
    data = bytes([0x1E, 0x70, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x41])
    reply = proto.decode_param_reply(proto.pack_id(0x11, 0x007F, 0xFD), data)
    assert reply.index == 0x701E
    assert reply.motor_id == 0x7F
    assert reply.ok
    assert P.get(0x701E).decode(reply.raw) == pytest.approx(30.0)


def test_write_frame_matches_studio_example():
    """Studio doc: payload 05 70 00 00 01 00 00 00 sets run_mode = 1."""
    frame = proto.param_write(0x01, 0x7005, P.get(0x7005).encode(1), host_id=0xFD)
    assert proto.unpack_id(frame.can_id) == (0x12, 0x00FD, 0x01)
    assert frame.data == bytes([0x05, 0x70, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00])


def test_float_roundtrip_is_within_one_lsb():
    limits = MODELS["RS04"]
    for value in (-12.0, -3.14, 0.0, 1.5, 12.0):
        raw = proto.float_to_uint(value, limits.p_min, limits.p_max)
        back = proto.uint_to_float(raw, limits.p_min, limits.p_max)
        lsb = (limits.p_max - limits.p_min) / 65535
        assert abs(back - value) <= lsb


def test_motion_control_payload_is_big_endian():
    limits = MODELS["RS04"]
    frame = proto.motion_control(1, torque=0.0, position=0.0, velocity=0.0,
                                 kp=0.0, kd=0.0, limits=limits)
    pos, vel, kp, kd = struct.unpack(">HHHH", frame.data)
    # Zero maps to mid-scale for symmetric ranges, and to 0 for one-sided ones.
    assert pos == proto.float_to_uint(0.0, limits.p_min, limits.p_max)
    assert kp == 0 and kd == 0


def test_feedback_decode_roundtrip():
    limits = MODELS["RS04"]
    payload = struct.pack(
        ">HHHH",
        proto.float_to_uint(1.234, limits.p_min, limits.p_max),
        proto.float_to_uint(-2.5, limits.v_min, limits.v_max),
        proto.float_to_uint(10.0, limits.t_min, limits.t_max),
        332,
    )
    # data2: motor id 5, no faults, Motor mode (2) in bits 14..15
    data2 = 0x05 | (proto.MotorMode.RUN << 14)
    fb = proto.decode_feedback(proto.pack_id(2, data2, 0xFD), payload, limits)
    assert fb.motor_id == 5
    assert fb.mode is proto.MotorMode.RUN
    assert fb.position == pytest.approx(1.234, abs=1e-3)
    assert fb.velocity == pytest.approx(-2.5, abs=1e-3)
    assert fb.torque == pytest.approx(10.0, abs=1e-2)
    assert fb.temperature == pytest.approx(33.2)
    assert fb.faults == []


def test_feedback_decodes_fault_bits():
    limits = MODELS["RS04"]
    # bit18 = overtemperature -> bit 2 of the 6-bit fault field
    data2 = 0x03 | (1 << (18 - 16 + 8)) | (proto.MotorMode.RUN << 14)
    fb = proto.decode_feedback(proto.pack_id(2, data2, 0xFD), b"\x00" * 8, limits)
    assert "overtemperature" in fb.faults


def test_param_scale_applies_to_temperature():
    """0x3006 motorTemp is stored as deci-degrees."""
    param = P.get(0x3006)
    assert param.decode(struct.pack("<h", 333)) == pytest.approx(33.3)
    assert param.encode(33.3) == struct.pack("<h", 333)[:2].ljust(4, b"\x00")


@pytest.mark.parametrize("model,v_max,t_max,kp_max,kd_max,i_max,gear", [
    ("RS00", 33.0, 14.0, 500.0, 5.0, 16.0, 10.0),
    ("RS01", 44.0, 17.0, 500.0, 5.0, 23.0, 7.75),
    ("RS02", 44.0, 17.0, 500.0, 5.0, 23.0, 7.75),
    ("RS03", 20.0, 60.0, 5000.0, 100.0, 43.0, 9.0),
    ("RS04", 15.0, 120.0, 5000.0, 100.0, 90.0, 9.0),
])
def test_model_constants_match_official_manuals(model, v_max, t_max, kp_max,
                                                kd_max, i_max, gear):
    """Guards the per-model scaling table against silent regressions.

    Kp/Kd in particular differ by 10x between the small (RS00-02) and large
    (RS03/04) motors - copying RS04's values across would misscale gains.
    """
    limits = MODELS[model]
    assert limits.p_max == 12.57       # firmware literal, not 4*pi
    assert limits.v_max == v_max
    assert limits.t_max == t_max
    assert limits.kp_max == kp_max
    assert limits.kd_max == kd_max
    assert limits.i_max == i_max
    assert limits.gear_ratio == gear
    assert limits.verified


def test_kp_encoding_differs_between_small_and_large_models():
    """The same Kp must not encode identically on RS02 and RS04."""
    small, large = MODELS["RS02"], MODELS["RS04"]
    kp = 100.0
    assert (proto.float_to_uint(kp, small.kp_min, small.kp_max)
            != proto.float_to_uint(kp, large.kp_min, large.kp_max))


def test_parameter_table_has_no_duplicate_indices():
    seen = [p.index for p in P.PARAMS]
    assert len(seen) == len(set(seen))


def test_readonly_params_reject_writes():
    from robstride.motor import Motor

    class FakeLink:
        host_id = 0xFD
        channel = "test"
        def add_listener(self, *a): pass
        def send(self, *a): raise AssertionError("should not transmit")

    motor = Motor(FakeLink(), 1, "RS04")
    with pytest.raises(PermissionError):
        motor.write(0x3017, 1.0)      # mechPos is read-only
