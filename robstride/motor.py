"""High-level per-motor operations layered over :mod:`robstride.bus`."""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field

from . import params as P
from . import protocol as proto
from .bus import CanLink
from .models import DEFAULT_MODEL, MODELS

log = logging.getLogger(__name__)


@dataclass
class MotorState:
    """Latest known state, refreshed from every feedback frame."""

    position: float = 0.0
    velocity: float = 0.0
    torque: float = 0.0
    temperature: float = 0.0
    mode: proto.MotorMode = proto.MotorMode.RESET
    faults: list[str] = field(default_factory=list)
    updated: float = 0.0

    @property
    def age(self) -> float:
        return time.time() - self.updated if self.updated else float("inf")


class Motor:
    """One RobStride actuator on one CAN channel."""

    def __init__(self, link: CanLink, motor_id: int, model: str = DEFAULT_MODEL):
        self.link = link
        self.motor_id = motor_id
        self.model = model
        self.state = MotorState()
        self.uid: bytes | None = None
        self.firmware: str | None = None
        self._lock = threading.Lock()
        link.add_listener(proto.CommType.FEEDBACK, self._on_feedback)

    # -- model / scaling --------------------------------------------------

    @property
    def limits(self):
        return MODELS.get(self.model, MODELS[DEFAULT_MODEL])

    def set_model(self, model: str) -> None:
        self.model = model

    # -- feedback ---------------------------------------------------------

    def _on_feedback(self, can_id: int, data: bytes) -> None:
        _, data2, _ = proto.unpack_id(can_id)
        if (data2 & 0xFF) != self.motor_id:
            return
        fb = proto.decode_feedback(can_id, data, self.limits)
        with self._lock:
            self.state = MotorState(
                position=fb.position, velocity=fb.velocity, torque=fb.torque,
                temperature=fb.temperature, mode=fb.mode, faults=fb.faults,
                updated=time.time(),
            )

    # -- lifecycle commands ----------------------------------------------

    def ping(self, timeout: float = 0.2) -> bytes | None:
        """Type 0 - returns the MCU UID if the motor answers."""
        result: list[bytes | None] = [None]
        done = threading.Event()

        def on_reply(can_id: int, data: bytes) -> None:
            info = proto.decode_device_id(can_id, data)
            if info.motor_id == self.motor_id:
                result[0] = info.uid
                done.set()

        self.link.add_listener(proto.CommType.GET_ID, on_reply)
        try:
            self.link.send(proto.get_device_id(self.motor_id, self.link.host_id))
            done.wait(timeout)
        finally:
            self.link.remove_listener(proto.CommType.GET_ID, on_reply)
        self.uid = result[0]
        return self.uid

    def enable(self) -> None:
        self.link.send(proto.enable(self.motor_id, self.link.host_id))

    def stop(self, clear_fault: bool = False) -> None:
        self.link.send(proto.stop(self.motor_id, clear_fault, self.link.host_id))

    def set_zero(self) -> None:
        """Set current position as mechanical zero.

        Only valid in CSP and operation-control modes; the firmware blocks it
        in PP mode.
        """
        self.link.send(proto.set_zero(self.motor_id, self.link.host_id))

    def save(self) -> None:
        """Type 22 - persist the 0x20xx parameters to flash."""
        self.link.send(proto.save_params(self.motor_id, self.link.host_id))

    def set_active_report(self, enabled: bool) -> None:
        self.link.send(
            proto.set_active_report(self.motor_id, enabled, self.link.host_id))

    def change_can_id(self, new_id: int) -> None:
        self.link.send(proto.set_can_id(self.motor_id, new_id, self.link.host_id))
        self.motor_id = new_id

    # -- parameters -------------------------------------------------------

    def read(self, index: int, timeout: float = 0.25):
        """Read one parameter, returning its engineering value (or ``None``)."""
        reply = self.link.read_param_raw(self.motor_id, index, timeout)
        if reply is None or not reply.ok:
            return None
        param = P.get(index)
        return param.decode(reply.raw) if param else reply.raw

    def write(self, index: int, value) -> None:
        param = P.get(index)
        if param is None:
            raise KeyError(f"Unknown parameter index 0x{index:04X}")
        if not param.writable:
            raise PermissionError(f"{param.name} (0x{index:04X}) is read-only")
        self.link.send(
            proto.param_write(self.motor_id, index, param.encode(value),
                              self.link.host_id))

    def read_many(self, indices, timeout: float = 0.2, gap: float = 0.002) -> dict:
        """Read a batch of parameters sequentially. Missing ones map to ``None``."""
        out = {}
        for index in indices:
            out[index] = self.read(index, timeout)
            if gap:
                time.sleep(gap)
        return out

    # -- control modes ----------------------------------------------------

    def set_run_mode(self, mode: proto.RunMode) -> None:
        """Switch control mode. Do this while the motor is stopped."""
        self.write(0x7005, int(mode))

    def motion_control(self, torque: float = 0.0, position: float = 0.0,
                       velocity: float = 0.0, kp: float = 0.0,
                       kd: float = 0.0) -> None:
        """Type 1 - the 5-parameter operation-control command."""
        self.link.send(proto.motion_control(
            self.motor_id, torque, position, velocity, kp, kd, self.limits))

    def set_current(self, iq: float) -> None:
        self.write(0x7006, iq)

    def set_velocity(self, rad_per_s: float) -> None:
        self.write(0x700A, rad_per_s)

    def set_position(self, rad: float) -> None:
        self.write(0x7016, rad)

    # -- convenience mode entry ------------------------------------------

    def start_current_mode(self, iq: float = 0.0) -> None:
        self.set_run_mode(proto.RunMode.CURRENT)
        self.enable()
        self.set_current(iq)

    def start_velocity_mode(self, speed: float = 0.0, current_limit: float | None = None,
                            accel: float | None = None) -> None:
        self.set_run_mode(proto.RunMode.VELOCITY)
        self.enable()
        if current_limit is not None:
            self.write(0x7018, current_limit)
        if accel is not None:
            self.write(0x7022, accel)
        self.set_velocity(speed)

    def start_csp_mode(self, position: float = 0.0,
                       speed_limit: float | None = None) -> None:
        self.set_run_mode(proto.RunMode.POSITION_CSP)
        self.enable()
        if speed_limit is not None:
            self.write(0x7017, speed_limit)
        self.set_position(position)

    def start_pp_mode(self, position: float = 0.0, vel_max: float | None = None,
                      accel: float | None = None) -> None:
        self.set_run_mode(proto.RunMode.POSITION_PP)
        self.enable()
        if vel_max is not None:
            self.write(0x7024, vel_max)
        if accel is not None:
            self.write(0x7025, accel)
        self.set_position(position)


def scan(link: CanLink, ids=range(1, 128), timeout: float = 0.06,
         progress=None) -> list[tuple[int, bytes]]:
    """Sweep CAN ids with type-0 probes and return the ones that answer."""
    found: list[tuple[int, bytes]] = []
    replies: dict[int, bytes] = {}

    def on_reply(can_id: int, data: bytes) -> None:
        info = proto.decode_device_id(can_id, data)
        replies[info.motor_id] = info.uid

    link.add_listener(proto.CommType.GET_ID, on_reply)
    try:
        for i, motor_id in enumerate(ids):
            link.send(proto.get_device_id(motor_id, link.host_id))
            time.sleep(timeout)
            if progress is not None:
                progress(i, motor_id)
        time.sleep(0.1)
    finally:
        link.remove_listener(proto.CommType.GET_ID, on_reply)

    for motor_id in sorted(replies):
        found.append((motor_id, replies[motor_id]))
    return found
