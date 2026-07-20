"""Guards the control panel's command sequencing.

The bug these cover: "Go to position" used to write only loc_ref, assuming the
user had already switched mode and enabled. After a jog the motor sits in
velocity mode, where a loc_ref write is silently discarded by the firmware -
no motion, no error. Every apply path must now establish its own mode.
"""

import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from robstride import RunMode                      # noqa: E402
from robstride.models import MODELS                # noqa: E402


class FakeLink:
    channel = "test"
    host_id = 0xFD

    def add_listener(self, *args):
        pass

    def remove_listener(self, *args):
        pass

    def send(self, *args):
        pass


class FakeMotor:
    """Records the command sequence a control action produces."""

    def __init__(self, run_mode=RunMode.VELOCITY):
        self.link = FakeLink()
        self.motor_id = 1
        self.model = "RS04"
        self.limits = MODELS["RS04"]
        self.calls = []
        self._run_mode = int(run_mode)
        from robstride.motor import MotorState
        self.state = MotorState()

    def read(self, index, timeout=0.25):
        self.calls.append(("read", index))
        return self._run_mode if index == 0x7005 else 0.0

    def write(self, index, value):
        self.calls.append(("write", index, value))
        if index == 0x7005:
            self._run_mode = int(value)

    def enable(self):
        self.calls.append(("enable",))

    def stop(self, clear_fault=False):
        self.calls.append(("stop",))

    def set_run_mode(self, mode):
        self.write(0x7005, int(mode))

    def motion_control(self, **kwargs):
        self.calls.append(("motion_control", kwargs))

    def writes_to(self, index):
        return [c for c in self.calls if c[0] == "write" and c[1] == index]


@pytest.fixture(scope="module")
def qapp():
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    yield app


@pytest.fixture
def view(qapp):
    from gui.control_view import ControlView
    widget = ControlView()
    yield widget
    widget.shutdown()


def _select_mode(view, mode):
    for i in range(view.mode_box.count()):
        if view.mode_box.itemData(i) is mode:
            view.mode_box.setCurrentIndex(i)
            return
    raise AssertionError(f"mode {mode} not in the dropdown")


def test_go_to_position_sets_mode_when_motor_is_in_velocity(view):
    """The exact failure: jog left the motor in velocity mode."""
    motor = FakeMotor(run_mode=RunMode.VELOCITY)
    view.motor = motor
    _select_mode(view, RunMode.POSITION_CSP)
    view.pos_ref.setValue(1.5)

    view._apply_position()

    assert motor.writes_to(0x7005), "run_mode was never written"
    assert motor.writes_to(0x7005)[0][2] == int(RunMode.POSITION_CSP)
    assert ("enable",) in motor.calls, "motor was never enabled"

    loc_ref = motor.writes_to(0x7016)
    assert loc_ref, "loc_ref was never written"
    assert loc_ref[0][2] == pytest.approx(1.5)


def test_position_write_comes_after_mode_and_enable(view):
    """Ordering matters: a setpoint written before the switch is discarded."""
    motor = FakeMotor(run_mode=RunMode.VELOCITY)
    view.motor = motor
    _select_mode(view, RunMode.POSITION_CSP)
    view._apply_position()

    order = [c for c in motor.calls
             if c[0] in ("enable", "stop") or (c[0] == "write" and c[1] in (0x7005, 0x7016))]
    kinds = [c[1] if c[0] == "write" else c[0] for c in order]
    assert kinds.index(0x7005) < kinds.index("enable") < kinds.index(0x7016)


def test_pp_and_csp_use_their_own_speed_registers(view):
    """PP limits speed via 0x7024/0x7025; CSP via 0x7017."""
    pp = FakeMotor(run_mode=RunMode.VELOCITY)
    view.motor = pp
    _select_mode(view, RunMode.POSITION_PP)
    view._apply_position()
    assert pp.writes_to(0x7024) and pp.writes_to(0x7025)
    assert not pp.writes_to(0x7017)

    csp = FakeMotor(run_mode=RunMode.VELOCITY)
    view.motor = csp
    _select_mode(view, RunMode.POSITION_CSP)
    view._apply_position()
    assert csp.writes_to(0x7017)
    assert not csp.writes_to(0x7024)


def test_already_in_mode_does_not_stop_the_motor(view):
    """Re-applying a setpoint must not drop and re-grab the load."""
    motor = FakeMotor(run_mode=RunMode.POSITION_CSP)
    view.motor = motor
    _select_mode(view, RunMode.POSITION_CSP)
    view._apply_position()

    assert ("stop",) not in motor.calls
    assert not motor.writes_to(0x7005)
    assert motor.writes_to(0x7016)


def test_speed_and_current_paths_also_set_their_mode(view):
    speed = FakeMotor(run_mode=RunMode.POSITION_CSP)
    view.motor = speed
    view.speed_ref.setValue(1.0)
    view._apply_speed()
    assert speed.writes_to(0x7005)[0][2] == int(RunMode.VELOCITY)
    assert speed.writes_to(0x700A)

    current = FakeMotor(run_mode=RunMode.VELOCITY)
    view.motor = current
    view.current_ref.setValue(2.0)
    view._apply_current()
    assert current.writes_to(0x7005)[0][2] == int(RunMode.CURRENT)
    assert current.writes_to(0x7006)


def test_jog_still_forces_velocity_mode(view):
    motor = FakeMotor(run_mode=RunMode.POSITION_CSP)
    view.motor = motor
    view.jog_speed.setValue(1.0)
    view._jog(1)

    assert motor.writes_to(0x7005)[0][2] == int(RunMode.VELOCITY)
    assert motor.writes_to(0x700A)[0][2] == pytest.approx(1.0)
