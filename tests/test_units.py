"""Guards the degree/radian display layer.

The hazard this covers: the GUI shows degrees but the protocol, the parameter
tables and the manuals are all radians. Every conversion has to be undone
before a value reaches the wire. A leak in either direction is silent - the
motor accepts 90 as happily as 1.5708 and simply goes somewhere else.
"""

import contextlib
import math
import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from robstride import RunMode                      # noqa: E402


@pytest.fixture(scope="module")
def qapp():
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    yield app


@contextlib.contextmanager
def _auto_confirm():
    """Answer the write confirmation dialog with Yes."""
    from PySide6.QtWidgets import QMessageBox
    original = QMessageBox.question
    QMessageBox.question = staticmethod(lambda *a, **k: QMessageBox.Yes)
    try:
        yield
    finally:
        QMessageBox.question = original


@pytest.fixture
def prefs():
    """The preference is process-wide, so always restore it."""
    from gui.units import units
    before = units.degrees
    yield units
    units.set_degrees(before)


# -- the preference object ------------------------------------------------

def test_degrees_are_the_default(prefs):
    assert prefs.degrees is True


def test_only_angular_units_convert(prefs):
    prefs.set_degrees(True)
    assert prefs.factor("rad") == pytest.approx(180.0 / math.pi)
    assert prefs.factor("rad/s") == pytest.approx(180.0 / math.pi)
    assert prefs.factor("rad/s^2") == pytest.approx(180.0 / math.pi)
    # Current, torque, voltage and temperature must pass through untouched.
    for unit in ("A", "Nm", "V", "C", "Hz", "Nm/A", ""):
        assert prefs.factor(unit) == 1.0
        assert prefs.label(unit) == unit


def test_radians_mode_is_a_no_op(prefs):
    prefs.set_degrees(False)
    assert prefs.factor("rad") == 1.0
    assert prefs.label("rad") == "rad"
    assert prefs.to_display(1.5, "rad") == pytest.approx(1.5)


def test_round_trip_is_lossless(prefs):
    prefs.set_degrees(True)
    for unit in ("rad", "rad/s", "rad/s^2", "A"):
        assert prefs.to_canonical(prefs.to_display(1.234, unit), unit) \
            == pytest.approx(1.234)


# -- the spin box ---------------------------------------------------------

def test_spin_reports_radians_while_showing_degrees(qapp, prefs):
    from gui.units import AngleSpin
    prefs.set_degrees(True)
    spin = AngleSpin(-12.57, 12.57, 0.0, 0.1, "rad")

    spin.setValue(90.0)                       # the user types 90 deg
    assert spin.rad() == pytest.approx(math.pi / 2, rel=1e-4)

    spin.setRad(math.pi)                      # the app sets pi rad
    assert spin.value() == pytest.approx(180.0, abs=0.01)


def test_spin_range_follows_the_unit(qapp, prefs):
    from gui.units import AngleSpin
    prefs.set_degrees(True)
    spin = AngleSpin(-12.57, 12.57, 0.0, 0.1, "rad")
    assert spin.maximum() == pytest.approx(720.0, abs=0.5)

    prefs.set_degrees(False)
    assert spin.maximum() == pytest.approx(12.57)


def test_toggling_units_preserves_the_canonical_value(qapp, prefs):
    from gui.units import AngleSpin
    prefs.set_degrees(True)
    spin = AngleSpin(-12.57, 12.57, 0.0, 0.1, "rad")
    spin.setRad(1.234)

    for _ in range(5):                        # flipping must not drift
        prefs.set_degrees(False)
        prefs.set_degrees(True)
    assert spin.rad() == pytest.approx(1.234, rel=1e-6)


def test_non_angular_spin_is_untouched(qapp, prefs):
    from gui.units import AngleSpin
    prefs.set_degrees(True)
    spin = AngleSpin(0, 90, 10.0, 0.5, "A")
    spin.setValue(12.0)
    assert spin.rad() == pytest.approx(12.0)
    assert spin.suffix().strip() == "A"


# -- the control panel ----------------------------------------------------

def test_position_command_reaches_the_wire_in_radians(qapp, prefs):
    """The whole point: 90 on screen must be pi/2 in the 0x7016 write."""
    from gui.control_view import ControlView
    from tests.test_control_flow import FakeMotor

    prefs.set_degrees(True)
    view = ControlView()
    try:
        motor = FakeMotor(run_mode=RunMode.VELOCITY)
        view.motor = motor
        for i in range(view.mode_box.count()):
            if view.mode_box.itemData(i) is RunMode.POSITION_CSP:
                view.mode_box.setCurrentIndex(i)
        view.pos_ref.setValue(90.0)
        view._apply_position()

        loc_ref = motor.writes_to(0x7016)
        assert loc_ref, "loc_ref was never written"
        assert loc_ref[0][2] == pytest.approx(math.pi / 2, rel=1e-4)
    finally:
        view.shutdown()


def test_torque_and_current_are_not_scaled(qapp, prefs):
    """Nm and A share the panel with angles and must survive untouched."""
    from gui.control_view import ControlView
    from tests.test_control_flow import FakeMotor

    prefs.set_degrees(True)
    view = ControlView()
    try:
        motor = FakeMotor(run_mode=RunMode.VELOCITY)
        view.motor = motor
        view.current_ref.setValue(7.5)
        view._apply_current()
        assert motor.writes_to(0x7006)[0][2] == pytest.approx(7.5)
    finally:
        view.shutdown()


# -- the parameter table --------------------------------------------------

def test_param_write_converts_back_to_radians(qapp, prefs):
    from gui.params_view import COL_VALUE, ParamsView
    from tests.test_control_flow import FakeMotor

    prefs.set_degrees(True)
    view = ParamsView()
    try:
        motor = FakeMotor()
        view.motor = motor
        view._populate("RS04")

        # 0x7016 loc_ref is in rad; 0x7006 iq_ref is in A.
        for index, typed, expected in ((0x7016, "90", math.pi / 2),
                                       (0x7006, "7.5", 7.5)):
            item = view.table.item(view._rows[index], COL_VALUE)
            item.setText(typed)
            view._mark_dirty(item, True)

        with _auto_confirm():
            view._write_changed()

        assert motor.writes_to(0x7016)[0][2] == pytest.approx(math.pi / 2, rel=1e-4)
        assert motor.writes_to(0x7006)[0][2] == pytest.approx(7.5)
    finally:
        view.shutdown()


def test_param_display_converts_reads(qapp, prefs):
    from PySide6.QtCore import Qt
    from gui.params_view import COL_UNIT, COL_VALUE, ParamsView

    prefs.set_degrees(True)
    view = ParamsView()
    try:
        view._populate("RS04")
        view._set_value(0x7016, math.pi / 2)
        item = view.table.item(view._rows[0x7016], COL_VALUE)
        assert float(item.text()) == pytest.approx(90.0, abs=0.01)
        # The canonical value stays available for a later unit flip.
        assert item.data(Qt.UserRole) == pytest.approx(math.pi / 2)
        assert view.table.item(view._rows[0x7016], COL_UNIT).text() == "deg"

        prefs.set_degrees(False)
        assert float(item.text()) == pytest.approx(math.pi / 2, rel=1e-4)
        assert view.table.item(view._rows[0x7016], COL_UNIT).text() == "rad"
    finally:
        view.shutdown()
