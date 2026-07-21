"""Guards the joint/gripper motor mapping in the Kinematics panel.

The bug these cover: ``QComboBox.findData`` compares an arbitrary Python
object in a QVariant by identity rather than equality, so an equal-but-
distinct tuple reported "not found" for an entry that was plainly present.
Every inventory refresh then appended a duplicate '(offline)' row instead of
re-selecting the existing one, and a saved mapping never restored. Nothing
caught it because the tests never had a motor in the inventory.
"""

import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")


@pytest.fixture(scope="module")
def qapp():
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    yield app


class FakeState:
    position = 0.0
    age = 0.0


class FakeMotor:
    def __init__(self, motor_id=8):
        self.motor_id = motor_id
        self.state = FakeState()


class ReportingState:
    """A motor reporting continuously, so its feedback never goes stale.

    Not a real MotorState with ``updated`` stamped once: that makes the test
    depend on the wall clock, and a slow run trips the move's staleness
    guard and blocks on a modal dialog. Live hardware with active reporting
    refreshes every 10 ms, which is what this stands in for.
    """

    def __init__(self):
        self.position = 0.0

    @property
    def age(self):
        return 0.0


class DrivableMotor:
    """Records what a move actually puts on the bus."""

    def __init__(self, motor_id):
        self.motor_id = motor_id
        self.state = ReportingState()
        self.writes = []
        self.calls = []
        self._mode = 2                        # velocity, i.e. the wrong one

    def read(self, index, timeout=0.25):
        return self._mode if index == 0x7005 else 0.0

    def write(self, index, value):
        self.writes.append((index, value))
        if index == 0x7005:
            self._mode = int(value)

    def stop(self, clear_fault=False):
        self.calls.append("stop")

    def enable(self):
        self.calls.append("enable")


@pytest.fixture(scope="module")
def arm_urdf():
    """A 3-joint arm, enough to exercise the mapping and the move path."""
    import tempfile
    from pathlib import Path
    urdf = """<?xml version="1.0"?>
    <robot name="testarm">
      <link name="base"/><link name="l1"/><link name="l2"/>
      <link name="l3"/><link name="tip"/>
      <joint name="j1" type="revolute">
        <parent link="base"/><child link="l1"/>
        <origin xyz="0 0 0.1"/><axis xyz="0 0 1"/>
        <limit lower="-3" upper="3"/></joint>
      <joint name="j2" type="revolute">
        <parent link="l1"/><child link="l2"/>
        <origin xyz="0 0 0.2"/><axis xyz="0 1 0"/>
        <limit lower="-2" upper="2"/></joint>
      <joint name="j3" type="revolute">
        <parent link="l2"/><child link="l3"/>
        <origin xyz="0 0 0.2"/><axis xyz="0 1 0"/>
        <limit lower="-2" upper="2"/></joint>
      <joint name="fixed_tip" type="fixed">
        <parent link="l3"/><child link="tip"/>
        <origin xyz="0 0 0.1"/></joint>
    </robot>"""
    folder = tempfile.TemporaryDirectory()
    path = Path(folder.name) / "testarm.urdf"
    path.write_text(urdf, encoding="utf-8")
    yield path
    folder.cleanup()


def test_find_motor_index_matches_a_tuple(qapp):
    from PySide6.QtWidgets import QComboBox
    from gui.kinematics_view import find_motor_index

    box = QComboBox()
    box.addItem("- none -", None)
    box.addItem("can0  id 8", ("can0", 8))

    # Built at run time, so it is equal to the stored key but not the same
    # object - exactly what a config reload or an inventory rebuild yields.
    needle = tuple(["can0", 8])
    assert needle == ("can0", 8) and needle is not box.itemData(1)

    assert find_motor_index(box, needle) == 1
    assert find_motor_index(box, tuple(["can0", 9])) == -1
    assert find_motor_index(box, None) == 0
    # The behaviour this helper exists to work around: identity, not equality.
    assert box.findData(needle) == -1


def test_inventory_refresh_does_not_duplicate_a_mapping(qapp, tmp_path):
    from gui.kinematics_view import KinematicsView

    view = KinematicsView(config_path=tmp_path / "cfg.json")
    try:
        view.motors = {("can0", 8): FakeMotor()}
        view._refresh_motor_boxes()
        view.gripper_motor.setCurrentIndex(1)
        before = view.gripper_motor.count()

        for _ in range(3):
            view._refresh_motor_boxes()

        assert view.gripper_motor.count() == before
        assert view.gripper_motor.currentData() == ("can0", 8)
        assert view._gripper_motor() is not None
    finally:
        view.shutdown()


def test_a_motor_that_goes_offline_keeps_its_mapping(qapp, tmp_path):
    """A closed channel must not silently erase the calibration."""
    from gui.kinematics_view import KinematicsView

    view = KinematicsView(config_path=tmp_path / "cfg.json")
    try:
        view.motors = {("can0", 8): FakeMotor()}
        view._refresh_motor_boxes()
        view.gripper_motor.setCurrentIndex(1)

        view.motors = {}
        view._refresh_motor_boxes()

        assert view.gripper_motor.currentData() == ("can0", 8)
        assert "offline" in view.gripper_motor.currentText()
    finally:
        view.shutdown()


# -- move to target -------------------------------------------------------

def _mapped_view(tmp_path, urdf):
    """A view with a URDF loaded and every joint mapped to a fake motor."""
    from gui.kinematics_view import COL_MOTOR, KinematicsView, find_motor_index

    view = KinematicsView(config_path=tmp_path / "cfg.json")
    view._load_urdf(str(urdf))
    motors = {("can0", i + 1): DrivableMotor(i + 1)
              for i in range(len(view.chain.actuated))}
    view.set_inventory(motors)
    for index, joint in enumerate(view.chain.actuated):
        box = view.table.cellWidget(view._rows[joint.name], COL_MOTOR)
        box.setCurrentIndex(find_motor_index(box, ("can0", index + 1)))
    return view, motors


def test_solving_makes_the_move_button_usable(qapp, tmp_path, arm_urdf):
    """The regression: the button stayed dead after a successful solve.

    It was greyed out until a solution existed, and the line that re-enabled
    it ended up in the wrong method, so pressing it did nothing whatsoever -
    no motion, no dialog, no clue.
    """
    view, _ = _mapped_view(tmp_path, arm_urdf)
    try:
        view._copy_current_pose()
        view.target["z"].setValue(view.target["z"].value() + 50.0)
        view._solve_ik()

        assert view._solution is not None
        assert view.move_button.isEnabled()
    finally:
        view.shutdown()


def test_move_to_target_commands_every_joint(qapp, tmp_path, arm_urdf,
                                             monkeypatch):
    from PySide6.QtWidgets import QMessageBox
    monkeypatch.setattr(QMessageBox, "question",
                        staticmethod(lambda *a, **k: QMessageBox.Yes))

    view, motors = _mapped_view(tmp_path, arm_urdf)
    try:
        view._copy_current_pose()
        view.target["z"].setValue(view.target["z"].value() + 50.0)
        view._solve_ik()
        view._move_to_target()

        for motor in motors.values():
            modes = [v for i, v in motor.writes if i == 0x7005]
            assert modes and modes[0] == 5          # CSP before any setpoint
            assert any(i == 0x7017 for i, _ in motor.writes)   # speed limit
            assert any(i == 0x7016 for i, _ in motor.writes)   # the angle
            assert "enable" in motor.calls
    finally:
        view.shutdown()


def test_move_without_a_solution_says_so(qapp, tmp_path, arm_urdf,
                                         monkeypatch):
    """Never silent: the failure mode that hid the bug must be impossible."""
    from PySide6.QtWidgets import QMessageBox
    told = []
    monkeypatch.setattr(QMessageBox, "information",
                        staticmethod(lambda p, t, m, *a, **k: told.append(t)))

    view, _ = _mapped_view(tmp_path, arm_urdf)
    try:
        view._move_to_target()
        assert told == ["No solution yet"]
    finally:
        view.shutdown()
