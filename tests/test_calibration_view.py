"""The calibration wizard: navigation, shared state, and the round trip.

The maths is covered in test_calibration; what matters here is the plumbing.
The wizard borrows its arm model from the Kinematics tab rather than keeping
its own, applies what it identifies back through the same seam, and has to
throw the captured poses away when it does - otherwise a second solve adds
the same correction a second time.
"""

import math
import os

import numpy as np
import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import calibration as cal
import kinematics as kin

ARM = """<?xml version="1.0"?>
<robot name="arm4">
  <link name="base"/><link name="l1"/><link name="l2"/><link name="l3"/>
  <link name="l4"/><link name="tip"/>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="l1"/>
    <origin xyz="0 0 0.10"/><axis xyz="0 0 1"/>
    <limit lower="-2.9" upper="2.9"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="l1"/><child link="l2"/>
    <origin xyz="0 0 0.08"/><axis xyz="0 1 0"/>
    <limit lower="-2.5" upper="2.5"/>
  </joint>
  <joint name="j3" type="revolute">
    <parent link="l2"/><child link="l3"/>
    <origin xyz="0 0 0.30"/><axis xyz="0 0 1"/>
    <limit lower="-2.9" upper="2.9"/>
  </joint>
  <joint name="j4" type="revolute">
    <parent link="l3"/><child link="l4"/>
    <origin xyz="0 0 0.06"/><axis xyz="0 1 0"/>
    <limit lower="-2.5" upper="2.5"/>
  </joint>
  <joint name="tip_joint" type="fixed">
    <parent link="l4"/><child link="tip"/>
    <origin xyz="0.04 0 0.20"/>
  </joint>
</robot>
"""

SEED = np.array([0.25, -0.55, 0.40, 0.85])


@pytest.fixture(scope="module")
def qapp():
    from PySide6.QtWidgets import QApplication
    return QApplication.instance() or QApplication([])


@pytest.fixture
def chain(tmp_path):
    path = tmp_path / "arm4.urdf"
    path.write_text(ARM, encoding="utf-8")
    return kin.Robot.from_urdf(path).chain("tip")


class FakeLink:
    channel = "can0"


class FakeState:
    def __init__(self, position=0.0):
        from robstride import MotorMode
        self.position = position
        self.age = 0.0
        self.mode = MotorMode.RUN
        self.faults = []


class FakeMotor:
    """Enough of a Motor for the wizard: readings, reads, and a model."""

    def __init__(self, motor_id=1, position=0.0):
        self.motor_id = motor_id
        self.model = "RS04"
        self.link = FakeLink()
        self.state = FakeState(position)
        self.written = {}

    @property
    def limits(self):
        from robstride import MODELS
        return MODELS["RS04"]

    def read(self, index, timeout=0.25):
        from gui.calibration_core import MECH_POS
        return self.state.position if index == MECH_POS else 0.0

    def write(self, index, value):
        self.written[index] = value

    def stop(self, clear_fault=False):
        self.written["stop"] = True

    def save(self):
        self.written["save"] = True

    def set_zero(self):
        self.written["zero"] = True


class FakeArm:
    """Stands in for the Kinematics tab's three-method interface."""

    def __init__(self, chain):
        self._chain = chain
        self.motors = [FakeMotor(i + 1) for i in range(len(chain))]
        self.offsets = {joint.name: 0.0 for joint in chain.actuated}
        self.applied = []

    def calibration_chain(self):
        return self._chain

    def joint_map(self):
        from gui.kinematics_view import JointMapping
        return [JointMapping(joint=joint, motor=motor, sign=1.0,
                             offset=self.offsets[joint.name])
                for joint, motor in zip(self._chain.actuated, self.motors)]

    def apply_offsets(self, offsets):
        self.applied.append(dict(offsets))
        for name, value in offsets.items():
            self.offsets[name] += value
        return len(offsets)

    def pose(self, q):
        """Put the fake motors where a joint vector says."""
        for motor, value in zip(self.motors, np.asarray(q, dtype=float)):
            motor.state.position = float(value)


@pytest.fixture
def view(qapp, chain, tmp_path):
    from gui.calibration_view import CalibrationView
    arm = FakeArm(chain)
    widget = CalibrationView(arm=arm, config_path=tmp_path / "cal.json")
    widget.session.motors = {("can0", m.motor_id): m for m in arm.motors}
    yield widget
    widget.shutdown()


# -- the shell -------------------------------------------------------------


def test_every_step_is_present_and_reachable(view):
    from gui.calibration_view import STEPS
    assert len(view.steps) == len(STEPS) == 8
    for index in range(len(STEPS)):
        assert view._index_of(view._row_of(index)) == index


def test_group_headings_are_not_steps(view):
    # Two headings sit among the eight rows and must not select a page.
    assert view.list.count() == 10
    assert view._index_of(0) is None


def test_selecting_a_step_shows_it(view):
    view._select(5)
    assert view.title.text().startswith("6.")
    assert view.pages.currentWidget() is view.steps[5]


def test_next_and_back_stop_at_the_ends(view):
    view._select(0)
    view._step_by(-1)
    assert view._index_of(view.list.currentRow()) == 0
    view._select(7)
    view._step_by(1)
    assert view._index_of(view.list.currentRow()) == 7


def test_the_list_carries_each_step_state(view):
    view._refresh_list()
    states = {view.list.item(view._row_of(i)).data(0x0100)   # Qt.UserRole
              for i in range(len(view.steps))}
    assert states                      # every step answered with something


# -- the session -----------------------------------------------------------


def test_session_reads_the_arm_through_the_kinematics_tab(view, chain):
    view.session.arm.pose(SEED)
    values, missing = view.session.joint_vector()
    assert not missing
    assert np.allclose(values, SEED)


def test_session_applies_sign_and_existing_offset(view, chain):
    view.session.arm.pose(SEED)
    view.session.arm.offsets["j2"] = 0.1
    values, _ = view.session.joint_vector()
    assert values[1] == pytest.approx(SEED[1] + 0.1)


def test_stale_joints_are_reported_not_invented(view):
    view.session.arm.motors[2].state.age = 5.0
    values, missing = view.session.joint_vector()
    assert missing == ["j3"]
    assert values[2] == 0.0


def test_unmapped_joints_block_the_arm_steps(view):
    assert view.session.blocker() == ""
    view.session.arm.motors[1] = None
    assert "j2" in view.session.blocker()


def test_recommended_locks_follow_the_configuration(view):
    session = view.session
    session.point, session.fit_tool = None, True
    assert session.recommend_locks() == {"j1", "j4"}
    session.fit_tool = False
    assert session.recommend_locks() == {"j1"}
    session.point = np.zeros(3)
    assert session.recommend_locks() == set()


def test_session_round_trips_through_its_config(view, tmp_path):
    from gui.calibration_core import Session
    session = view.session
    session.tool = np.array([0.01, -0.02, 0.13])
    session.point = np.array([0.4, 0.1, 0.2])
    session.locked = {"j1", "j4"}
    session.precise = False
    session.snapshots[("can0", 3)] = {"mech_angle_rotat": 2}
    session.save()

    restored = Session(config_path=tmp_path / "cal.json")
    restored.load()
    assert np.allclose(restored.tool, session.tool)
    assert np.allclose(restored.point, session.point)
    assert restored.locked == {"j1", "j4"}
    assert restored.precise is False
    assert restored.snapshots[("can0", 3)]["mech_angle_rotat"] == 2


def test_a_missing_config_is_not_an_error(tmp_path):
    from gui.calibration_core import Session
    session = Session(config_path=tmp_path / "nope.json")
    session.load()
    assert session.point is None


# -- capture and solve -----------------------------------------------------


def capture_poses(view, chain, truth, count=10):
    """Drive the fake arm through poses that all reach one point."""
    poses = cal.candidates(chain, SEED, count=count, attempts=count * 40)
    assert len(poses) >= count
    step = view.steps[6]                      # PosesStep
    for q in poses[:count]:
        view.session.arm.pose(q - truth)
        step._capture()
    return poses[:count]


def test_capture_records_what_the_motors_report(view, chain):
    view.session.locked = {"j1"}
    capture_poses(view, chain, np.zeros(len(chain)), count=4)
    assert len(view.session.poses) == 4


def test_solving_recovers_the_planted_offsets(view, chain):
    truth = np.array([0.0, 0.030, -0.022, 0.018])
    view.session.locked = {"j1"}
    view.session.fit_tool = False
    capture_poses(view, chain, truth, count=10)

    solve = view.steps[7]
    solve._solve()
    fit = view.session.fit
    assert fit is not None
    assert fit.after.rms < fit.before.rms / 50
    assert np.allclose(fit.offsets, truth, atol=5e-4)


def test_applying_hands_the_offsets_to_the_kinematics_tab(view, chain):
    from PySide6.QtWidgets import QMessageBox
    truth = np.array([0.0, 0.025, -0.019, 0.011])
    view.session.locked = {"j1"}
    view.session.fit_tool = False
    capture_poses(view, chain, truth, count=10)
    view.steps[7]._solve()

    received = []
    view.offsets_applied.connect(received.append)
    # The confirmation is modal; the wizard is being driven headless here.
    monkey = QMessageBox.question
    QMessageBox.question = staticmethod(lambda *a, **k: QMessageBox.Yes)
    try:
        view.steps[7]._apply()
    finally:
        QMessageBox.question = monkey

    assert len(received) == 1
    assert "j1" not in received[0]            # locked, so never applied
    assert received[0]["j2"] == pytest.approx(truth[1], abs=5e-4)


def test_the_offsets_reach_the_arm_once_the_signal_is_wired(view, chain):
    """What MainWindow connects, connected here to prove the seam closes."""
    from PySide6.QtWidgets import QMessageBox
    arm = view.session.arm
    view.offsets_applied.connect(arm.apply_offsets)
    truth = np.array([0.0, 0.025, -0.019, 0.011])
    view.session.locked = {"j1"}
    view.session.fit_tool = False
    capture_poses(view, chain, truth, count=10)
    view.steps[7]._solve()

    monkey = QMessageBox.question
    QMessageBox.question = staticmethod(lambda *a, **k: QMessageBox.Yes)
    try:
        view.steps[7]._apply()
    finally:
        QMessageBox.question = monkey

    assert len(arm.applied) == 1
    assert arm.offsets["j2"] == pytest.approx(truth[1], abs=5e-4)
    # And the arm now reads out as calibrated: the joint values the session
    # sees have the correction folded in.
    arm.pose(SEED - truth)
    values, _ = view.session.joint_vector()
    assert np.allclose(values[1:], SEED[1:], atol=5e-4)


def test_applying_clears_the_capture_so_it_cannot_be_applied_twice(view, chain):
    from PySide6.QtWidgets import QMessageBox
    view.session.locked = {"j1"}
    view.session.fit_tool = False
    capture_poses(view, chain, np.array([0.0, 0.02, 0.0, 0.0]), count=8)
    view.steps[7]._solve()

    monkey = QMessageBox.question
    QMessageBox.question = staticmethod(lambda *a, **k: QMessageBox.Yes)
    try:
        view.steps[7]._apply()
    finally:
        QMessageBox.question = monkey

    assert view.session.poses == []
    assert view.session.fit is None


def test_the_report_names_what_was_not_identified(view, chain):
    view.session.locked = set()               # nothing locked on purpose
    view.session.fit_tool = False
    capture_poses(view, chain, np.zeros(len(chain)), count=8)
    view.steps[7]._solve()
    report = view.report()
    assert "not identified" in report
    assert "j1" in report


def test_report_before_solving_says_so(view):
    assert "No calibration" in view.report()


# -- suggestions -----------------------------------------------------------


def test_suggested_poses_reach_the_same_point(view, chain):
    view.session.arm.pose(SEED)
    view.session.locked = {"j1"}
    step = view.steps[6]
    step.count.setValue(4)
    step._generate()
    assert len(view.session.variants) == 4
    target = chain.position(SEED)
    for q in view.session.variants:
        assert np.linalg.norm(chain.position(q) - target) < 1e-3


def test_selecting_a_suggestion_previews_it(view, chain):
    view.session.arm.pose(SEED)
    step = view.steps[6]
    step.count.setValue(3)
    step._generate()
    step._rebuild_variants()
    step.variants.selectRow(1)
    assert step._preview is not None
    assert np.allclose(step._preview, view.session.variants[1])


# -- the steps that only read ----------------------------------------------


def test_map_step_reports_where_the_tip_goes_per_joint(view, chain):
    view.session.arm.pose(SEED)
    step = view.steps[4]
    step.entered()
    assert step.table.rowCount() == len(chain)
    # Every row says something in millimetres about one degree of motor.
    for row in range(len(chain)):
        assert "mm" in step.table.item(row, 5).text()


def test_motor_steps_list_the_inventory(view):
    for index in (0, 1, 2, 3):
        step = view.steps[index]
        step.entered()
        assert step.table.rowCount() == len(view.session.motors)


def test_motors_step_blocks_on_an_uncalibrated_encoder(view):
    from gui.calibration_core import State
    step = view.steps[0]
    step.entered()
    assert step.state()[0] is not State.FAIL
    list(view.session.motors.values())[0].state.faults = ["uncalibrated"]
    assert step.state()[0] is State.FAIL


def test_tool_step_stores_what_is_typed(view):
    step = view.steps[5]
    step.entered()
    step.tool["z"].setValue(120.0)
    step.fit_tool.setChecked(True)
    assert view.session.tool[2] == pytest.approx(0.120)
    assert view.session.fit_tool is True


def test_tool_step_lock_button_locks_the_unidentifiable(view):
    step = view.steps[5]
    step.entered()
    step.fit_tool.setChecked(True)
    step.known.setChecked(False)
    step._recommend()
    assert view.session.locked == {"j1", "j4"}


def test_measured_fixture_enables_its_coordinates(view):
    step = view.steps[5]
    step.entered()
    step.known.setChecked(True)
    step.point["x"].setValue(400.0)
    assert view.session.point is not None
    assert view.session.point[0] == pytest.approx(0.4)
    step.known.setChecked(False)
    assert view.session.point is None


def test_degrees_of_separation_warning_is_in_the_capture_path(view, chain):
    """Capturing the same pose twice must not silently double-count it."""
    from PySide6.QtWidgets import QMessageBox
    view.session.arm.pose(SEED)
    step = view.steps[6]
    step._capture()
    asked = []
    monkey = QMessageBox.question

    def spy(*args, **kwargs):
        asked.append(args[1] if len(args) > 1 else "")
        return QMessageBox.No

    QMessageBox.question = staticmethod(spy)
    try:
        step._capture()
    finally:
        QMessageBox.question = monkey
    assert asked and len(view.session.poses) == 1


def test_units_of_the_scatter_readout(view, chain):
    view.session.locked = {"j1"}
    capture_poses(view, chain, np.array([0.0, 0.05, 0.0, 0.0]), count=5)
    step = view.steps[6]
    step._update_scatter()
    assert "mm RMS" in step.scatter.text()
