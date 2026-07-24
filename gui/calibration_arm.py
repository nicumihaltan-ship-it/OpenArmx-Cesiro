"""Calibration steps that work against the URDF.

The procedure: the arm holds a tool, the tool tip sits in a fixture that does
not move, and the arm is worked into as many different poses as it can reach
without the tip leaving the fixture. Every one of those poses must, if the
model were right, compute the same tip position. The scatter between them is
the error, and the joint offsets that remove it are what this identifies.

Nothing about the fixture has to be measured. That is the whole appeal - see
the module docstring of :mod:`calibration` for what it costs, which is that
a fixed point cannot pin the arm's rotation about its own base.
"""

from __future__ import annotations

import logging
import math
import time

import numpy as np
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QGuiApplication
from PySide6.QtWidgets import (
    QCheckBox, QDoubleSpinBox, QGroupBox, QHBoxLayout, QLabel, QMessageBox,
    QPushButton, QSpinBox, QSplitter, QTableWidgetItem, QVBoxLayout, QWidget,
)

import calibration as cal
from robstride import RunMode

from .calibration_core import (
    ADD_OFFSET, LIMIT_CUR, LIMIT_SPD, LOC_REF, MECH_POS, RUN_MODE,
    State, Step, note, table,
)
from .scene_gl import MEASURED_COLOR, TARGET_COLOR, SceneGL
from .units import units

log = logging.getLogger(__name__)


def _uncertainty(value: float) -> str:
    """Identifiability, phrased so it cannot be mistaken for an offset."""
    if value is None or not np.isfinite(value):
        return "not identified"
    if value > cal.WEAK:
        return f"{value:.1f} deg/mm - weak"
    return f"{value:.3f} deg/mm"


def _spin(minimum, maximum, value=0.0, step=1.0, suffix=" mm", decimals=2):
    box = QDoubleSpinBox()
    box.setRange(minimum, maximum)
    box.setSingleStep(step)
    box.setDecimals(decimals)
    box.setValue(value)
    box.setSuffix(suffix)
    return box


# --------------------------------------------------------------------------
# 5 - the map the fit will use
# --------------------------------------------------------------------------


class MapStep(Step):
    TITLE = "Arm model"
    BLURB = ("The URDF, the chain and the joint-to-motor map this "
             "calibration will be run against.")

    HEADERS = ["URDF joint", "Motor", "Sign", "Motor reading", "Joint value",
               "Tip moves per +1 deg of motor"]

    open_kinematics = Signal()

    def __init__(self, session, parent=None):
        super().__init__(session, parent)
        self.summary = QLabel("-")
        self.summary.setStyleSheet("font-weight: 600;")
        self.table = table(self.HEADERS, stretch=5)

        jump = QPushButton("Open the Kinematics tab")
        jump.clicked.connect(self.open_kinematics)

        layout = QVBoxLayout(self)
        layout.addWidget(note(
            "The map lives on the Kinematics tab and is only shown here. Two "
            "maps for one arm would drift apart, and the one that was wrong "
            "would be whichever you were not looking at."))
        layout.addWidget(self.summary)
        layout.addWidget(self.table, 1)
        layout.addWidget(note(
            "The last column is worth a minute of your time. It is where the "
            "model says the tool tip goes when that motor turns one degree "
            "the positive way. Nudge each joint and watch: if the tip moves "
            "the other way, that joint's <b>sign</b> is wrong, and no offset "
            "will ever fix a wrong sign - the fit will just spread the "
            "contradiction over the other six joints."))
        buttons = QHBoxLayout()
        buttons.addWidget(jump)
        buttons.addStretch(1)
        layout.addLayout(buttons)

    def entered(self) -> None:
        self._rebuild()

    def _rebuild(self) -> None:
        rows = self.session.mapping()
        self.table.setRowCount(len(rows))
        for row, mapping in enumerate(rows):
            self.table.setItem(row, 0, QTableWidgetItem(mapping.name))
            for column in range(1, len(self.HEADERS)):
                self.table.setItem(row, column, QTableWidgetItem("-"))
        self.refresh()

    def refresh(self) -> None:
        rows = self.session.mapping()
        if self.table.rowCount() != len(rows):
            self._rebuild()
            return
        chain = self.session.chain()
        if chain is None:
            self.summary.setText(self.session.blocker())
            return
        blocker = self.session.blocker()
        self.summary.setText(
            blocker or f"{chain.base} -> {chain.tip}, {len(chain)} joints, "
                       f"all mapped")
        self.summary.setStyleSheet(
            "font-weight: 600; color: %s;" % ("#c0392b" if blocker else "black"))

        values, _ = self.session.joint_vector()
        sensitivity = self._sensitivity(chain, values, rows)
        for row, mapping in enumerate(rows):
            motor = mapping.motor
            self.table.item(row, 1).setText(
                "-" if motor is None else
                f"{motor.link.channel}  id {motor.motor_id}")
            self.table.item(row, 2).setText("+" if mapping.sign > 0 else "-")
            self.table.item(row, 3).setText(
                units.text(motor.state.position, "rad", sign=True)
                if mapping.live else "stale")
            item = self.table.item(row, 4)
            item.setText(units.text(values[row], "rad", sign=True))
            item.setForeground(Qt.black if mapping.live else Qt.gray)
            self.table.item(row, 5).setText(sensitivity[row])

    @staticmethod
    def _sensitivity(chain, values, rows) -> list[str]:
        """Tip motion, in mm, for one degree of positive motor rotation."""
        jac = chain.jacobian(values)          # metres of tip per radian
        out = []
        for index, mapping in enumerate(rows):
            move = jac[:, index] * math.radians(1.0) * mapping.sign * 1000.0
            dominant = int(np.argmax(np.abs(move)))
            out.append(f"{move[0]:+.2f}, {move[1]:+.2f}, {move[2]:+.2f} mm  "
                       f"(mostly {'+' if move[dominant] > 0 else '-'}"
                       f"{'XYZ'[dominant]})")
        return out

    def state(self) -> tuple[State, str]:
        blocker = self.session.blocker()
        if blocker:
            return State.FAIL, blocker
        stale = [row.name for row in self.session.mapping() if not row.live]
        if stale:
            return State.WARN, "no fresh feedback from " + ", ".join(stale)
        chain = self.session.chain()
        return State.OK, f"{len(chain)} joints mapped and reporting"


# --------------------------------------------------------------------------
# 6 - the tool and the fixture
# --------------------------------------------------------------------------


class ToolStep(Step):
    TITLE = "Tool and fixture"
    BLURB = ("Where the tool tip is relative to the tip frame, and what is "
             "known about the fixture holding it.")

    def __init__(self, session, parent=None):
        super().__init__(session, parent)
        self._locks: dict[str, QCheckBox] = {}
        self._loading = False

        # -- tool
        self.tool = {axis: _spin(-1000, 1000, 0.0, 1.0)
                     for axis in ("x", "y", "z")}
        self.fit_tool = QCheckBox("Fit the tool tip as well (recommended)")
        self.fit_tool.setToolTip(
            "The tool was made, not measured. Fitting it costs three "
            "parameters and the last joint's offset, and saves you a CMM.")
        tool_row = QHBoxLayout()
        tool_row.addWidget(QLabel("Tool tip in the tip frame"))
        for axis, box in self.tool.items():
            tool_row.addWidget(QLabel(axis.upper()))
            tool_row.addWidget(box)
        tool_row.addWidget(self.fit_tool)
        tool_row.addStretch(1)
        tool_box = QGroupBox("Tool")
        tool_layout = QVBoxLayout(tool_box)
        tool_layout.addLayout(tool_row)
        tool_layout.addWidget(note(
            "Measured off the drawing is fine as a starting value - the fit "
            "refines it. What matters is that the tool is <b>rigid</b> and "
            "that the gripper holds it the same way for the whole session. "
            "A tool that shifts in the jaws between poses is indis-"
            "tinguishable from a joint offset, and the fit will report it as "
            "one."))

        # -- fixture
        self.known = QCheckBox("The fixture position is measured")
        self.known.setToolTip(
            "Leave this off unless you really have measured the fixture into "
            "the URDF base frame. Off is the normal case and costs only the "
            "first joint's offset.")
        self.known.toggled.connect(self._on_known)
        self.point = {axis: _spin(-5000, 5000, 0.0, 1.0)
                      for axis in ("x", "y", "z")}
        point_row = QHBoxLayout()
        point_row.addWidget(self.known)
        for axis, box in self.point.items():
            point_row.addWidget(QLabel(axis.upper()))
            point_row.addWidget(box)
        point_row.addStretch(1)
        fixture_box = QGroupBox("Fixture")
        fixture_layout = QVBoxLayout(fixture_box)
        fixture_layout.addLayout(point_row)
        fixture_layout.addWidget(note(
            "An unmeasured fixture is fitted as three more unknowns, which "
            "works because the procedure only ever needed the tip to be in "
            "the <i>same</i> place, not in a <i>known</i> place. The price "
            "is that any rotation of the whole arm about its base axis moves "
            "the fitted fixture point with it and leaves every residual "
            "untouched - so the first joint's offset cannot be identified. "
            "Measure the fixture and it can."))

        # -- capture precision
        self.precise = QCheckBox(
            "Read mechPos directly at capture (slower, much finer)")
        self.precise.setToolTip(
            "The feedback frame quantises position to 16 bits over "
            "+/-12.57 rad: 0.022 deg a step. Reading 0x7019 per joint costs "
            "a round trip each and gives the float the firmware actually "
            "holds.")

        # -- locks
        self.locks_box = QGroupBox("Offsets to hold at zero")
        self.locks_layout = QVBoxLayout(self.locks_box)
        self.locks_note = note("")
        self.locks_layout.addWidget(self.locks_note)
        recommend = QPushButton("Lock what cannot be identified")
        recommend.clicked.connect(self._recommend)
        self.locks_layout.addWidget(recommend)

        self.verdict = QLabel("-")
        self.verdict.setWordWrap(True)
        self.verdict.setStyleSheet("font-weight: 600;")

        layout = QVBoxLayout(self)
        layout.addWidget(tool_box)
        layout.addWidget(fixture_box)
        layout.addWidget(self.precise)
        layout.addWidget(self.locks_box)
        layout.addWidget(self.verdict)
        layout.addStretch(1)

        for box in (*self.tool.values(), *self.point.values()):
            box.valueChanged.connect(self._store)
        self.fit_tool.toggled.connect(self._store)
        self.known.toggled.connect(self._store)
        self.precise.toggled.connect(self._store)

    def entered(self) -> None:
        self._loading = True
        session = self.session
        for axis, value in zip(("x", "y", "z"), session.tool * 1000.0):
            self.tool[axis].setValue(float(value))
        self.fit_tool.setChecked(session.fit_tool)
        self.known.setChecked(session.point is not None)
        if session.point is not None:
            for axis, value in zip(("x", "y", "z"), session.point * 1000.0):
                self.point[axis].setValue(float(value))
        self.precise.setChecked(session.precise)
        self._build_locks()
        self._loading = False
        self._on_known(self.known.isChecked())
        self.refresh()

    def _build_locks(self) -> None:
        for box in self._locks.values():
            box.setParent(None)
        self._locks = {}
        chain = self.session.chain()
        for joint in (chain.actuated if chain else []):
            box = QCheckBox(joint.name)
            box.setChecked(joint.name in self.session.locked)
            box.toggled.connect(self._store)
            self.locks_layout.addWidget(box)
            self._locks[joint.name] = box
        self.locks_note.setText(
            "A locked offset is left at zero instead of being given whatever "
            "value the damping happened to settle on. Lock the ones the "
            "procedure structurally cannot see - the button below picks "
            "them - so the report says 'not identified' rather than quietly "
            "handing you a number.")

    def _on_known(self, on: bool) -> None:
        for box in self.point.values():
            box.setEnabled(on)

    def _recommend(self) -> None:
        recommended = self.session.recommend_locks()
        for name, box in self._locks.items():
            box.setChecked(name in recommended)
        self._store()

    def _store(self) -> None:
        if self._loading:
            return
        session = self.session
        session.tool = np.array(
            [self.tool[a].value() for a in ("x", "y", "z")]) / 1000.0
        session.fit_tool = self.fit_tool.isChecked()
        session.point = (np.array([self.point[a].value()
                                   for a in ("x", "y", "z")]) / 1000.0
                         if self.known.isChecked() else None)
        session.locked = {name for name, box in self._locks.items()
                          if box.isChecked()}
        session.precise = self.precise.isChecked()
        session.save()
        self.refresh()
        self.changed.emit()

    def refresh(self) -> None:
        chain = self.session.chain()
        if chain is None:
            self.verdict.setText(self.session.blocker())
            return
        recommended = self.session.recommend_locks()
        missing = sorted(recommended - self.session.locked)
        free = len(chain) - len(self.session.locked)
        text = (f"{free} of {len(chain)} joint offsets will be fitted, plus "
                f"{'the tool tip and ' if self.session.fit_tool else ''}"
                f"{'the fixture point' if self.session.point is None else 'nothing else'}.")
        if missing:
            text += ("   Not identifiable in this configuration and still "
                     "unlocked: " + ", ".join(missing))
        self.verdict.setText(text)
        self.verdict.setStyleSheet(
            "font-weight: 600; color: %s;" % ("#e67e22" if missing else "black"))

    def state(self) -> tuple[State, str]:
        if self.session.chain() is None:
            return State.FAIL, "no chain"
        missing = sorted(self.session.recommend_locks() - self.session.locked)
        if missing:
            return State.WARN, "unidentifiable and unlocked: " + ", ".join(missing)
        if not np.any(self.session.tool) and not self.session.fit_tool:
            return State.WARN, "the tool tip is the tip frame and is not fitted"
        return State.OK, "configured"


# --------------------------------------------------------------------------
# 7 - the poses
# --------------------------------------------------------------------------


class PosesStep(Step):
    TITLE = "Poses"
    BLURB = ("Work the arm into as many different configurations as it "
             "reaches with the tool still in the fixture, and capture each.")

    VARIANT_HEADERS = ["#", "Largest joint move", "Joint changes"]
    POSE_HEADERS = ["#", "Tip X", "Tip Y", "Tip Z", "Off centre"]

    def __init__(self, session, parent=None):
        super().__init__(session, parent)
        self._preview: np.ndarray | None = None

        self.count = QSpinBox()
        self.count.setRange(1, 40)
        self.count.setValue(8)
        generate = QPushButton("Suggest poses")
        generate.setToolTip(
            "Null-space walk from where the arm is now, then the subset that "
            "pins the offsets down best")
        generate.clicked.connect(self._generate)
        self.drive_button = QPushButton("Drive to the selected pose")
        self.drive_button.setStyleSheet(
            "background: #c0392b; color: white; font-weight: bold;")
        self.drive_button.clicked.connect(self._drive_to)
        self.speed = QDoubleSpinBox()
        self.speed.setRange(0.02, 1.0)
        self.speed.setValue(0.12)
        self.speed.setDecimals(2)
        self.speed.setSuffix(" rad/s")
        self.current = QDoubleSpinBox()
        self.current.setRange(0.5, 40.0)
        self.current.setValue(4.0)
        self.current.setSuffix(" A")

        self.variants = table(self.VARIANT_HEADERS, stretch=2)
        self.variants.itemSelectionChanged.connect(self._on_variant_selected)
        self.poses = table(self.POSE_HEADERS, stretch=4)

        capture = QPushButton("Capture this pose")
        capture.setStyleSheet("font-weight: 600; padding: 6px;")
        capture.clicked.connect(self._capture)
        drop = QPushButton("Delete selected")
        drop.clicked.connect(self._drop)
        clear = QPushButton("Clear all")
        clear.clicked.connect(self._clear)

        self.gain = QLabel("-")
        self.gain.setStyleSheet("font-weight: 600;")
        self.scatter = QLabel("-")

        self.scene = SceneGL()
        self.scene.set_origin_visible(True)

        top = QHBoxLayout()
        top.addWidget(QLabel("Suggest"))
        top.addWidget(self.count)
        top.addWidget(generate)
        top.addSpacing(16)
        top.addWidget(QLabel("Speed"))
        top.addWidget(self.speed)
        top.addWidget(QLabel("Current"))
        top.addWidget(self.current)
        top.addWidget(self.drive_button)
        top.addStretch(1)

        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.addWidget(QLabel("Suggested poses"))
        left_layout.addWidget(self.variants, 1)
        left_layout.addWidget(QLabel("Captured"))
        left_layout.addWidget(self.poses, 2)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left)
        splitter.addWidget(self.scene)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

        actions = QHBoxLayout()
        actions.addWidget(capture)
        actions.addWidget(drop)
        actions.addWidget(clear)
        actions.addStretch(1)

        layout = QVBoxLayout(self)
        layout.addWidget(note(
            "Put the tool in the fixture, then move the arm without taking "
            "it out. A seven-joint arm holding its tip still has four "
            "degrees of freedom left - the elbow swings right round - and "
            "those are exactly the poses worth capturing. <b>Hand-guiding is "
            "the safe way</b>: stop the motors, or put them in current mode "
            "at zero, and push. Driving to a suggested pose only makes sense "
            "if the fixture is a ball-and-socket that lets the tool pivot; "
            "against a rigid clamp the arm will fight the fixture and one of "
            "them will lose.", "#c0392b"))
        layout.addLayout(top)
        layout.addWidget(splitter, 1)
        layout.addWidget(self.gain)
        layout.addWidget(self.scatter)
        layout.addLayout(actions)

    # -- suggestions ------------------------------------------------------

    def entered(self) -> None:
        self._rebuild_variants()
        self._rebuild_poses()
        chain = self.session.chain()
        if chain is not None:
            values, _ = self.session.joint_vector()
            self.scene.fit(np.array([t[:3, 3]
                                     for _, t in chain.frames(values)]))
            self.scene.set_marker_radius(0.012)

    def _generate(self) -> None:
        chain = self.session.chain()
        if chain is None:
            QMessageBox.information(self, "No chain", self.session.blocker())
            return
        values, missing = self.session.joint_vector()
        if missing:
            QMessageBox.warning(
                self, "Incomplete feedback",
                "These joints are not reporting: " + ", ".join(missing)
                + "\n\nThe suggestions start from where the arm is, so they "
                "would be generated around a pose that is partly invented.")
            return
        self.status.emit("Searching the null space...")
        QGuiApplication.processEvents()
        self.session.variants = cal.variants(
            chain, values, count=self.count.value(),
            tool=self.session.tool, fit_tool=self.session.fit_tool,
            free_point=self.session.point is None,
            locked=tuple(sorted(self.session.locked)))
        self._rebuild_variants()
        self.status.emit(f"{len(self.session.variants)} poses suggested")

    def _rebuild_variants(self) -> None:
        rows = self.session.variants
        self.variants.setRowCount(len(rows))
        chain = self.session.chain()
        names = [joint.name for joint in chain.actuated] if chain else []
        current, _ = self.session.joint_vector() if chain else (np.zeros(0), [])
        for row, q in enumerate(rows):
            delta = q - current if len(current) == len(q) else q
            biggest = float(np.max(np.abs(delta))) if delta.size else 0.0
            detail = ", ".join(
                f"{name} {math.degrees(value):+.0f}"
                for name, value in zip(names, delta)
                if abs(value) > math.radians(2.0))
            self.variants.setItem(row, 0, QTableWidgetItem(str(row + 1)))
            self.variants.setItem(
                row, 1, QTableWidgetItem(f"{math.degrees(biggest):.0f} deg"))
            self.variants.setItem(row, 2, QTableWidgetItem(detail or "-"))

    def _on_variant_selected(self) -> None:
        rows = {item.row() for item in self.variants.selectedItems()}
        chain = self.session.chain()
        if len(rows) != 1 or chain is None:
            self._preview = None
            self.scene.set_preview(None)
            return
        self._preview = self.session.variants[rows.pop()]
        self.scene.set_preview(
            np.array([t[:3, 3] for _, t in chain.frames(self._preview)]))

    # -- capture ----------------------------------------------------------

    def _capture(self) -> None:
        chain = self.session.chain()
        if chain is None:
            QMessageBox.information(self, "No chain", self.session.blocker())
            return
        values, missing = self.session.joint_vector(self.session.precise)
        if missing:
            QMessageBox.warning(
                self, "Incomplete reading",
                "No usable reading from: " + ", ".join(missing)
                + "\n\nA captured pose with an invented joint value in it is "
                "worse than one fewer pose - the fit cannot tell which.")
            return
        near = [i + 1 for i, pose in enumerate(self.session.poses)
                if np.max(np.abs(pose.q - values)) < cal.SEPARATION]
        if near and QMessageBox.question(
                self, "Nearly the same pose",
                f"This is within {math.degrees(cal.SEPARATION):.0f} deg of "
                f"pose {', '.join(str(n) for n in near)} on every joint.\n\n"
                "Near-duplicates add measurement noise but no new "
                "information, and they make the fit look better than it is. "
                "Capture anyway?") != QMessageBox.Yes:
            return
        self.session.poses.append(
            cal.Pose(values, label=f"pose {len(self.session.poses) + 1}"))
        self.session.fit = None
        self._rebuild_poses()
        self.changed.emit()
        self.status.emit(f"Captured {len(self.session.poses)} pose(s)")

    def _drop(self) -> None:
        rows = sorted({item.row() for item in self.poses.selectedItems()},
                      reverse=True)
        for row in rows:
            if 0 <= row < len(self.session.poses):
                del self.session.poses[row]
        if rows:
            self.session.fit = None
            self._rebuild_poses()
            self.changed.emit()

    def _clear(self) -> None:
        self.session.poses = []
        self.session.fit = None
        self._rebuild_poses()
        self.changed.emit()

    def _rebuild_poses(self) -> None:
        poses = self.session.poses
        self.poses.setRowCount(len(poses))
        chain = self.session.chain()
        if chain is None or not poses:
            for row in range(len(poses)):
                for column in range(len(self.POSE_HEADERS)):
                    self.poses.setItem(row, column, QTableWidgetItem("-"))
            self._update_scatter()
            return
        points = cal.tool_points(chain, poses, np.zeros(len(chain)),
                                 self.session.tool)
        result = cal.spread(points, self.session.point)
        for row, point in enumerate(points):
            self.poses.setItem(row, 0, QTableWidgetItem(str(row + 1)))
            for column, value in enumerate(point * 1000.0, start=1):
                self.poses.setItem(row, column,
                                   QTableWidgetItem(f"{value:+.2f}"))
            item = QTableWidgetItem(f"{result.distances[row] * 1000:.2f} mm")
            item.setForeground(
                Qt.red if result.distances[row] > 2 * result.rms else Qt.black)
            self.poses.setItem(row, 4, item)
        self._update_scatter()

    def _update_scatter(self) -> None:
        chain = self.session.chain()
        poses = self.session.poses
        if chain is None or not poses:
            self.scatter.setText("Nothing captured yet.")
            return
        points = cal.tool_points(chain, poses, np.zeros(len(chain)),
                                 self.session.tool)
        result = cal.spread(points, self.session.point)
        self.scatter.setText(
            f"{len(poses)} pose(s). As the model stands they disagree about "
            f"where the tip is by {result.rms * 1000:.2f} mm RMS, "
            f"{result.span * 1000:.2f} mm between the two worst. That "
            f"disagreement is the error being calibrated out.")

    # -- live -------------------------------------------------------------

    def refresh(self) -> None:
        chain = self.session.chain()
        if chain is None:
            self.gain.setText(self.session.blocker())
            return
        values, missing = self.session.joint_vector()
        frames = chain.frames(values)
        self.scene.set_skeleton(np.array([t[:3, 3] for _, t in frames]))
        tip = frames[-1][1]
        self.scene.set_tip(tip[:3, :3] @ self.session.tool + tip[:3, 3])

        points = [(point, MEASURED_COLOR, 9.0) for point in cal.tool_points(
            chain, self.session.poses, np.zeros(len(chain)), self.session.tool)]
        if self.session.point is not None:
            points.append((self.session.point, TARGET_COLOR, 13.0))
        self.scene.set_points(points)

        if missing:
            self.gain.setText("Waiting for feedback from: " + ", ".join(missing))
            self.gain.setStyleSheet("font-weight: 600; color: #c0392b;")
            return
        now = self.session.observability()
        here = self.session.observability(extra=values)
        self.gain.setText(
            "Least-determined joint offset: "
            f"{_uncertainty(now.worst_joint() if now else None)}"
            f"   ->   capturing here: "
            f"{_uncertainty(here.worst_joint() if here else None)}")
        improved = (now is not None and here is not None
                    and here.worst_joint() < now.worst_joint() * 0.95)
        self.gain.setStyleSheet(
            "font-weight: 600; color: %s;" % ("#27ae60" if improved else "gray"))

    # -- driving ----------------------------------------------------------

    def _drive_to(self) -> None:
        if self._preview is None:
            QMessageBox.information(
                self, "No pose selected",
                "Select one of the suggested poses first.")
            return
        rows = self.session.mapping()
        values, missing = self.session.joint_vector()
        if missing:
            QMessageBox.warning(self, "Stale feedback",
                                "No fresh reading from: " + ", ".join(missing))
            return
        deltas = self._preview - values
        detail = "\n".join(
            f"  {row.name}: {math.degrees(delta):+.1f} deg"
            for row, delta in zip(rows, deltas))
        if QMessageBox.question(
                self, "Drive the arm to this pose?",
                "The tool is supposed to be in the fixture. This only works "
                "if the fixture lets the tool pivot - a ball in a socket. "
                "Against a rigid clamp the arm is a closed chain and will "
                "load the fixture until something gives.\n\n"
                f"{detail}\n\nSpeed {self.speed.value():g} rad/s, current "
                f"limited to {self.current.value():g} A. No collision "
                "checking. Is everyone clear?") != QMessageBox.Yes:
            return
        moved = 0
        for row, target in zip(rows, self._preview):
            try:
                self._drive(row.motor, row.command(target))
                moved += 1
            except Exception as exc:
                QMessageBox.critical(
                    self, "Move failed",
                    f"{row.name}: {exc}\n\n{moved} joint(s) were already "
                    "commanded - the arm is part-way there.")
                return
        self.status.emit(f"Commanded {moved} joints")

    def _drive(self, motor, position: float) -> None:
        """CSP with the current limited, mirroring the Control tab."""
        current = motor.read(RUN_MODE, timeout=0.2)
        if current is None or int(current) != int(RunMode.POSITION_CSP):
            motor.stop()
            time.sleep(0.01)
            motor.write(RUN_MODE, int(RunMode.POSITION_CSP))
            time.sleep(0.01)
        motor.enable()
        motor.write(LIMIT_SPD, self.speed.value())
        motor.write(LIMIT_CUR, self.current.value())
        motor.write(LOC_REF, position)

    def state(self) -> tuple[State, str]:
        chain = self.session.chain()
        if chain is None:
            return State.FAIL, "no chain"
        count = len(self.session.poses)
        if not count:
            return State.TODO, "nothing captured"
        report = self.session.observability()
        weak = [name for name in (report.weak() if report else [])
                if name in {j.name for j in chain.actuated}]
        if weak:
            return State.WARN, (f"{count} pose(s); still not identified: "
                                + ", ".join(weak))
        return State.OK, f"{count} pose(s), every free offset identified"


# --------------------------------------------------------------------------
# 8 - the fit
# --------------------------------------------------------------------------


class SolveStep(Step):
    TITLE = "Solve and apply"
    BLURB = "Fit the offsets, see what the data actually determined, apply it."

    OFFSET_HEADERS = ["Joint", "Offset", "Identifiability", "Verdict"]
    POSE_HEADERS = ["#", "Before", "After"]

    offsets_applied = Signal(dict)

    def __init__(self, session, parent=None):
        super().__init__(session, parent)

        solve = QPushButton("Solve")
        solve.setStyleSheet("font-weight: 600; padding: 6px;")
        solve.clicked.connect(self._solve)
        self.apply_button = QPushButton("Apply to the Kinematics tab")
        self.apply_button.clicked.connect(self._apply)
        self.bake_button = QPushButton("Bake into the motors...")
        self.bake_button.setToolTip(
            "Writes add_offset (0x702B) so the motors themselves report the "
            "corrected angle. Optional, riskier, and reversible only if you "
            "do not save to flash.")
        self.bake_button.clicked.connect(self._bake)
        copy = QPushButton("Copy report")
        copy.clicked.connect(self._copy)

        self.summary = QLabel("Not solved yet.")
        self.summary.setWordWrap(True)
        self.summary.setStyleSheet("font-weight: 600;")
        self.extra = QLabel("")
        self.extra.setWordWrap(True)
        self.extra.setStyleSheet("color: gray;")

        self.offsets = table(self.OFFSET_HEADERS, stretch=3)
        self.residuals = table(self.POSE_HEADERS, stretch=2)

        buttons = QHBoxLayout()
        buttons.addWidget(solve)
        buttons.addWidget(self.apply_button)
        buttons.addWidget(self.bake_button)
        buttons.addWidget(copy)
        buttons.addStretch(1)

        tables = QHBoxLayout()
        tables.addWidget(self.offsets, 3)
        tables.addWidget(self.residuals, 1)

        layout = QVBoxLayout(self)
        layout.addLayout(buttons)
        layout.addWidget(self.summary)
        layout.addWidget(self.extra)
        layout.addLayout(tables, 1)
        layout.addWidget(note(
            "Read the identifiability column before the offset column. An "
            "offset the poses did not constrain is not a small number, it is "
            "no number at all, and the only honest thing to do with it is "
            "capture more poses or lock it. A fit that halves the scatter "
            "while leaving three offsets unidentified has told you about "
            "four joints, not seven."))

    def entered(self) -> None:
        self._render()

    # -- solving ----------------------------------------------------------

    def _solve(self) -> None:
        chain = self.session.chain()
        if chain is None or not self.session.poses:
            QMessageBox.information(
                self, "Nothing to solve",
                self.session.blocker()
                or "Capture a few poses on the previous step first.")
            return
        try:
            self.session.fit = cal.solve_fixed_point(
                chain, self.session.poses, **self.session.fit_kwargs())
        except Exception as exc:
            QMessageBox.critical(self, "The fit failed", str(exc))
            return
        self._render()
        self.changed.emit()
        fit = self.session.fit
        self.status.emit(
            f"Scatter {fit.before.rms * 1000:.2f} -> {fit.after.rms * 1000:.2f} mm")

    def _report(self):
        """Observability at the fitted offsets, or at zero if unsolved."""
        chain = self.session.chain()
        if chain is None:
            return None
        return cal.observability(
            chain, self.session.poses,
            offsets=self.session.fit.offsets if self.session.fit else None,
            **self.session.observe_kwargs())

    def _render(self) -> None:
        fit = self.session.fit
        chain = self.session.chain()
        self.apply_button.setEnabled(fit is not None)
        self.bake_button.setEnabled(fit is not None)
        if fit is None or chain is None:
            self.summary.setText("Not solved yet.")
            self.extra.setText("")
            self.offsets.setRowCount(0)
            self.residuals.setRowCount(0)
            return

        report = self._report()
        uncertainty = report.by_name() if report else {}
        self.summary.setText(
            f"Tip scatter {fit.before.rms * 1000:.2f} mm RMS -> "
            f"{fit.after.rms * 1000:.2f} mm over {len(self.session.poses)} "
            f"poses ({fit.improvement:.1f}x better), worst pose "
            f"{fit.after.worst * 1000:.2f} mm, "
            f"{fit.iterations} iterations"
            f"{'' if fit.converged else ' - DID NOT CONVERGE'}.")
        self.summary.setStyleSheet(
            "font-weight: 600; color: %s;"
            % ("#27ae60" if fit.converged and fit.improvement > 2
               else "#e67e22"))

        parts = []
        if self.session.fit_tool:
            parts.append("fitted tool tip "
                         + ", ".join(f"{v * 1000:+.2f}" for v in fit.tool)
                         + " mm")
        parts.append(("fitted" if self.session.point is None else "given")
                     + " fixture point "
                     + ", ".join(f"{v * 1000:+.1f}" for v in fit.point) + " mm")
        self.extra.setText(".   ".join(parts))

        joints = list(chain.actuated)
        self.offsets.setRowCount(len(joints))
        for row, (joint, offset) in enumerate(zip(joints, fit.offsets)):
            locked = joint.name in self.session.locked
            value = uncertainty.get(joint.name)
            self.offsets.setItem(row, 0, QTableWidgetItem(joint.name))
            self.offsets.setItem(row, 1, QTableWidgetItem(
                f"{math.degrees(offset):+.3f} deg"))
            self.offsets.setItem(row, 2, QTableWidgetItem(
                "locked at zero" if locked else _uncertainty(value)))
            if locked:
                verdict, bad = "held at zero, not fitted", False
            elif value is None or not np.isfinite(value):
                verdict, bad = "the poses do not determine this", True
            elif value > cal.WEAK:
                verdict, bad = "barely determined - capture more poses", True
            else:
                verdict, bad = "identified", False
            item = QTableWidgetItem(verdict)
            item.setForeground(Qt.red if bad else Qt.black)
            self.offsets.setItem(row, 3, item)

        self.residuals.setRowCount(len(self.session.poses))
        for row in range(len(self.session.poses)):
            self.residuals.setItem(row, 0, QTableWidgetItem(str(row + 1)))
            self.residuals.setItem(row, 1, QTableWidgetItem(
                f"{fit.before.distances[row] * 1000:.2f} mm"))
            item = QTableWidgetItem(f"{fit.after.distances[row] * 1000:.2f} mm")
            item.setForeground(
                Qt.red if fit.after.distances[row] > 2 * fit.after.rms
                else Qt.black)
            self.residuals.setItem(row, 2, item)

    # -- applying ---------------------------------------------------------

    def _fitted(self) -> dict:
        """Identified offsets only - the locked ones are zero by construction."""
        fit = self.session.fit
        return {name: value for name, value in fit.named().items()
                if name not in self.session.locked}

    def _apply(self) -> None:
        if self.session.fit is None:
            return
        offsets = self._fitted()
        detail = "\n".join(f"  {name}: {math.degrees(value):+.3f} deg"
                           for name, value in offsets.items())
        if QMessageBox.question(
                self, "Apply these offsets?",
                f"Added to the offsets already on the Kinematics tab:\n\n"
                f"{detail}\n\nNothing is written to any motor. The correction "
                "lives in this tool's own configuration, which is the "
                "reversible place for it to live.") != QMessageBox.Yes:
            return
        self.offsets_applied.emit(offsets)
        self.status.emit(f"Applied {len(offsets)} offsets to the Kinematics tab")

    def _copy(self) -> None:
        QGuiApplication.clipboard().setText(self.text_report())
        self.status.emit("Report copied to the clipboard")

    def text_report(self) -> str:
        fit = self.session.fit
        chain = self.session.chain()
        if fit is None or chain is None:
            return "No calibration has been solved."
        report = self._report()
        uncertainty = report.by_name() if report else {}
        lines = [
            "OpenArmX fixed-point calibration",
            f"chain            {chain.base} -> {chain.tip}, "
            f"{len(chain)} joints",
            f"poses            {len(self.session.poses)}",
            "tool tip         "
            + ", ".join(f"{v * 1000:+.2f}" for v in fit.tool) + " mm"
            + ("  (fitted)" if self.session.fit_tool else "  (given)"),
            "fixture point    "
            + ", ".join(f"{v * 1000:+.1f}" for v in fit.point) + " mm"
            + ("  (fitted)" if self.session.point is None else "  (measured)"),
            f"tip scatter      {fit.before.rms * 1000:.3f} mm RMS -> "
            f"{fit.after.rms * 1000:.3f} mm",
            f"worst pose       {fit.before.worst * 1000:.3f} mm -> "
            f"{fit.after.worst * 1000:.3f} mm",
            f"converged        {fit.converged} after {fit.iterations} iterations",
            "",
            f"{'joint':<20}{'offset (deg)':>14}{'identifiability':>20}",
        ]
        for joint, offset in zip(chain.actuated, fit.offsets):
            locked = joint.name in self.session.locked
            lines.append(
                f"{joint.name:<20}{math.degrees(offset):>+14.4f}"
                f"{'locked' if locked else _uncertainty(uncertainty.get(joint.name)):>20}")
        lines += ["", f"{'pose':<8}{'before (mm)':>14}{'after (mm)':>14}"]
        for row in range(len(self.session.poses)):
            lines.append(f"{row + 1:<8}{fit.before.distances[row] * 1000:>14.3f}"
                         f"{fit.after.distances[row] * 1000:>14.3f}")
        return "\n".join(lines)

    # -- baking into the motors -------------------------------------------

    def _bake(self) -> None:
        """Push the offsets into add_offset, and check they landed.

        0x702B is documented as "zero offset" and nothing states which way it
        shifts the reported angle, so this measures it rather than assuming:
        write, read mechPos back, and compare against what was expected. A
        register whose sign convention has been checked on the bench is worth
        more than one taken on trust from a translated table.
        """
        fit = self.session.fit
        if fit is None:
            return
        rows = {row.name: row for row in self.session.mapping()}
        wanted = {name: value for name, value in self._fitted().items()
                  if name in rows and rows[name].motor is not None}
        if not wanted:
            QMessageBox.information(self, "Nothing to bake",
                                    "No identified offset maps to a motor.")
            return
        if QMessageBox.question(
                self, "Write add_offset on these motors?",
                f"{len(wanted)} motor(s) will be STOPPED, then have "
                "add_offset (0x702B) written so they report the corrected "
                "angle themselves.\n\nThe register's sign convention is not "
                "stated in any manual, so each write is read back and "
                "measured; you will be shown what actually happened and can "
                "undo it.\n\nThe write is volatile - nothing survives a "
                "power cycle unless you save to flash afterwards. The arm "
                "will be left disabled.") != QMessageBox.Yes:
            return

        original, results = {}, []
        for name, offset in wanted.items():
            row = rows[name]
            motor = row.motor
            try:
                motor.stop()
                before_offset = motor.read(ADD_OFFSET, timeout=0.3) or 0.0
                before_pos = motor.read(MECH_POS, timeout=0.3)
                # joint = sign * motor + offset, so the motor has to report
                # offset/sign more for the correction to fold into it.
                expected = offset / row.sign
                motor.write(ADD_OFFSET, float(before_offset) + expected)
                time.sleep(0.2)
                after_pos = motor.read(MECH_POS, timeout=0.3)
                original[name] = (motor, float(before_offset))
                observed = (None if before_pos is None or after_pos is None
                            else float(after_pos) - float(before_pos))
                results.append((name, expected, observed))
            except Exception:
                results.append((name, float("nan"), None))
                log.exception("baking %s failed", name)

        detail = "\n".join(
            f"  {name}: expected {math.degrees(expected):+.3f} deg, "
            + ("no readback" if observed is None
               else f"measured {math.degrees(observed):+.3f} deg")
            for name, expected, observed in results)
        agree = [observed is not None and expected != 0
                 and abs(observed - expected) < abs(expected) * 0.2
                 for _, expected, observed in results]
        inverted = [observed is not None and expected != 0
                    and abs(observed + expected) < abs(expected) * 0.2
                    for _, expected, observed in results]

        if all(agree) and agree:
            verdict = ("Every motor moved its reported angle by what was "
                       "asked. Keep it?")
        elif all(inverted) and inverted:
            verdict = ("Every motor shifted by the OPPOSITE of what was "
                       "asked - add_offset subtracts on this firmware. "
                       "Undo it? (then re-run: the sign is now known)")
        else:
            verdict = ("The readback does not match what was written. Undo "
                       "and keep the correction on the Kinematics tab "
                       "instead, where it is known to work.")

        keep = QMessageBox.question(
            self, "What the motors actually did",
            f"{detail}\n\n{verdict}",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.Yes if all(agree) and agree else QMessageBox.No)

        if keep == QMessageBox.Yes and all(agree) and agree:
            self.status.emit(
                f"add_offset written on {len(results)} motor(s) - save to "
                "flash on the 'Zero and range' step to make it survive a "
                "power cycle")
            return
        for name, (motor, value) in original.items():
            try:
                motor.write(ADD_OFFSET, value)
            except Exception:
                log.exception("could not restore add_offset on %s", name)
        self.status.emit("add_offset restored on every motor")

    def state(self) -> tuple[State, str]:
        fit = self.session.fit
        if fit is None:
            return State.TODO, "not solved"
        report = self._report()
        names = {joint.name for joint in self.session.chain().actuated}
        weak = [name for name in (report.weak() if report else [])
                if name in names and name not in self.session.locked]
        if weak:
            return State.WARN, "not identified: " + ", ".join(weak)
        if not fit.converged:
            return State.WARN, "the fit did not converge"
        return State.OK, (f"{fit.before.rms * 1000:.2f} -> "
                          f"{fit.after.rms * 1000:.2f} mm RMS")
