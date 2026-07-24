"""Forward kinematics and joint-offset calibration.

Answers "where is the tool tip?" from the angles the motors are reporting,
and turns that into a calibration procedure: pose the arm, measure where the
tip really is, and let the solver work out the joint zero offsets that
reconcile the two.

The URDF is never bundled - OpenArmX's description is CC BY-NC-SA - so the
path to a local copy, along with the joint-to-motor map, lives in a config
file under the user's own application data.
"""

from __future__ import annotations

import json
import logging
import math
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from PySide6.QtCore import QStandardPaths, Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QAbstractItemView, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog,
    QGroupBox, QHBoxLayout, QHeaderView, QLabel, QLineEdit, QMessageBox,
    QPushButton, QSplitter, QTableWidget, QTableWidgetItem, QVBoxLayout,
    QWidget,
)

import kinematics as kin
from robstride import RunMode

from .scene_gl import (
    MEASURED_COLOR, SKELETON_COLOR, TARGET_COLOR, TCP_COLOR, SceneGL,
)
from .units import AngleSpin, units

log = logging.getLogger(__name__)

COL_JOINT, COL_MOTOR, COL_SIGN, COL_READING, COL_OFFSET, COL_VALUE, \
    COL_LIMIT = range(7)

HEADERS = ["URDF joint", "Motor", "Sign", "Motor reading", "Offset",
           "Joint value", "URDF limit"]

#: Leaf-link names that look like a tool frame, best first.
TIP_HINTS = ("tcp", "tool", "_ee", "grasp", "hand", "gripper")


def _ranked_tips(robot: kin.Robot) -> list[str]:
    """Leaf links, tool-looking ones first.

    A URDF's leaves include stubs and fingers - OpenArmX's ``link4_ext`` is
    a mounting boss - and the first one in file order is rarely the frame
    anybody wants to measure. Ranking beats making the user hunt.
    """
    def rank(link: str) -> tuple:
        lowered = link.lower()
        for position, hint in enumerate(TIP_HINTS):
            if hint in lowered:
                return (0, position, link)
        return (1, 0, link)

    return sorted(robot.tips(), key=rank)


def find_motor_index(box: QComboBox, key) -> int:
    """Index of the combo entry carrying ``key``, or -1.

    Not ``QComboBox.findData``: for an arbitrary Python object in a QVariant
    it compares by **identity**, not equality, so an equal-but-distinct
    tuple reports "not found" for an entry that is plainly there. Every
    motor key here is rebuilt - from a config file, from the inventory dict
    - and is therefore never the same object twice.

    The consequence was silent: each inventory refresh appended another
    '(offline)' duplicate instead of re-selecting the existing row, and a
    saved mapping never restored.
    """
    for index in range(box.count()):
        if box.itemData(index) == key:
            return index
    return -1


def _config_path() -> Path:
    base = QStandardPaths.writableLocation(QStandardPaths.AppConfigLocation)
    return Path(base or ".") / "openarmx_kinematics.json"


@dataclass
class JointMapping:
    """One row of the joint table, as the calibration tab sees it."""

    joint: object                       # kin.Joint
    motor: object | None                # robstride.Motor, or None if unmapped
    sign: float
    offset: float                       # radians

    @property
    def name(self) -> str:
        return self.joint.name

    @property
    def live(self) -> bool:
        return self.motor is not None and self.motor.state.age <= 1.0

    def value(self) -> float:
        """The URDF joint value this motor is currently reporting."""
        if self.motor is None:
            return 0.0
        return self.sign * self.motor.state.position + self.offset

    def command(self, value: float) -> float:
        """The motor angle that would put this joint at ``value``."""
        return (value - self.offset) / self.sign


class KinematicsView(QWidget):
    """Live tool-tip pose plus the offset-identification workflow."""

    status = Signal(str)

    def __init__(self, parent=None, config_path: Path | None = None):
        super().__init__(parent)
        # Injectable so tests never read or overwrite the real user config -
        # shutdown() saves, so an un-isolated test run silently rewrites
        # whatever mapping the operator had built up.
        self._config_path = Path(config_path) if config_path else _config_path()
        self.robot: kin.Robot | None = None
        self.chain: kin.Chain | None = None
        self.motors: dict[tuple[str, int], object] = {}
        self.samples: list[kin.Sample] = []
        self._rows: dict[str, int] = {}
        self._loading = False
        self._meshes = kin.MeshCache()
        self._rescaled: list[str] = []
        self._solution: kin.IKResult | None = None
        self._relaxed: kin.IKResult | None = None
        self.gripper: kin.Gripper | None = None

        self._build_ui()
        self._load_config()

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(100)

    # -- construction -----------------------------------------------------

    def _build_ui(self) -> None:
        self.urdf_path = QLineEdit()
        self.urdf_path.setPlaceholderText(
            "Path to a URDF describing the arms...")
        self.urdf_path.setReadOnly(True)
        browse = QPushButton("Browse")
        browse.clicked.connect(self._browse)
        reload_button = QPushButton("Reload")
        reload_button.clicked.connect(lambda: self._load_urdf(
            self.urdf_path.text(), remember_chain=True))

        self.chain_box = QComboBox()
        self.chain_box.setMinimumWidth(240)
        self.chain_box.currentIndexChanged.connect(self._on_chain_changed)

        top = QHBoxLayout()
        top.addWidget(QLabel("URDF"))
        top.addWidget(self.urdf_path, 3)
        top.addWidget(browse)
        top.addWidget(reload_button)
        top.addSpacing(16)
        top.addWidget(QLabel("Tip frame"))
        top.addWidget(self.chain_box, 2)

        # -- meshes
        self.mesh_root = QLineEdit()
        self.mesh_root.setPlaceholderText(
            "Folder holding the STL meshes - optional if they sit beside the "
            "URDF")
        self.mesh_root.setReadOnly(True)
        self.mesh_root.setToolTip(
            "URDF meshes are referenced as package:// URIs, which only "
            "resolve inside a ROS workspace. Point this at the folder the "
            "meshes actually live in; sub-folders are searched too.")
        mesh_browse = QPushButton("Browse")
        mesh_browse.clicked.connect(self._browse_meshes)
        mesh_clear = QPushButton("Clear")
        mesh_clear.clicked.connect(lambda: (self.mesh_root.clear(),
                                            self._rebuild_meshes(),
                                            self._save_config()))
        self.mesh_label = QLabel("no meshes loaded")
        self.mesh_label.setStyleSheet("color: gray;")

        mesh_row = QHBoxLayout()
        mesh_row.addWidget(QLabel("Meshes"))
        mesh_row.addWidget(self.mesh_root, 3)
        mesh_row.addWidget(mesh_browse)
        mesh_row.addWidget(mesh_clear)
        mesh_row.addWidget(self.mesh_label, 2)
        self.show_components = QCheckBox("XYZ legs")
        self.show_components.setChecked(True)
        self.show_components.setToolTip(
            "Draw the tip's X, Y and Z components as three orthogonal legs "
            "from the origin, so its position can be read off the scene")
        mesh_row.addWidget(self.show_components)
        fit_view = QPushButton("Fit view")
        fit_view.clicked.connect(lambda: (self.scene.reset_view(),
                                          self._fit_view()))
        mesh_row.addWidget(fit_view)

        # -- joints
        self.table = QTableWidget(0, len(HEADERS))
        self.table.setHorizontalHeaderLabels(HEADERS)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.verticalHeader().setVisible(False)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(COL_JOINT, QHeaderView.Stretch)
        for col in (COL_MOTOR, COL_SIGN, COL_READING, COL_OFFSET, COL_VALUE,
                    COL_LIMIT):
            header.setSectionResizeMode(col, QHeaderView.ResizeToContents)

        # -- gripper (a branch off the chain, not part of the arm's IK)
        self.gripper_motor = QComboBox()
        self.gripper_motor.currentIndexChanged.connect(self._save_config)
        self.gripper_sign = QComboBox()
        self.gripper_sign.addItem("+", 1.0)
        self.gripper_sign.addItem("-", -1.0)
        self.gripper_sign.currentIndexChanged.connect(self._save_config)

        self.gripper_ratio = QDoubleSpinBox()
        self.gripper_ratio.setRange(0.01, 500.0)
        self.gripper_ratio.setDecimals(3)
        self.gripper_ratio.setSingleStep(0.1)
        self.gripper_ratio.setValue(7.0)
        self.gripper_ratio.setSuffix(" mm/rad")
        self.gripper_ratio.setToolTip(
            "Finger stroke per radian of motor. The URDF describes the "
            "fingers but not the transmission that drives them, so this one "
            "cannot be derived - measure it and type it in. The default "
            "assumes full stroke over one motor revolution.")
        self.gripper_ratio.valueChanged.connect(self._save_config)

        self.gripper_command = QDoubleSpinBox()
        self.gripper_command.setRange(0.0, 1000.0)
        self.gripper_command.setDecimals(1)
        self.gripper_command.setSingleStep(1.0)
        self.gripper_command.setSuffix(" mm")
        self.gripper_command.setToolTip("Stroke to drive the fingers to")
        self.gripper_apply = QPushButton("Apply")
        self.gripper_apply.clicked.connect(self._apply_gripper)

        self.gripper_label = QLabel("-")
        self.gripper_label.setStyleSheet("font-weight: 600;")

        self.gripper_box = QGroupBox("Gripper")
        gripper_row = QHBoxLayout(self.gripper_box)
        gripper_row.addWidget(QLabel("Motor"))
        gripper_row.addWidget(self.gripper_motor)
        gripper_row.addWidget(self.gripper_sign)
        gripper_row.addWidget(self.gripper_ratio)
        gripper_row.addWidget(self.gripper_label, 2)
        gripper_row.addWidget(QLabel("Go to"))
        gripper_row.addWidget(self.gripper_command)
        gripper_row.addWidget(self.gripper_apply)

        self.report_button = QPushButton("Enable active reporting on mapped motors")
        self.report_button.setToolTip(
            "The joint values come from feedback frames. Without active "
            "reporting the motors only answer when polled, and the readings "
            "here go stale.")
        self.report_button.clicked.connect(self._enable_reporting)

        # -- tip pose
        self.pose_labels: dict[str, QLabel] = {}
        pose_box = QGroupBox("Tool tip, relative to the URDF base")
        pose_grid = QHBoxLayout(pose_box)
        for key, title in [("x", "X"), ("y", "Y"), ("z", "Z"),
                           ("roll", "roll"), ("pitch", "pitch"),
                           ("yaw", "yaw")]:
            cell = QVBoxLayout()
            caption = QLabel(title)
            caption.setStyleSheet("color: gray;")
            value = QLabel("-")
            value.setStyleSheet("font-size: 15px; font-weight: 600;")
            cell.addWidget(caption)
            cell.addWidget(value)
            pose_grid.addLayout(cell)
            self.pose_labels[key] = value

        self.scene = SceneGL()

        # -- calibration
        self.measured = {}
        measure_row = QHBoxLayout()
        measure_row.addWidget(QLabel("Measured tip"))
        for axis in ("x", "y", "z"):
            spin = QDoubleSpinBox()
            spin.setRange(-5000, 5000)
            spin.setDecimals(1)
            spin.setSingleStep(1.0)
            spin.setSuffix(" mm")
            spin.setToolTip(f"Measured {axis.upper()} of the tip, in the "
                            "URDF base frame")
            measure_row.addWidget(QLabel(axis.upper()))
            measure_row.addWidget(spin)
            self.measured[axis] = spin

        self.capture_button = QPushButton("Capture sample")
        self.capture_button.setToolTip(
            "Records the current joint readings against the measured tip "
            "position typed to the left.")
        self.capture_button.clicked.connect(self._capture)
        self.solve_button = QPushButton("Solve offsets")
        self.solve_button.clicked.connect(self._solve)
        self.clear_button = QPushButton("Clear samples")
        self.clear_button.clicked.connect(self._clear_samples)
        measure_row.addSpacing(12)
        measure_row.addWidget(self.capture_button)
        measure_row.addWidget(self.solve_button)
        measure_row.addWidget(self.clear_button)
        measure_row.addStretch(1)

        self.fit_label = QLabel("No samples captured")
        self.fit_label.setStyleSheet("color: gray;")

        calib_box = QGroupBox("Offset calibration")
        calib_layout = QVBoxLayout(calib_box)
        calib_layout.addLayout(measure_row)
        calib_layout.addWidget(self.fit_label)

        # -- inverse kinematics
        self.target = {}
        target_row = QHBoxLayout()
        target_row.addWidget(QLabel("Target"))
        for axis in ("x", "y", "z"):
            spin = QDoubleSpinBox()
            spin.setRange(-5000, 5000)
            spin.setDecimals(1)
            spin.setSingleStep(10.0)
            spin.setSuffix(" mm")
            target_row.addWidget(QLabel(axis.upper()))
            target_row.addWidget(spin)
            self.target[axis] = spin
        for angle in ("roll", "pitch", "yaw"):
            spin = QDoubleSpinBox()
            spin.setRange(-180.0, 180.0)
            spin.setDecimals(1)
            spin.setSingleStep(5.0)
            spin.setSuffix(" deg")
            spin.setWrapping(True)
            target_row.addWidget(QLabel(angle))
            target_row.addWidget(spin)
            self.target[angle] = spin

        copy_current = QPushButton("Copy current")
        copy_current.setToolTip("Fill the target with the tip pose right now")
        copy_current.clicked.connect(self._copy_current_pose)
        self.solve_ik_button = QPushButton("Solve IK")
        self.solve_ik_button.clicked.connect(self._solve_ik)
        self.relax_button = QPushButton("Relax orientation")
        self.relax_button.setToolTip(
            "Replace the requested orientation with one the arm can actually "
            "hold at that point, and solve again")
        self.relax_button.clicked.connect(self._relax_orientation)
        self.relax_button.setEnabled(False)
        self.clear_preview_button = QPushButton("Clear preview")
        self.clear_preview_button.clicked.connect(self._clear_preview)
        target_row.addSpacing(10)
        target_row.addWidget(copy_current)
        target_row.addWidget(self.solve_ik_button)
        target_row.addWidget(self.relax_button)
        target_row.addWidget(self.clear_preview_button)
        target_row.addStretch(1)

        self.ik_label = QLabel("No solution")
        self.ik_label.setStyleSheet("color: gray;")

        self.move_speed = AngleSpin(0.02, 3.0, 0.20, 0.05, "rad/s")
        self.move_speed.setToolTip(
            "Speed limit written to every joint before the move (0x7017)")
        self.move_button = QPushButton("MOVE TO TARGET")
        self.move_button.setStyleSheet(
            "background: #c0392b; color: white; font-weight: bold; padding: 6px;")
        self.move_button.setToolTip(
            "Drives every mapped joint to the solved pose. There is no "
            "collision checking of any kind.")
        self.move_button.clicked.connect(self._move_to_target)

        warning = QLabel(
            "No collision checking — the arm will take whatever path the "
            "joints happen to sweep. Check the preview, keep clear, and keep "
            "STOP in reach.")
        warning.setStyleSheet("color: #c0392b;")
        warning.setWordWrap(True)

        move_row = QHBoxLayout()
        move_row.addWidget(self.ik_label, 3)
        move_row.addWidget(QLabel("Speed limit"))
        move_row.addWidget(self.move_speed)
        move_row.addWidget(self.move_button)

        ik_box = QGroupBox("Move to a point (inverse kinematics)")
        ik_layout = QVBoxLayout(ik_box)
        ik_layout.addLayout(target_row)
        ik_layout.addLayout(move_row)
        ik_layout.addWidget(warning)

        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.addWidget(self.table, 1)
        left_layout.addWidget(self.gripper_box)
        left_layout.addWidget(self.report_button)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left)
        splitter.addWidget(self.scene)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

        self.info = QLabel("Load a URDF to begin")
        self.info.setStyleSheet("color: gray;")

        layout = QVBoxLayout(self)
        layout.addLayout(top)
        layout.addLayout(mesh_row)
        layout.addWidget(self.info)
        layout.addWidget(splitter, 1)
        layout.addWidget(pose_box)
        layout.addWidget(ik_box)
        layout.addWidget(calib_box)

    # -- URDF -------------------------------------------------------------

    def _browse(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Open URDF", self.urdf_path.text(), "URDF (*.urdf *.xml)")
        if path:
            self._load_urdf(path)

    def _load_urdf(self, path: str, remember_chain: bool = False) -> None:
        if not path:
            return
        wanted = self.chain_box.currentText() if remember_chain else None
        try:
            robot = kin.Robot.from_urdf(path)
        except Exception as exc:
            QMessageBox.critical(self, "Could not read the URDF", str(exc))
            return
        self.robot = robot
        self.urdf_path.setText(path)

        self._loading = True
        self.chain_box.clear()
        for tip in _ranked_tips(robot):
            self.chain_box.addItem(tip)
        self._loading = False

        if wanted and self.chain_box.findText(wanted) >= 0:
            self.chain_box.setCurrentText(wanted)
        else:
            self.chain_box.setCurrentIndex(0 if self.chain_box.count() else -1)
        self._on_chain_changed()
        self.status.emit(
            f"Loaded {robot.name}: {len(robot.links)} links, "
            f"{len(robot.tips())} tip frames")

    def _on_chain_changed(self) -> None:
        if self._loading or self.robot is None:
            return
        tip = self.chain_box.currentText()
        if not tip:
            return
        try:
            self.chain = self.robot.chain(tip)
        except Exception as exc:
            self.info.setText(f"{tip}: {exc}")
            self.chain = None
            return
        self.gripper = self.robot.gripper(self.chain)
        self.gripper_box.setEnabled(self.gripper is not None)
        if self.gripper is not None:
            self.gripper_command.setRange(self.gripper.lower * 1000.0,
                                          self.gripper.upper * 1000.0)
        self._clear_samples()
        self._clear_preview()
        self._build_rows()
        self._rebuild_meshes()
        self.info.setText(
            f"{self.robot.name}: {self.chain.base} -> {tip}, "
            f"{len(self.chain)} actuated joints")

    # -- meshes -----------------------------------------------------------

    def _browse_meshes(self) -> None:
        folder = QFileDialog.getExistingDirectory(
            self, "Folder containing the meshes", self.mesh_root.text())
        if folder:
            self.mesh_root.setText(folder)
            self._rebuild_meshes()
            self._save_config()

    def _mesh_roots(self) -> list[Path]:
        """Where to look for a mesh, most specific first.

        The URDF's own ancestors are always searched, so a description laid
        out the usual way - ``<pkg>/urdf/robot/x.urdf`` beside
        ``<pkg>/meshes/`` - needs no folder configured at all.
        """
        roots: list[Path] = []
        chosen = self.mesh_root.text().strip()
        if chosen:
            roots.append(Path(chosen))
        urdf = self.urdf_path.text().strip()
        if urdf:
            parent = Path(urdf).parent
            roots += [parent, *list(parent.parents)[:3]]
        seen, unique = set(), []
        for root in roots:
            if root not in seen:
                seen.add(root)
                unique.append(root)
        return unique

    def _rebuild_meshes(self) -> None:
        self.scene.clear_meshes()
        self._meshes.clear()
        if self.robot is None or self.chain is None:
            self.mesh_label.setText("no meshes loaded")
            return

        roots = self._mesh_roots()
        reference = self._reach()
        triangles = 0
        rescaled: list[str] = []
        wanted = [name for name, _ in self.chain.frames(np.zeros(len(self.chain)))]
        if self.gripper is not None:
            wanted += [joint.child for joint in self.gripper.joints]
        for name in wanted:
            link = self.robot.links.get(name)
            if link is None:
                continue
            for geometry in link.geometry("collision"):
                mesh = self._meshes.triangles(geometry.mesh, roots)
                if mesh is None or not len(mesh):
                    continue
                scale = geometry.scale
                correction = kin.unit_correction(mesh, scale, reference)
                if correction is not None:
                    scale = scale * correction
                    rescaled.append(name)
                    log.warning("%s: mesh %s looks like millimetres but the "
                                "URDF declares scale %s - applying %g",
                                name, geometry.mesh, geometry.scale, correction)
                self.scene.add_mesh(name, mesh, geometry.origin, scale)
                triangles += len(mesh)

        self._rescaled = rescaled
        missing = len(self._meshes.missing)
        if not self.scene.mesh_count:
            self.mesh_label.setText(
                f"no meshes found ({missing} unresolved)" if missing
                else "no meshes referenced")
            self.mesh_label.setStyleSheet("color: #e67e22;")
            return
        note = f"{self.scene.mesh_count} meshes, {triangles:,} triangles"
        if missing:
            # DAE visuals are the usual cause: only STL is read here.
            note += f"   [{missing} unresolved]"
        if rescaled:
            note += (f"   [rescaled mm->m: {', '.join(sorted(set(rescaled)))}]")
        self.mesh_label.setText(note)
        self.mesh_label.setStyleSheet(
            "color: #e67e22;" if (missing or rescaled) else "color: gray;")
        self._fit_view()

    def _reach(self) -> float:
        """Rough kinematic extent of the chain, used as a sanity reference."""
        if self.chain is None:
            return 0.0
        spans = []
        for pose in (np.zeros(len(self.chain)), self._mid_pose()):
            origins = np.array([t[:3, 3] for _, t in self.chain.frames(pose)])
            spans.append(np.linalg.norm(origins.max(axis=0)
                                        - origins.min(axis=0)))
        return float(max(spans))

    def _mid_pose(self) -> np.ndarray:
        return np.array([
            0.0 if joint.lower is None or joint.upper is None
            else (joint.lower + joint.upper) / 2.0
            for joint in self.chain.actuated])

    def _fit_view(self) -> None:
        """Frame the camera on the chain at its mid-range pose."""
        if self.chain is None:
            return
        origins = np.array([t[:3, 3]
                            for _, t in self.chain.frames(self._mid_pose())])
        self.scene.fit(origins)
        # The markers have to read as landmarks, not planets.
        self.scene.set_marker_radius(max(self._reach() * 0.018, 0.005))

    # -- joint table ------------------------------------------------------

    def _build_rows(self) -> None:
        self.table.setRowCount(0)
        self._rows = {}
        if self.chain is None:
            return
        self.table.setRowCount(len(self.chain.actuated))
        for row, joint in enumerate(self.chain.actuated):
            self._rows[joint.name] = row

            name = QTableWidgetItem(joint.name)
            name.setToolTip(f"{joint.type}, axis {joint.axis.round(3)}")
            self.table.setItem(row, COL_JOINT, name)

            motor_box = QComboBox()
            motor_box.currentIndexChanged.connect(self._save_config)
            self.table.setCellWidget(row, COL_MOTOR, motor_box)

            sign_box = QComboBox()
            sign_box.addItem("+", 1.0)
            sign_box.addItem("-", -1.0)
            sign_box.setToolTip(
                "Whether the motor turns the same way as the URDF joint axis")
            sign_box.currentIndexChanged.connect(self._save_config)
            self.table.setCellWidget(row, COL_SIGN, sign_box)

            self.table.setItem(row, COL_READING, QTableWidgetItem("-"))

            offset = QDoubleSpinBox()
            offset.setRange(-360.0, 360.0)
            offset.setDecimals(3)
            offset.setSingleStep(0.1)
            offset.setSuffix(" deg")
            offset.setToolTip(
                "Added to the signed motor reading to give the URDF joint "
                "value. 'Solve offsets' fills these in.")
            offset.valueChanged.connect(self._save_config)
            self.table.setCellWidget(row, COL_OFFSET, offset)

            self.table.setItem(row, COL_VALUE, QTableWidgetItem("-"))

            limit = "-"
            if joint.lower is not None and joint.upper is not None:
                limit = (f"{math.degrees(joint.lower):+.1f} .. "
                         f"{math.degrees(joint.upper):+.1f} deg")
            self.table.setItem(row, COL_LIMIT, QTableWidgetItem(limit))
        self._refresh_motor_boxes()

    def set_inventory(self, motors: dict) -> None:
        """Called whenever the connection panel discovers or drops motors."""
        self.motors = dict(motors)
        self._refresh_motor_boxes()

    def _refresh_motor_boxes(self) -> None:
        keys = sorted(self.motors)
        self._loading = True
        boxes = [self.table.cellWidget(row, COL_MOTOR)
                 for row in range(self.table.rowCount())]
        boxes.append(self.gripper_motor)
        for box in boxes:
            if box is None:
                continue
            previous = box.currentData()
            box.clear()
            box.addItem("- none -", None)
            for channel, motor_id in keys:
                box.addItem(f"{channel}  id {motor_id}", (channel, motor_id))
            if previous is not None:
                index = find_motor_index(box, previous)
                # Keep a mapping to a motor that is momentarily missing, so a
                # closed channel does not quietly erase the calibration.
                if index < 0:
                    box.addItem(f"{previous[0]}  id {previous[1]}  (offline)",
                                previous)
                    index = box.count() - 1
                box.setCurrentIndex(index)
        self._loading = False

    # -- live state -------------------------------------------------------

    def _joint_values(self) -> tuple[np.ndarray, bool]:
        """Corrected joint vector, and whether every joint had fresh data."""
        values = np.zeros(len(self.chain)) if self.chain else np.zeros(0)
        complete = True
        for index, joint in enumerate(self.chain.actuated):
            row = self._rows[joint.name]
            box = self.table.cellWidget(row, COL_MOTOR)
            sign = self.table.cellWidget(row, COL_SIGN).currentData()
            offset = math.radians(self.table.cellWidget(row, COL_OFFSET).value())
            motor = self.motors.get(box.currentData()) if box else None
            if motor is None:
                complete = False
                self.table.item(row, COL_READING).setText("-")
                self.table.item(row, COL_VALUE).setText("-")
                continue
            reading = motor.state.position
            stale = motor.state.age > 1.0
            value = sign * reading + offset
            values[index] = value
            self.table.item(row, COL_READING).setText(
                units.text(reading, "rad", sign=True))
            item = self.table.item(row, COL_VALUE)
            item.setText(units.text(value, "rad", sign=True))
            outside = (joint.lower is not None and value < joint.lower) or \
                      (joint.upper is not None and value > joint.upper)
            item.setForeground(Qt.red if outside else
                               (Qt.gray if stale else Qt.black))
            if stale:
                complete = False
        return values, complete

    def _refresh(self) -> None:
        if self.chain is None or not self.isVisible():
            return
        values, _ = self._joint_values()
        frames = self.chain.frames(values)
        transform = frames[-1][1]
        position = transform[:3, 3]
        roll, pitch, yaw = kin.matrix_to_rpy(transform)

        for key, value in zip(("x", "y", "z"), position * 1000.0):
            self.pose_labels[key].setText(f"{value:+.1f} mm")
        for key, value in zip(("roll", "pitch", "yaw"),
                              (roll, pitch, yaw)):
            self.pose_labels[key].setText(f"{math.degrees(value):+.1f} deg")

        self._refresh_gripper()
        self._draw(frames, transform)

    def _draw(self, frames, tip_transform) -> None:
        transforms = {name: transform for name, transform in frames}
        if self.gripper is not None:
            # The fingers hang off a chain link, so they need the arm's
            # solution before they can be placed.
            parent = transforms.get(self.gripper.parent)
            if parent is not None:
                stroke = self._gripper_stroke() or 0.0
                transforms.update(dict(self.gripper.frames(parent, stroke)))
        self.scene.set_pose(transforms)
        self.scene.set_skeleton(np.array([t[:3, 3] for _, t in frames]))

        axes = [(frames[step][1], 0.035)
                for step, joint in enumerate(self.chain.joints, start=1)
                if joint.actuated]
        axes.append((tip_transform, 0.08))
        self.scene.set_frames(axes)

        tip = tip_transform[:3, 3]
        self.scene.set_tip(tip)
        self.scene.set_components(tip if self.show_components.isChecked()
                                  else None)

        points = [(sample.measured, MEASURED_COLOR, 9.0)
                  for sample in self.samples]
        if self._solution is not None:
            points.append((self._target_pose()[:3, 3], TARGET_COLOR, 13.0))
        self.scene.set_points(points)

    # -- reporting --------------------------------------------------------

    def _enable_reporting(self) -> None:
        touched = 0
        for row in range(self.table.rowCount()):
            box = self.table.cellWidget(row, COL_MOTOR)
            motor = self.motors.get(box.currentData()) if box else None
            if motor is None:
                continue
            try:
                motor.set_active_report(True)
                touched += 1
            except Exception as exc:
                log.debug("active report failed on %s: %s", motor, exc)
        self.status.emit(f"Active reporting enabled on {touched} motor(s)")

    # -- gripper ----------------------------------------------------------

    def _gripper_motor(self):
        return self.motors.get(self.gripper_motor.currentData())

    def _gripper_stroke(self) -> float | None:
        """Finger stroke in metres from the motor reading, or ``None``."""
        motor = self._gripper_motor()
        if self.gripper is None or motor is None:
            return None
        sign = self.gripper_sign.currentData()
        metres_per_rad = self.gripper_ratio.value() / 1000.0
        stroke = sign * motor.state.position * metres_per_rad
        return float(np.clip(stroke, self.gripper.lower, self.gripper.upper))

    def _refresh_gripper(self) -> None:
        if self.gripper is None:
            self.gripper_label.setText("no finger joints in this URDF")
            self.gripper_label.setStyleSheet("color: gray;")
            return
        motor = self._gripper_motor()
        if motor is None:
            self.gripper_label.setText(
                f"unmapped   (stroke 0 .. {self.gripper.upper * 1000:.0f} mm)")
            self.gripper_label.setStyleSheet("color: gray;")
            return
        stroke = self._gripper_stroke()
        stale = motor.state.age > 1.0
        self.gripper_label.setText(
            f"{units.text(motor.state.position, 'rad', sign=True)}   ->   "
            f"stroke {stroke * 1000:.1f} mm, "
            f"fingers {self.gripper.separation(stroke) * 1000:.1f} mm apart")
        self.gripper_label.setStyleSheet(
            "font-weight: 600; color: %s;" % ("gray" if stale else "black"))

    def _apply_gripper(self) -> None:
        motor = self._gripper_motor()
        if self.gripper is None or motor is None:
            QMessageBox.warning(self, "Gripper not mapped",
                                "Select the motor that drives the fingers.")
            return
        stroke = np.clip(self.gripper_command.value() / 1000.0,
                         self.gripper.lower, self.gripper.upper)
        sign = self.gripper_sign.currentData()
        metres_per_rad = self.gripper_ratio.value() / 1000.0
        if metres_per_rad <= 0:
            return
        command = stroke / metres_per_rad / sign

        if QMessageBox.question(
                self, "Move the gripper?",
                f"Drive motor {motor.motor_id} to "
                f"{units.text(command, 'rad', sign=True)}.\n\n"
                f"That is a {stroke * 1000:.1f} mm stroke — fingers "
                f"{self.gripper.separation(stroke) * 1000:.1f} mm apart — "
                f"at {self.gripper_ratio.value():g} mm/rad.\n\n"
                "If that ratio is wrong the fingers will travel the wrong "
                "distance, and the gripper will drive into its own stop.") \
                != QMessageBox.Yes:
            return
        try:
            self._drive(motor, command)
        except Exception as exc:
            QMessageBox.critical(self, "Gripper move failed", str(exc))
            return
        self.status.emit(f"Gripper commanded to {stroke * 1000:.1f} mm")

    # -- inverse kinematics -----------------------------------------------

    def _target_pose(self) -> np.ndarray:
        return kin.pose(
            [self.target[a].value() / 1000.0 for a in ("x", "y", "z")],
            [math.radians(self.target[a].value())
             for a in ("roll", "pitch", "yaw")])

    def _copy_current_pose(self) -> None:
        if self.chain is None:
            return
        values, _ = self._joint_values()
        transform = self.chain.fk(values)
        for axis, value in zip(("x", "y", "z"), transform[:3, 3] * 1000.0):
            self.target[axis].setValue(float(value))
        for axis, value in zip(("roll", "pitch", "yaw"),
                               kin.matrix_to_rpy(transform)):
            self.target[axis].setValue(math.degrees(value))

    def _clear_preview(self) -> None:
        self._solution = None
        self._relaxed = None
        self.relax_button.setEnabled(False)
        self.scene.set_preview(None)
        self.ik_label.setText("No solution")
        self.ik_label.setStyleSheet("color: gray;")

    def _solve_ik(self) -> None:
        if self.chain is None:
            QMessageBox.warning(self, "No chain", "Load a URDF first.")
            return
        seed, _ = self._joint_values()
        target = self._target_pose()
        result = kin.solve_ik(self.chain, target, seed=seed)
        self._solution = result
        self._relaxed = None

        frames = self.chain.frames(result.q)
        self.scene.set_preview(np.array([t[:3, 3] for _, t in frames]))

        note = (f"pos error {result.position_error * 1000:.2f} mm, "
                f"orientation {math.degrees(result.orientation_error):.2f} deg, "
                f"{result.iterations} iterations")
        if result.at_limit:
            note += f"   [at limit: {', '.join(result.at_limit)}]"

        if result.converged:
            self.ik_label.setStyleSheet("color: #27ae60;")
        else:
            # Six constraints on seven joints fails far more often than the
            # point being out of range, so say which of the two it was -
            # "unreachable" alone sends you moving the arm for no reason.
            position_only = kin.solve_ik(self.chain, target, seed=seed,
                                         orientation=False)
            if position_only.converged:
                self._relaxed = position_only
                achievable = np.degrees(
                    kin.matrix_to_rpy(position_only.reached))
                note = ("POINT IS REACHABLE, that orientation there is not.  "
                        + note
                        + "   Achievable orientation: "
                        + ", ".join(f"{v:+.1f}" for v in achievable) + " deg")
                self.ik_label.setStyleSheet("color: #e67e22;")
            else:
                note = ("POINT IS OUT OF RANGE - closest pose shown.  " + note)
                self.ik_label.setStyleSheet("color: #c0392b;")
        self.relax_button.setEnabled(self._relaxed is not None)
        self.ik_label.setText(note)

    def _relax_orientation(self) -> None:
        """Adopt the orientation the arm can actually hold at that point."""
        if self._relaxed is None:
            return
        for axis, value in zip(("roll", "pitch", "yaw"),
                               kin.matrix_to_rpy(self._relaxed.reached)):
            self.target[axis].setValue(math.degrees(value))
        self._solve_ik()

    def _move_to_target(self) -> None:
        # Deliberately always clickable. This button was previously greyed
        # out until a solution existed, and a bug left it greyed out after
        # a successful solve too - so pressing it did nothing at all, with
        # no message to say why. A primary action must always answer.
        if self.chain is None:
            QMessageBox.information(self, "No chain",
                                    "Load a URDF and pick a tip frame first.")
            return
        if self._solution is None:
            QMessageBox.information(
                self, "No solution yet",
                "Type a target pose and press 'Solve IK' first.\n\n"
                "An unreachable target still produces a solution - the "
                "closest pose the arm can hold - and that can be moved to.")
            return
        result = self._solution

        commands = []
        for index, joint in enumerate(self.chain.actuated):
            row = self._rows[joint.name]
            box = self.table.cellWidget(row, COL_MOTOR)
            motor = self.motors.get(box.currentData()) if box else None
            if motor is None:
                QMessageBox.warning(
                    self, "Joint not mapped",
                    f"{joint.name} has no motor. Every joint in the chain "
                    "must be mapped before the arm can be driven.")
                return
            if motor.state.age > 1.0:
                QMessageBox.warning(
                    self, "Stale feedback",
                    f"{joint.name} has not reported for "
                    f"{motor.state.age:.1f} s. Where the arm is now is not "
                    "known, so it must not be commanded.\n\n"
                    "Use 'Enable active reporting on mapped motors' first.")
                return
            sign = self.table.cellWidget(row, COL_SIGN).currentData()
            offset = math.radians(
                self.table.cellWidget(row, COL_OFFSET).value())
            # joint = sign * motor + offset, so motor = (joint - offset)/sign
            commands.append((joint, motor,
                             (result.q[index] - offset) / sign,
                             result.q[index] - (sign * motor.state.position
                                                + offset)))

        biggest = max(abs(delta) for _, _, _, delta in commands)
        detail = "\n".join(
            f"  {joint.name}: {units.text(delta, 'rad', sign=True)}"
            for joint, _, _, delta in commands)
        shortfall = ""
        if not result.converged:
            shortfall = (f"\nTHE TARGET IS NOT REACHABLE. This moves to the "
                         f"closest pose, {result.position_error * 1000:.1f} mm "
                         f"away from the point you asked for.\n")

        if QMessageBox.question(
                self, "Move the arm?",
                f"Drive {len(commands)} joints to the solved pose.\n{shortfall}"
                f"\nLargest single joint move: "
                f"{units.text(biggest, 'rad')}\n\n{detail}\n\n"
                f"Speed limit {units.text(self.move_speed.rad(), 'rad/s')} "
                f"per joint.\n\n"
                "There is NO collision checking. The arm will sweep whatever "
                "is between here and there. Is everyone clear of it?") \
                != QMessageBox.Yes:
            return

        moved = 0
        for joint, motor, command, _ in commands:
            try:
                self._drive(motor, command)
                moved += 1
            except Exception as exc:
                QMessageBox.critical(
                    self, "Move failed",
                    f"{joint.name} on motor {motor.motor_id}: {exc}\n\n"
                    f"{moved} joint(s) were already commanded - the arm is "
                    "part-way to the target. Check the Control tab.")
                return
        self.status.emit(f"Commanded {moved} joints to the solved pose")

    def _drive(self, motor, position: float) -> None:
        """Put one motor in CSP at the given speed limit and send the angle.

        Mirrors the Control tab's sequencing: a setpoint written in the wrong
        mode is discarded by the firmware without an error, and the manual
        requires the motor be stopped before a mode change.
        """
        current = motor.read(0x7005, timeout=0.2)
        if current is None or int(current) != int(RunMode.POSITION_CSP):
            motor.stop()
            time.sleep(0.01)
            motor.write(0x7005, int(RunMode.POSITION_CSP))
            time.sleep(0.01)
        motor.enable()
        motor.write(0x7017, self.move_speed.rad())
        motor.write(0x7016, position)

    # -- calibration ------------------------------------------------------

    def _capture(self) -> None:
        if self.chain is None:
            return
        values, complete = self._joint_values()
        if not complete:
            if QMessageBox.question(
                    self, "Incomplete joint data",
                    "Some joints are unmapped or their readings are stale.\n\n"
                    "Capturing now records zeros for those, which will bias "
                    "the fit. Capture anyway?") != QMessageBox.Yes:
                return
        measured = np.array([self.measured[a].value() for a in ("x", "y", "z")])
        self.samples.append(kin.Sample(q=values, measured=measured / 1000.0))
        self._update_sample_label()
        self.status.emit(f"Captured sample {len(self.samples)}")

    def _clear_samples(self) -> None:
        self.samples = []
        self._update_sample_label()

    def _update_sample_label(self) -> None:
        count = len(self.samples)
        if not count:
            self.fit_label.setText("No samples captured")
            self.fit_label.setStyleSheet("color: gray;")
            return
        needed = len(self.chain) if self.chain else 0
        note = f"{count} sample(s) captured"
        if count * 3 < needed:
            note += (f"   [under-determined: {needed} offsets need at least "
                     f"{-(-needed // 3)} well-spread poses]")
            self.fit_label.setStyleSheet("color: #e67e22;")
        else:
            self.fit_label.setStyleSheet("color: gray;")
        self.fit_label.setText(note)

    def _solve(self) -> None:
        if self.chain is None or not self.samples:
            QMessageBox.information(
                self, "Nothing to solve",
                "Pose the arm, type the measured tip position, and press "
                "'Capture sample' a few times first.")
            return
        try:
            fit = kin.solve_offsets(self.chain, self.samples)
        except Exception as exc:
            QMessageBox.critical(self, "Solve failed", str(exc))
            return

        detail = "\n".join(
            f"  {joint.name}: {math.degrees(offset):+.3f} deg"
            for joint, offset in zip(self.chain.actuated, fit.offsets))
        if QMessageBox.question(
                self, "Apply the fitted offsets?",
                f"Tip error RMS {fit.rms_before * 1000:.1f} mm -> "
                f"{fit.rms * 1000:.1f} mm over {len(self.samples)} sample(s), "
                f"{fit.iterations} iterations"
                f"{'' if fit.converged else ' (did NOT converge)'}.\n\n"
                f"Offsets to add:\n{detail}\n\n"
                "These are added to the offsets already in the table.") \
                != QMessageBox.Yes:
            return

        for joint, offset in zip(self.chain.actuated, fit.offsets):
            spin = self.table.cellWidget(self._rows[joint.name], COL_OFFSET)
            spin.setValue(spin.value() + math.degrees(offset))
        self.fit_label.setText(
            f"Applied: RMS {fit.rms_before * 1000:.1f} -> {fit.rms * 1000:.1f} mm "
            f"over {len(self.samples)} sample(s)")
        self.fit_label.setStyleSheet(
            "color: #27ae60;" if fit.converged else "color: #c0392b;")
        self.status.emit("Offsets applied - use 'Set zero here' on each motor "
                         "to bake them in, or keep them here as a correction")

    # -- shared with the calibration tab ----------------------------------
    #
    # The calibration procedure needs the same URDF, the same chain and the
    # same joint-to-motor map this tab already holds, and hands offsets back
    # when it has identified them. Rebuilding that UI over there would give
    # the operator two mappings to keep in step, and they would not stay in
    # step. These three methods are the whole interface between the tabs.

    def calibration_chain(self):
        """The chain the calibration tab should work on, or ``None``."""
        return self.chain

    def joint_map(self) -> list[JointMapping]:
        """Motor, sign and offset for every actuated joint, in chain order."""
        out = []
        for joint in self.chain.actuated if self.chain else []:
            row = self._rows[joint.name]
            box = self.table.cellWidget(row, COL_MOTOR)
            out.append(JointMapping(
                joint=joint,
                motor=self.motors.get(box.currentData()) if box else None,
                sign=self.table.cellWidget(row, COL_SIGN).currentData(),
                offset=math.radians(
                    self.table.cellWidget(row, COL_OFFSET).value())))
        return out

    def apply_offsets(self, offsets: dict[str, float]) -> int:
        """Add identified offsets, in radians, to the offset column."""
        applied = 0
        for name, offset in offsets.items():
            row = self._rows.get(name)
            if row is None:
                continue
            spin = self.table.cellWidget(row, COL_OFFSET)
            spin.setValue(spin.value() + math.degrees(offset))
            applied += 1
        self._save_config()
        return applied

    # -- config -----------------------------------------------------------

    def _save_config(self) -> None:
        if self._loading or self.chain is None:
            return
        joints = {}
        for joint in self.chain.actuated:
            row = self._rows[joint.name]
            motor = self.table.cellWidget(row, COL_MOTOR).currentData()
            joints[joint.name] = {
                "channel": motor[0] if motor else None,
                "motor_id": motor[1] if motor else None,
                "sign": self.table.cellWidget(row, COL_SIGN).currentData(),
                "offset_deg": self.table.cellWidget(row, COL_OFFSET).value(),
            }
        gripper_motor = self.gripper_motor.currentData()
        payload = {
            "urdf": self.urdf_path.text(),
            "mesh_root": self.mesh_root.text(),
            "tip": self.chain_box.currentText(),
            "joints": joints,
            "gripper": {
                "channel": gripper_motor[0] if gripper_motor else None,
                "motor_id": gripper_motor[1] if gripper_motor else None,
                "sign": self.gripper_sign.currentData(),
                "mm_per_rad": self.gripper_ratio.value(),
            },
        }
        try:
            path = self._config_path
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        except Exception as exc:
            log.debug("could not save kinematics config: %s", exc)

    def _load_config(self) -> None:
        try:
            payload = json.loads(self._config_path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return
        except Exception as exc:
            log.debug("could not read kinematics config: %s", exc)
            return

        urdf = payload.get("urdf")
        if not urdf or not Path(urdf).exists():
            return
        # Set the mesh folder first so the URDF load resolves meshes in one
        # pass rather than loading them twice.
        mesh_root = payload.get("mesh_root") or ""
        if mesh_root and Path(mesh_root).is_dir():
            self.mesh_root.setText(mesh_root)
        self._load_urdf(urdf)
        tip = payload.get("tip")
        if tip and self.chain_box.findText(tip) >= 0:
            self.chain_box.setCurrentText(tip)

        self._loading = True
        for name, entry in (payload.get("joints") or {}).items():
            row = self._rows.get(name)
            if row is None:
                continue
            channel, motor_id = entry.get("channel"), entry.get("motor_id")
            if channel is not None and motor_id is not None:
                box = self.table.cellWidget(row, COL_MOTOR)
                key = (channel, motor_id)
                if find_motor_index(box, key) < 0:
                    box.addItem(f"{channel}  id {motor_id}  (offline)", key)
                box.setCurrentIndex(find_motor_index(box, key))
            sign_box = self.table.cellWidget(row, COL_SIGN)
            sign_box.setCurrentIndex(0 if entry.get("sign", 1.0) > 0 else 1)
            self.table.cellWidget(row, COL_OFFSET).setValue(
                float(entry.get("offset_deg", 0.0)))

        gripper = payload.get("gripper") or {}
        channel, motor_id = gripper.get("channel"), gripper.get("motor_id")
        if channel is not None and motor_id is not None:
            key = (channel, motor_id)
            if find_motor_index(self.gripper_motor, key) < 0:
                self.gripper_motor.addItem(
                    f"{channel}  id {motor_id}  (offline)", key)
            self.gripper_motor.setCurrentIndex(
                find_motor_index(self.gripper_motor, key))
        self.gripper_sign.setCurrentIndex(
            0 if gripper.get("sign", 1.0) > 0 else 1)
        self.gripper_ratio.setValue(float(gripper.get("mm_per_rad", 7.0)))
        self._loading = False

    def shutdown(self) -> None:
        self._timer.stop()
        self._save_config()
