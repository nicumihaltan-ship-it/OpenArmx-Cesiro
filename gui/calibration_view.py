"""Calibration, one step at a time.

Two halves, in that order. The first four steps are about the motors alone
and need no model of the arm: is the encoder calibrated, what angle range
does each motor report, does the power-on position bootstrap repeat, how much
backlash is in the joint. The last four are about the arm as a whole and need
the URDF: they identify the joint zero offsets from a tool tip held in a
fixture.

The order is not decoration. Arm-level calibration assumes each motor reports
its own angle correctly; run it on top of an uncalibrated encoder or an
unconfirmed feedback scaling and it will produce seven confident numbers that
encode somebody else's problem.
"""

from __future__ import annotations

import logging

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QHBoxLayout, QLabel, QListWidget, QListWidgetItem, QPushButton,
    QStackedWidget, QVBoxLayout, QWidget,
)

from .calibration_arm import MapStep, PosesStep, SolveStep, ToolStep
from .calibration_core import Session, State, Step
from .calibration_motors import BootstrapStep, MotionStep, MotorsStep, ZeroStep

log = logging.getLogger(__name__)

#: The steps, in order, and which half of the procedure they belong to.
STEPS = [
    ("Motors", MotorsStep),
    ("Motors", ZeroStep),
    ("Motors", BootstrapStep),
    ("Motors", MotionStep),
    ("Arm", MapStep),
    ("Arm", ToolStep),
    ("Arm", PosesStep),
    ("Arm", SolveStep),
]


class CalibrationView(QWidget):
    """The wizard shell: a list of steps on the left, the current one right."""

    status = Signal(str)
    #: Joint name -> radians, for the Kinematics tab to fold into its table.
    offsets_applied = Signal(dict)
    #: Asked for by the step that can only point at the other tab.
    show_kinematics = Signal()

    def __init__(self, arm=None, parent=None, config_path=None):
        super().__init__(parent)
        self.session = Session(arm=arm, config_path=config_path)
        self.session.load()

        self.steps: list[Step] = []
        self._build_ui()

        # Slower than the other tabs' 100 ms: these pages recompute an SVD
        # over every captured pose to keep the identifiability readout live,
        # and nothing on them changes fast enough to notice the difference.
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(400)

    # -- construction -----------------------------------------------------

    def _build_ui(self) -> None:
        self.list = QListWidget()
        self.list.setMaximumWidth(300)
        self.list.currentRowChanged.connect(self._on_step_changed)

        self.pages = QStackedWidget()
        self.title = QLabel("")
        self.title.setStyleSheet("font-size: 16px; font-weight: 600;")
        self.blurb = QLabel("")
        self.blurb.setWordWrap(True)
        self.blurb.setStyleSheet("color: gray;")

        previous = QPushButton("< Back")
        previous.clicked.connect(lambda: self._step_by(-1))
        following = QPushButton("Next >")
        following.clicked.connect(lambda: self._step_by(1))
        self.detail = QLabel("")
        self.detail.setWordWrap(True)

        half = ""
        for index, (group, factory) in enumerate(STEPS):
            step = factory(self.session, self)
            step.status.connect(self.status)
            step.changed.connect(self._refresh_list)
            if isinstance(step, MapStep):
                step.open_kinematics.connect(self.show_kinematics)
            if isinstance(step, SolveStep):
                step.offsets_applied.connect(self._on_offsets_applied)
            self.steps.append(step)
            self.pages.addWidget(step)

            if group != half:
                half = group
                heading = QListWidgetItem(
                    "The motors, without a model"
                    if group == "Motors" else "The arm, against the URDF")
                heading.setFlags(Qt.NoItemFlags)
                heading.setForeground(Qt.gray)
                self.list.addItem(heading)
            self.list.addItem(QListWidgetItem(f"  {index + 1}. {step.TITLE}"))

        header = QVBoxLayout()
        header.setSpacing(2)
        header.addWidget(self.title)
        header.addWidget(self.blurb)

        footer = QHBoxLayout()
        footer.addWidget(self.detail, 1)
        footer.addWidget(previous)
        footer.addWidget(following)

        right = QVBoxLayout()
        right.addLayout(header)
        right.addWidget(self.pages, 1)
        right.addLayout(footer)

        layout = QHBoxLayout(self)
        layout.addWidget(self.list)
        layout.addLayout(right, 1)

        self._select(0)

    # -- navigation -------------------------------------------------------

    def _row_of(self, index: int) -> int:
        """List row carrying step ``index`` - the group headings shift it."""
        seen, half = 0, ""
        for row, (group, _) in enumerate(STEPS):
            if group != half:
                half = group
                seen += 1
            if row == index:
                return seen + row
        return 0

    def _index_of(self, row: int) -> int | None:
        for index in range(len(STEPS)):
            if self._row_of(index) == row:
                return index
        return None

    def _select(self, index: int) -> None:
        self.list.setCurrentRow(self._row_of(index))

    def _step_by(self, delta: int) -> None:
        index = self._index_of(self.list.currentRow())
        if index is None:
            index = 0
        self._select(max(0, min(len(STEPS) - 1, index + delta)))

    def _on_step_changed(self, row: int) -> None:
        index = self._index_of(row)
        if index is None:
            return                      # a group heading; not selectable
        step = self.steps[index]
        self.pages.setCurrentWidget(step)
        self.title.setText(f"{index + 1}. {step.TITLE}")
        self.blurb.setText(step.BLURB)
        try:
            step.entered()
        except Exception:
            log.exception("entering %s failed", type(step).__name__)
        self._refresh_list()

    # -- live state -------------------------------------------------------

    def _tick(self) -> None:
        if not self.isVisible():
            return
        step = self.pages.currentWidget()
        if isinstance(step, Step):
            try:
                step.refresh()
            except Exception:
                log.exception("refreshing %s failed", type(step).__name__)
        self._refresh_list()

    def _refresh_list(self) -> None:
        current = self._index_of(self.list.currentRow())
        for index, step in enumerate(self.steps):
            try:
                state, detail = step.state()
            except Exception:
                log.exception("state of %s failed", type(step).__name__)
                state, detail = State.WARN, "internal error - see the log"
            item = self.list.item(self._row_of(index))
            item.setText(f"  {state.glyph} {index + 1}. {step.TITLE}")
            item.setToolTip(detail)
            item.setForeground(QColor(state.colour))
            item.setData(Qt.UserRole, state.name)
            if index == current:
                self.detail.setText(detail)
                self.detail.setStyleSheet(f"color: {state.colour};")

    # -- wiring -----------------------------------------------------------

    def set_inventory(self, motors: dict) -> None:
        self.session.motors = dict(motors)

    def _on_offsets_applied(self, offsets: dict) -> None:
        self.offsets_applied.emit(offsets)
        # The offsets are now folded into the joint values, so every captured
        # pose and the fit built on them describe an arm that no longer
        # exists. Keeping them would let a second solve add the same
        # correction twice.
        self.session.poses = []
        self.session.fit = None
        self.session.save()
        self._refresh_list()

    def report(self) -> str:
        """The last solved calibration as text, for tests and the clipboard."""
        for step in self.steps:
            if isinstance(step, SolveStep):
                return step.text_report()
        return ""

    def shutdown(self) -> None:
        self._timer.stop()
        for step in self.steps:
            try:
                step.shutdown()
            except Exception:
                log.exception("shutdown of %s failed", type(step).__name__)
        self.session.save()
