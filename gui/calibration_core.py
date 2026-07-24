"""Shared machinery for the calibration wizard.

The wizard is a list of steps, each a self-contained page that knows how to
report its own state. This module holds what they all need: the state
vocabulary, the session they share, the base page, and a way to run a timed
motor test without freezing the interface that carries the stop button.
"""

from __future__ import annotations

import json
import logging
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path

import numpy as np
from PySide6.QtCore import QObject, QStandardPaths, QTimer, Signal
from PySide6.QtWidgets import (
    QAbstractItemView, QHeaderView, QLabel, QTableWidget, QWidget,
)

import calibration as cal
from robstride import params as P

log = logging.getLogger(__name__)

# --------------------------------------------------------------------------
# Register handles.
#
# Everything reached by a literal index here lives in the 0x70xx runtime
# range, which is verified identical on RS00, RS03 and RS04. That matters
# more than usual for calibration: the arms mix models, and the 0x20xx/0x30xx
# index of any given name is different on each of them. Where a stored
# register really is needed, it is looked up by name instead.
# --------------------------------------------------------------------------

#: Load-side mechanical angle, as a float. The type-2 feedback frame carries
#: the same number quantised to 16 bits over +/-12.57 rad - 0.022 deg a step,
#: which is a fifth of a millimetre at half a metre of reach. Fine for
#: watching the arm move, coarse enough to be worth avoiding when the whole
#: exercise is measuring sub-degree errors.
MECH_POS = 0x7019
ZERO_STA = 0x7029
ADD_OFFSET = 0x702B
RUN_MODE = 0x7005
LOC_REF = 0x7016
LIMIT_SPD = 0x7017
LIMIT_CUR = 0x7018

#: Feedback older than this is not evidence of where the arm is now.
STALE = 1.0


class State(Enum):
    """What a step has to say about itself, for the list on the left."""

    TODO = ("○", "#7f8c8d")
    OK = ("✓", "#27ae60")
    WARN = ("!", "#e67e22")
    FAIL = ("✗", "#c0392b")

    @property
    def glyph(self) -> str:
        return self.value[0]

    @property
    def colour(self) -> str:
        return self.value[1]


# --------------------------------------------------------------------------
# Small helpers
# --------------------------------------------------------------------------


def read_named(motor, name: str, timeout: float = 0.25):
    """Read a register by name, or ``None`` if this model has no such one.

    Names are portable across models where indices are not, so this is how
    the diagnostics reach registers like ``mech_angle_rotat`` without
    hard-coding one model's map onto another's silicon.
    """
    index = P.index_of(name, motor.model)
    if index is None:
        return None
    try:
        return motor.read(index, timeout)
    except Exception as exc:
        log.debug("read %s on %s failed: %s", name, motor.motor_id, exc)
        return None


def motor_label(key) -> str:
    channel, motor_id = key
    return f"{channel}  id {motor_id}"


def number(value, digits: int = 4, suffix: str = "") -> str:
    """Format a possibly-missing reading."""
    if value is None:
        return "-"
    if isinstance(value, str):
        return value
    return f"{float(value):.{digits}f}{suffix}"


def table(headers: list[str], stretch: int = 0) -> QTableWidget:
    widget = QTableWidget(0, len(headers))
    widget.setHorizontalHeaderLabels(headers)
    widget.setSelectionBehavior(QAbstractItemView.SelectRows)
    widget.setEditTriggers(QAbstractItemView.NoEditTriggers)
    widget.verticalHeader().setVisible(False)
    head = widget.horizontalHeader()
    for column in range(len(headers)):
        head.setSectionResizeMode(
            column, QHeaderView.Stretch if column == stretch
            else QHeaderView.ResizeToContents)
    return widget


def note(text: str, colour: str = "gray") -> QLabel:
    label = QLabel(text)
    label.setWordWrap(True)
    label.setStyleSheet(f"color: {colour};")
    return label


def _config_path() -> Path:
    base = QStandardPaths.writableLocation(QStandardPaths.AppConfigLocation)
    return Path(base or ".") / "openarmx_calibration.json"


# --------------------------------------------------------------------------
# The session
# --------------------------------------------------------------------------


@dataclass
class Session:
    """State the steps share, and the bits of it worth keeping.

    The URDF, the chain and the joint-to-motor map are deliberately *not*
    here: they belong to the Kinematics tab, and duplicating them would give
    the operator two maps to keep in step. This holds only what calibration
    adds on top.
    """

    #: The Kinematics tab, or any object with the same three methods.
    arm: object | None = None
    motors: dict = field(default_factory=dict)

    #: Tool tip in the tip-link frame, metres.
    tool: np.ndarray = field(default_factory=lambda: np.zeros(3))
    fit_tool: bool = True
    #: Fixture point in the base frame, or None to fit it as an unknown.
    point: np.ndarray | None = None
    locked: set = field(default_factory=set)
    precise: bool = True

    poses: list = field(default_factory=list)
    variants: list = field(default_factory=list)
    fit: object | None = None
    #: Motor key -> the last bootstrap reading, for power-cycle comparison.
    snapshots: dict = field(default_factory=dict)

    config_path: Path | None = None

    # -- the arm ----------------------------------------------------------

    def chain(self):
        return self.arm.calibration_chain() if self.arm is not None else None

    def mapping(self) -> list:
        return self.arm.joint_map() if self.arm is not None else []

    def blocker(self) -> str:
        """Why the arm-level steps cannot run yet, or ``""``."""
        chain = self.chain()
        if chain is None:
            return ("No chain. Load a URDF and pick a tip frame on the "
                    "Kinematics tab.")
        unmapped = [row.name for row in self.mapping() if row.motor is None]
        if unmapped:
            return ("Unmapped joints: " + ", ".join(unmapped)
                    + ". Every joint in the chain needs a motor.")
        return ""

    def joint_vector(self, precise: bool = False):
        """The joint values now, and the names of any that are not live.

        ``precise`` swaps the feedback frame for a direct read of mechPos,
        which costs a round trip per joint and buys the difference between a
        16-bit quantisation and a float. Worth it for a capture, not for the
        60-times-a-minute refresh.
        """
        values, missing = [], []
        for row in self.mapping():
            if not row.live:
                missing.append(row.name)
                values.append(0.0)
                continue
            reading = row.motor.state.position
            if precise:
                exact = None
                try:
                    exact = row.motor.read(MECH_POS, timeout=0.25)
                except Exception as exc:
                    log.debug("precise read failed on %s: %s", row.name, exc)
                if exact is None:
                    missing.append(row.name)
                else:
                    reading = float(exact)
            values.append(row.sign * reading + row.offset)
        return np.array(values, dtype=float), missing

    # -- how the fit is configured ----------------------------------------

    def fit_kwargs(self) -> dict:
        return {
            "tool": self.tool,
            "fit_tool": self.fit_tool,
            "point": self.point,
            "locked": tuple(sorted(self.locked)),
        }

    def observe_kwargs(self) -> dict:
        return {
            "tool": self.tool,
            "fit_tool": self.fit_tool,
            "free_point": self.point is None,
            "locked": tuple(sorted(self.locked)),
        }

    def observability(self, extra=None):
        """Identifiability of the offsets given what has been captured."""
        chain = self.chain()
        if chain is None:
            return None
        poses = list(self.poses)
        if extra is not None:
            poses.append(cal.Pose(extra))
        return cal.observability(chain, poses, **self.observe_kwargs())

    def recommend_locks(self) -> set:
        """Joints this configuration cannot identify, whatever is captured.

        Structural, not statistical: the first joint always goes, because a
        free fixture point absorbs any rotation about the base axis, and the
        last one goes whenever the tool offset is being fitted alongside it,
        because turning the tool and rotating the tool vector are the same
        motion.
        """
        chain = self.chain()
        if chain is None or not len(chain):
            return set()
        names = [joint.name for joint in chain.actuated]
        out = set()
        if self.point is None:
            out.add(names[0])
        if self.fit_tool:
            out.add(names[-1])
        return out

    # -- persistence ------------------------------------------------------

    def path(self) -> Path:
        return self.config_path or _config_path()

    def save(self) -> None:
        payload = {
            "tool_mm": [float(v) * 1000.0 for v in self.tool],
            "fit_tool": bool(self.fit_tool),
            "point_mm": (None if self.point is None
                         else [float(v) * 1000.0 for v in self.point]),
            "locked": sorted(self.locked),
            "precise": bool(self.precise),
            "snapshots": {f"{c}|{i}": v for (c, i), v in self.snapshots.items()},
        }
        try:
            path = self.path()
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        except Exception as exc:
            log.debug("could not save the calibration config: %s", exc)

    def load(self) -> None:
        try:
            payload = json.loads(self.path().read_text(encoding="utf-8"))
        except FileNotFoundError:
            return
        except Exception as exc:
            log.debug("could not read the calibration config: %s", exc)
            return
        tool = payload.get("tool_mm")
        if tool and len(tool) == 3:
            self.tool = np.array(tool, dtype=float) / 1000.0
        self.fit_tool = bool(payload.get("fit_tool", True))
        point = payload.get("point_mm")
        self.point = (np.array(point, dtype=float) / 1000.0
                      if point and len(point) == 3 else None)
        self.locked = set(payload.get("locked") or [])
        self.precise = bool(payload.get("precise", True))
        for key, value in (payload.get("snapshots") or {}).items():
            channel, _, motor_id = key.rpartition("|")
            if motor_id.isdigit():
                self.snapshots[(channel, int(motor_id))] = value


# --------------------------------------------------------------------------
# Pages
# --------------------------------------------------------------------------


class Step(QWidget):
    """One page of the wizard."""

    status = Signal(str)
    #: Emitted when this page's :meth:`state` may have changed.
    changed = Signal()

    TITLE = ""
    BLURB = ""

    def __init__(self, session: Session, parent=None):
        super().__init__(parent)
        self.session = session

    def entered(self) -> None:
        """Called each time the page is shown."""

    def refresh(self) -> None:
        """Called a few times a second while the page is shown."""

    def state(self) -> tuple[State, str]:
        return State.TODO, ""

    def shutdown(self) -> None:
        """Called once when the window is closing."""


# --------------------------------------------------------------------------
# Timed motor tests
# --------------------------------------------------------------------------


class Sequence(QObject):
    """A list of actions with waits between them, run on the event loop.

    Every motor test here is command-wait-measure, and the waits are for real
    mechanical settling rather than for tidiness. Doing them with
    ``time.sleep`` would block the event loop for the whole test, which is
    precisely when the operator most needs the stop button to answer.
    """

    #: True when every action ran, False on abort or on an exception.
    finished = Signal(bool)
    progress = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._steps: list = []
        self._index = 0
        self._running = False
        self.error: str = ""

    @property
    def running(self) -> bool:
        return self._running

    def run(self, steps) -> None:
        """``steps`` is a sequence of ``(label, wait_ms, action)``."""
        if self._running:
            raise RuntimeError("a sequence is already running")
        self._steps = list(steps)
        self._index = 0
        self._running = True
        self.error = ""
        self._advance()

    def abort(self) -> None:
        if not self._running:
            return
        self._running = False
        self.error = "aborted"
        self.finished.emit(False)

    def _advance(self) -> None:
        if not self._running:
            return
        if self._index >= len(self._steps):
            self._running = False
            self.finished.emit(True)
            return
        label, wait, action = self._steps[self._index]
        self._index += 1
        if label:
            self.progress.emit(label)
        try:
            action()
        except Exception as exc:
            log.exception("calibration sequence step %r failed", label)
            self._running = False
            self.error = f"{type(exc).__name__}: {exc}"
            self.finished.emit(False)
            return
        QTimer.singleShot(int(wait), self._advance)
