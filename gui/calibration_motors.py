"""Calibration steps that need no model of the arm.

Everything here is per motor: is its encoder calibrated, what angle range
does it report, did its power-on position bootstrap come up the same as last
time, and how much backlash and hysteresis does the joint actually have.

These come first on purpose. A joint whose encoder is uncalibrated, or whose
turn number resolves differently on each power cycle, produces readings that
no amount of arm-level fitting can rescue - the fit will happily absorb the
nonsense into offsets that are wrong in a different way tomorrow. And the
backlash number here is the floor on what the arm-level calibration can
possibly achieve, so it is worth knowing before spending an afternoon
chasing a residual below it.
"""

from __future__ import annotations

import logging

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QDoubleSpinBox, QGroupBox, QHBoxLayout, QLabel, QMessageBox,
    QPushButton, QTableWidgetItem, QVBoxLayout,
)

from robstride import RunMode
from robstride import params as P

from .calibration_core import (
    ADD_OFFSET, LIMIT_CUR, LIMIT_SPD, LOC_REF, MECH_POS, RUN_MODE, STALE,
    ZERO_STA, Sequence, State, Step, motor_label, note, number, read_named,
    table,
)
from .units import units

log = logging.getLogger(__name__)

#: faultSta bits that make a calibration meaningless rather than merely
#: inconvenient. Both are documented in the manual's fault table.
BLOCKING_FAULTS = {7: "encoder uncalibrated", 9: "position initialisation"}

#: The bootstrap registers worth watching, in the order they are computed at
#: power-on. Looked up by name because their indices differ per model.
BOOTSTRAP = ["chasu_offset", "chasu_angle", "chasu_angle_init",
             "mech_angle_init2", "mech_angle_rotat", "rotation", "mechPos"]


# --------------------------------------------------------------------------
# 1 - motors
# --------------------------------------------------------------------------


class MotorsStep(Step):
    TITLE = "Motors"
    BLURB = ("Every motor that will take part, and whether it is in a fit "
             "state to be calibrated at all.")

    HEADERS = ["Motor", "Model", "Feedback scaling", "Firmware", "Mode",
               "Faults", "Encoder"]

    def __init__(self, session, parent=None):
        super().__init__(session, parent)
        self._readings: dict = {}

        self.table = table(self.HEADERS, stretch=5)
        read = QPushButton("Read firmware and fault words")
        read.clicked.connect(self._read_all)

        layout = QVBoxLayout(self)
        layout.addWidget(note(
            "Two things here can invalidate a calibration before it starts. "
            "An <b>uncalibrated encoder</b> (fault bit 7) means the motor "
            "does not know its own rotor angle, and the fix is RobStride's "
            "magnetic-encoder calibration, not this tool. A <b>feedback "
            "scaling</b> that nobody has confirmed means position is decoded "
            "against the wrong full-scale value, so every angle is wrong by "
            "a constant factor - and a scale error is not an offset error. "
            "The offset fit will absorb it into seven meaningless numbers."))
        layout.addWidget(self.table, 1)
        buttons = QHBoxLayout()
        buttons.addWidget(read)
        buttons.addStretch(1)
        layout.addLayout(buttons)

    def entered(self) -> None:
        self._rebuild()

    def _rebuild(self) -> None:
        keys = sorted(self.session.motors)
        self.table.setRowCount(len(keys))
        for row, key in enumerate(keys):
            item = QTableWidgetItem(motor_label(key))
            item.setData(Qt.UserRole, key)
            self.table.setItem(row, 0, item)
            for column in range(1, len(self.HEADERS)):
                self.table.setItem(row, column, QTableWidgetItem("-"))
        self.refresh()

    def refresh(self) -> None:
        keys = sorted(self.session.motors)
        if self.table.rowCount() != len(keys):
            self._rebuild()
            return
        for row, key in enumerate(keys):
            motor = self.session.motors[key]
            reading = self._readings.get(key, {})
            scaling = ("verified" if motor.limits.verified
                       else "UNVERIFIED - check models.json")
            if not P.has_table(motor.model):
                scaling += "; no parameter table"
            faults = ", ".join(motor.state.faults) or "none"
            if motor.state.age > STALE:
                faults = "no feedback"
            self._set(row, 1, motor.model)
            self._set(row, 2, scaling,
                      warn=not motor.limits.verified
                      or not P.has_table(motor.model))
            self._set(row, 3, reading.get("firmware", "-"))
            self._set(row, 4, motor.state.mode.name
                      if motor.state.age <= STALE else "-")
            self._set(row, 5, faults, warn=bool(motor.state.faults))
            self._set(row, 6, self._encoder(key, motor),
                      warn=bool(self._blocking(key, motor)))

    def _set(self, row: int, column: int, text: str, warn: bool = False) -> None:
        item = self.table.item(row, column)
        item.setText(text)
        item.setForeground(Qt.red if warn else Qt.black)

    def _blocking(self, key, motor) -> list[str]:
        """Reasons this motor cannot be calibrated as it stands."""
        out = []
        if "uncalibrated" in motor.state.faults:
            out.append("encoder uncalibrated")
        word = self._readings.get(key, {}).get("faultSta")
        if isinstance(word, (int, float)):
            out += [name for bit, name in BLOCKING_FAULTS.items()
                    if int(word) & (1 << bit)]
        return sorted(set(out))

    def _encoder(self, key, motor) -> str:
        blocking = self._blocking(key, motor)
        if blocking:
            return "FAULT: " + ", ".join(blocking)
        if "faultSta" not in self._readings.get(key, {}):
            return "not read"
        return "calibrated"

    def _read_all(self) -> None:
        for key, motor in sorted(self.session.motors.items()):
            entry = self._readings.setdefault(key, {})
            entry["firmware"] = motor.read(0x1003, timeout=0.3) or "-"
            entry["faultSta"] = read_named(motor, "faultSta")
        self.refresh()
        self.changed.emit()
        self.status.emit(f"Read {len(self._readings)} motor(s)")

    def state(self) -> tuple[State, str]:
        motors = self.session.motors
        if not motors:
            return State.FAIL, "no motors - scan a channel first"
        blocked = [motor_label(key) for key, motor in motors.items()
                   if self._blocking(key, motor)]
        if blocked:
            return State.FAIL, "faults on " + ", ".join(blocked)
        unverified = [motor_label(key) for key, motor in motors.items()
                      if not motor.limits.verified]
        if unverified:
            return State.WARN, "unverified scaling on " + ", ".join(unverified)
        if not self._readings:
            return State.TODO, f"{len(motors)} motor(s), not yet read"
        return State.OK, f"{len(motors)} motor(s) healthy"


# --------------------------------------------------------------------------
# 2 - the zero and the range it is measured in
# --------------------------------------------------------------------------


class ZeroStep(Step):
    TITLE = "Zero and range"
    BLURB = ("Where each motor thinks zero is, and over what range it "
             "reports angles.")

    HEADERS = ["Motor", "Range (zero_sta)", "add_offset", "mechPos (0x7019)",
               "Feedback frame"]

    def __init__(self, session, parent=None):
        super().__init__(session, parent)
        self._readings: dict = {}

        self.table = table(self.HEADERS, stretch=1)

        read = QPushButton("Read all")
        read.clicked.connect(self._read_all)
        span = QPushButton("Set -pi..pi on every motor")
        span.setToolTip(
            "Writes zero_sta = 1 (0x7029). With zero_sta = 0 a joint sitting "
            "just below its zero reports about 6.28 rad instead of a small "
            "negative number, and every model that consumes the reading has "
            "to know to unwrap it.")
        span.clicked.connect(lambda: self._write_all(ZERO_STA, 1, "zero_sta"))
        clear = QPushButton("Clear add_offset")
        clear.setToolTip("Writes 0 to 0x702B on every motor")
        clear.clicked.connect(
            lambda: self._write_all(ADD_OFFSET, 0.0, "add_offset"))
        zero = QPushButton("Set zero here (selected)")
        zero.clicked.connect(self._set_zero)
        save = QPushButton("Save to flash (selected)")
        save.setToolTip("Type-22 save: without it every 0x20xx change is lost "
                        "at the next power-off")
        save.clicked.connect(self._save)

        layout = QVBoxLayout(self)
        layout.addWidget(note(
            "There are two different zeros, and confusing them wastes a day. "
            "<b>Set zero here</b> is the type-6 command: it makes the motor's "
            "present position its mechanical zero, permanently and in the "
            "motor. <b>add_offset</b> (0x702B) is a live software shift of "
            "the same reading, lost at power-off unless saved. And the "
            "offsets this wizard identifies are a third thing again - they "
            "live in the Kinematics tab and correct the URDF's idea of the "
            "joint, touching nothing in the motor at all. Prefer that one: "
            "it is the only one you can undo by editing a number."))
        layout.addWidget(self.table, 1)
        buttons = QHBoxLayout()
        for button in (read, span, clear, zero, save):
            buttons.addWidget(button)
        buttons.addStretch(1)
        layout.addLayout(buttons)

    def entered(self) -> None:
        self._rebuild()

    def _rebuild(self) -> None:
        keys = sorted(self.session.motors)
        self.table.setRowCount(len(keys))
        for row, key in enumerate(keys):
            item = QTableWidgetItem(motor_label(key))
            item.setData(Qt.UserRole, key)
            self.table.setItem(row, 0, item)
            for column in range(1, len(self.HEADERS)):
                self.table.setItem(row, column, QTableWidgetItem("-"))
        self.refresh()

    def refresh(self) -> None:
        keys = sorted(self.session.motors)
        if self.table.rowCount() != len(keys):
            self._rebuild()
            return
        for row, key in enumerate(keys):
            motor = self.session.motors[key]
            entry = self._readings.get(key, {})
            span = entry.get("zero_sta")
            self.table.item(row, 1).setText(
                "-" if span is None else
                ("-pi .. pi" if int(span) == 1 else "0 .. 2pi"))
            self.table.item(row, 2).setText(
                number(entry.get("add_offset"), 4, " rad"))
            self.table.item(row, 3).setText(
                number(entry.get("mechPos"), 5, " rad"))
            self.table.item(row, 4).setText(
                units.text(motor.state.position, "rad", sign=True)
                if motor.state.age <= STALE else "stale")

    def _selected(self) -> list:
        keys = []
        for item in self.table.selectedItems():
            key = self.table.item(item.row(), 0).data(Qt.UserRole)
            if key is not None and key not in keys:
                keys.append(key)
        return keys

    def _read_all(self) -> None:
        for key, motor in sorted(self.session.motors.items()):
            self._readings[key] = {
                "zero_sta": motor.read(0x7029, timeout=0.3),
                "add_offset": motor.read(ADD_OFFSET, timeout=0.3),
                "mechPos": motor.read(MECH_POS, timeout=0.3),
            }
        self.refresh()
        self.changed.emit()
        self.status.emit("Read the zero and range of every motor")

    def _write_all(self, index: int, value, name: str) -> None:
        if QMessageBox.question(
                self, f"Write {name} on every motor?",
                f"This writes {name} (0x{index:04X}) = {value} to all "
                f"{len(self.session.motors)} motor(s).\n\nThe write is live "
                "but volatile: it takes effect now and is lost at the next "
                "power-off unless you also save to flash.") != QMessageBox.Yes:
            return
        failed = []
        for key, motor in sorted(self.session.motors.items()):
            try:
                motor.write(index, value)
            except Exception as exc:
                failed.append(f"{motor_label(key)}: {exc}")
        if failed:
            QMessageBox.warning(self, f"Some {name} writes failed",
                                "\n".join(failed))
        self._read_all()

    def _set_zero(self) -> None:
        keys = self._selected()
        if not keys:
            QMessageBox.information(self, "Nothing selected",
                                    "Select the motor rows to zero.")
            return
        if QMessageBox.question(
                self, "Set mechanical zero",
                "Make the present position the mechanical zero of:\n\n  "
                + "\n  ".join(motor_label(k) for k in keys)
                + "\n\nEvery angle these motors report will shift, so any "
                "offset already identified against the old zero stops being "
                "correct. Do this before the arm-level calibration, not "
                "after it.") != QMessageBox.Yes:
            return
        for key in keys:
            self.session.motors[key].set_zero()
        self.status.emit(f"Zero set on {len(keys)} motor(s)")
        self._read_all()

    def _save(self) -> None:
        keys = self._selected()
        if not keys:
            QMessageBox.information(self, "Nothing selected",
                                    "Select the motor rows to save.")
            return
        for key in keys:
            self.session.motors[key].save()
        self.status.emit(f"Type-22 save sent to {len(keys)} motor(s)")

    def state(self) -> tuple[State, str]:
        if not self.session.motors:
            return State.FAIL, "no motors"
        if len(self._readings) < len(self.session.motors):
            return State.TODO, "not read yet"
        spans = {int(entry["zero_sta"]) for entry in self._readings.values()
                 if entry.get("zero_sta") is not None}
        if len(spans) > 1:
            return State.WARN, "the motors disagree on the angle range"
        offsets = [entry.get("add_offset") for entry in self._readings.values()]
        live = [v for v in offsets if v not in (None, 0.0)]
        if live:
            return State.WARN, f"{len(live)} motor(s) carry an add_offset"
        return State.OK, "one range, no stray software offsets"


# --------------------------------------------------------------------------
# 3 - the power-on position bootstrap
# --------------------------------------------------------------------------


class BootstrapStep(Step):
    TITLE = "Power-on bootstrap"
    BLURB = ("Whether each motor comes up believing it is where it really "
             "is - checked across a power cycle.")

    HEADERS = ["Motor", "chasu_offset", "chasu_angle_init", "mech_angle_init2",
               "turn no.", "mechPos", "vs snapshot"]

    def __init__(self, session, parent=None):
        super().__init__(session, parent)
        self._readings: dict = {}

        self.table = table(self.HEADERS, stretch=6)
        read = QPushButton("Read all")
        read.clicked.connect(self._read_all)
        snapshot = QPushButton("Snapshot (before the power cycle)")
        snapshot.clicked.connect(self._snapshot)
        forget = QPushButton("Forget snapshots")
        forget.clicked.connect(self._forget)

        layout = QVBoxLayout(self)
        layout.addWidget(note(
            "These motors carry two encoders - a fine one on the rotor and a "
            "coarse one on the output shaft - and combine them at power-on "
            "to work out which of the nine rotor turns the output is in. "
            "When that goes wrong the motor comes up believing it is a "
            "fraction of a turn from where it actually is, and everything "
            "downstream inherits the error.<br><br>"
            "<b>The check:</b> park the arm, press Snapshot, power-cycle the "
            "motors <i>without moving anything</i>, then Read all. The turn "
            "number must come back identical and mech_angle_init2 must agree "
            "to well under 2*pi/9 = 0.70 rad. A chasu_offset of zero means "
            "the encoder calibration never completed or was never saved."))
        layout.addWidget(self.table, 1)
        buttons = QHBoxLayout()
        for button in (read, snapshot, forget):
            buttons.addWidget(button)
        buttons.addStretch(1)
        layout.addLayout(buttons)

    def entered(self) -> None:
        self._rebuild()

    def _rebuild(self) -> None:
        keys = sorted(self.session.motors)
        self.table.setRowCount(len(keys))
        for row, key in enumerate(keys):
            item = QTableWidgetItem(motor_label(key))
            item.setData(Qt.UserRole, key)
            self.table.setItem(row, 0, item)
            for column in range(1, len(self.HEADERS)):
                self.table.setItem(row, column, QTableWidgetItem("-"))
        self.refresh()

    def refresh(self) -> None:
        keys = sorted(self.session.motors)
        if self.table.rowCount() != len(keys):
            self._rebuild()
            return
        for row, key in enumerate(keys):
            entry = self._readings.get(key, {})
            for column, name in enumerate(
                    ["chasu_offset", "chasu_angle_init", "mech_angle_init2",
                     "mech_angle_rotat", "mechPos"], start=1):
                self.table.item(row, column).setText(number(entry.get(name), 4))
            verdict, bad = self._compare(key)
            item = self.table.item(row, 6)
            item.setText(verdict)
            item.setForeground(Qt.red if bad else Qt.black)

    def _compare(self, key) -> tuple[str, bool]:
        """This motor's bootstrap against the snapshot taken before."""
        entry = self._readings.get(key)
        old = self.session.snapshots.get(key)
        if not entry:
            return "not read", False
        if not old:
            return "no snapshot", False
        if entry.get("chasu_offset") in (None, 0.0):
            return "chasu_offset is zero - encoder never calibrated", True
        turns_now, turns_then = (entry.get("mech_angle_rotat"),
                                 old.get("mech_angle_rotat"))
        if turns_now is not None and turns_then is not None \
                and int(turns_now) != int(turns_then):
            return (f"TURN NUMBER CHANGED {int(turns_then)} -> "
                    f"{int(turns_now)}"), True
        init_now, init_then = (entry.get("mech_angle_init2"),
                               old.get("mech_angle_init2"))
        if init_now is not None and init_then is not None:
            drift = abs(float(init_now) - float(init_then))
            # A tenth of the 2*pi/9 output sector: beyond that the bootstrap
            # is not merely noisy, it is landing somewhere else.
            return f"init drift {drift:.4f} rad", drift > 0.07
        return "repeats", False

    def _read_all(self) -> None:
        for key, motor in sorted(self.session.motors.items()):
            if not P.has_table(motor.model):
                self._readings[key] = {}
                continue
            self._readings[key] = {name: read_named(motor, name)
                                   for name in BOOTSTRAP}
        self.refresh()
        self.changed.emit()
        self.status.emit("Read the position bootstrap of every motor")

    def _snapshot(self) -> None:
        if not self._readings:
            self._read_all()
        self.session.snapshots.update(
            {key: dict(value) for key, value in self._readings.items() if value})
        self.session.save()
        self.refresh()
        self.changed.emit()
        self.status.emit(
            f"Snapshot taken for {len(self.session.snapshots)} motor(s) - "
            "now power-cycle without moving the arm, then Read all")

    def _forget(self) -> None:
        self.session.snapshots.clear()
        self.session.save()
        self.refresh()
        self.changed.emit()

    def state(self) -> tuple[State, str]:
        if not self._readings:
            return State.TODO, "not read yet"
        supported = [k for k, v in self._readings.items() if v]
        if not supported:
            return State.WARN, "no confirmed parameter table for these models"
        bad = [motor_label(key) for key in supported if self._compare(key)[1]]
        if bad:
            return State.FAIL, "bootstrap unreliable on " + ", ".join(bad)
        if not self.session.snapshots:
            return State.WARN, "read, but not yet checked across a power cycle"
        return State.OK, "the bootstrap repeats across a power cycle"


# --------------------------------------------------------------------------
# 4 - what the joint mechanically does
# --------------------------------------------------------------------------


class MotionStep(Step):
    TITLE = "Backlash and repeatability"
    BLURB = ("Move one joint and measure what comes back: response, "
             "hysteresis, and how well it returns to the same place.")

    HEADERS = ["Motor", "Response", "Backlash", "Repeatability", "Note"]

    def __init__(self, session, parent=None):
        super().__init__(session, parent)
        self._results: dict = {}
        self._test: dict = {}
        self._sequence = Sequence(self)
        self._sequence.finished.connect(self._finished)
        self._sequence.progress.connect(self.status)

        self.table = table(self.HEADERS, stretch=4)

        self.amplitude = QDoubleSpinBox()
        self.amplitude.setRange(0.5, 45.0)
        self.amplitude.setValue(8.0)
        self.amplitude.setSuffix(" deg")
        self.amplitude.setToolTip(
            "How far the joint is driven either side of where it is now. Big "
            "enough to clear the backlash it is measuring, small enough that "
            "the arm cannot reach anything.")
        self.settle = QDoubleSpinBox()
        self.settle.setRange(0.3, 8.0)
        self.settle.setValue(1.5)
        self.settle.setSuffix(" s")
        self.settle.setToolTip("Wait after each move before reading back")
        self.speed = QDoubleSpinBox()
        self.speed.setRange(0.02, 3.0)
        self.speed.setValue(0.25)
        self.speed.setDecimals(2)
        self.speed.setSuffix(" rad/s")
        self.current = QDoubleSpinBox()
        self.current.setRange(0.5, 40.0)
        self.current.setValue(4.0)
        self.current.setSuffix(" A")
        self.current.setToolTip(
            "Current ceiling for the test (0x7018). Keep it low: it is the "
            "difference between a joint that stalls against an obstruction "
            "and one that pushes through it.")

        self.run_button = QPushButton("Run the test on the selected motor")
        self.run_button.clicked.connect(self._run)
        self.stop_button = QPushButton("STOP")
        self.stop_button.setStyleSheet(
            "background: #c0392b; color: white; font-weight: bold; padding: 8px;")
        self.stop_button.clicked.connect(self._stop)

        settings = QHBoxLayout()
        for label, widget in (("Amplitude", self.amplitude),
                              ("Settle", self.settle),
                              ("Speed limit", self.speed),
                              ("Current limit", self.current)):
            settings.addWidget(QLabel(label))
            settings.addWidget(widget)
        settings.addStretch(1)
        settings.addWidget(self.run_button)
        settings.addWidget(self.stop_button)

        box = QGroupBox("Test")
        box_layout = QVBoxLayout(box)
        box_layout.addLayout(settings)

        layout = QVBoxLayout(self)
        layout.addWidget(note(
            "<b>This moves the arm.</b> The joint is driven a few degrees "
            "either side of where it is now, in position mode, with the "
            "current limited. Nothing else is checked - clear the space "
            "around the arm and keep STOP in reach.", "#c0392b"))
        layout.addWidget(note(
            "Backlash is measured the only way it can be: settle at the same "
            "command having arrived from above, then from below, and take "
            "the difference. Whatever it comes to is the floor on the "
            "arm-level calibration - a fit that reports a residual well "
            "below the backlash is fitting noise, and the same arm will read "
            "differently the next time it approaches a pose from the other "
            "side. This step is optional; skip it and the later steps still "
            "work, you just will not know what number to trust."))
        layout.addWidget(box)
        layout.addWidget(self.table, 1)

    def entered(self) -> None:
        self._rebuild()

    def _rebuild(self) -> None:
        keys = sorted(self.session.motors)
        self.table.setRowCount(len(keys))
        for row, key in enumerate(keys):
            item = QTableWidgetItem(motor_label(key))
            item.setData(Qt.UserRole, key)
            self.table.setItem(row, 0, item)
            for column in range(1, len(self.HEADERS)):
                self.table.setItem(row, column, QTableWidgetItem("-"))
        self.refresh()

    def refresh(self) -> None:
        keys = sorted(self.session.motors)
        if self.table.rowCount() != len(keys):
            self._rebuild()
            return
        self.run_button.setEnabled(not self._sequence.running)
        for row, key in enumerate(keys):
            result = self._results.get(key)
            if not result:
                continue
            self.table.item(row, 1).setText(f"{result['response'] * 100:.1f} %")
            self.table.item(row, 2).setText(
                f"{result['backlash']:.3f} deg")
            self.table.item(row, 3).setText(
                f"{result['repeat']:.3f} deg")
            item = self.table.item(row, 4)
            item.setText(result["note"])
            item.setForeground(Qt.red if result["bad"] else Qt.black)

    def _selected_key(self):
        rows = {item.row() for item in self.table.selectedItems()}
        if len(rows) != 1:
            return None
        return self.table.item(rows.pop(), 0).data(Qt.UserRole)

    # -- the test ---------------------------------------------------------

    def _run(self) -> None:
        key = self._selected_key()
        if key is None:
            QMessageBox.information(
                self, "Select one motor",
                "Pick exactly one row. The test drives that joint.")
            return
        motor = self.session.motors[key]
        if motor.state.age > STALE:
            QMessageBox.warning(
                self, "No feedback",
                f"{motor_label(key)} has not reported for "
                f"{motor.state.age:.1f} s. Where it is now is not known, so "
                "it must not be commanded.")
            return
        amplitude = self.amplitude.value()
        if QMessageBox.question(
                self, "Move this joint?",
                f"{motor_label(key)} will be driven {amplitude:g} deg above "
                f"and below where it is now, six moves in all, and returned "
                f"to the start.\n\nSpeed limit {self.speed.value():g} rad/s, "
                f"current limit {self.current.value():g} A.\n\nThere is no "
                "collision checking. Is the arm clear?") != QMessageBox.Yes:
            return

        import math
        step = math.radians(amplitude)
        settle = int(self.settle.value() * 1000)
        self._test = {"key": key, "motor": motor, "readings": {}}

        def prepare() -> None:
            motor.stop()
            motor.write(RUN_MODE, int(RunMode.POSITION_CSP))
            motor.write(LIMIT_SPD, self.speed.value())
            motor.write(LIMIT_CUR, self.current.value())
            motor.enable()
            start = motor.read(MECH_POS, timeout=0.3)
            self._test["start"] = float(
                start if start is not None else motor.state.position)
            motor.write(LOC_REF, self._test["start"])

        def go(delta: float):
            def move() -> None:
                motor.write(LOC_REF, self._test["start"] + delta)
            return move

        def grab(name: str):
            def read() -> None:
                value = motor.read(MECH_POS, timeout=0.3)
                self._test["readings"][name] = float(
                    value if value is not None else motor.state.position)
            return read

        self._sequence.run([
            ("preparing", 500, prepare),
            ("settling at the start", settle, go(0.0)),
            ("", 0, grab("start")),
            ("driving up", settle, go(step)),
            ("", 0, grab("up")),
            ("returning from above", settle, go(0.0)),
            ("", 0, grab("above")),
            ("driving down", settle, go(-step)),
            ("", 0, grab("down")),
            ("returning from below", settle, go(0.0)),
            ("", 0, grab("below")),
            ("driving up again", settle, go(step)),
            ("returning from above again", settle, go(0.0)),
            ("", 0, grab("again")),
            ("stopping", 0, motor.stop),
        ])
        self.refresh()

    def _stop(self) -> None:
        self._sequence.abort()
        motor = self._test.get("motor")
        if motor is not None:
            try:
                motor.stop()
            except Exception:
                log.exception("could not stop %s", motor)
        self.status.emit("Test aborted, motor stopped")
        self.refresh()

    def _finished(self, completed: bool) -> None:
        self.refresh()
        if not completed:
            self.status.emit(f"Test did not finish: {self._sequence.error}")
            return
        import math
        readings = self._test["readings"]
        step = math.radians(self.amplitude.value())
        response = abs(readings["up"] - readings["start"]) / step if step else 0.0
        backlash = math.degrees(abs(readings["above"] - readings["below"]))
        repeat = math.degrees(abs(readings["again"] - readings["above"]))

        message, bad = "", False
        if response < 0.8:
            message = ("the joint moved far less than commanded - raise the "
                       "current limit, or something is in the way")
            bad = True
        elif backlash > 1.0:
            message = "that is a lot of backlash for a calibration to sit on"
            bad = True
        else:
            message = (f"the arm-level fit cannot beat about "
                       f"{backlash:.2f} deg at this joint")
        self._results[self._test["key"]] = {
            "response": response, "backlash": backlash, "repeat": repeat,
            "note": message, "bad": bad,
        }
        self.refresh()
        self.changed.emit()
        self.status.emit(f"Backlash {backlash:.3f} deg, "
                         f"repeatability {repeat:.3f} deg")

    def shutdown(self) -> None:
        if self._sequence.running:
            self._stop()

    def state(self) -> tuple[State, str]:
        if not self._results:
            return State.TODO, "optional - not measured"
        bad = [motor_label(key) for key, value in self._results.items()
               if value["bad"]]
        if bad:
            return State.WARN, "look at " + ", ".join(bad)
        worst = max(value["backlash"] for value in self._results.values())
        return State.OK, (f"{len(self._results)} joint(s) measured, worst "
                          f"backlash {worst:.2f} deg")
