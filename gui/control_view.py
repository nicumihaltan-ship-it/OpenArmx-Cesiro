"""Motor control: enable/stop, mode selection, setpoints and jog.

Everything here moves real hardware. The panel keeps the motor disabled until
you explicitly enable it, shows live feedback, and offers a stop button that
stays reachable at all times.
"""

from __future__ import annotations

import logging
import time

from PySide6.QtCore import QTimer, Signal
from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QMessageBox, QPushButton, QVBoxLayout, QWidget,
)

from robstride import Motor, RunMode

from .units import AngleSpin, units

log = logging.getLogger(__name__)

MODE_LABELS = [
    ("Operation control (MIT-style)", RunMode.OPERATION),
    ("Position - PP (profile)", RunMode.POSITION_PP),
    ("Velocity", RunMode.VELOCITY),
    ("Current (Iq)", RunMode.CURRENT),
    ("Position - CSP (cyclic sync)", RunMode.POSITION_CSP),
]


def _spin(minimum, maximum, value=0.0, step=0.1, suffix="", decimals=3):
    box = QDoubleSpinBox()
    box.setRange(minimum, maximum)
    box.setSingleStep(step)
    box.setDecimals(decimals)
    box.setValue(value)
    if suffix:
        box.setSuffix(f" {suffix}")
    return box


class ControlView(QWidget):
    status = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.motor: Motor | None = None
        self._last_mode: RunMode | None = None
        self._build_ui()
        units.changed.connect(self._on_units_changed)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._update_state)
        self._timer.start(100)

    # -- construction -----------------------------------------------------

    def _build_ui(self) -> None:
        # -- live state
        self.state_labels: dict[str, QLabel] = {}
        state_box = QGroupBox("Live feedback")
        state_grid = QGridLayout(state_box)
        for col, (key, title) in enumerate([
                ("position", "Position"), ("velocity", "Velocity"),
                ("torque", "Torque"), ("temperature", "Temperature"),
                ("mode", "Mode"), ("faults", "Faults")]):
            value = QLabel("-")
            value.setStyleSheet("font-size: 15px; font-weight: 600;")
            state_grid.addWidget(QLabel(title), 0, col)
            state_grid.addWidget(value, 1, col)
            self.state_labels[key] = value

        # -- lifecycle
        self.enable_button = QPushButton("Enable")
        self.enable_button.clicked.connect(self._enable)
        self.stop_button = QPushButton("STOP")
        self.stop_button.setStyleSheet(
            "background: #c0392b; color: white; font-weight: bold; padding: 8px;")
        self.stop_button.clicked.connect(self._stop)
        self.clear_fault_button = QPushButton("Clear faults")
        self.clear_fault_button.clicked.connect(lambda: self._stop(clear=True))
        self.zero_button = QPushButton("Set zero here")
        self.zero_button.clicked.connect(self._set_zero)

        lifecycle = QHBoxLayout()
        lifecycle.addWidget(self.enable_button)
        lifecycle.addWidget(self.stop_button, 2)
        lifecycle.addWidget(self.clear_fault_button)
        lifecycle.addWidget(self.zero_button)

        # -- mode selection
        self.mode_box = QComboBox()
        for label, mode in MODE_LABELS:
            self.mode_box.addItem(label, mode)
        self.apply_mode_button = QPushButton("Apply mode")
        self.apply_mode_button.setToolTip(
            "Writes 0x7005. Not required before the Apply buttons below - "
            "each of those sets the mode it needs on its own.")
        self.apply_mode_button.clicked.connect(self._apply_mode)

        # The dropdown is only a request; this shows what the motor is really
        # in, read back from 0x7005. They drift apart easily - jogging, for
        # instance, forces velocity mode.
        self.actual_mode = QLabel("motor mode: unknown")
        self.actual_mode.setStyleSheet("color: gray;")
        self.refresh_mode_button = QPushButton("Read")
        self.refresh_mode_button.setToolTip("Re-read run_mode (0x7005)")
        self.refresh_mode_button.clicked.connect(self._read_mode)

        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel("Control mode"))
        mode_row.addWidget(self.mode_box, 1)
        mode_row.addWidget(self.apply_mode_button)
        mode_row.addWidget(self.actual_mode)
        mode_row.addWidget(self.refresh_mode_button)

        # -- per-mode setpoints
        self.op_torque = _spin(-120, 120, 0.0, 0.5, "Nm")
        self.op_position = AngleSpin(-12.57, 12.57, 0.0, 0.1, "rad")
        self.op_velocity = AngleSpin(-15, 15, 0.0, 0.1, "rad/s")
        self.op_kp = _spin(0, 500, 0.0, 1.0)
        self.op_kd = _spin(0, 100, 1.0, 0.1)
        self.op_start = QPushButton("Enter mode")
        self.op_start.setToolTip(
            "Switch to operation control and enable, before streaming type-1 "
            "commands with Send.")
        self.op_start.clicked.connect(self._start_motion_mode)
        self.op_send = QPushButton("Send")
        self.op_send.clicked.connect(self._send_motion)
        self.op_stream = QCheckBox("Stream at 100 Hz")
        self.op_stream.toggled.connect(self._toggle_stream)

        op_box = QGroupBox("Operation control (type 1)")
        op_form = QFormLayout(op_box)
        op_form.addRow("Feed-forward torque", self.op_torque)
        op_form.addRow("Target position", self.op_position)
        op_form.addRow("Target velocity", self.op_velocity)
        op_form.addRow("Kp", self.op_kp)
        op_form.addRow("Kd", self.op_kd)
        op_row = QHBoxLayout()
        op_row.addWidget(self.op_start)
        op_row.addWidget(self.op_send)
        op_row.addWidget(self.op_stream)
        op_form.addRow(op_row)

        self.current_ref = _spin(-90, 90, 0.0, 0.1, "A")
        self.current_send = QPushButton("Apply Iq")
        self.current_send.clicked.connect(self._apply_current)

        current_box = QGroupBox("Current mode")
        current_form = QFormLayout(current_box)
        current_form.addRow("Iq command", self.current_ref)
        current_form.addRow(self.current_send)

        self.speed_ref = AngleSpin(-20, 20, 0.0, 0.1, "rad/s")
        self.speed_limit_cur = _spin(0, 90, 10.0, 0.5, "A")
        self.speed_accel = AngleSpin(0, 200, 15.0, 1.0, "rad/s^2")
        self.speed_send = QPushButton("Apply speed")
        self.speed_send.clicked.connect(self._apply_speed)

        speed_box = QGroupBox("Velocity mode")
        speed_form = QFormLayout(speed_box)
        speed_form.addRow("Speed command", self.speed_ref)
        speed_form.addRow("Current limit", self.speed_limit_cur)
        speed_form.addRow("Acceleration", self.speed_accel)
        speed_form.addRow(self.speed_send)

        self.pos_ref = AngleSpin(-12.57, 12.57, 0.0, 0.05, "rad")
        self.pos_speed = AngleSpin(0, 20, 2.0, 0.1, "rad/s")
        self.pos_accel = AngleSpin(0, 200, 10.0, 1.0, "rad/s^2")
        self.pos_send = QPushButton("Go to position")
        self.pos_send.setToolTip(
            "Sets the position mode selected above, enables the motor, then "
            "sends the target. No need to press Enable or Apply mode first.")
        self.pos_send.clicked.connect(self._apply_position)

        pos_box = QGroupBox("Position mode (PP / CSP)")
        pos_form = QFormLayout(pos_box)
        pos_form.addRow("Position command", self.pos_ref)
        pos_form.addRow("Speed limit", self.pos_speed)
        pos_form.addRow("Acceleration (PP)", self.pos_accel)
        pos_form.addRow(self.pos_send)

        # -- jog
        self.jog_speed = AngleSpin(0.05, 10, 1.0, 0.1, "rad/s")
        jog_minus = QPushButton("JOG -")
        jog_plus = QPushButton("JOG +")
        jog_stop = QPushButton("Jog stop")
        jog_minus.pressed.connect(lambda: self._jog(-1))
        jog_plus.pressed.connect(lambda: self._jog(1))
        jog_stop.clicked.connect(lambda: self._jog(0))

        jog_box = QGroupBox("Jog")
        jog_row = QHBoxLayout(jog_box)
        jog_row.addWidget(QLabel("speed"))
        jog_row.addWidget(self.jog_speed)
        jog_row.addWidget(jog_minus)
        jog_row.addWidget(jog_plus)
        jog_row.addWidget(jog_stop)

        modes = QGridLayout()
        modes.addWidget(op_box, 0, 0, 2, 1)
        modes.addWidget(current_box, 0, 1)
        modes.addWidget(speed_box, 1, 1)
        modes.addWidget(pos_box, 0, 2, 2, 1)

        self.info = QLabel("No motor selected")
        self.info.setStyleSheet("color: gray;")

        layout = QVBoxLayout(self)
        layout.addWidget(self.info)
        layout.addWidget(state_box)
        layout.addLayout(lifecycle)
        layout.addLayout(mode_row)
        layout.addLayout(modes)
        layout.addWidget(jog_box)
        layout.addStretch(1)

        self._stream_timer = QTimer(self)
        self._stream_timer.timeout.connect(self._send_motion)

    # -- motor binding ----------------------------------------------------

    def set_motor(self, motor: Motor | None) -> None:
        self.op_stream.setChecked(False)
        self.motor = motor
        self._last_mode = None
        if motor is None:
            self.info.setText("No motor selected")
            self.actual_mode.setText("motor mode: unknown")
            self.actual_mode.setStyleSheet("color: gray;")
            return
        limits = motor.limits
        self.info.setText(
            f"Motor id {motor.motor_id} on {motor.link.channel} - model "
            f"{motor.model}" + ("" if limits.verified else
                                "   [scaling constants UNVERIFIED for this model]"))
        # Retune the spin boxes to the selected model's real envelope.
        self.op_torque.setRange(limits.t_min, limits.t_max)
        self.op_position.setRadRange(limits.p_min, limits.p_max)
        self.op_velocity.setRadRange(limits.v_min, limits.v_max)
        # Kp/Kd scale differently per model (500/5 on RS00-02, 5000/100 on
        # RS03/04), so the spin boxes have to follow the selected model.
        self.op_kp.setRange(limits.kp_min, limits.kp_max)
        self.op_kd.setRange(limits.kd_min, limits.kd_max)
        self.pos_ref.setRadRange(limits.p_min, limits.p_max)
        self.current_ref.setRange(-limits.i_max, limits.i_max)
        self.speed_limit_cur.setRange(0, limits.i_max)
        self._read_mode()

    def _require(self) -> Motor | None:
        if self.motor is None:
            QMessageBox.warning(self, "No motor", "Select a motor first.")
        return self.motor

    # -- lifecycle --------------------------------------------------------

    def _enable(self) -> None:
        motor = self._require()
        if motor is None:
            return
        if QMessageBox.question(
                self, "Enable motor",
                f"Enable motor {motor.motor_id} on {motor.link.channel}?\n\n"
                "The actuator will start holding torque and may move.") \
                != QMessageBox.Yes:
            return
        try:
            motor.enable()
            self.status.emit(f"Motor {motor.motor_id} enabled")
        except Exception as exc:
            QMessageBox.critical(self, "Enable failed", str(exc))

    def _stop(self, clear: bool = False) -> None:
        motor = self.motor
        if motor is None:
            return
        self.op_stream.setChecked(False)
        try:
            motor.stop(clear_fault=clear)
            self.status.emit(
                f"Motor {motor.motor_id} stopped" + (" and faults cleared" if clear else ""))
        except Exception as exc:
            QMessageBox.critical(self, "Stop failed", str(exc))

    def _set_zero(self) -> None:
        motor = self._require()
        if motor is None:
            return
        if QMessageBox.question(
                self, "Set mechanical zero",
                f"Set the current position of motor {motor.motor_id} as zero?\n\n"
                "Not available in PP mode - the firmware blocks it there.") \
                != QMessageBox.Yes:
            return
        motor.set_zero()
        self.status.emit(f"Zero set on motor {motor.motor_id}")

    # -- modes ------------------------------------------------------------

    def _apply_mode(self) -> None:
        motor = self._require()
        if motor is None:
            return
        mode = self.mode_box.currentData()
        try:
            # Stop first: the manual is explicit that a mode change while
            # running is undefined behaviour.
            motor.stop()
            time.sleep(0.01)
            motor.set_run_mode(mode)
            self._refresh_mode_label(mode)
            self.status.emit(
                f"run_mode = {mode.name} on motor {motor.motor_id} "
                f"(motor is stopped - press Enable, or just use an Apply button)")
        except Exception as exc:
            QMessageBox.critical(self, "Mode change failed", str(exc))

    def _write(self, index: int, value: float, label: str) -> None:
        motor = self._require()
        if motor is None:
            return
        try:
            motor.write(index, value)
            self.status.emit(f"{label} = {value:g} on motor {motor.motor_id}")
        except Exception as exc:
            QMessageBox.critical(self, "Write failed", str(exc))

    def _read_mode(self) -> None:
        motor = self.motor
        if motor is None:
            return
        raw = motor.read(0x7005, timeout=0.3)
        if raw is None:
            self.actual_mode.setText("motor mode: no reply")
            self.actual_mode.setStyleSheet("color: #c0392b;")
            return
        try:
            self._refresh_mode_label(RunMode(int(raw)))
        except ValueError:
            self.actual_mode.setText(f"motor mode: unknown ({int(raw)})")
            self.actual_mode.setStyleSheet("color: #c0392b;")

    def _refresh_mode_label(self, mode: RunMode) -> None:
        self.actual_mode.setText(f"motor mode: {mode.name}")
        matches = mode is self.mode_box.currentData()
        self.actual_mode.setStyleSheet(
            "color: #27ae60;" if matches else "color: #e67e22;")

    def _start_motion_mode(self) -> None:
        """Enter operation-control mode, ready for type-1 commands."""
        motor = self._require()
        if motor is None:
            return
        try:
            self._ensure_mode(motor, RunMode.OPERATION)
            self.status.emit("Operation control mode ready")
        except Exception as exc:
            QMessageBox.critical(self, "Mode change failed", str(exc))

    def _send_motion(self) -> None:
        motor = self.motor
        if motor is None:
            return
        try:
            motor.motion_control(
                torque=self.op_torque.value(), position=self.op_position.rad(),
                velocity=self.op_velocity.rad(), kp=self.op_kp.value(),
                kd=self.op_kd.value())
        except Exception as exc:
            self.op_stream.setChecked(False)
            QMessageBox.critical(self, "Motion command failed", str(exc))

    def _toggle_stream(self, on: bool) -> None:
        if on and self.motor is None:
            self.op_stream.setChecked(False)
            return
        if on:
            self._stream_timer.start(10)
        else:
            self._stream_timer.stop()

    def _ensure_mode(self, motor: Motor, mode: RunMode) -> None:
        """Put the motor into ``mode`` and enable it, ready for a setpoint.

        Writing a setpoint that belongs to a different mode is silently
        ignored by the firmware - a loc_ref written while in velocity mode
        does nothing and reports no error. So every "apply" path has to
        guarantee the mode itself rather than assume the user sequenced the
        buttons correctly.

        The manual requires the motor be stopped before a mode change, so
        switching goes stop -> write run_mode -> enable. When the motor is
        already in the right mode that dance is skipped, since a needless
        disable/enable makes the joint drop and re-grab its load.
        """
        current = motor.read(0x7005, timeout=0.2)
        if current is None or int(current) != int(mode):
            motor.stop()
            time.sleep(0.01)
            motor.write(0x7005, int(mode))
            time.sleep(0.01)
        motor.enable()
        self._last_mode = mode
        self._refresh_mode_label(mode)

    def _apply_speed(self) -> None:
        motor = self._require()
        if motor is None:
            return
        try:
            self._ensure_mode(motor, RunMode.VELOCITY)
            motor.write(0x7018, self.speed_limit_cur.value())
            motor.write(0x7022, self.speed_accel.rad())
            motor.write(0x700A, self.speed_ref.rad())
            self.status.emit(
                f"Speed {units.text(self.speed_ref.rad(), 'rad/s')} applied")
        except Exception as exc:
            QMessageBox.critical(self, "Write failed", str(exc))

    def _apply_current(self) -> None:
        motor = self._require()
        if motor is None:
            return
        try:
            self._ensure_mode(motor, RunMode.CURRENT)
            motor.write(0x7006, self.current_ref.value())
            self.status.emit(f"Iq {self.current_ref.value():g} A applied")
        except Exception as exc:
            QMessageBox.critical(self, "Write failed", str(exc))

    def _apply_position(self) -> None:
        motor = self._require()
        if motor is None:
            return
        # PP and CSP are different firmware modes with different speed-limit
        # registers, so honour whichever the dropdown selects.
        mode = self.mode_box.currentData()
        if mode not in (RunMode.POSITION_PP, RunMode.POSITION_CSP):
            mode = RunMode.POSITION_CSP
        try:
            self._ensure_mode(motor, mode)
            if mode is RunMode.POSITION_PP:
                motor.write(0x7024, self.pos_speed.rad())
                motor.write(0x7025, self.pos_accel.rad())
            else:
                motor.write(0x7017, self.pos_speed.rad())
            motor.write(0x7016, self.pos_ref.rad())
            self.status.emit(
                f"Moving to {units.text(self.pos_ref.rad(), 'rad')} in {mode.name}")
        except Exception as exc:
            QMessageBox.critical(self, "Write failed", str(exc))

    def _jog(self, direction: int) -> None:
        motor = self._require()
        if motor is None:
            return
        speed = self.jog_speed.rad() * direction
        try:
            self._ensure_mode(motor, RunMode.VELOCITY)
            motor.write(0x700A, speed)
            self.status.emit(f"Jog at {units.text(speed, 'rad/s', sign=True)}"
                             if direction else "Jog stopped")
        except Exception as exc:
            QMessageBox.critical(self, "Jog failed", str(exc))

    # -- live state -------------------------------------------------------

    def _on_units_changed(self, _degrees: bool) -> None:
        # The spin boxes re-render themselves; the feedback labels are
        # rewritten on the next tick, which is 100 ms away at most.
        self._update_state()

    def _update_state(self) -> None:
        if self.motor is None:
            return
        state = self.motor.state
        stale = state.age > 1.0
        self.state_labels["position"].setText(
            units.text(state.position, "rad", sign=True))
        self.state_labels["velocity"].setText(
            units.text(state.velocity, "rad/s", rad=3, sign=True))
        self.state_labels["torque"].setText(f"{state.torque:+.3f} Nm")
        self.state_labels["temperature"].setText(f"{state.temperature:.1f} C")
        self.state_labels["mode"].setText(state.mode.name)
        faults = ", ".join(state.faults) if state.faults else "none"
        self.state_labels["faults"].setText(faults)
        self.state_labels["faults"].setStyleSheet(
            "font-size: 15px; font-weight: 600; color: %s;"
            % ("#c0392b" if state.faults else "#27ae60"))

        colour = "gray" if stale else "black"
        for key in ("position", "velocity", "torque", "temperature", "mode"):
            self.state_labels[key].setStyleSheet(
                f"font-size: 15px; font-weight: 600; color: {colour};")

    def shutdown(self) -> None:
        self._stream_timer.stop()
        self._timer.stop()
