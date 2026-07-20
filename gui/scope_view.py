"""Real-time oscilloscope.

Two data sources feed the plot:

* **Feedback frames** (type 2) - position, velocity, torque and temperature.
  These are the fastest stream the firmware offers; enable active reporting
  and they arrive every 10 ms without any request traffic.
* **Polled parameters** (type 17) - anything else in the table, round-robined
  by a :class:`~robstride.poller.ParamPoller`.
"""

from __future__ import annotations

import csv
import logging
import time
from collections import deque

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QCheckBox, QDoubleSpinBox, QFileDialog, QHBoxLayout, QLabel,
    QListWidget, QListWidgetItem, QMessageBox, QPushButton, QSpinBox,
    QSplitter, QVBoxLayout, QWidget,
)

from robstride import Motor, protocol as proto
from robstride import params as P
from robstride.poller import ParamPoller, Sample

log = logging.getLogger(__name__)

pg.setConfigOptions(antialias=True, background=None, foreground="k")

PEN_COLORS = ["#1f77b4", "#d62728", "#2ca02c", "#ff7f0e", "#9467bd",
              "#8c564b", "#17becf", "#bcbd22"]

#: Pseudo-indices for the fields carried in a feedback frame.
FEEDBACK_SOURCES = {
    -1: ("position (feedback)", "rad"),
    -2: ("velocity (feedback)", "rad/s"),
    -3: ("torque (feedback)", "Nm"),
    -4: ("temperature (feedback)", "C"),
}


class Channel:
    """One plotted trace."""

    __slots__ = ("index", "label", "unit", "times", "values", "curve")

    def __init__(self, index: int, label: str, unit: str, depth: int, curve):
        self.index = index
        self.label = label
        self.unit = unit
        self.times: deque[float] = deque(maxlen=depth)
        self.values: deque[float] = deque(maxlen=depth)
        self.curve = curve

    def append(self, t: float, v: float) -> None:
        self.times.append(t)
        self.values.append(v)

    def clear(self) -> None:
        self.times.clear()
        self.values.clear()


class ScopeView(QWidget):
    status = Signal(str)
    sample_ready = Signal(int, float, float)   # index, timestamp, value

    def __init__(self, parent=None):
        super().__init__(parent)
        self.motor: Motor | None = None
        self.poller: ParamPoller | None = None
        self.channels: dict[int, Channel] = {}
        self._running = False
        self._t0 = time.time()

        self.sample_ready.connect(self._append_sample)
        self._build_ui()

    # -- construction -----------------------------------------------------

    def _build_ui(self) -> None:
        self.source_list = QListWidget()
        self.source_list.setSelectionMode(QListWidget.NoSelection)
        for index, (label, unit) in FEEDBACK_SOURCES.items():
            self._add_source_item(index, label, unit, checked=index in (-1, -2, -3))
        for param in P.PARAMS:
            if param.dtype == "string":
                continue
            self._add_source_item(param.index,
                                  f"0x{param.index:04X}  {param.name}",
                                  param.unit,
                                  checked=param.index in P.SCOPE_DEFAULTS)
        self.source_list.itemChanged.connect(self._rebuild_channels)

        self.start_button = QPushButton("Start")
        self.start_button.setCheckable(True)
        self.start_button.toggled.connect(self._toggle_run)

        self.clear_button = QPushButton("Clear")
        self.clear_button.clicked.connect(self._clear)

        self.export_button = QPushButton("Export CSV")
        self.export_button.clicked.connect(self._export)

        self.active_report = QCheckBox("Active reporting")
        self.active_report.setToolTip(
            "Type 24 - motor pushes feedback frames without being polled")
        self.active_report.toggled.connect(self._toggle_active_report)

        self.report_interval = QSpinBox()
        self.report_interval.setRange(1, 200)
        self.report_interval.setValue(1)
        self.report_interval.setToolTip(
            "0x7026 EPScan_time: 1 = 10 ms, each step adds 5 ms")
        self.report_interval.valueChanged.connect(self._set_report_interval)

        self.poll_rate = QDoubleSpinBox()
        self.poll_rate.setRange(0.01, 2.0)
        self.poll_rate.setSingleStep(0.01)
        self.poll_rate.setValue(0.05)
        self.poll_rate.setSuffix(" s")
        self.poll_rate.valueChanged.connect(
            lambda v: self.poller and self.poller.set_interval(v))

        self.window = QDoubleSpinBox()
        self.window.setRange(1.0, 600.0)
        self.window.setValue(20.0)
        self.window.setSuffix(" s window")

        self.depth = QSpinBox()
        self.depth.setRange(100, 200000)
        self.depth.setValue(20000)
        self.depth.setToolTip("Samples kept per channel")

        self.autoscale = QCheckBox("Auto Y")
        self.autoscale.setChecked(True)

        self.plot = pg.PlotWidget()
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setLabel("bottom", "time", units="s")
        self.legend = self.plot.addLegend(offset=(-10, 10))

        controls = QHBoxLayout()
        controls.addWidget(self.start_button)
        controls.addWidget(self.clear_button)
        controls.addWidget(self.export_button)
        controls.addSpacing(12)
        controls.addWidget(self.active_report)
        controls.addWidget(self.report_interval)
        controls.addSpacing(12)
        controls.addWidget(QLabel("poll"))
        controls.addWidget(self.poll_rate)
        controls.addWidget(self.window)
        controls.addWidget(self.depth)
        controls.addWidget(self.autoscale)
        controls.addStretch(1)

        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.addWidget(QLabel("Channels"))
        left_layout.addWidget(self.source_list)

        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.addLayout(controls)
        right_layout.addWidget(self.plot, 1)

        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(left)
        splitter.addWidget(right)
        splitter.setStretchFactor(1, 4)

        layout = QVBoxLayout(self)
        layout.addWidget(splitter)

        self._timer = pg.QtCore.QTimer(self)
        self._timer.timeout.connect(self._redraw)
        self._timer.start(50)

        self._rebuild_channels()

    def _add_source_item(self, index: int, label: str, unit: str,
                         checked: bool) -> None:
        item = QListWidgetItem(f"{label}  [{unit}]" if unit else label)
        item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
        item.setCheckState(Qt.Checked if checked else Qt.Unchecked)
        item.setData(Qt.UserRole, (index, label, unit))
        self.source_list.addItem(item)

    # -- motor binding ----------------------------------------------------

    def set_motor(self, motor: Motor | None) -> None:
        was_running = self._running
        if was_running:
            self.start_button.setChecked(False)

        if self.motor is not None:
            self.motor.link.remove_listener(proto.CommType.FEEDBACK,
                                            self._on_feedback_frame)
        self.motor = motor
        if motor is not None:
            motor.link.add_listener(proto.CommType.FEEDBACK,
                                    self._on_feedback_frame)
            self.poller = ParamPoller(motor, self._on_poll_sample,
                                      interval=self.poll_rate.value())
            self._rebuild_channels()
        else:
            self.poller = None

    # -- channels ---------------------------------------------------------

    def _selected_sources(self) -> list[tuple[int, str, str]]:
        out = []
        for row in range(self.source_list.count()):
            item = self.source_list.item(row)
            if item.checkState() == Qt.Checked:
                out.append(item.data(Qt.UserRole))
        return out

    def _rebuild_channels(self) -> None:
        selected = self._selected_sources()
        wanted = {index for index, _, _ in selected}

        for index in list(self.channels):
            if index not in wanted:
                channel = self.channels.pop(index)
                self.plot.removeItem(channel.curve)
                try:
                    self.legend.removeItem(channel.label)
                except Exception:
                    pass

        for position, (index, label, unit) in enumerate(selected):
            if index in self.channels:
                continue
            pen = pg.mkPen(PEN_COLORS[position % len(PEN_COLORS)], width=2)
            curve = self.plot.plot([], [], pen=pen, name=label)
            self.channels[index] = Channel(index, label, unit,
                                           self.depth.value(), curve)

        if self.poller is not None:
            self.poller.set_indices([i for i in wanted if i >= 0])

    # -- data sources -----------------------------------------------------

    def _on_feedback_frame(self, can_id: int, data: bytes) -> None:
        if self.motor is None or not self._running:
            return
        _, data2, _ = proto.unpack_id(can_id)
        if (data2 & 0xFF) != self.motor.motor_id:
            return
        fb = proto.decode_feedback(can_id, data, self.motor.limits)
        now = time.time()
        for index, value in ((-1, fb.position), (-2, fb.velocity),
                             (-3, fb.torque), (-4, fb.temperature)):
            if index in self.channels:
                self.sample_ready.emit(index, now, value)

    def _on_poll_sample(self, sample: Sample) -> None:
        if not self._running:
            return
        try:
            value = float(sample.value)
        except (TypeError, ValueError):
            return
        self.sample_ready.emit(sample.index, sample.timestamp, value)

    def _append_sample(self, index: int, timestamp: float, value: float) -> None:
        channel = self.channels.get(index)
        if channel is not None:
            channel.append(timestamp - self._t0, value)

    # -- run control ------------------------------------------------------

    def _toggle_run(self, on: bool) -> None:
        if on and self.motor is None:
            QMessageBox.warning(self, "No motor", "Select a motor first.")
            self.start_button.setChecked(False)
            return
        self._running = on
        self.start_button.setText("Stop" if on else "Start")
        if self.poller is None:
            return
        if on:
            self.poller.set_indices([i for i in self.channels if i >= 0])
            self.poller.start()
            self.status.emit("Scope running")
        else:
            self.poller.stop()
            self.status.emit("Scope stopped")

    def _clear(self) -> None:
        self._t0 = time.time()
        for channel in self.channels.values():
            channel.clear()

    def _redraw(self) -> None:
        if not self.channels:
            return
        span = self.window.value()
        latest = 0.0
        for channel in self.channels.values():
            if channel.times:
                latest = max(latest, channel.times[-1])

        for channel in self.channels.values():
            if not channel.times:
                channel.curve.setData([], [])
                continue
            times = np.fromiter(channel.times, dtype=float)
            values = np.fromiter(channel.values, dtype=float)
            mask = times >= latest - span
            channel.curve.setData(times[mask], values[mask])

        if latest > span:
            self.plot.setXRange(latest - span, latest, padding=0)
        if self.autoscale.isChecked():
            self.plot.enableAutoRange(axis="y")

    # -- motor-side reporting --------------------------------------------

    def _toggle_active_report(self, on: bool) -> None:
        if self.motor is None:
            if on:
                self.active_report.setChecked(False)
            return
        try:
            self.motor.set_active_report(on)
            self.status.emit(
                f"Active reporting {'enabled' if on else 'disabled'} "
                f"on motor {self.motor.motor_id}")
        except Exception as exc:
            QMessageBox.critical(self, "Command failed", str(exc))

    def _set_report_interval(self, value: int) -> None:
        if self.motor is None:
            return
        try:
            self.motor.write(0x7026, value)
            self.status.emit(f"Report interval set to {10 + (value - 1) * 5} ms")
        except Exception as exc:
            log.debug("EPScan_time write failed: %s", exc)

    # -- export -----------------------------------------------------------

    def _export(self) -> None:
        if not any(c.times for c in self.channels.values()):
            QMessageBox.information(self, "Nothing to export",
                                    "Capture some data first.")
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "Export waveform", "robstride_scope.csv", "CSV (*.csv)")
        if not path:
            return
        try:
            with open(path, "w", newline="", encoding="utf-8") as fh:
                writer = csv.writer(fh)
                writer.writerow(["channel", "unit", "time_s", "value"])
                for channel in self.channels.values():
                    for t, v in zip(channel.times, channel.values):
                        writer.writerow([channel.label, channel.unit,
                                         f"{t:.6f}", v])
        except Exception as exc:
            QMessageBox.critical(self, "Export failed", str(exc))
            return
        self.status.emit(f"Waveform exported to {path}")

    def shutdown(self) -> None:
        self._timer.stop()
        if self.poller is not None:
            self.poller.stop()
        if self.motor is not None:
            self.motor.link.remove_listener(proto.CommType.FEEDBACK,
                                            self._on_feedback_frame)
