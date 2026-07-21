"""Main window: connection dock plus the four working tabs."""

from __future__ import annotations

import logging

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QComboBox, QDockWidget, QLabel, QMainWindow, QMessageBox, QStatusBar,
    QTabWidget, QToolBar,
)

from robstride import unverified

from .connection import ConnectionPanel
from .control_view import ControlView
from .params_view import ParamsView
from .scope_view import ScopeView
from .trace_view import TraceView
from .units import units

log = logging.getLogger(__name__)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("OpenArmX - RobStride CAN configurator")
        self.resize(1500, 950)

        self.connection = ConnectionPanel()
        dock = QDockWidget("Connection", self)
        dock.setWidget(self.connection)
        dock.setAllowedAreas(Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea)
        self.addDockWidget(Qt.LeftDockWidgetArea, dock)

        self.params_view = ParamsView()
        self.scope_view = ScopeView()
        self.trace_view = TraceView()
        self.control_view = ControlView()

        self._build_toolbar()

        self.tabs = QTabWidget()
        self.tabs.addTab(self.params_view, "Parameters")
        self.tabs.addTab(self.scope_view, "Oscilloscope")
        self.tabs.addTab(self.trace_view, "CAN trace")
        self.tabs.addTab(self.control_view, "Control")
        self.setCentralWidget(self.tabs)

        self.setStatusBar(QStatusBar())
        self.bus_label = QLabel("")
        self.statusBar().addPermanentWidget(self.bus_label)

        self.connection.motor_selected.connect(self._on_motor_selected)
        self.connection.inventory_changed.connect(self._on_inventory_changed)
        for view in (self.params_view, self.scope_view, self.trace_view,
                     self.control_view):
            view.status.connect(self.statusBar().showMessage)

        self._stats_timer = QTimer(self)
        self._stats_timer.timeout.connect(self._update_stats)
        self._stats_timer.start(500)

        QTimer.singleShot(400, self._warn_unverified_models)

    # -- construction -----------------------------------------------------

    def _build_toolbar(self) -> None:
        """The angle-unit selector, which every tab honours."""
        self.angle_units = QComboBox()
        self.angle_units.addItem("Degrees", True)
        self.angle_units.addItem("Radians", False)
        self.angle_units.setCurrentIndex(0 if units.degrees else 1)
        self.angle_units.setToolTip(
            "Display and entry only. The protocol, the parameter files and "
            "the manuals are all radians, and exports stay canonical - switch "
            "to radians when cross-checking a value against the manual.")
        self.angle_units.currentIndexChanged.connect(
            lambda _: units.set_degrees(self.angle_units.currentData()))

        toolbar = QToolBar("View", self)
        toolbar.setMovable(False)
        toolbar.addWidget(QLabel("Angles  "))
        toolbar.addWidget(self.angle_units)
        self.addToolBar(toolbar)

    # -- wiring -----------------------------------------------------------

    def _on_motor_selected(self, motor) -> None:
        self.params_view.set_motor(motor)
        self.scope_view.set_motor(motor)
        self.control_view.set_motor(motor)
        if motor is not None:
            self.statusBar().showMessage(
                f"Selected motor {motor.motor_id} on {motor.link.channel}")

    def _on_inventory_changed(self) -> None:
        self.trace_view.set_links(self.connection.open_links())

    def _update_stats(self) -> None:
        links = self.connection.open_links()
        if not links:
            self.bus_label.setText("no channel open")
            return
        parts = [f"{l.channel}: TX {l.tx_count} / RX {l.rx_count}"
                 + (f" / ERR {l.error_count}" if l.error_count else "")
                 for l in links]
        self.bus_label.setText("   |   ".join(parts))

    def _warn_unverified_models(self) -> None:
        pending = unverified()
        if not pending:
            return
        QMessageBox.warning(
            self, "Check the scaling constants",
            "Position, velocity and torque in the feedback frames are uint16 "
            "values scaled against per-model limits, so a wrong constant "
            "produces plausible but incorrect readings.\n\n"
            "These models are using unverified defaults:\n\n    "
            + ", ".join(pending)
            + "\n\nConfirm each against its own manual and correct them in "
              "robstride/models.json before trusting the readouts.")

    # -- shutdown ---------------------------------------------------------

    def closeEvent(self, event) -> None:
        for view in (self.params_view, self.scope_view, self.trace_view,
                     self.control_view):
            try:
                view.shutdown()
            except Exception:
                log.exception("shutdown failed for %s", type(view).__name__)
        self.connection.shutdown()
        super().closeEvent(event)
