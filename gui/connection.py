"""Connection dock: channels, bus scan and the discovered-motor list."""

from __future__ import annotations

import logging

from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtWidgets import (
    QComboBox, QGroupBox, QHBoxLayout, QHeaderView, QLabel, QMessageBox,
    QProgressBar, QPushButton, QSpinBox, QTreeWidget, QTreeWidgetItem,
    QVBoxLayout, QWidget,
)

from robstride import BITRATES, CanError, CanLink, Motor, model_names, scan
from robstride.bus import (
    INTERFACES, available_channels, channel_candidates, default_channels,
    default_interface,
)
from robstride.models import DEFAULT_MODEL

log = logging.getLogger(__name__)


class ScanWorker(QThread):
    """Sweeps CAN ids on one link without blocking the UI."""

    progress = Signal(int)
    finished_scan = Signal(str, list)   # channel, [(motor_id, uid), ...]
    failed = Signal(str, str)

    def __init__(self, link: CanLink, id_from: int, id_to: int, parent=None):
        super().__init__(parent)
        self.link = link
        self.id_from = id_from
        self.id_to = id_to

    def run(self) -> None:
        ids = range(self.id_from, self.id_to + 1)
        total = max(1, len(ids))
        try:
            found = scan(
                self.link, ids,
                progress=lambda i, _mid: self.progress.emit(int(100 * (i + 1) / total)),
            )
        except CanError as exc:
            self.failed.emit(self.link.channel, str(exc))
            return
        self.finished_scan.emit(self.link.channel, found)


class ChannelWidget(QGroupBox):
    """Open/close controls for a single CAN channel."""

    state_changed = Signal()

    def __init__(self, title: str, channel_slot: int, parent=None):
        super().__init__(title, parent)
        self.link: CanLink | None = None
        self._slot = channel_slot

        self.interface_box = QComboBox()
        self.interface_box.addItems(INTERFACES)
        self.interface_box.setCurrentText(default_interface())
        self.interface_box.currentTextChanged.connect(self._on_interface_changed)

        self.channel_box = QComboBox()
        self.channel_box.setEditable(True)
        self._reload_channels(default_interface())

        self.bitrate_box = QComboBox()
        for code, rate in BITRATES.items():
            self.bitrate_box.addItem(f"{rate // 1000} kbit/s", rate)
        self.bitrate_box.setCurrentIndex(0)   # 1 Mbit/s, the motor default

        self.host_id = QSpinBox()
        self.host_id.setRange(0, 255)
        self.host_id.setValue(0xFD)
        self.host_id.setPrefix("host id ")

        self.open_button = QPushButton("Open")
        self.open_button.setCheckable(True)
        self.open_button.toggled.connect(self._toggle)

        self.status = QLabel("closed")
        self.status.setStyleSheet("color: gray;")

        row1 = QHBoxLayout()
        row1.addWidget(self.interface_box, 1)
        row1.addWidget(self.channel_box, 2)
        row1.addWidget(self.bitrate_box, 1)

        row2 = QHBoxLayout()
        row2.addWidget(self.host_id)
        row2.addWidget(self.open_button)

        layout = QVBoxLayout(self)
        layout.addLayout(row1)
        layout.addLayout(row2)
        layout.addWidget(self.status)

    def _reload_channels(self, interface: str) -> None:
        candidates = channel_candidates(interface)[:8] or default_channels()[:8]
        self.channel_box.clear()
        self.channel_box.addItems(candidates)
        if self._slot < len(candidates):
            self.channel_box.setCurrentText(candidates[self._slot])

    def _on_interface_changed(self, interface: str) -> None:
        self._reload_channels(interface)
        # SocketCAN takes its bitrate from `ip link`, so the selector is moot.
        self.bitrate_box.setEnabled(interface != "socketcan")
        self.bitrate_box.setToolTip(
            "Set with: sudo ip link set <iface> up type can bitrate 1000000"
            if interface == "socketcan" else "")

    def _toggle(self, checked: bool) -> None:
        if checked:
            self._open()
        else:
            self._close()
        self.state_changed.emit()

    def _open(self) -> None:
        channel = self.channel_box.currentText().strip()
        bitrate = self.bitrate_box.currentData()
        interface = self.interface_box.currentText()
        link = CanLink(channel, bitrate=bitrate, interface=interface,
                       host_id=self.host_id.value())
        try:
            link.open()
        except CanError as exc:
            self.open_button.setChecked(False)
            self.status.setText("failed")
            self.status.setStyleSheet("color: #c0392b;")
            # CanError already carries the platform-specific hint.
            QMessageBox.critical(self, "Cannot open channel", str(exc))
            return
        self.link = link
        self.open_button.setText("Close")
        rate = "set via ip link" if interface == "socketcan" \
            else f"{bitrate // 1000} kbit/s"
        self.status.setText(f"open - {interface}:{channel} @ {rate}")
        self.status.setStyleSheet("color: #27ae60;")

    def _close(self) -> None:
        if self.link is not None:
            self.link.close()
            self.link = None
        self.open_button.setText("Open")
        self.status.setText("closed")
        self.status.setStyleSheet("color: gray;")

    def shutdown(self) -> None:
        self._close()


class ConnectionPanel(QWidget):
    """Both channels plus the shared motor inventory."""

    motor_selected = Signal(object)     # Motor or None
    inventory_changed = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.motors: dict[tuple[str, int], Motor] = {}
        self._scan_workers: list[ScanWorker] = []

        self.channel_a = ChannelWidget("Arm A", 0)
        self.channel_b = ChannelWidget("Arm B", 1)
        for ch in (self.channel_a, self.channel_b):
            ch.state_changed.connect(self.inventory_changed)

        self.detect_button = QPushButton("Detect adapters")
        self.detect_button.clicked.connect(self._detect)

        self.scan_from = QSpinBox()
        self.scan_from.setRange(0, 127)
        self.scan_from.setValue(1)
        self.scan_to = QSpinBox()
        self.scan_to.setRange(0, 127)
        self.scan_to.setValue(20)

        self.scan_button = QPushButton("Scan bus")
        self.scan_button.clicked.connect(self._scan)

        self.progress = QProgressBar()
        self.progress.setVisible(False)

        self.tree = QTreeWidget()
        self.tree.setColumnCount(4)
        self.tree.setHeaderLabels(["Motor", "Channel", "Model", "MCU UID"])
        self.tree.header().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.tree.itemSelectionChanged.connect(self._on_selection)
        self.tree.setRootIsDecorated(False)

        self.add_manual = QPushButton("Add motor by id")
        self.add_manual.clicked.connect(self._add_manual)
        self.manual_id = QSpinBox()
        self.manual_id.setRange(0, 127)
        self.manual_id.setValue(1)

        scan_row = QHBoxLayout()
        scan_row.addWidget(QLabel("ids"))
        scan_row.addWidget(self.scan_from)
        scan_row.addWidget(QLabel("to"))
        scan_row.addWidget(self.scan_to)
        scan_row.addWidget(self.scan_button, 1)

        manual_row = QHBoxLayout()
        manual_row.addWidget(self.manual_id)
        manual_row.addWidget(self.add_manual, 1)

        layout = QVBoxLayout(self)
        layout.addWidget(self.detect_button)
        layout.addWidget(self.channel_a)
        layout.addWidget(self.channel_b)
        layout.addLayout(scan_row)
        layout.addWidget(self.progress)
        layout.addWidget(QLabel("Discovered motors"))
        layout.addWidget(self.tree, 1)
        layout.addLayout(manual_row)

    # -- links ------------------------------------------------------------

    def open_links(self) -> list[CanLink]:
        return [ch.link for ch in (self.channel_a, self.channel_b)
                if ch.link is not None]

    def _detect(self) -> None:
        interface = self.channel_a.interface_box.currentText()
        found = available_channels(interface)
        if found:
            QMessageBox.information(
                self, "Adapters detected",
                f"Channels available on the {interface} backend:\n  "
                + "\n  ".join(found)
                + ("\n\nSocketCAN interfaces that are still down are listed "
                   "too - bring them up with 'ip link' before opening."
                   if interface == "socketcan" else ""))
            self.channel_a.channel_box.setCurrentText(found[0])
            if len(found) > 1:
                self.channel_b.channel_box.setCurrentText(found[1])
            return

        if interface == "socketcan":
            detail = ("No SocketCAN interfaces found.\n\n"
                      "Load the PEAK driver and bring the interface up:\n"
                      "    sudo modprobe peak_usb\n"
                      "    sudo ip link set can0 up type can bitrate 1000000")
        else:
            detail = ("python-can reported no PCAN channels.\n\n"
                      "Install the PEAK-System driver package (which provides "
                      "PCANBasic.dll) and make sure the PCAN-USB Pro FD is "
                      "connected.")
        QMessageBox.warning(self, "No adapter found", detail)

    # -- scanning ---------------------------------------------------------

    def _scan(self) -> None:
        links = self.open_links()
        if not links:
            QMessageBox.warning(self, "No open channel",
                                "Open at least one CAN channel first.")
            return
        self.scan_button.setEnabled(False)
        self.progress.setVisible(True)
        self.progress.setValue(0)

        self._scan_workers = []
        for link in links:
            worker = ScanWorker(link, self.scan_from.value(), self.scan_to.value(), self)
            worker.progress.connect(self.progress.setValue)
            worker.finished_scan.connect(self._on_scan_done)
            worker.failed.connect(self._on_scan_failed)
            worker.finished.connect(self._maybe_finish_scan)
            self._scan_workers.append(worker)
            worker.start()

    def _maybe_finish_scan(self) -> None:
        if all(w.isFinished() for w in self._scan_workers):
            self.scan_button.setEnabled(True)
            self.progress.setVisible(False)

    def _on_scan_failed(self, channel: str, message: str) -> None:
        QMessageBox.critical(self, f"Scan failed on {channel}", message)

    def _on_scan_done(self, channel: str, found: list) -> None:
        link = next((l for l in self.open_links() if l.channel == channel), None)
        if link is None:
            return
        for motor_id, uid in found:
            self._register(link, motor_id, uid)
        self._rebuild_tree()
        self.inventory_changed.emit()

    def _add_manual(self) -> None:
        links = self.open_links()
        if not links:
            QMessageBox.warning(self, "No open channel",
                                "Open a CAN channel first.")
            return
        self._register(links[0], self.manual_id.value(), None)
        self._rebuild_tree()
        self.inventory_changed.emit()

    def _register(self, link: CanLink, motor_id: int, uid) -> None:
        key = (link.channel, motor_id)
        if key not in self.motors:
            self.motors[key] = Motor(link, motor_id, DEFAULT_MODEL)
        if uid is not None:
            self.motors[key].uid = uid

    # -- tree -------------------------------------------------------------

    def _rebuild_tree(self) -> None:
        self.tree.clear()
        for (channel, motor_id), motor in sorted(self.motors.items()):
            uid = motor.uid.hex(" ").upper() if motor.uid else "-"
            item = QTreeWidgetItem([f"id {motor_id}", channel, "", uid])
            item.setData(0, Qt.UserRole, (channel, motor_id))
            self.tree.addTopLevelItem(item)

            combo = QComboBox()
            combo.addItems(model_names())
            combo.setCurrentText(motor.model)
            combo.currentTextChanged.connect(
                lambda name, m=motor: self._set_model(m, name))
            self.tree.setItemWidget(item, 2, combo)

    def _set_model(self, motor: Motor, name: str) -> None:
        motor.set_model(name)
        self.inventory_changed.emit()

    def _on_selection(self) -> None:
        items = self.tree.selectedItems()
        if not items:
            self.motor_selected.emit(None)
            return
        key = items[0].data(0, Qt.UserRole)
        self.motor_selected.emit(self.motors.get(key))

    def current_motor(self) -> Motor | None:
        items = self.tree.selectedItems()
        if not items:
            return None
        return self.motors.get(items[0].data(0, Qt.UserRole))

    def shutdown(self) -> None:
        for worker in self._scan_workers:
            worker.requestInterruption()
            worker.wait(1000)
        self.channel_a.shutdown()
        self.channel_b.shutdown()
