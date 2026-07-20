"""CAN trace: a decoded, filterable view of every frame on the bus.

This is the CANalyzer-style pane - raw frames with the RobStride 29-bit id
split into communication type / data area 2 / destination, plus a
human-readable interpretation per frame type, and a manual transmit box.
"""

from __future__ import annotations

import csv
import logging
import struct
import time

from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QAbstractItemView, QCheckBox, QComboBox, QFileDialog, QHBoxLayout, QLabel,
    QLineEdit, QMessageBox, QPushButton, QTableWidget, QTableWidgetItem,
    QVBoxLayout, QWidget,
)

from robstride import CanLink, protocol as proto
from robstride import params as P
from robstride.bus import TraceEntry

log = logging.getLogger(__name__)

HEADERS = ["Time", "Dir", "CAN ID", "Type", "Data2", "Dest", "Data", "Decoded"]

COMM_NAMES = {
    0: "get device id", 1: "motion control", 2: "feedback", 3: "enable",
    4: "stop / version", 6: "set zero", 7: "set CAN id",
    17: "param read", 18: "param write", 21: "fault report", 22: "save",
    23: "set baud", 24: "active report", 25: "set protocol",
}


def describe(entry: TraceEntry) -> str:
    """Best-effort human reading of a frame."""
    ct, data, dest = entry.comm_type, entry.data, entry.dest
    try:
        if ct == proto.CommType.FEEDBACK and len(data) >= 8:
            pos, vel, tor, temp = struct.unpack(">HHHH", data[:8])
            mode = proto.MotorMode((entry.data2 >> 14) & 0x03).name
            faults = [n for b, n in proto.FAULT_BITS.items()
                      if (entry.data2 >> 8) & (1 << (b - 16))]
            text = (f"motor {entry.data2 & 0xFF} {mode} "
                    f"pos_raw={pos} vel_raw={vel} torque_raw={tor} "
                    f"temp={temp / 10:.1f}C")
            return text + (f" FAULT: {', '.join(faults)}" if faults else "")

        if ct in (proto.CommType.PARAM_READ, proto.CommType.PARAM_WRITE) \
                and len(data) >= 8:
            index = struct.unpack("<H", data[0:2])[0]
            param = P.get(index)
            name = param.name if param else "unknown"
            verb = "read" if ct == proto.CommType.PARAM_READ else "write"
            if param is not None and any(data[4:8]):
                try:
                    value = param.decode(data[4:8])
                    return f"{verb} 0x{index:04X} {name} = {value}"
                except Exception:
                    pass
            return f"{verb} 0x{index:04X} {name}"

        if ct == proto.CommType.GET_ID:
            return f"device id, uid={data.hex(' ').upper()}"

        if ct == proto.CommType.FAULT_FEEDBACK and len(data) >= 8:
            report = proto.decode_fault_frame(
                proto.pack_id(ct, entry.data2, dest), data)
            parts = report.faults + [f"warn: {w}" for w in report.warnings]
            return "; ".join(parts) if parts else "no faults"

        if ct == proto.CommType.MOTION_CONTROL and len(data) >= 8:
            pos, vel, kp, kd = struct.unpack(">HHHH", data[:8])
            return (f"torque_raw={entry.data2} pos_raw={pos} vel_raw={vel} "
                    f"kp_raw={kp} kd_raw={kd}")

        if ct in (proto.CommType.SET_BAUD, proto.CommType.SET_PROTOCOL,
                  proto.CommType.SET_ACTIVE_REPORT) and len(data) >= 7:
            return f"F_CMD={data[6]}"

        if ct == proto.CommType.SET_CAN_ID:
            return f"new CAN id = {(entry.data2 >> 8) & 0xFF}"
    except Exception:
        log.debug("decode failed for %08X", entry.can_id, exc_info=True)
    return ""


class TraceView(QWidget):
    status = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.links: list[CanLink] = []
        self._paused = False
        self._seen = 0
        self._build_ui()

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(120)

    def _build_ui(self) -> None:
        self.pause_button = QPushButton("Pause")
        self.pause_button.setCheckable(True)
        self.pause_button.toggled.connect(self._toggle_pause)

        self.clear_button = QPushButton("Clear")
        self.clear_button.clicked.connect(self._clear)

        self.export_button = QPushButton("Export CSV")
        self.export_button.clicked.connect(self._export)

        self.autoscroll = QCheckBox("Auto-scroll")
        self.autoscroll.setChecked(True)

        self.type_filter = QComboBox()
        self.type_filter.addItem("All types", None)
        for code, name in sorted(COMM_NAMES.items()):
            self.type_filter.addItem(f"{code} - {name}", code)

        self.id_filter = QLineEdit()
        self.id_filter.setPlaceholderText("Filter: motor id, hex CAN id, or text")

        self.counter = QLabel("0 frames")
        self.counter.setStyleSheet("color: gray;")

        # -- manual transmit
        self.tx_channel = QComboBox()
        self.tx_id = QLineEdit()
        self.tx_id.setPlaceholderText("CAN id hex, e.g. 1200FD01")
        self.tx_data = QLineEdit()
        self.tx_data.setPlaceholderText("data hex, e.g. 05 70 00 00 01 00 00 00")
        self.tx_button = QPushButton("Send")
        self.tx_button.clicked.connect(self._send_manual)

        self.table = QTableWidget(0, len(HEADERS))
        self.table.setHorizontalHeaderLabels(HEADERS)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setColumnWidth(0, 110)
        self.table.setColumnWidth(6, 200)

        top = QHBoxLayout()
        top.addWidget(self.pause_button)
        top.addWidget(self.clear_button)
        top.addWidget(self.export_button)
        top.addWidget(self.autoscroll)
        top.addWidget(self.type_filter)
        top.addWidget(self.id_filter, 1)
        top.addWidget(self.counter)

        tx = QHBoxLayout()
        tx.addWidget(QLabel("TX"))
        tx.addWidget(self.tx_channel)
        tx.addWidget(self.tx_id, 1)
        tx.addWidget(self.tx_data, 2)
        tx.addWidget(self.tx_button)

        layout = QVBoxLayout(self)
        layout.addLayout(top)
        layout.addWidget(self.table, 1)
        layout.addLayout(tx)

    # -- links ------------------------------------------------------------

    def set_links(self, links: list[CanLink]) -> None:
        self.links = links
        current = self.tx_channel.currentText()
        self.tx_channel.clear()
        self.tx_channel.addItems([l.channel for l in links])
        if current:
            self.tx_channel.setCurrentText(current)

    # -- refresh ----------------------------------------------------------

    def _toggle_pause(self, on: bool) -> None:
        self._paused = on
        self.pause_button.setText("Resume" if on else "Pause")

    def _clear(self) -> None:
        for link in self.links:
            link.clear_trace()
        self.table.setRowCount(0)
        self._seen = 0

    def _matches(self, entry: TraceEntry, decoded: str) -> bool:
        wanted_type = self.type_filter.currentData()
        if wanted_type is not None and entry.comm_type != wanted_type:
            return False
        needle = self.id_filter.text().strip().lower()
        if not needle:
            return True
        haystack = (f"{entry.can_id:08x} {entry.dest} {entry.data2 & 0xFF} "
                    f"{entry.data.hex(' ')} {decoded}").lower()
        return needle in haystack

    def _refresh(self) -> None:
        if self._paused or not self.links:
            return

        entries: list[TraceEntry] = []
        for link in self.links:
            entries.extend(link.snapshot_trace())
        entries.sort(key=lambda e: e.timestamp)

        # Only append what is new since the last refresh.
        new = entries[self._seen:]
        if not new:
            self.counter.setText(f"{len(entries)} frames")
            return
        self._seen = len(entries)

        for entry in new:
            decoded = describe(entry)
            if not self._matches(entry, decoded):
                continue
            row = self.table.rowCount()
            self.table.insertRow(row)
            stamp = time.strftime("%H:%M:%S", time.localtime(entry.timestamp))
            stamp += f".{int(entry.timestamp % 1 * 1000):03d}"
            values = [
                stamp,
                entry.direction,
                f"{entry.can_id:08X}",
                f"{entry.comm_type} {COMM_NAMES.get(entry.comm_type, '')}".strip(),
                f"0x{entry.data2:04X}",
                str(entry.dest),
                entry.data.hex(" ").upper(),
                decoded,
            ]
            for col, text in enumerate(values):
                item = QTableWidgetItem(text)
                if entry.direction == "TX":
                    item.setForeground(Qt.darkBlue)
                self.table.setItem(row, col, item)

        # Keep the widget bounded regardless of how long a session runs.
        excess = self.table.rowCount() - 5000
        if excess > 0:
            for _ in range(excess):
                self.table.removeRow(0)

        if self.autoscroll.isChecked():
            self.table.scrollToBottom()
        self.counter.setText(f"{len(entries)} frames")

    # -- manual transmit --------------------------------------------------

    def _send_manual(self) -> None:
        link = next((l for l in self.links
                     if l.channel == self.tx_channel.currentText()), None)
        if link is None:
            QMessageBox.warning(self, "No channel", "Open a CAN channel first.")
            return
        try:
            can_id = int(self.tx_id.text().strip().replace("0x", ""), 16)
            raw = bytes.fromhex(self.tx_data.text().replace(",", " ").strip())
        except ValueError as exc:
            QMessageBox.warning(self, "Bad input", f"Could not parse: {exc}")
            return
        try:
            link.send_raw(can_id, raw)
        except Exception as exc:
            QMessageBox.critical(self, "Send failed", str(exc))
            return
        self.status.emit(f"Sent {can_id:08X} {raw.hex(' ').upper()}")

    # -- export -----------------------------------------------------------

    def _export(self) -> None:
        path, _ = QFileDialog.getSaveFileName(
            self, "Export trace", "robstride_trace.csv", "CSV (*.csv)")
        if not path:
            return
        try:
            with open(path, "w", newline="", encoding="utf-8") as fh:
                writer = csv.writer(fh)
                writer.writerow(HEADERS)
                for row in range(self.table.rowCount()):
                    writer.writerow([
                        self.table.item(row, col).text() if self.table.item(row, col)
                        else "" for col in range(len(HEADERS))])
        except Exception as exc:
            QMessageBox.critical(self, "Export failed", str(exc))
            return
        self.status.emit(f"Trace exported to {path}")

    def shutdown(self) -> None:
        self._timer.stop()
