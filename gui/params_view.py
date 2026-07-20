"""Parameter table: read every parameter, watch selected ones live, write and save."""

from __future__ import annotations

import csv
import json
import logging
import time
from pathlib import Path

from PySide6.QtCore import Qt, QThread, Signal
from PySide6.QtGui import QBrush, QColor
from PySide6.QtWidgets import (
    QAbstractItemView, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog,
    QHBoxLayout, QHeaderView, QLabel, QLineEdit, QMessageBox, QPushButton,
    QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget,
)

from robstride import Motor
from robstride import params as P
from robstride.poller import ParamPoller, Sample

log = logging.getLogger(__name__)

COL_INDEX, COL_NAME, COL_GROUP, COL_TYPE, COL_ACCESS, COL_VALUE, COL_UNIT, \
    COL_RANGE, COL_WATCH, COL_NOTE = range(10)

HEADERS = ["Index", "Name", "Group", "Type", "Access", "Value", "Unit",
           "Range", "Watch", "Note"]


class ReadAllWorker(QThread):
    """Reads the whole table once, off the UI thread."""

    value_ready = Signal(int, object)
    progress = Signal(int, int)
    done = Signal(int, int)     # ok, failed

    def __init__(self, motor: Motor, indices: list[int], parent=None):
        super().__init__(parent)
        self.motor = motor
        self.indices = indices

    def run(self) -> None:
        ok = failed = 0
        for i, index in enumerate(self.indices):
            if self.isInterruptionRequested():
                break
            try:
                value = self.motor.read(index, timeout=0.15)
            except Exception as exc:
                log.debug("read 0x%04X failed: %s", index, exc)
                value = None
            if value is None:
                failed += 1
            else:
                ok += 1
                self.value_ready.emit(index, value)
            self.progress.emit(i + 1, len(self.indices))
        self.done.emit(ok, failed)


class ParamsView(QWidget):
    """The full 0x0000-0x30xx / 0x70xx parameter table."""

    status = Signal(str)
    #: Emitted from the poller thread; Qt queues it onto the UI thread.
    sample_ready = Signal(int, object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.sample_ready.connect(self._set_value)
        self.motor: Motor | None = None
        self.poller: ParamPoller | None = None
        self._reader: ReadAllWorker | None = None
        self._rows: dict[int, int] = {}      # param index -> table row
        self._suppress_edit = False

        self._build_ui()
        self._populate()

    # -- construction -----------------------------------------------------

    def _build_ui(self) -> None:
        self.group_filter = QComboBox()
        self.group_filter.addItem("All groups", None)
        for group in P.Group:
            self.group_filter.addItem(group.value, group)
        self.group_filter.currentIndexChanged.connect(self._apply_filter)

        self.search = QLineEdit()
        self.search.setPlaceholderText("Filter by name or index...")
        self.search.textChanged.connect(self._apply_filter)

        self.read_all_button = QPushButton("Read all")
        self.read_all_button.clicked.connect(self._read_all)

        self.write_button = QPushButton("Write changed")
        self.write_button.clicked.connect(self._write_changed)

        self.save_button = QPushButton("Save to flash")
        self.save_button.setToolTip(
            "Type 22 - persists 0x20xx parameters so they survive a power cycle")
        self.save_button.clicked.connect(self._save_to_flash)

        self.watch_check = QCheckBox("Poll watched")
        self.watch_check.toggled.connect(self._toggle_watch)

        self.rate = QDoubleSpinBox()
        self.rate.setRange(0.01, 5.0)
        self.rate.setSingleStep(0.01)
        self.rate.setValue(0.05)
        self.rate.setSuffix(" s")
        self.rate.setToolTip("Poll interval per full watch cycle")
        self.rate.valueChanged.connect(
            lambda v: self.poller and self.poller.set_interval(v))

        self.export_button = QPushButton("Export")
        self.export_button.clicked.connect(self._export)
        self.import_button = QPushButton("Import")
        self.import_button.clicked.connect(self._import)

        self.info = QLabel("No motor selected")
        self.info.setStyleSheet("color: gray;")

        self.table = QTableWidget(0, len(HEADERS))
        self.table.setHorizontalHeaderLabels(HEADERS)
        self.table.setEditTriggers(QAbstractItemView.DoubleClicked
                                   | QAbstractItemView.EditKeyPressed)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.verticalHeader().setVisible(False)
        self.table.itemChanged.connect(self._on_item_changed)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(COL_NOTE, QHeaderView.Stretch)
        for col in (COL_INDEX, COL_NAME, COL_GROUP, COL_TYPE, COL_ACCESS,
                    COL_VALUE, COL_UNIT, COL_RANGE, COL_WATCH):
            header.setSectionResizeMode(col, QHeaderView.ResizeToContents)

        top = QHBoxLayout()
        top.addWidget(self.group_filter)
        top.addWidget(self.search, 2)
        top.addWidget(self.read_all_button)
        top.addWidget(self.write_button)
        top.addWidget(self.save_button)

        second = QHBoxLayout()
        second.addWidget(self.watch_check)
        second.addWidget(self.rate)
        second.addStretch(1)
        second.addWidget(self.export_button)
        second.addWidget(self.import_button)

        layout = QVBoxLayout(self)
        layout.addLayout(top)
        layout.addLayout(second)
        layout.addWidget(self.info)
        layout.addWidget(self.table, 1)

    def _populate(self) -> None:
        self._suppress_edit = True
        self.table.setRowCount(len(P.PARAMS))
        for row, param in enumerate(P.PARAMS):
            self._rows[param.index] = row

            def cell(text, editable=False):
                item = QTableWidgetItem(text)
                flags = item.flags() & ~Qt.ItemIsEditable
                if editable:
                    flags |= Qt.ItemIsEditable
                item.setFlags(flags)
                return item

            self.table.setItem(row, COL_INDEX, cell(f"0x{param.index:04X}"))
            self.table.setItem(row, COL_NAME, cell(param.name))
            self.table.setItem(row, COL_GROUP, cell(param.group.value))
            self.table.setItem(row, COL_TYPE, cell(param.dtype))
            self.table.setItem(row, COL_ACCESS, cell(param.access.value))
            self.table.setItem(row, COL_VALUE, cell("", param.writable))
            self.table.setItem(row, COL_UNIT, cell(param.unit))

            if param.minimum is not None or param.maximum is not None:
                rng = f"{param.minimum if param.minimum is not None else ''}" \
                      f" .. {param.maximum if param.maximum is not None else ''}"
            else:
                rng = ""
            self.table.setItem(row, COL_RANGE, cell(rng))

            watch = QTableWidgetItem()
            watch.setFlags(Qt.ItemIsUserCheckable | Qt.ItemIsEnabled)
            watch.setCheckState(Qt.Unchecked)
            self.table.setItem(row, COL_WATCH, watch)

            self.table.setItem(row, COL_NOTE, cell(param.note))

            if param.access is P.Access.RO:
                for col in range(len(HEADERS)):
                    item = self.table.item(row, col)
                    if item is not None:
                        item.setForeground(Qt.darkGray)
        self._suppress_edit = False

    # -- motor binding ----------------------------------------------------

    def set_motor(self, motor: Motor | None) -> None:
        self._stop_poller()
        self.motor = motor
        if motor is None:
            self.info.setText("No motor selected")
            return
        self.info.setText(
            f"Motor id {motor.motor_id} on {motor.link.channel} - model {motor.model}")
        self.poller = ParamPoller(motor, self._on_sample,
                                  interval=self.rate.value())
        self.poller.set_indices(self._watched())
        if self.watch_check.isChecked():
            self.poller.start()

    def _stop_poller(self) -> None:
        if self.poller is not None:
            self.poller.stop()
            self.poller = None

    # -- filtering --------------------------------------------------------

    def _apply_filter(self) -> None:
        group = self.group_filter.currentData()
        needle = self.search.text().strip().lower()
        for param in P.PARAMS:
            row = self._rows[param.index]
            visible = True
            if group is not None and param.group is not group:
                visible = False
            if needle:
                haystack = f"{param.name} 0x{param.index:04x} {param.note}".lower()
                if needle not in haystack:
                    visible = False
            self.table.setRowHidden(row, not visible)

    # -- reading ----------------------------------------------------------

    def _read_all(self) -> None:
        if self.motor is None:
            QMessageBox.warning(self, "No motor", "Select a motor first.")
            return
        if self._reader is not None and self._reader.isRunning():
            return
        indices = [p.index for p in P.PARAMS]
        self.read_all_button.setEnabled(False)
        self._reader = ReadAllWorker(self.motor, indices, self)
        self._reader.value_ready.connect(self._set_value)
        self._reader.progress.connect(
            lambda i, n: self.status.emit(f"Reading parameters {i}/{n}"))
        self._reader.done.connect(self._read_all_done)
        self._reader.start()

    def _read_all_done(self, ok: int, failed: int) -> None:
        self.read_all_button.setEnabled(True)
        self.status.emit(f"Read {ok} parameters, {failed} did not answer")

    def _set_value(self, index: int, value) -> None:
        row = self._rows.get(index)
        if row is None:
            return
        text = f"{value:.6g}" if isinstance(value, float) else str(value)
        self._suppress_edit = True
        item = self.table.item(row, COL_VALUE)
        item.setText(text)
        item.setData(Qt.UserRole, value)
        self._suppress_edit = False

    def _on_sample(self, sample: Sample) -> None:
        # Runs on the poller thread - hand off via a queued signal.
        self.sample_ready.emit(sample.index, sample.value)

    # -- watching ---------------------------------------------------------

    def _watched(self) -> list[int]:
        out = []
        for param in P.PARAMS:
            item = self.table.item(self._rows[param.index], COL_WATCH)
            if item is not None and item.checkState() == Qt.Checked:
                out.append(param.index)
        return out

    def _toggle_watch(self, on: bool) -> None:
        if self.poller is None:
            if on:
                QMessageBox.warning(self, "No motor", "Select a motor first.")
                self.watch_check.setChecked(False)
            return
        self.poller.set_indices(self._watched())
        if on:
            self.poller.start()
        else:
            self.poller.stop()

    # -- editing / writing ------------------------------------------------

    DIRTY_ROLE = Qt.UserRole + 1

    def _mark_dirty(self, item: QTableWidgetItem, dirty: bool) -> None:
        item.setData(self.DIRTY_ROLE, dirty)
        item.setBackground(QBrush(QColor("#fff3a3")) if dirty else QBrush())

    def _is_dirty(self, item: QTableWidgetItem) -> bool:
        return bool(item.data(self.DIRTY_ROLE))

    def _on_item_changed(self, item: QTableWidgetItem) -> None:
        if self._suppress_edit:
            return
        if item.column() == COL_WATCH and self.poller is not None:
            self.poller.set_indices(self._watched())
            return
        if item.column() == COL_VALUE:
            self._mark_dirty(item, True)

    def _write_changed(self) -> None:
        if self.motor is None:
            QMessageBox.warning(self, "No motor", "Select a motor first.")
            return
        pending = []
        for param in P.PARAMS:
            if not param.writable:
                continue
            item = self.table.item(self._rows[param.index], COL_VALUE)
            if self._is_dirty(item) and item.text().strip():
                pending.append((param, item))
        if not pending:
            self.status.emit("Nothing changed")
            return

        names = ", ".join(p.name for p, _ in pending[:8])
        if len(pending) > 8:
            names += f" and {len(pending) - 8} more"
        if QMessageBox.question(
                self, "Write parameters",
                f"Write {len(pending)} parameter(s) to motor "
                f"{self.motor.motor_id}?\n\n{names}") != QMessageBox.Yes:
            return

        written = 0
        for param, item in pending:
            try:
                value = item.text().strip()
                self.motor.write(param.index,
                                 value if param.is_string else float(value))
                self._mark_dirty(item, False)
                written += 1
                time.sleep(0.003)
            except Exception as exc:
                QMessageBox.critical(self, "Write failed",
                                     f"{param.name}: {exc}")
                break
        self.status.emit(f"Wrote {written} parameter(s). "
                         f"Use 'Save to flash' to make 0x20xx values persistent.")

    def _save_to_flash(self) -> None:
        if self.motor is None:
            return
        if QMessageBox.question(
                self, "Save to flash",
                "Persist the current 0x20xx parameters on motor "
                f"{self.motor.motor_id}?\n\nThis writes the motor's non-volatile "
                "memory.") != QMessageBox.Yes:
            return
        self.motor.save()
        self.status.emit("Save frame (type 22) sent")

    # -- import / export --------------------------------------------------

    def _export(self) -> None:
        path, selected = QFileDialog.getSaveFileName(
            self, "Export parameters", "robstride_params.json",
            "JSON (*.json);;CSV (*.csv)")
        if not path:
            return
        rows = []
        for param in P.PARAMS:
            item = self.table.item(self._rows[param.index], COL_VALUE)
            text = item.text().strip()
            if not text:
                continue
            rows.append({
                "index": f"0x{param.index:04X}", "name": param.name,
                "type": param.dtype, "access": param.access.value,
                "unit": param.unit, "value": text,
            })
        try:
            if path.lower().endswith(".csv") or "CSV" in selected:
                with open(path, "w", newline="", encoding="utf-8") as fh:
                    writer = csv.DictWriter(fh, fieldnames=list(rows[0]) if rows
                                            else ["index", "name", "value"])
                    writer.writeheader()
                    writer.writerows(rows)
            else:
                meta = {
                    "motor_id": self.motor.motor_id if self.motor else None,
                    "model": self.motor.model if self.motor else None,
                    "exported": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "params": rows,
                }
                Path(path).write_text(json.dumps(meta, indent=2), encoding="utf-8")
        except Exception as exc:
            QMessageBox.critical(self, "Export failed", str(exc))
            return
        self.status.emit(f"Exported {len(rows)} parameters to {path}")

    def _import(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Import parameters", "", "JSON (*.json);;CSV (*.csv)")
        if not path:
            return
        try:
            if path.lower().endswith(".csv"):
                with open(path, newline="", encoding="utf-8") as fh:
                    rows = list(csv.DictReader(fh))
            else:
                rows = json.loads(Path(path).read_text(encoding="utf-8"))["params"]
        except Exception as exc:
            QMessageBox.critical(self, "Import failed", str(exc))
            return

        loaded = 0
        for row in rows:
            try:
                index = int(str(row["index"]), 16) if str(row["index"]).lower() \
                    .startswith("0x") else int(row["index"])
            except (KeyError, ValueError):
                continue
            table_row = self._rows.get(index)
            param = P.get(index)
            if table_row is None or param is None or not param.writable:
                continue
            self._suppress_edit = True
            item = self.table.item(table_row, COL_VALUE)
            item.setText(str(row.get("value", "")))
            self._mark_dirty(item, True)
            self._suppress_edit = False
            loaded += 1
        self.status.emit(
            f"Loaded {loaded} values into the table - review them, then 'Write changed'")

    def shutdown(self) -> None:
        self._stop_poller()
        if self._reader is not None and self._reader.isRunning():
            self._reader.requestInterruption()
            self._reader.wait(1500)
