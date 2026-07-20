"""Post-build self-check.

A frozen build can start its GUI happily and still fail the moment you open a
bus, because python-can resolves backends through entry points that PyInstaller
cannot enumerate. This exercises the parts that only break once packaged.

Run with ``--selftest``; results go to stdout and to ``selftest-report.txt``
next to the executable (Windows GUI builds have no console to print to).
"""

from __future__ import annotations

import platform
import struct
import sys
import traceback
from pathlib import Path


def _check(name: str, fn) -> tuple[str, bool, str]:
    try:
        detail = fn()
        return name, True, detail or "ok"
    except Exception as exc:
        return name, False, f"{type(exc).__name__}: {exc}"


def _backend(interface: str) -> str:
    """Import a python-can backend the way can.Bus() would."""
    import importlib
    module = importlib.import_module(f"can.interfaces.{interface}")
    return f"{interface} -> {module.__name__}"


def _virtual_bus_roundtrip() -> str:
    """Send and receive a real frame over the virtual backend."""
    import can
    from robstride import protocol as proto

    frame = proto.param_read(0x7F, 0x701E, host_id=0xFD)
    with can.Bus(interface="virtual", channel="selftest") as tx, \
            can.Bus(interface="virtual", channel="selftest") as rx:
        tx.send(can.Message(arbitration_id=frame.can_id, data=frame.data,
                            is_extended_id=True))
        msg = rx.recv(timeout=2.0)
    if msg is None:
        raise RuntimeError("no frame received on the virtual bus")
    if msg.arbitration_id != 0x1100FD7F:
        raise RuntimeError(f"unexpected id {msg.arbitration_id:08X}")
    return f"id {msg.arbitration_id:08X} data {bytes(msg.data).hex(' ')}"


def _model_constants() -> str:
    from robstride.models import MODELS
    expected = {"RS00": (33.0, 14.0, 500.0), "RS01": (44.0, 17.0, 500.0),
                "RS02": (44.0, 17.0, 500.0), "RS03": (20.0, 60.0, 5000.0),
                "RS04": (15.0, 120.0, 5000.0)}
    for name, (v, t, kp) in expected.items():
        limits = MODELS[name]
        if (limits.v_max, limits.t_max, limits.kp_max) != (v, t, kp):
            raise RuntimeError(f"{name} constants drifted: {limits}")
    return f"{len(MODELS)} models verified"


def _protocol_roundtrip() -> str:
    from robstride import protocol as proto
    from robstride import params as P
    from robstride.models import MODELS

    limits = MODELS["RS04"]
    payload = struct.pack(
        ">HHHH",
        proto.float_to_uint(1.5, limits.p_min, limits.p_max),
        proto.float_to_uint(-2.0, limits.v_min, limits.v_max),
        proto.float_to_uint(7.5, limits.t_min, limits.t_max), 331)
    fb = proto.decode_feedback(
        proto.pack_id(2, 0x05 | (2 << 14), 0xFD), payload, limits)
    if abs(fb.position - 1.5) > 1e-3 or abs(fb.torque - 7.5) > 1e-2:
        raise RuntimeError(f"feedback decode drifted: {fb}")

    value = P.get(0x701E).decode(
        bytes([0x00, 0x00, 0xF0, 0x41]))
    if abs(value - 30.0) > 1e-6:
        raise RuntimeError(f"IEEE-754 decode drifted: {value}")
    return f"pos {fb.position:.3f} rad, torque {fb.torque:.2f} Nm, loc_kp {value}"


def _param_table() -> str:
    from robstride import params as P
    if len(P.PARAMS) < 140:
        raise RuntimeError(f"parameter table looks truncated: {len(P.PARAMS)}")
    return f"{len(P.PARAMS)} parameters"


def _qt() -> str:
    import PySide6
    import PySide6.QtWidgets  # the module the GUI actually needs at runtime
    return f"PySide6 {PySide6.__version__}"


def _pyqtgraph() -> str:
    import pyqtgraph
    return f"pyqtgraph {pyqtgraph.__version__}"


CHECKS = [
    ("Qt runtime", _qt),
    ("pyqtgraph", _pyqtgraph),
    ("parameter table", _param_table),
    ("model constants", _model_constants),
    ("protocol decode", _protocol_roundtrip),
    ("python-can: pcan backend", lambda: _backend("pcan")),
    ("python-can: socketcan backend", lambda: _backend("socketcan")),
    ("python-can: virtual backend", lambda: _backend("virtual")),
    ("virtual bus round-trip", _virtual_bus_roundtrip),
]


def run() -> int:
    frozen = getattr(sys, "frozen", False)
    lines = [
        "OpenArmX RobStride configurator - self test",
        f"platform : {platform.system()} {platform.release()} "
        f"({platform.machine()})",
        f"python   : {platform.python_version()}",
        f"build    : {'frozen executable' if frozen else 'source checkout'}",
        "",
    ]

    failures = 0
    for name, fn in CHECKS:
        label, ok, detail = _check(name, fn)
        if not ok:
            failures += 1
        lines.append(f"[{'PASS' if ok else 'FAIL'}] {label:32s} {detail}")

    # The socketcan backend imports but cannot open a bus on Windows; that is
    # expected and not a failure of the build.
    lines += ["", f"{len(CHECKS) - failures}/{len(CHECKS)} checks passed."]
    if failures:
        lines.append("Build is NOT usable - see the failures above.")
    else:
        lines.append("Build looks good. Connect the adapter and open a channel.")

    report = "\n".join(lines)

    base = Path(sys.executable).parent if frozen else Path(__file__).parent
    report_path = base / "selftest-report.txt"
    try:
        report_path.write_text(report, encoding="utf-8")
        report += f"\n\nReport written to {report_path}"
    except OSError as exc:
        report += f"\n\nCould not write report: {exc}"

    has_console = _write_stdout(report)

    # Only pop a dialog when there is genuinely nowhere to print AND somebody
    # is there to dismiss it - a windowed build double-clicked from Explorer.
    # A Windows GUI build has no stdout even in CI, so the console check alone
    # is not enough; blocking there would hang the job forever.
    if not has_console and not _headless():
        _show_dialog(report, failures, len(CHECKS))

    return 1 if failures else 0


def _headless() -> bool:
    """True when no human is watching: CI, or an explicitly offscreen Qt."""
    import os
    return bool(os.environ.get("CI")
                or os.environ.get("OPENARMX_NO_DIALOG")
                or os.environ.get("QT_QPA_PLATFORM") == "offscreen")


def _write_stdout(text: str) -> bool:
    """Print the report. Returns False when there is no usable stdout."""
    stream = sys.stdout
    if stream is None:
        return False
    try:
        stream.write(text + "\n")
        stream.flush()
        return True
    except (OSError, ValueError):
        return False


def _show_dialog(report: str, failures: int, total: int) -> None:
    try:
        from PySide6.QtWidgets import QApplication, QMessageBox
        app = QApplication.instance() or QApplication([])
        box = QMessageBox()
        box.setWindowTitle("Self test")
        box.setIcon(QMessageBox.Critical if failures else QMessageBox.Information)
        box.setText(f"{total - failures}/{total} checks passed")
        box.setDetailedText(report)
        box.exec()
        del app
    except Exception:
        traceback.print_exc()


if __name__ == "__main__":
    sys.exit(run())
