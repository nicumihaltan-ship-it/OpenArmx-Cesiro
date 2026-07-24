"""Post-build self-check.

A frozen build can start its GUI happily and still fail the moment you open a
bus, because python-can resolves backends through entry points that PyInstaller
cannot enumerate. This exercises the parts that only break once packaged.

Run with ``--selftest``; results go to stdout and to ``selftest-report.txt``
next to the executable (Windows GUI builds have no console to print to).
"""

from __future__ import annotations

import math
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

    counts = {}
    for model in ("RS00", "RS03", "RS04"):
        table = P.params_for(model)
        if len(table) < 140:
            raise RuntimeError(f"{model} table looks truncated: {len(table)}")
        counts[model] = len(table)

    # The layouts must stay distinct - see PARAMETERS.md. Flattening them back
    # into one shared table is the failure this guards against.
    if P.get(0x2009, "RS00").name != "motor_baud":
        raise RuntimeError("RS00 0x2009 should be motor_baud, not CAN_ID")
    if P.get(0x2009, "RS03").name != "CAN_ID":
        raise RuntimeError("RS03 0x2009 should be CAN_ID")

    # Name-based lookup is how the calibration tab reaches per-model registers
    # without hard-coding one model's map onto another's. mechPos really does
    # sit at a different index on each, so the by-name handle has to track it.
    if P.index_of("mechPos", "RS04") != 0x3017:
        raise RuntimeError("RS04 mechPos should resolve to 0x3017")
    if P.index_of("mechPos", "RS00") != 0x3016:
        raise RuntimeError("RS00 mechPos should resolve to 0x3016")
    if P.index_of("chasu_offset", "RS02") is not None:
        raise RuntimeError("RS02 has no confirmed table; by-name must be None")

    return ", ".join(f"{m} {n}" for m, n in counts.items())


def _opengl() -> str:
    """The 3D view's stack, which is all dynamic imports underneath.

    pyqtgraph reaches QtOpenGL through importlib, and PyOpenGL resolves its
    platform backend by name, so a frozen build can lose either one without
    any build-time complaint. Importing is enough to catch that - creating a
    real GL context needs a display this check cannot assume.
    """
    import OpenGL  # noqa: F401
    import OpenGL.arrays.numpymodule  # noqa: F401
    import PySide6.QtOpenGL  # noqa: F401
    import PySide6.QtOpenGLWidgets  # noqa: F401
    import pyqtgraph.opengl as gl

    for name in ("GLViewWidget", "GLMeshItem", "GLLinePlotItem",
                 "GLScatterPlotItem", "GLGridItem", "MeshData"):
        if not hasattr(gl, name):
            raise RuntimeError(f"pyqtgraph.opengl is missing {name}")
    return f"PyOpenGL {OpenGL.__version__}, QtOpenGL present"


def _kinematics() -> str:
    """The FK module, exercised on a chain whose answer is known by hand.

    A frozen build that cannot import numpy through this path, or that ships
    a broken copy, fails here rather than in front of an arm.
    """
    import tempfile
    from pathlib import Path

    import kinematics as kin

    urdf = """<?xml version="1.0"?>
    <robot name="selftest">
      <link name="base"/><link name="link1"/><link name="tip"/>
      <joint name="j1" type="revolute">
        <parent link="base"/><child link="link1"/>
        <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
        <limit lower="-3.14" upper="3.14"/>
      </joint>
      <joint name="j2" type="fixed">
        <parent link="link1"/><child link="tip"/>
        <origin xyz="1 0 0" rpy="0 0 0"/>
      </joint>
    </robot>"""
    with tempfile.TemporaryDirectory() as folder:
        path = Path(folder) / "selftest.urdf"
        path.write_text(urdf, encoding="utf-8")
        chain = kin.Robot.from_urdf(path).chain("tip")

    if len(chain) != 1:
        raise RuntimeError(f"expected 1 actuated joint, got {len(chain)}")
    # A quarter turn about +Z takes the 1 m tip from +X to +Y.
    tip = chain.position([math.pi / 2])
    if abs(tip[0]) > 1e-9 or abs(tip[1] - 1.0) > 1e-9:
        raise RuntimeError(f"tip landed at {tip.round(6)}, expected [0, 1, 0]")
    return f"{len(chain)} DOF chain, tip at {tip.round(3)}"


def _calibration() -> str:
    """The fixed-point offset fit, on a chain whose answer is known.

    Plant an offset on one joint of a two-link arm, capture a few poses that
    all reach the same tip point, and confirm the fit recovers it. Catches a
    frozen build that ships a broken numpy linear-algebra path, which the
    forward-kinematics check alone does not exercise.
    """
    import tempfile
    from pathlib import Path

    import numpy as np

    import calibration as cal
    import kinematics as kin

    # Four joints, so that holding the tip at a point leaves a null space to
    # walk - a shorter chain has only discrete elbow-up/down solutions and
    # the walk finds nothing between them.
    urdf = """<?xml version="1.0"?>
    <robot name="cal">
      <link name="base"/><link name="l1"/><link name="l2"/><link name="l3"/>
      <link name="l4"/><link name="tip"/>
      <joint name="j1" type="revolute">
        <parent link="base"/><child link="l1"/>
        <origin xyz="0 0 0.1"/><axis xyz="0 0 1"/>
        <limit lower="-2.9" upper="2.9"/>
      </joint>
      <joint name="j2" type="revolute">
        <parent link="l1"/><child link="l2"/>
        <origin xyz="0 0 0.08"/><axis xyz="0 1 0"/>
        <limit lower="-2.5" upper="2.5"/>
      </joint>
      <joint name="j3" type="revolute">
        <parent link="l2"/><child link="l3"/>
        <origin xyz="0 0 0.3"/><axis xyz="0 0 1"/>
        <limit lower="-2.9" upper="2.9"/>
      </joint>
      <joint name="j4" type="revolute">
        <parent link="l3"/><child link="l4"/>
        <origin xyz="0 0 0.06"/><axis xyz="0 1 0"/>
        <limit lower="-2.5" upper="2.5"/>
      </joint>
      <joint name="tip_joint" type="fixed">
        <parent link="l4"/><child link="tip"/>
        <origin xyz="0.04 0 0.2"/>
      </joint>
    </robot>"""
    with tempfile.TemporaryDirectory() as folder:
        path = Path(folder) / "cal.urdf"
        path.write_text(urdf, encoding="utf-8")
        chain = kin.Robot.from_urdf(path).chain("tip")

    seed = np.array([0.25, -0.55, 0.4, 0.85])
    poses = cal.candidates(chain, seed, count=6, attempts=300)
    if len(poses) < 4:
        raise RuntimeError(f"only generated {len(poses)} poses")
    truth = np.array([0.0, 0.05, -0.03, 0.02])
    fit = cal.solve_fixed_point(chain, [cal.Pose(q - truth) for q in poses],
                                locked=["j1"])
    if np.max(np.abs(fit.offsets - truth)) > 5e-3:
        raise RuntimeError(f"offset fit drifted: {fit.offsets.round(4)}")
    return (f"recovered {len(poses)} poses, "
            f"j2 {math.degrees(fit.offsets[1]):.2f} deg")


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
    ("forward kinematics", _kinematics),
    ("offset calibration", _calibration),
    ("OpenGL stack", _opengl),
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
