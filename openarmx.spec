# PyInstaller spec, shared by the Windows and Linux builds.
#
# PyInstaller does not cross-compile: run it on Windows for the .exe and on
# Linux for the Linux binary. Both produce a single self-contained file.
#
#     pyinstaller openarmx.spec --noconfirm

import sys

block_cipher = None

# python-can discovers backends through entry points, which a frozen build
# cannot enumerate - the backend modules have to be named explicitly or
# opening a bus fails at runtime with "unknown interface".
hidden = [
    "can.interfaces.pcan",
    "can.interfaces.pcan.pcan",
    "can.interfaces.socketcan",
    "can.interfaces.socketcan.socketcan",
    "can.interfaces.virtual",
]

# The 3D kinematics view needs OpenGL, and both halves of that stack resolve
# their real modules at runtime rather than by a visible import:
#
#  * pyqtgraph reaches QtOpenGL through importlib.import_module(f"{QT_LIB}.
#    QtOpenGL"), so static analysis never sees PySide6.QtOpenGL at all.
#  * PyOpenGL picks its platform backend and its array handlers by name.
#
# Without these the build succeeds and then dies when the Kinematics tab is
# opened. The 'OpenGL stack' self-test check exists to catch exactly that.
hidden += [
    "PySide6.QtOpenGL",
    "PySide6.QtOpenGLWidgets",
    "OpenGL.arrays.ctypesarrays",
    "OpenGL.arrays.ctypesparameters",
    "OpenGL.arrays.ctypespointers",
    "OpenGL.arrays.lists",
    "OpenGL.arrays.nones",
    "OpenGL.arrays.numbers",
    "OpenGL.arrays.numpymodule",
    "OpenGL.arrays.strings",
    "OpenGL.arrays.vbo",
    "OpenGL.platform.win32" if sys.platform == "win32"
    else "OpenGL.platform.glx",
]

a = Analysis(
    ["app.py"],
    pathex=[],
    binaries=[],
    datas=[],
    # selftest is imported lazily behind --selftest, so it needs naming here.
    hiddenimports=hidden + ["selftest"],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    # Qt ships several large modules this app never touches; dropping them
    # roughly halves the binary.
    excludes=[
        "PySide6.QtWebEngineCore", "PySide6.QtWebEngineWidgets",
        "PySide6.QtQuick", "PySide6.QtQml", "PySide6.Qt3DCore",
        "PySide6.QtMultimedia", "PySide6.QtCharts", "PySide6.QtDataVisualization",
        "PySide6.QtPdf", "PySide6.QtDesigner", "PySide6.QtBluetooth",
        "tkinter", "matplotlib", "PIL", "IPython", "pytest",
    ],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

exe = EXE(
    pyz,
    a.scripts,
    a.binaries,
    a.zipfiles,
    a.datas,
    [],
    name="OpenArmX-RobStride" if sys.platform == "win32" else "openarmx-robstride",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    runtime_tmpdir=None,
    console=False,          # GUI app: no console window on Windows
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
