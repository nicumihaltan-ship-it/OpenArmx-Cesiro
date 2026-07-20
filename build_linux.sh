#!/usr/bin/env bash
# Build the Linux executable. Run this ON a Linux machine - PyInstaller does
# not cross-compile, so a Windows host cannot produce this binary.
#
#     chmod +x build_linux.sh && ./build_linux.sh
#
# The result lands in dist/openarmx-robstride as a single self-contained file.
set -euo pipefail

cd "$(dirname "$0")"

# glibc is forward- but not backward-compatible: a binary built on Ubuntu 24.04
# will not start on 22.04. Build on the oldest distro you need to support.
if command -v ldd >/dev/null 2>&1; then
    echo "Building against glibc: $(ldd --version | head -1)"
    echo "The result will run on this glibc version or newer, not older."
    echo
fi

PYTHON="${PYTHON:-python3}"
if ! "$PYTHON" -c 'import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)'; then
    echo "Need Python 3.10 or newer; found $($PYTHON --version)" >&2
    exit 1
fi

if [ ! -d .venv ]; then
    echo "Creating virtual environment..."
    "$PYTHON" -m venv .venv
fi

./.venv/bin/python -m pip install --upgrade pip
./.venv/bin/python -m pip install -r requirements.txt pyinstaller

echo
echo "Running tests..."
./.venv/bin/python -m pytest tests -q

echo
echo "Building..."
./.venv/bin/python -m PyInstaller openarmx.spec --noconfirm --clean

echo
echo "Verifying the build..."
# Qt needs a display; use the offscreen platform so this works over SSH too.
QT_QPA_PLATFORM=offscreen ./dist/openarmx-robstride --selftest

echo
echo "Done: dist/openarmx-robstride"
echo
echo "Runtime notes:"
echo "  * PySide6 needs system Qt libraries. On Debian/Ubuntu:"
echo "      sudo apt install libegl1 libxkbcommon-x11-0 libxcb-cursor0 \\"
echo "                       libxcb-icccm4 libxcb-keysyms1 libxcb-shape0"
echo "  * The PEAK adapter appears as SocketCAN via the peak_usb module."
echo "    Bring each channel up before connecting:"
echo "      sudo modprobe peak_usb"
echo "      sudo ip link set can0 up type can bitrate 1000000"
echo "      sudo ip link set can1 up type can bitrate 1000000"
echo "  * To use CAN without root, grant the binary the capability:"
echo "      sudo setcap cap_net_raw+ep ./dist/openarmx-robstride"
