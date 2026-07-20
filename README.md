# OpenArmX — RobStride CAN configurator

A CANalyzer-style tool for reading, configuring and driving RobStride actuators
over a PEAK PCAN-USB Pro FD, built for the two OpenArmX arms.

## Executables

A prebuilt Windows binary is in `dist/OpenArmX-RobStride.exe` (64 MB,
self-contained — no Python needed). Verify any build with:

```
OpenArmX-RobStride.exe --selftest
```

It checks the Qt runtime, the parameter table, the model constants, protocol
decoding, and that all three python-can backends load and can pass a real
frame. Results go to `selftest-report.txt` next to the binary.

### Building

PyInstaller does not cross-compile, so each binary must be built on its own
platform:

```
# Windows
.venv\Scripts\python.exe -m PyInstaller openarmx.spec --noconfirm --clean

# Linux
./build_linux.sh
```

**No Linux machine?** Run the `Build executables` workflow
(`.github/workflows/build.yml`) from the Actions tab:

```
gh workflow run "Build executables" --ref main
gh run download <run-id> --dir ci-artifacts
```

It builds both binaries and attaches them to the run; pushing a `v*` tag also
creates a release. The Linux job builds on Ubuntu 22.04 deliberately — glibc is
not backward-compatible, so a binary built on a newer release will refuse to
start on an older one. Building there means the binary runs on glibc 2.35 and
newer (Ubuntu 22.04+, Debian 12+).

## Running from source

```
git clone https://github.com/nicumihaltan-ship-it/OpenArmx-Cesiro.git
cd OpenArmx-Cesiro
py -3.11 -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements.txt
.venv\Scripts\python.exe app.py          # add --debug for verbose logging
```

On Windows, **clone into a short path** such as `C:\OpenArmx`. PySide6 has
deeply nested internal files, and installing it under an already-deep directory
fails with `WinError 206: The filename or extension is too long` — the classic
260-character `MAX_PATH` limit. Enabling long paths
(`LongPathsEnabled` in the registry) also works.

Afterwards:

```
run.bat                                  # Windows
./.venv/bin/python app.py                # Linux
```

## Before it will talk to hardware

The same PCAN-USB Pro FD is reached through **different backends per
platform**, and the connection panel lets you pick which:

### Windows — the `pcan` backend

1. **Install the PEAK-System driver package.** It provides `PCANBasic.dll`,
   which `python-can` loads. It was *not* present on this machine when the tool
   was built — `Detect adapters` will report nothing until you install it.
   Download: <https://www.peak-system.com/Drivers.523.0.html>
2. The adapter's two channels appear as `PCAN_USBBUS1` and `PCAN_USBBUS2` —
   one arm per channel. Bitrate is set in the app.

### Linux — the `socketcan` backend

The `peak_usb` kernel module (in mainline Linux) presents the adapter as
ordinary SocketCAN interfaces, so `PCANBasic` is not involved at all.

```bash
sudo modprobe peak_usb
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
ip -details link show can0            # confirm it is up
```

The bitrate comes from `ip link`, not from the app — the selector greys out
when SocketCAN is chosen. `Detect adapters` also lists interfaces that exist
but are still down, since those are the ones you need to bring up.

To use CAN without running as root:

```bash
sudo setcap cap_net_raw+ep ./openarmx-robstride
```

PySide6 also needs system Qt libraries:

```bash
sudo apt install libegl1 libxkbcommon-x11-0 libxcb-cursor0 \
                 libxcb-icccm4 libxcb-keysyms1 libxcb-shape0
```

### Both platforms

- Terminate the bus with 120 Ω at both ends. The Pro FD can supply internal
  termination; otherwise fit resistors.
- Motors ship at **1 Mbit/s**.

## Wiring

| Wire | Signal |
|---|---|
| blue | CAN_H |
| brown | CAN_L |
| black | GND |
| red | VBAT+ (24–60 VDC, 48 V nominal) |

## The four tabs

**Parameters** — the full table: identity, firmware version, the flash-stored
`0x20xx` block, the read-only `0x30xx` observation values, and the runtime
`0x70xx` controls. `Read all` sweeps everything; tick `Watch` on any row and
enable `Poll watched` to see it update live. Edited cells highlight; `Write
changed` sends them, and `Save to flash` (type 22) makes `0x20xx` values
survive a power cycle. Export/import as JSON or CSV.

**Oscilloscope** — multi-channel real-time plot. Two data sources feed it:

- *Feedback frames* (type 2) give position, velocity, torque and temperature.
  This is the fast path — tick `Active reporting` and the motor pushes a frame
  every 10 ms with no request traffic.
- *Polled parameters* (type 17) cover everything else, round-robined at the
  configured interval. Polling many channels at once slows each one down.

**CAN trace** — every frame, decoded. The 29-bit id is split into
communication type / data area 2 / destination, with a plain-language reading
per frame. Filter by type or text, and transmit raw frames by hand.

**Control** — enable/stop, mode switching, per-mode setpoints, and jog.
The motor stays disabled until you explicitly enable it.

## Safety

These are 120 Nm actuators. The panel confirms before enabling, and `STOP`
stays reachable at all times, but:

- **Switch modes while stopped.** The manual is explicit that changing control
  mode mid-run causes undefined behaviour.
- **Don't touch torque limit, protection temperature or over-temperature time**
  (`0x2007`, and the equivalents). RobStride disclaims liability for damage
  caused by changing them.
- Zero calibration works in CSP and operation-control modes; the firmware
  blocks it in PP mode.
- Setting `damper` (`0x702A`/`0x2028`) to 1 disables post-power-off
  anti-backdrive protection. Leave it alone unless you know why you want that.

## Per-model scaling — the thing most likely to bite

Position, velocity, torque, Kp and Kd all travel as uint16 values scaled
against **per-model** limits. A wrong constant produces readings that look
entirely plausible and are wrong.

| | RS00 | RS01 | RS02 | RS03 | RS04 |
|---|---|---|---|---|---|
| P_MAX (rad) | 12.57 | 12.57 | 12.57 | 12.57 | 12.57 |
| V_MAX (rad/s) | 33 | 44 | 44 | 20 | 15 |
| T_MAX (Nm) | 14 | 17 | 17 | 60 | 120 |
| KP_MAX | 500 | 500 | 500 | 5000 | 5000 |
| KD_MAX | 5 | 5 | 5 | 100 | 100 |
| Current (A) | 16 | 23 | 23 | 43 | 90 |
| Gear ratio | 10:1 | 7.75:1 | 7.75:1 | 9:1 | 9:1 |

Confirmed against the official RobStride manuals for each model and
cross-checked against the `kscalelabs/actuator` driver. Two traps worth
knowing:

- **Kp/Kd differ by 10x** between the small motors and RS03/04. Reusing RS04's
  values on an RS02 misscales every gain you send.
- **RS01 and RS02 share protocol constants** despite being physically
  different motors. RS01's protocol V_MAX of 44 rad/s over-provisions its real
  capability (~33 rad/s). Do not "correct" it — that breaks the encoding.
- **Firmware older than 0.0.2.6 used P_MAX 12.5, not 12.57.** Check
  `AppCodeVersion` (`0x1003`) on old units.

**The parameter tables differ per model too**, which is more dangerous than
the scaling constants. `CAN_ID` is at `0x200A` on RS00 but `0x2009` on
RS03/RS04 — and on RS00, `0x2009` is the baud rate. The tool keys its tables
by model and refuses `0x20xx`/`0x30xx` writes for any model whose table is
unconfirmed. Confirmed tables: **RS00, RS03, RS04**. Only the `0x7005`–`0x702E`
runtime range is identical across models. See [PARAMETERS.md](PARAMETERS.md).

Set each motor's model in the connection panel's `Model` column. To override
any constant, drop a `robstride/models.json`:

```json
{"RS02": {"v_max": 44.0, "t_max": 17.0, "verified": true}}
```

## Protocol notes

- CAN 2.0B extended frames, 8-byte payloads, 1 Mbit/s.
- 29-bit id: `bits 28..24` communication type, `bits 23..8` data area 2,
  `bits 7..0` destination address.
- Motion-control payloads (type 1) are **big-endian**; parameter payloads
  (types 17/18) are **little-endian**. This asymmetry is in the firmware, not
  a bug here.
- Default host id is `0xFD` (253), configurable per channel.
- The motors also speak CANopen (CiA 402) and MIT protocol — switchable via
  type 25, effective after a power cycle. **This tool implements the private
  protocol only.** If a motor has been switched away from private, it will not
  answer a bus scan; switch it back before use.

## What is verified and what is not

Verified by automated test against the manual's own worked examples:
id packing, the type-17 read frame, IEEE-754 parameter decoding, the type-18
write frame, feedback decode and fault-bit extraction, and every per-model
scaling constant. Run them with:

```
.venv\Scripts\python.exe -m pytest tests -q
```

Both executables additionally pass all nine `--selftest` checks as frozen
builds, verified in CI — that is what catches PyInstaller's usual failure mode
(backends that resolve from source but not once packaged). The Linux binary is
built and checked on Ubuntu 22.04 / glibc 2.35.

**Not verified: anything involving real hardware.** No PCAN driver or adapter
was present on the machine this was written on, so the transport layer, bus
scanning, live polling and all motor commands have never been exercised
against an actual motor. In particular the SocketCAN path — the one the Linux
build depends on — has never seen a real adapter. Bring up one motor on the
bench, watch the CAN trace tab to confirm frames look right, and only then
connect a full arm.

**Not verified: `build_linux.sh`.** CI builds Linux with the same spec file, so
the packaging is proven, but that script itself has never been run.

One other gap: the manual documents `0x7005`–`0x702E` for RS04, and the same
table is *very likely* identical on the other models — but that table is a
raster image in the other manuals and could not be confirmed. Spot-check a
couple of indices per model.

## Layout

```
robstride/          protocol, parameter table, model constants, transport
  protocol.py       frame construction and decoding
  params.py         the ~150-entry parameter table
  models.py         per-model scaling constants (+ models.json override)
  bus.py            python-can/PCAN transport, RX thread, trace buffer
  motor.py          per-motor operations and bus scanning
  poller.py         background parameter polling
gui/                Qt front-end, one module per tab
tests/              protocol tests against the manual's examples
app.py              entry point
```
