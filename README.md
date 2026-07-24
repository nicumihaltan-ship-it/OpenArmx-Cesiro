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
decoding, forward kinematics, the offset-calibration fit, the OpenGL stack,
and that all three python-can backends load and can pass a real frame. Results
go to `selftest-report.txt` next to the binary.

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
ordinary SocketCAN interfaces, so `PCANBasic` is not involved at all. No PEAK
driver download is needed.

Work through these in order. Each step is checkable, so a failure tells you
where the problem is instead of leaving you guessing at the GUI.

**1. Get the binary**

```bash
# from the Releases page, or:
gh release download v1.0.1 --pattern openarmx-robstride
chmod +x openarmx-robstride
```

**2. Confirm the kernel sees the adapter**

```bash
lsusb | grep -i peak                  # expect 0c72:0012
dmesg | grep -i peak_usb              # expect the driver binding, 2 channels
```

If `lsusb` shows nothing, it is a cable or power problem, not software.

**3. Bring the interfaces up**

```bash
sudo modprobe peak_usb
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
ip -details link show can0            # expect "state UP" and bitrate 1000000
```

The bitrate comes from `ip link`, not from the app — the selector greys out
when SocketCAN is chosen. `Detect adapters` also lists interfaces that exist
but are still down, since those are the ones you need to bring up.

**4. Sanity-check the bus before involving this tool**

Worth doing once. It separates "the bus is wrong" from "the app is wrong".

```bash
sudo apt install can-utils
candump can0                          # in one terminal
cansend can0 '00000000#'              # in another - candump should show it
```

With motors powered and the bus terminated, `candump can0` stays quiet until
something is asked to talk — the motors do not chatter unprompted unless
active reporting was enabled. If you see errors scrolling, suspect
termination or bitrate before anything else.

**5. Install the Qt runtime libraries**

```bash
sudo apt install libegl1 libgl1 libxkbcommon-x11-0 libxcb-cursor0 \
                 libxcb-icccm4 libxcb-keysyms1 libxcb-shape0
```

`libgl1` is for the Kinematics tab's 3D view. Without it the rest of the app
still runs, but opening that tab fails to create a GL context.

**6. Verify the build itself**

```bash
./openarmx-robstride --selftest       # expect 11/11
```

**7. Run it without root**

```bash
sudo setcap cap_net_raw+ep ./openarmx-robstride
./openarmx-robstride
```

In the app: set the backend to `socketcan`, channel `can0` / `can1`, open, then
**Scan bus**. Set each motor's model before touching anything — see the model
warnings above; on this rig that means RS00, RS03 and RS04.

**First contact with a motor, in order:** scan → select → set the model →
`Read all` in the Parameters tab → watch the CAN trace to confirm frames look
sane. Only then go near the Control tab, and expect the joint to move the
moment you enable.

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

## The six tabs

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

**Kinematics** — where the tool tip is, relative to the base, given what the
motors are reporting. Map each URDF joint to a motor and the tip pose updates
live alongside a rotatable 3D view of the arm. From there,
[drive the tip to a point](#driving-the-tip-to-a-point) with inverse
kinematics, or do a quick [tape-measure offset
fit](#a-quick-offset-fit-from-measured-tip-positions).

**Calibration** — a step-by-step procedure for the whole calibration, laid
out as a wizard. Two halves: four steps on the motors alone (encoder health,
zero and angle range, the power-on position bootstrap, and the joint's
backlash), then four on the arm as a whole (the joint-to-motor map, the tool
and fixture, capturing poses, and the fit). See
[calibrating the arm](#calibrating-the-arm).

## The gripper

The fingers are not part of the arm's kinematics: they hang off a link on
the way to the tool frame rather than affecting where that frame is, so the
joint table and the IK solver both ignore them. They still get their own row.

Map the motor that drives them, and the panel reports the stroke and how far
apart the fingers are, animating them in the 3D view. `Go to` drives them.

Two things about this are worth knowing:

- **Both fingers run off one motor.** URDF has no way to say "one actuator,
  two slides" without a `mimic` tag, and OpenArmX's description does not use
  one, so it declares them as independent prismatic joints. The hardware
  couples them, and one stroke value drives both — their opposing axes make
  that symmetric for free.
- **The transmission is not in the URDF.** The description says how far the
  fingers travel but nothing about the screw or linkage that moves them, so
  the motor-radians-to-millimetres ratio cannot be derived — measure it and
  type it into `mm/rad`. The default assumes full stroke over one motor
  revolution, which is a guess, and the confirmation dialog says so: get it
  wrong and the fingers travel the wrong distance into their own stop.

Stroke on these arms runs 0–44 mm per finger, which is 12–100 mm between the
finger frames.

## Driving the tip to a point

Type a target pose — XYZ in millimetres, roll/pitch/yaw in degrees, in the
same base frame as the tip readout — and `Solve IK` finds the joint values
that put the tool there. `Copy current` fills the fields from wherever the
tip is now, which is the easiest way to get a nearby target to start from.

The solution is shown as a **teal skeleton** next to the live pose, with the
target point in purple, and nothing is commanded until you press
`MOVE TO TARGET` and confirm.

**Reachability, and why it usually fails.** The arm has seven joints and a
full pose is six constraints, so a target that misses is far more often an
impossible *orientation* than an out-of-range *point*. The panel separates
the two:

- `POINT IS REACHABLE, that orientation there is not` — it also prints an
  orientation the arm can hold at that exact point, and `Relax orientation`
  adopts it and re-solves.
- `POINT IS OUT OF RANGE` — the point itself is beyond the arm.

Either way the closest achievable pose is shown and can still be moved to;
the confirmation says how far short it lands.

**What the solver does.** Damped least squares seeded from the current pose.
The damping is what makes an unreachable target safe to ask for: an undamped
pseudo-inverse blows up near a singularity and throws the arm somewhere
unrelated, while this one stops short and reports the shortfall. On seven
joints there is still a null space, and the minimum-norm step resolves it by
moving as little as possible from where the arm already is. Restarts use a
seeded generator, so the same target always yields the same pose — a solver
that quietly picked a different branch each run has no business driving
hardware.

**Before it moves**, every joint must be mapped and reporting fresh feedback.
A stale joint aborts the move: if where the arm *is* is unknown, it must not
be told where to go. The confirmation lists the per-joint deltas and the
largest single move, and the speed limit (default 0.2 rad/s) is written to
every joint before the setpoints.

> **There is no collision checking of any kind.** Not against the body, the
> other arm, the table, or you. The arm sweeps whatever lies between its
> current pose and the solved one, and the preview shows only the endpoint,
> not the path. Check it, stand clear, and keep `STOP` on the Control tab
> within reach.

## Setting up the arm model

Both the Kinematics tab and the Calibration tab work against a URDF and a
joint-to-motor map. That map lives in one place — the Kinematics tab — and
the Calibration tab reads it from there, so there is only ever one map to
keep correct.

1. **Point it at a URDF.** `Browse` to a description of the arms. Nothing is
   bundled — OpenArmX's own description is CC BY-NC-SA, so the tool reads
   whatever local copy you have rather than shipping one. The path and the
   whole joint map are remembered in `openarmx_kinematics.json` under your
   user config directory, not in this repository.
2. **Pick the tip frame.** Tool-looking leaf links (`..._hand_tcp`) are
   offered first; the rest of the leaves are still in the list.
3. **Point it at the meshes**, if they are not already beside the URDF.
   Mesh references in a URDF are `package://` URIs, which only resolve
   inside a ROS workspace — there isn't one here, so the folder is searched
   instead. The URDF's own parent folders are always searched first, so a
   description laid out the usual way (`<pkg>/urdf/robot/x.urdf` next to
   `<pkg>/meshes/`) needs nothing configured. A flat folder of loose STLs
   works too: the last resort is a match on the bare filename.
   **Only STL is read.** Descriptions commonly ship DAE visual meshes and
   STL collision meshes, so collision geometry is preferred and anything
   unresolved is counted next to the path.
4. **Map the joints.** One row per actuated joint: choose the motor, and set
   `Sign` to `-` where the motor turns opposite to the URDF axis. A joint
   value outside its URDF limit turns red, which is usually a wrong sign.
5. **Wake the feedback up.** `Enable active reporting on mapped motors`.
   Without it the motors only speak when polled and the readings go stale —
   stale joints are greyed out and excluded from a capture.

### A quick offset fit from measured tip positions

The Kinematics tab turns a tape measure into a calibration. The idea is that
the tip position the URDF predicts and the tip position you can measure
disagree by exactly the joint zero errors, and enough poses make those errors
separable.

**Capture poses.** Move the arm somewhere, measure where the tip really is in
the base frame, type the three numbers in millimetres, and press `Capture
sample`. Repeat. Seven offsets need at least three well-spread poses; the
panel warns while you are under that. Poses that are nearly identical will
report a flattering residual and offsets that mean nothing.

**Solve.** `Solve offsets` runs a damped Gauss-Newton fit and shows the
tip-error RMS before and after before you accept it. Accepting adds the
result to the offsets already in the table.

This is the fast path when you have a way to measure the tip into the base
frame. When you do not — the usual case on a bench — use the Calibration tab
below, which needs no external metrology at all.

## Calibrating the arm

The Calibration tab runs the whole procedure as a wizard, and it is built
around one idea that removes the tape measure: **a tool held in a fixture
that does not move.** If the model were perfect, every arm configuration that
still seats the tool in the fixture would compute the same tip position. It
does not, and that scatter is the joint error. Nothing about *where* the
fixture is has to be known — a rigid tool and a machined seat are the whole
apparatus.

The steps run in order, and the order matters: the arm-level fit assumes each
motor reports its own angle correctly, so the motor-level checks come first.
The list on the left carries a live state glyph per step (`○` to do, `✓` good,
`!` worth a look, `✗` blocking).

**The motors, without a model:**

1. **Motors** — every motor that will take part, and whether it is fit to be
   calibrated at all. An uncalibrated encoder (fault bit 7) or an unconfirmed
   feedback scaling invalidates the whole calibration before it starts: a
   scale error is not an offset error, and the fit would absorb it into seven
   meaningless numbers.
2. **Zero and range** — where each motor thinks zero is (`zero_sta`,
   `add_offset`, `mechPos`), with buttons to set the `-π..π` range, clear
   stray software offsets, set a mechanical zero, and save to flash. The three
   different "zeros" are spelled out here so they do not get confused.
3. **Power-on bootstrap** — these motors combine two encoders at power-on to
   resolve which of nine rotor turns the output is in, and when that goes
   wrong the motor comes up believing it is a fraction of a turn from where it
   is. Snapshot, power-cycle without moving the arm, and read back: the turn
   number must repeat and `mech_angle_init2` must agree to well under
   2π/9 ≈ 0.70 rad. (See [PARAMETERS.md](PARAMETERS.md) for the register
   chain.)
4. **Backlash and repeatability** — drives one joint a few degrees either
   side of where it is, current-limited, and measures the hysteresis by
   arriving at the same command from above and from below. Whatever that comes
   to is the floor on what the arm-level fit can achieve; a residual below the
   backlash is fitting noise. Optional, but worth knowing before you chase a
   number you cannot reach.

**The arm, against the URDF:**

5. **Arm model** — the chain and the joint map, read from the Kinematics tab.
   Its one original readout is the last column: where the tip moves for one
   degree of each motor. If it moves the wrong way, that joint's sign is
   wrong, and no offset fixes a wrong sign.
6. **Tool and fixture** — the tool tip in the tip frame (fit it as well and
   you never have to measure it), and whether the fixture position is known.
   An unmeasured fixture is fitted as three more unknowns, which is what frees
   you from measuring it — at the cost of the first joint's offset, which a
   fixed point structurally cannot see. `Lock what cannot be identified`
   parks those so the report says "not identified" rather than handing you a
   number the data never determined.
7. **Poses** — put the tool in the fixture and work the arm into as many
   different configurations as it reaches without the tip leaving. A
   seven-joint arm holding its tip still has four degrees of freedom left, and
   those are the poses worth capturing. **Suggest poses** generates them by a
   null-space walk and keeps the subset that pins the offsets down best; a
   live readout says how much the pose you are in would add. Hand-guiding is
   the safe way to reach them — driving there only makes sense if the fixture
   lets the tool pivot.
8. **Solve and apply** — the fit, and next to each offset an *identifiability*
   figure in degrees per millimetre of measurement noise. Read that column
   first: an offset the poses did not constrain is not a small number, it is
   no number, and the honest thing to do is capture more poses or lock it.
   `Apply` folds the identified offsets into the Kinematics tab's table and
   throws the poses away so the same correction cannot be applied twice.
   `Bake into the motors` optionally writes `add_offset` instead — it measures
   the readback to check the register's undocumented sign convention, and lets
   you undo it.

As with the quick fit, the offsets are a correction this tool holds. To bake
them into the motors instead, use `Bake into the motors` on the last step, or
drive each joint to its true zero and use `Set zero here` on the Control tab.

The 3D view renders the real mesh geometry through OpenGL, on top of the
skeleton, the joint axes and the tool frame. Drag to orbit, wheel to zoom,
right-drag to pan, `Fit view` to re-frame. Captured samples appear as orange
markers, so a bad measurement is obvious next to the predicted tip.

Two spheres carry the frame you are working in: a **yellow one at the base
origin**, and a **red one at the tool tip**. Between them, `XYZ legs` draws
the tip's position as three orthogonal segments — red along X, then green
along Y, then blue along Z — so the tip's coordinates can be read off the
scene instead of only off the numbers below it.

All three are drawn without depth testing. The origin sits inside the base
plate of a floor-standing robot and the tip disappears behind the arm's own
geometry from half the viewing angles; a landmark you cannot see is not a
landmark. Back-face culling stands in for the depth test on the spheres,
which are convex, so they still shade as spheres rather than flat discs.

### When a mesh is in the wrong units

Descriptions mix millimetres and metres more often than they admit, and
OpenArmX's own URDF does it: the body mesh is authored in millimetres but
declares `scale="1.0 1.0 1.0"`, which draws a 773-metre column that swallows
the entire scene. The arm meshes beside it really are metres, and the hand
meshes correctly declare `0.001`.

A mesh whose extent exceeds twenty times the robot's own reach is therefore
treated as millimetres, converted, and **named in the mesh label and the
log** — a silent geometry fix is exactly the sort of thing that bites later.
Anything that is still absurd after the conversion is left alone, because
millimetres did not explain it and guessing further would be worse than
showing you the problem.

One arm is about 43,500 triangles. That is nothing for a GPU — it measures
around 13 ms a frame — but it is far too much to project in software, which
came out at roughly 660 ms a frame on the same geometry. Hence the PyOpenGL
dependency. Both halves of that stack import dynamically (pyqtgraph reaches
`QtOpenGL` through `importlib`, PyOpenGL picks its platform backend by
name), so `openarmx.spec` names them explicitly and the `OpenGL stack`
self-test check exists to catch a frozen build that lost them.

## Degrees or radians

The `Angles` selector in the toolbar switches every angular quantity in the
GUI between degrees and radians. It defaults to **degrees**, which is easier
to reason about when you are eyeballing a joint against its mechanical stop.

It is a display layer and nothing more. Positions, speeds and accelerations
are converted on their way into a widget and converted straight back before
anything is written, so the protocol, the parameter tables and every
`0x70xx` register stay in radians. Non-angular units — current, torque,
voltage, temperature — are never touched.

Two things deliberately stay canonical:

- **Exported JSON/CSV is always radians**, whichever unit is on screen. A
  capture taken in degrees imports cleanly into a session in radians.
- **The `Range` column and the manuals agree.** Switch the selector to
  radians when cross-checking a value against RobStride's documentation,
  rather than converting in your head.

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

Both executables additionally pass every `--selftest` check as frozen
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
kinematics.py       URDF parsing, forward and inverse kinematics, offset fitting
gui/                Qt front-end, one module per tab
  units.py          the degree/radian display layer
  scene_gl.py       OpenGL 3D view: meshes, skeleton, frames
tests/              protocol, kinematics, units and panel-behaviour tests
app.py              entry point
```
