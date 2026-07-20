# RobStride parameter reference

Notes on what the parameters actually mean, beyond the manual's often terse or
repeated descriptions. Everything here is marked as either **documented**
(stated in a RobStride manual), **derived** (arithmetic that checks out against
the manual's own sample values), or **inferred** (reasoning from naming and
hardware, not confirmed).

## How the address space is organised

| Range | Contents | Storage | Access |
|---|---|---|---|
| `0x0000`–`0x0001` | Name, barcode | flash | R/W |
| `0x1000`–`0x1007` | Bootloader and firmware version strings | flash | R |
| `0x2000`–`0x2029` | Configuration and calibration | flash — needs a **type-22 save** | R/W |
| `0x3000`–`0x3048` | Live observation values | volatile | **R only** |
| `0x7005`–`0x702E` | Runtime control | volatile (some mirrored to `0x20xx`) | R/W |

Two practical consequences:

- Writing a `0x20xx` parameter changes behaviour immediately but is **lost on
  power-off** unless you follow it with a type-22 save.
- Several names appear twice, once in `0x20xx` and once in `0x70xx`
  (`limit_spd`, `limit_cur`, `cur_kp`, `zero_sta`, `damper`, `add_offset`).
  The `0x70xx` copy is the live one the control loop reads; the `0x20xx` copy
  is what gets restored at boot.

## What you actually use for control

Most of the table is diagnostic. For closed-loop work you need only these:

**Read** — or better, get them free in the type-2 feedback frame:

| Index | Name | Meaning |
|---|---|---|
| `0x3017` | `mechPos` | Load-side position, rad — **the one that matters** |
| `0x3018` | `mechVel` | Load-side speed, rad/s |
| `0x302C` | `torque_fdb` | Torque feedback, Nm |
| `0x3015` | `rotation` | Turn count |
| `0x3006` | `motorTemp` | Motor NTC, ×10 |
| `0x300C` | `VBUS` | Bus voltage, V |
| `0x3023` | `faultSta` | Fault word |

**Write:**

| Index | Name | Used in |
|---|---|---|
| `0x7005` | `run_mode` | always — 0 operation, 1 PP, 2 velocity, 3 current, 5 CSP |
| `0x7016` | `loc_ref` | position modes |
| `0x700A` | `spd_ref` | velocity mode |
| `0x7006` | `iq_ref` | current mode |
| `0x7017` | `limit_spd` | CSP speed limit |
| `0x7024` / `0x7025` | `vel_max` / `acc_set` | PP speed and acceleration |
| `0x7018` | `limit_cur` | current ceiling in velocity/position modes |

The feedback frame already carries position, velocity, torque and temperature
at up to 100 Hz, so polling `0x3017`/`0x3018` is usually wasted bus bandwidth —
enable active reporting (type 24) instead.

---

## The position chain: `0x3028`–`0x302A` and `0x3033`–`0x3037`

This is the group the manual labels, unhelpfully, *"Motor position
determination parameters"* on five consecutive registers. Here is what they
appear to be.

### The hardware problem being solved

**Documented:** the RS04 has a **14-bit absolute** magnetic encoder and a
**9:1** gear reduction.

"Absolute" there means absolute *within one turn* of the encoder — 16384 counts
per revolution. But the encoder sits on the motor shaft, which spins 9 times per
output revolution. So at power-on the firmware knows the motor angle precisely
and the output angle only modulo 1/9th of a turn. It cannot tell which of the
nine motor turns the output shaft is currently in.

**Documented:** the manual refers to a second, *"differential magnetic
encoder"* (`cs_angle`, `chasu_angle`).

**Inferred:** this is a vernier (nonius) arrangement — a second encoder geared
differently from the first, so the *difference* between the two readings is
unique across the whole output revolution. That difference resolves the turn
number, turning a single-turn absolute encoder into a multi-turn absolute one.
"chasu" is pinyin (差数 / 差速, "difference"). This explains the hardware but is
not stated anywhere in the manual.

### The registers

| Index | Name | Side | Role |
|---|---|---|---|
| `0x3004` | `encoderRaw` | motor | Raw 14-bit encoder count, 0–16383 |
| `0x3028` | `as_angle` | motor | Main encoder angle at initialisation |
| `0x3029` | `cs_angle` | — | Differential encoder angle at initialisation |
| `0x302A` | `chasu_angle` | — | Difference between the two, raw |
| `0x2006` | `chasu_offset` | — | Calibrated zero for that difference (**flash**) |
| `0x3033` | `chasu_angle_init` | — | Difference after the offset is removed |
| `0x3034` | `chasu_angle_out` | motor | That difference scaled by the gear ratio |
| `0x3036` | `mech_angle_init2` | load | Resolved output angle at initialisation |
| `0x3035` | `motormechinit` | motor | Same, expressed motor-side |
| `0x3037` | `mech_angle_rotat` | — | Resolved turn number (int16) |
| `0x3015` | `rotation` | — | Running turn count during operation |
| `0x3016` | `modPos` | motor | Live single-turn motor angle |
| `0x3017` | `mechPos` | load | Live output angle — the useful one |

### The arithmetic — derived, and it checks out

The manual's own "Current value (for reference)" column is a consistent
snapshot. Three relations hold **exactly** on those numbers:

```
chasu_angle_init = chasu_angle - chasu_offset
    0.075549     =  4.822069   -   4.74652        exact to floating point

chasu_angle_out  = chasu_angle_init * 9           (9 = the RS04 gear ratio)
    0.679938     =     0.075549     * 9           exact to display rounding

motormechinit    = mech_angle_init2 * 9
    0.528840     =      0.05876     * 9           exact
```

Also `encoderRaw / 16384 × 2π ≈ modPos` (4.3703 vs 4.3634), confirming
`modPos` is the raw single-turn motor angle, and `mech_angle_init2 == loc_reff`
(`0x3030`) exactly — the initial position becomes the first position-loop
setpoint, which is why a correctly initialised motor does not lurch at enable.

The gear ratio appearing in two independent exact relations is the strongest
evidence for the reading above. Note the `_init` family is internally
consistent because it is a single power-on snapshot; the *live* values
(`modPos`, `mechPos`, `position`) were sampled at different instants in the
manual's table and do not cross-check.

### How this is used in control — the short answer: it isn't

These registers are **power-on bootstrap state, not control inputs.** The
sequence at boot is roughly:

1. Read both encoders → `as_angle`, `cs_angle`
2. Take the difference → `chasu_angle`, remove the calibrated offset →
   `chasu_angle_init`
3. Scale by the gear ratio → `chasu_angle_out`, resolve the turn number →
   `mech_angle_rotat`
4. Combine with the fine encoder reading → `mech_angle_init2`, the absolute
   output angle
5. Seed the position loop with it (`loc_reff`) so the motor holds still at
   enable

After that the control loop runs on `mechPos` / `mechVel`, and this group just
sits there recording what happened at boot. **You never write them** — they are
read-only, and the thing that *is* writable (`chasu_offset`, `0x2006`) is set by
the magnetic encoder calibration procedure, not by hand.

### When you would actually look at them

They are a diagnostic for exactly one class of failure: **the motor comes up
believing it is somewhere it is not.** Symptoms are a joint that jumps on
enable, reports a position offset by a fraction of a turn, or reads differently
after each power cycle.

Check, in order:

- `faultSta` (`0x3023`) bit 7 — encoder uncalibrated — and bit 9, position
  initialisation fault.
- `chasu_offset` (`0x2006`) — if this is zero or obviously wrong, the encoder
  calibration never completed or was not saved to flash.
- `mech_angle_rotat` (`0x3037`) across several power cycles at the *same*
  physical position. It should land on the same turn number every time. If it
  wanders, the vernier resolution is marginal — usually mechanical (backlash,
  a loose encoder magnet) or a botched calibration.
- `mech_angle_init2` (`0x3036`) against where the joint physically is. A
  mismatch of roughly 2π/9 ≈ 0.7 rad on the output is the signature of an
  off-by-one turn resolution.

The manual's own remedy for a bad state here: recalibrate the magnetic encoder
(required after reopening the motor or changing the three-phase wiring order),
and consider setting `iq_test` (`0x702D`) to 1, which lengthens initialisation
for a more accurate reference.

---

## Verifying any of this on your own hardware

The claims above marked *inferred* can be settled empirically with the
oscilloscope tab, and you have the motors:

**Is it really a vernier pair?** Plot `0x3028 as_angle` and `0x3029 cs_angle`
while slowly back-driving the output shaft through a full revolution with the
motor unpowered. If the hypothesis holds, both ramp and wrap, but at different
rates, and their difference (`0x302A`) advances monotonically across the whole
output turn without repeating. If the two track each other identically, the
hypothesis is wrong.

**Does the turn resolution repeat?** Park the joint, note `0x3037` and `0x3036`,
power-cycle, read again. Repeat five times. Same values each time means the
absolute bootstrap is healthy.

**What is `position` (`0x3032`)?** Undocumented beyond the same generic label.
Plot it against `mechPos` and `modPos` while jogging slowly and the
relationship should become obvious — it is likely the unwrapped multi-turn
output position, but that is a guess.

If you run any of these, the results are worth recording here — that turns
inference into documentation.

## Parameters not to touch

The manual is explicit, and RobStride disclaims liability for damage from
changing them:

- `0x2007` `Status1` — torque limitation
- Protection temperature and over-temperature time
- `0x2005` `MechOffset`, `0x2006` `chasu_offset` — encoder calibration outputs;
  writing these by hand desynchronises the position bootstrap described above
- `0x702A` / `0x2028` `damper` — setting 1 disables post-power-off
  anti-backdrive protection, which exists to stop voltage surges when the joint
  is spun fast while unpowered
