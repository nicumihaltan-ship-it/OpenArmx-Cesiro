# RobStride parameter reference

Notes on what the parameters actually mean, beyond the manual's often terse or
repeated descriptions. Everything here is marked as either **confirmed**
(stated in a RobStride manual), **derived** (arithmetic that checks out against
the manual's own sample values), or **inferred** (reasoning from naming and
hardware, not confirmed).

> **You will need the manuals.** They are not included in this repository —
> they are RobStride's copyrighted material and this repo is public. Get them
> from the vendor, and prefer the **Chinese** editions:
> [github.com/RobStride/Product_Information](https://github.com/RobStride/Product_Information)
> (`产品资料/RS0N/`). They are consistently more precise than the English
> translations, which contain outright errors — `0x3007` is labelled "Bus
> voltage" in English when it is actually the second encoder's raw sample, and
> `mechPos` is described as a single-turn value when it is multi-turn. Cross-
> reading models helps too: each manual leaks different fragments of the same
> register map. This matters for the OpenArmX arms, which mix RS00/01/02 with
> RS04.

## How the address space is organised

| Range | Contents | Storage | Access | Same on every model? |
|---|---|---|---|---|
| `0x0000`–`0x0001` | Name, barcode | flash | R/W | yes |
| `0x1000`–`0x1007` | Bootloader and firmware version strings | flash | R | yes |
| `0x2000`–`0x20xx` | Configuration and calibration | flash — needs a **type-22 save** | R/W | **NO** |
| `0x3000`–`0x30xx` | Live observation values | volatile | **R only** | **NO** |
| `0x7005`–`0x702E` | Runtime control | volatile (some mirrored to `0x20xx`) | R/W | **yes** |

Three practical consequences:

- Writing a `0x20xx` parameter changes behaviour immediately but is **lost on
  power-off** unless you follow it with a type-22 save.
- Several names appear twice, once in `0x20xx` and once in `0x70xx`
  (`limit_spd`, `limit_cur`, `cur_kp`, `zero_sta`, `damper`, `add_offset`).
  The `0x70xx` copy is the live one the control loop reads; the `0x20xx` copy
  is what gets restored at boot. **Prefer the `0x70xx` handle** — it is
  model-independent, so using it sidesteps the hazard below entirely.
- The `0x20xx` and `0x30xx` layouts **differ per model**. Read on.

## The register layout is model-specific — and this one bites

This is the single most dangerous thing about the parameter interface, and no
manual states it, because each manual only describes its own model.

Comparing the RS00, RS03 and RS04 manuals directly: RS00 and RS03 disagree on
**21 of 40** config indices and **33** observation indices. The identity
registers are among them:

| Parameter | RS00 | RS03 | RS04 |
|---|---|---|---|
| `CAN_ID` | **`0x200A`** | `0x2009` | `0x2009` |
| `CAN_MASTER` | `0x200B` | `0x200A` | `0x200A` |
| baud rate | **`0x2009`** (`motor_baud`) | `0x2022` | `0x2024` |
| `CAN_TIMEOUT` | `0x200C` | `0x200B` | `0x200B` |
| `protocol_1` | `0x2022` | `0x2025` | `0x2027` |
| `zero_sta` | `0x2021` | `0x2023` | `0x2025` |
| `damper` | `0x2023` | `0x2026` | `0x2028` |
| `MechOffset` | `0x2005` | `0x2005` | `0x2005` |

Note the first two rows together. On an RS03 or RS04, `0x2009` is the CAN id.
On an **RS00 it is the baud rate.** Change a motor's CAN id using the wrong
model's table and you will instead set its bitrate — the motor drops off the
bus at the next power cycle and will not answer a scan until you find it again
at whatever rate you accidentally selected.

Because of this, the tool keys its parameter tables by model
(`robstride/params.py`), never shares them, and **refuses to write** any
`0x20xx`/`0x30xx` index for a model whose table has not been confirmed. Set
each motor's model in the connection panel before touching anything in those
ranges.

By contrast the `0x7005`–`0x702E` runtime range is **verified identical**
across RS00, RS03 and RS04 — all 29 entries agree on index, name, type and
permission. Only the documented value *ranges* differ, tracking each motor's
rating (`iq_ref` ±16 A on RS00 versus ±43 A on RS03), plus one default
(`loc_kp` 40 versus 60). So motion-control code is portable between models;
configuration code is not.

### Provenance

The RS00 and RS03 tables here were extracted from their own manuals twice —
once from flattened text, once from cell-aligned table extraction — with both
passes required to agree. Eleven rows where the secondary fields (min/max or
remark) printed inconsistently are marked `[UNVERIFIED]` in the tool's note
column and carry no range. The names and indices on those rows are still
trustworthy; only the ranges and remarks are doubtful.

One known defect worth naming: in the RS03 manual, the remark column across
`0x2009`–`0x200C` is shifted one row against the names, so `CAN_ID` carries a
remark reading "Motor index" and `CAN_TIMEOUT` prints as `uint8` even though
§3.3.6 says 20000 = 1 s, which cannot fit in a byte. The *names* were
cross-checked and are correct; treat that block's types and ranges with
suspicion and read back before relying on them.

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

**Confirmed** — from RobStride's Chinese-language manuals, which are more
explicit than the English translations
([github.com/RobStride/Product_Information](https://github.com/RobStride/Product_Information)):

- There really are **two magnetic encoders**. The RS02 bill of materials lists
  磁编码器芯片 **AS5047P — 2 PCS**; RS03 the same. Both 14-bit single-turn
  absolute (14bit 单圈绝对值).
- The second one is the **差速磁编码器** ("chasu magnetic encoder").
  `0x3007 encoder2raw` is its raw sample — the English table's "Bus voltage"
  there is simply a mistranslation. RS00 and RS05 name the register
  `chasu_coder_raw` outright.
- **chasu (差速) sits on the low-speed, i.e. output, end.** The RS00 config
  table pairs `position_offset` = 高速段偏置 (high-speed *stage* offset) with
  `chasu_angle_offset` = 低速端偏置 (low-speed *end* offset). That is the
  firmware's own vocabulary.
- The Chinese names for the three registers: `as_angle` = 磁编初始角 (encoder
  initial angle), `cs_angle` = 差速磁编初始角 (**chasu** encoder initial
  angle), `chasu_angle` = 差速角度.
- Power-on absolute range is **one output revolution** — 0–2π by default,
  −π–π with `zero_sta` = 1. That is precisely what a rotor + output encoder
  pair buys you.

So the arrangement is a **coarse/fine pair**: a fine encoder on the fast rotor
and a coarse one on the slow output shaft, turning at a 9:1 speed ratio. The
output-side encoder says roughly where the joint is, the rotor-side one says
precisely where within that, and together they give absolute position across
the full output turn.

An earlier draft of this document called it a *vernier/nonius* scheme. That
was wrong and has been retired: 差速 means the two encoders turn at *different
speeds*, whereas classic nonius uses two *nearly equal* ratios. Coarse/fine is
the accurate description.

**Still not confirmed:** the actual algorithm. No source states how
`as_angle`, `cs_angle` and `chasu_angle` combine to produce `rotation`.
Whether it is a phase-difference computation or a straightforward coarse
quadrant lookup from the output encoder remains inference.

### The registers

| Index | Name | Side | Role |
|---|---|---|---|
| `0x3004` | `encoderRaw` | rotor | Raw 14-bit count from the rotor encoder, 0–16383 |
| `0x3007` | `encoder2raw` | output | Raw count from the chasu (output-side) encoder |
| `0x3028` | `as_angle` | rotor | Rotor encoder angle at initialisation |
| `0x3029` | `cs_angle` | output | Chasu encoder angle at initialisation |
| `0x302A` | `chasu_angle` | output | Chasu angle, raw |
| `0x2006` | `chasu_offset` | output | Calibrated zero for it (**flash, writable**) |
| `0x3033` | `chasu_angle_init` | output | Chasu angle after the offset is removed |
| `0x3034` | `chasu_angle_out` | rotor | That angle scaled by the gear ratio |
| `0x3036` | `mech_angle_init2` | output | Resolved output angle at initialisation |
| `0x3035` | `motormechinit` | rotor | Same, expressed rotor-side |
| `0x3037` | `mech_angle_rotat` | — | Resolved turn number (int16) |
| `0x3015` | `rotation` | — | 圈数, running turn count |
| `0x3016` | `modPos` | rotor | 未计圈 — **non**-turn-counted rotor angle |
| `0x3017` | `mechPos` | output | 计圈 — **turn-counted** output angle, the useful one |

Note on `mechPos`: the English manual renders it "load end loop mechanical
Angle", which reads as a single-turn value. The Chinese is 负载端**计圈**机械角度
— *turn-counted*. `mechPos` is the **multi-turn** output position, which is
what you want for control; `modPos` is the wrapped rotor angle.

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

This arithmetic matters for a second reason. In the RS04 English manual the
description column in this block is misaligned by one row against the Chinese
original, so the name-to-register mapping there cannot be taken on trust. The
relations above were derived from the *values*, and they come out exact using
the names as labelled — which independently corroborates that the mapping is
right for at least these five registers.

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

**Confirm the 9:1 coarse/fine split.** Plot `0x3004 encoderRaw` against
`0x3007 encoder2raw` while slowly back-driving the output shaft through one
full revolution, motor unpowered. The rotor encoder should wrap **nine times**
for one wrap of the chasu encoder. That single plot settles the architecture.

**How does the turn number get resolved?** This is the part no source
documents. Plot `0x302A chasu_angle` and `0x3037 mech_angle_rotat` over the
same slow revolution and watch where the turn number steps. If it steps at
even 1/9th intervals of the chasu angle, it is a straight quadrant lookup from
the output encoder; anything more elaborate would show up as a different
pattern.

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
