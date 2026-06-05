# Investigation: `LabwareHeightError` with the wireless color‑sensor custom labware

**Status:** Investigation / findings report
**Context issues:**
- [Opentrons/ot2#10](https://github.com/Opentrons/ot2/issues/10) – the original error report ("custom labware … too tall")
- [vertical-cloud-lab/byu-vcl#33](https://github.com/vertical-cloud-lab/byu-vcl/issues/33) – "Recreate wireless color sensor tool"
  - [comment with reference links](https://github.com/vertical-cloud-lab/byu-vcl/issues/33#issuecomment-4511778595)
  - [second report of the same error after downgrading to v8.8.1](https://github.com/vertical-cloud-lab/byu-vcl/issues/33#issuecomment-4626373005)

---

## TL;DR

The error is **not** caused by a recent Opentrons OT‑2 / Opentrons App update. It is a
**fixed geometric limit of the OT‑2 hardware** that has been enforced by the same piece of
software since 2020, long before any v6/v7/v8/v9 release.

The custom labware `ac_color_sensor_charging_port.json` declares a total height of
**100 mm**. The tallest point an OT‑2 pipette can be raised to is **≈96.3 mm above the deck**
for the pipette currently installed. Because `100 mm > 96.3 mm`, the motion planner refuses to
move and raises `LabwareHeightError`.

This is exactly why **downgrading the robot and app to v8.8.1 did not change anything**
([byu-vcl#33 comment](https://github.com/vertical-cloud-lab/byu-vcl/issues/33#issuecomment-4626373005)):
there is **no version of the OT‑2 software in which a 100 mm labware will clear the gantry.**
The fix has to be a **hardware / labware‑definition change**, not a software downgrade.

> **"Highest version that still works":** None — and equally, *every* version works *once the
> loaded labware is ≤ ~96 mm tall*. The constraint is hardware, so the right lever is the
> labware height (and/or the pipette), not the software release.

---

## 1. The exact error

From [Opentrons/ot2#10](https://github.com/Opentrons/ot2/issues/10):

```
File "/usr/lib/python3.12/site-packages/opentrons/protocols/geometry/planning.py", line 182,
  in _build_safe_height
raise LabwareHeightError(
opentrons.protocols.geometry.planning.LabwareHeightError: The BYU wireless color sensor
charging port on 8 has a total height of 100.0 mm, which is too tall for your current pipette
configurations. The longest pipette on your robot can only be raised to 96.30000000000001 mm
above the deck. This may be because the labware is incorrectly defined, incorrectly calibrated,
or physically too tall. This could also be caused by the pipette and its tipracks being
mismatched. Please check your protocol, labware definitions and calibrations.
```

Two symptoms were reported, and both are the *same* underlying problem:

1. During **Tip Length Calibration**, the error appears after confirming the labware/cal‑block
   placement (Step 1/6) or after answering "Yes, it picked up the tip" (Step 4/6).
2. The error is sometimes preceded by a separate, *mechanical* failure where the printed
   sensor body will not stay on the pipette nozzle (see "Secondary issue" below).

---

## 2. The custom labware involved

The labware referenced in the issue is
[`ac_color_sensor_charging_port.json`](https://github.com/AccelerationConsortium/ac-dev-lab/blob/e85dd45e19480068f9e8323dad1a6294dabe8075/src/ac_training_lab/ot-2/_scripts/ac_color_sensor_charging_port.json).
The relevant fields are:

```jsonc
{
  "dimensions": { "xDimension": 128, "yDimension": 86, "zDimension": 100 },  // <-- 100 mm tall
  "parameters": {
    "format": "irregular",
    "isTiprack": true,
    "tipLength": 84,
    "loadName": "ac_color_sensor_charging_port"
  },
  "namespace": "custom_beta",
  "schemaVersion": 2
}
```

The labware is loaded as a **tip rack** (`isTiprack: true`) and declares `zDimension: 100`.
That 100 mm value is what flows straight into the error message as the labware's
`highest_z`.

---

## 3. Root cause — where the 96.3 mm ceiling comes from

### 3.1 The check that raises the error

The error is raised inside the **legacy motion planner** used by the calibration flows:
[`api/src/opentrons/protocols/geometry/planning.py`](https://github.com/Opentrons/opentrons/blob/edge/api/src/opentrons/protocols/geometry/planning.py),
function `_build_safe_height(...)`.

When the planner has to retract the pipette to a safe travel height before a move, it computes:

```python
to_safety = deck.highest_z + constraints.lw_z_margin   # tallest thing on the deck + margin
if to_safety > constraints.instr_max_height:           # taller than the pipette can reach?
    if constraints.instr_max_height >= (deck.highest_z + constraints.minimum_lw_z_margin):
        to_safety = constraints.instr_max_height       # OK, just clip to max reach
    else:
        raise LabwareHeightError(...)                   # <-- physically cannot clear it
```

So the error fires when the **tallest labware on the deck** (`deck.highest_z`, here = 100 mm)
is too tall for the pipette to clear (`instr_max_height`, here ≈ 96.3 mm).

### 3.2 Where `instr_max_height` (≈96.3 mm) comes from

The calibration flow asks the hardware for the pipette's maximum reach
([`robot-server/robot_server/robot/calibration/util.py`](https://github.com/Opentrons/opentrons/blob/edge/robot-server/robot_server/robot/calibration/util.py),
`move()`):

```python
max_height = user_flow.hardware.get_instrument_max_height(user_flow.mount)
safe = planning.safe_height(from_loc, to_loc, user_flow.deck, max_height)
```

`get_instrument_max_height` is computed in
[`pipette_handler.py`](https://github.com/Opentrons/opentrons/blob/edge/api/src/opentrons/hardware_control/instruments/ot2/pipette_handler.py):

```python
max_height = pip.config.mount_configurations.homePosition - retract_distance + cp.z
```

and the protocol‑engine equivalent in
[`protocol_engine/state/pipettes.py`](https://github.com/Opentrons/opentrons/blob/edge/api/src/opentrons/protocol_engine/state/pipettes.py)
states it even more plainly:

```python
return config.home_position - Z_RETRACT_DISTANCE + config.nozzle_offset_z
```

The terms are all **hardware / pipette‑definition constants**:

| Term | Source | Value (GEN2 single, example) |
|------|--------|------------------------------|
| `homePosition` | `shared-data/pipette/.../pipetteNameSpecs.json` | `172.15` mm (GEN1 pipettes use `220`) |
| `Z_RETRACT_DISTANCE` | `api/src/opentrons/config/defaults_ot2.py` | `2` mm |
| critical‑point `z` / `nozzle_offset_z` | pipette model definition + tip state | pipette‑specific (negative) |

For the pipette currently installed, that arithmetic evaluates to **≈96.3 mm**. The number
is a property of the gantry travel and the pipette, **not** of the app or robot‑software
version. (It changes only if you physically change the pipette, e.g. a GEN1 vs GEN2 model, or
the critical point because a tip is/ isn't attached.)

---

## 4. What changed across versions — and what did *not*

### 4.1 The height check is old (so downgrading cannot remove it)

`LabwareHeightError` and the `_build_safe_height` logic that raises it have lived in
`planning.py` since the **2020** motion‑planning refactors. Relevant commit history for the
file:

- `6187e58` *"refactor(api): add waypoint planning without the use of Location"* — **2020‑11‑06**
- `e5e1b8c` *"refactor(api): protocol_api package shuffling"* (created `protocols.geometry.planning`) — **2020‑09‑08**

Every Opentrons App / OT‑2 release the lab is likely to use (v4 → v5 → v6 → v7 → v8 → v9)
contains this same check. That is consistent with the observation that **v8.8.1 and v9 behave
identically** — there is no earlier release that omits the 96.3 mm ceiling.

### 4.2 Things that *did* change in recent versions (and why they are not the cause here)

These are real changes in the v6→v9 window, listed so they can be ruled out:

- **Tip Length Calibration moved on‑robot / into Protocol Engine.** The calibration UX and the
  code path were reorganized across v6/v7, but the *geometry* check (`safe_height` →
  `_build_safe_height`) is shared and unchanged.
- **Stricter labware‑schema handling.** Newer software asserts custom tip racks used for
  calibration are **schema 2** (`assert tip_rack_definition["schemaVersion"] == 2` in
  `calibration/util.py`; non‑schema‑2 racks are filtered out in `get_default_tipracks`). The
  color‑sensor labware *is* schema 2, so it passes — this is not the blocker, but it is a real
  reason an *older* schema‑1 definition can suddenly be rejected after an update.
- **`custom_beta` namespace / labware upload validation** was tightened over time. Again, the
  file here is accepted; the failure happens later, during motion planning.

**Net:** none of the version‑to‑version changes introduced the 96.3 mm limit. The limit is the
hardware envelope, and the labware is simply taller than it.

---

## 5. Why the "previous team" seemingly got it to work

The most likely explanations (in rough order of probability):

1. **A shorter labware definition was used previously.** The trigger is purely
   `zDimension`/`highest_z`. A version of this charging‑port/tip definition with
   `zDimension ≤ ~96 mm` (or a thinner "fake tip" design) would never trip the check. The
   current `zDimension: 100` is the single value that needs to come down.
2. **The pipette was different.** A different pipette model / critical point yields a different
   `instr_max_height`. If the previous setup used a pipette/tip combination with a taller reach
   (or never raised the gantry while this labware was the tallest deck item), the error would
   not appear.
3. **The workflow avoided the calibration retract.** The error is raised when the planner has to
   compute a *safe travel height* above the tallest labware. A protocol/flow that never asked the
   gantry to clear the 100 mm item would not hit it.

All three point to the same conclusion: the issue is **geometry + definition**, which is why a
software downgrade is the one thing that *cannot* fix it.

---

## 6. How to adapt and keep using this setup

Pick whichever is most compatible with the sensor mechanical design discussed in
[byu-vcl#33](https://github.com/vertical-cloud-lab/byu-vcl/issues/33):

### A. Reduce the loaded labware height to ≤ ~96 mm (recommended, software‑only)
- Edit the labware definition so `dimensions.zDimension` (and the physical part) is at or below
  the pipette's reach (target **≤ 95 mm** to leave margin below the measured 96.3 mm).
- Keep the well/tip geometry consistent with the real part so calibration still lands correctly.
- This is the smallest change and works on **any** app/robot version.

### B. Re‑design the sensor / charging‑port so the loaded part is shorter
- This matches @falbiston's idea in the thread of a **replaceable nozzle/tip on a fixed body**:
  make the portion that is loaded as labware short enough to clear the gantry, and keep the tall
  body off‑deck or below the deck plane.
- Re‑printing fresh also addresses the *mechanical* "won't stay on the nozzle" problem reported
  separately (a stretched/worn press‑fit tip).

### C. Use a pipette with greater vertical reach
- `instr_max_height = homePosition − 2 mm + critical_point_z`. A different pipette model changes
  `homePosition` and the critical point and therefore the ceiling. This is the least convenient
  option and may still not buy the full 3.7 mm needed.

### D. Avoid the calibration retract entirely
- If the 100 mm part does not actually need to be a *deck* labware that the gantry must clear,
  load it so it is not the tallest deck item during the move that triggers the retract. (Most
  fragile; A or B is preferred.)

**Recommended path:** **A** (bring `zDimension` to ≤ ~95 mm) combined with **B** (a shorter,
replaceable loaded section). That removes the `LabwareHeightError` on every Opentrons version
and is robust to future updates.

---

## 7. Secondary (mechanical) issue noted in the thread

Separately from the software limit, @swcharles reported in
[byu-vcl#33](https://github.com/vertical-cloud-lab/byu-vcl/issues/33#issuecomment-4626373005)
that after the printed sensor "tip" was left hanging on the nozzle overnight, the robot could no
longer pick it up — i.e. the press‑fit stretched/wore out. This is independent of the height
error and is best resolved by **re‑printing** and/or moving to the **replaceable‑nozzle** design.
It explains why, after downgrading, the behavior looked "worse": two different failures were in
play at once.

---

## 8. References (code, as of `Opentrons/opentrons@edge`)

- `api/src/opentrons/protocols/geometry/planning.py` — `LabwareHeightError`, `_build_safe_height`, `safe_height`
- `robot-server/robot_server/robot/calibration/util.py` — calibration `move()` calls `get_instrument_max_height` then `planning.safe_height`
- `api/src/opentrons/hardware_control/instruments/ot2/pipette_handler.py` — `instrument_max_height = homePosition − retract_distance + cp.z`
- `api/src/opentrons/hardware_control/api.py` — `get_instrument_max_height(...)` passes `self._config.z_retract_distance`
- `api/src/opentrons/protocol_engine/state/pipettes.py` — `home_position − Z_RETRACT_DISTANCE + nozzle_offset_z`
- `api/src/opentrons/config/defaults_ot2.py` — `Z_RETRACT_DISTANCE = 2`
- `shared-data/pipette/definitions/1/pipetteNameSpecs.json` — pipette `homePosition` values (GEN2 single `172.15`, GEN1 `220`)
- Custom labware: `AccelerationConsortium/ac-dev-lab` → `src/ac_training_lab/ot-2/_scripts/ac_color_sensor_charging_port.json` (`zDimension: 100`)
