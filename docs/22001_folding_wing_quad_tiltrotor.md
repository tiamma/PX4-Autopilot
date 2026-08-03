# Folding Wing Quad Tiltrotor (Airframe 22001)

Source file: `ROMFS/px4fmu_common/init.d/airframes/22001_folding_wing_quad_tiltrotor`

## Overview

| Field | Value |
|---|---|
| Name | Folding Wing Quad Tiltrotor |
| Type | VTOL Tiltrotor |
| Class | VTOL |
| Excluded boards | `px4_fmu-v2`, `bitcraze_crazyflie`, `holybro_kakutef7` |

This airframe defines a **quadrotor VTOL where all four rotors tilt** between a
vertical thrust orientation (multicopter/MC mode) and a horizontal thrust
orientation (fixed-wing/FW mode). Unlike a "standard VTOL" (separate MC and FW
motors) or a tailsitter (whole airframe tilts), each of the four rotors here
has its **own independently controlled tilt servo/actuator**, hence "Tiltrotor
VTOL" (`CA_AIRFRAME 3`).

It inherits common VTOL defaults from `rc.vtol_defaults` before applying its
own configuration.

```sh
. ${R}etc/init.d/rc.vtol_defaults
```

---

## 1. Vehicle type and control allocation method

```sh
param set-default MAV_TYPE 21
param set-default VT_TYPE 1
param set-default CA_AIRFRAME 3
```

| Parameter | Value | Meaning |
|---|---|---|
| `MAV_TYPE` | `21` | MAVLink vehicle type: `MAV_TYPE_VTOL_TILTROTOR` |
| `VT_TYPE` | `1` | VTOL type used by `vtol_att_control`: `1` = Tiltrotor |
| `CA_AIRFRAME` | `3` | Control allocation airframe: `3` = Tiltrotor VTOL |

`CA_AIRFRAME` selects which effectiveness model the `control_allocator`
module builds. Selecting `Tiltrotor VTOL` enables the rotor position **and**
per-rotor tilt-servo mapping shown below.

---

## 2. Rotor tilt assignment

```sh
param set-default CA_ROTOR0_TILT 1
param set-default CA_ROTOR1_TILT 2
param set-default CA_ROTOR2_TILT 3
param set-default CA_ROTOR3_TILT 4
param set-default CA_SV_TL_COUNT 1
```

| Parameter | Value | Meaning |
|---|---|---|
| `CA_ROTOR0_TILT` | `1` | Rotor 0 is tilted by tilt servo/actuator **1** |
| `CA_ROTOR1_TILT` | `2` | Rotor 1 is tilted by tilt servo/actuator **2** |
| `CA_ROTOR2_TILT` | `3` | Rotor 2 is tilted by tilt servo/actuator **3** |
| `CA_ROTOR3_TILT` | `4` | Rotor 3 is tilted by tilt servo/actuator **4** |
| `CA_SV_TL_COUNT` | `1` | Total number of distinct tilt servo *outputs* declared |

Each rotor is tilted by its own actuator (1, 2, 3, 4 — four independent
tilts), even though `CA_SV_TL_COUNT` is set to `1`. This means the airframe is
configured to expose **one tilt servo output group** in the actuator
mapping, but each rotor still references a distinct tilt index
(1–4) for its own actuator channel. When wiring outputs in
`Actuators` configuration (QGC), you assign each `Tilt ${i}` output its own
PWM/AUX channel — the count/index scheme here is how the control allocation
matrix associates a given rotor with a given tilt channel, not how many
physical servos exist.

> Note: In this specific airframe, all four rotors fold/tilt in unison during
> a VTOL transition (see `Folding Wing` in the name), driven by the tilt
> control logic in `Tiltrotor` (`src/modules/vtol_att_control/tiltrotor.cpp`).

---

## 3. Fixed-wing control surfaces

```sh
param set-default CA_SV_CS_COUNT 4
param set-default CA_SV_CS0_TYPE 1
param set-default CA_SV_CS0_TRQ_R -0.5
param set-default CA_SV_CS1_TYPE 2
param set-default CA_SV_CS1_TRQ_R 0.5
param set-default CA_SV_CS2_TYPE 3
param set-default CA_SV_CS2_TRQ_P 1
param set-default CA_SV_CS3_TYPE 4
param set-default CA_SV_CS3_TRQ_Y 1
```

`CA_SV_CS_COUNT = 4` declares four fixed-wing control surfaces used in FW
(and blended MC/FW transition) mode:

| Index | `CA_SV_CS${i}_TYPE` | Surface | Torque scaling |
|---|---|---|---|
| 0 | `1` | Left Aileron | `CA_SV_CS0_TRQ_R = -0.5` (roll) |
| 1 | `2` | Right Aileron | `CA_SV_CS1_TRQ_R = 0.5` (roll) |
| 2 | `3` | Elevator | `CA_SV_CS2_TRQ_P = 1` (pitch) |
| 3 | `4` | Rudder | `CA_SV_CS3_TRQ_Y = 1` (yaw) |

The `TRQ_R` / `TRQ_P` / `TRQ_Y` parameters scale how much each control
surface contributes to commanded roll/pitch/yaw torque, and their sign
determines direction of deflection (ailerons have opposite signs since they
move in opposite directions for roll).

---

## 4. Rotor geometry (square quad-X)

```sh
param set-default CA_ROTOR_COUNT 4
param set-default CA_ROTOR0_PX 1
param set-default CA_ROTOR0_PY 1
param set-default CA_ROTOR1_PX -1
param set-default CA_ROTOR1_PY -1
param set-default CA_ROTOR2_PX 1
param set-default CA_ROTOR2_PY -1
param set-default CA_ROTOR2_KM -0.05
param set-default CA_ROTOR3_PX -1
param set-default CA_ROTOR3_PY 1
param set-default CA_ROTOR3_KM -0.05
```

`CA_ROTOR_COUNT = 4` declares a quadrotor MC layout. Positions are in PX4
body axes: **+X forward, +Y right, +Z down** (down/Z omitted here, i.e. 0).

| Rotor | `PX` (fwd/aft) | `PY` (left/right) | Position (approx.) | `KM` (spin dir) |
|---|---|---|---|---|
| 0 | `1` | `1` | Front-right | default (`0.05`, CW) |
| 1 | `-1` | `-1` | Rear-left | default (`0.05`, CW) |
| 2 | `1` | `-1` | Front-left | `-0.05` (CCW) |
| 3 | `-1` | `1` | Rear-right | `-0.05` (CCW) |

This forms a standard **quad-X** motor arrangement:

```
        FRONT
   2 (CCW)   0 (CW)
      \       /
       \     /
       /     \
      /       \
   1 (CW)    3 (CCW)
        REAR
```

`CA_ROTOR${i}_KM` is the reaction-torque (moment) coefficient. Rotors 0 and 1
use the module default (`0.05`), while rotors 2 and 3 are explicitly set to
`-0.05` to spin the opposite direction — this alternating CW/CCW pattern is
required so that motor torques cancel out and yaw can be controlled by
differential motor speed, exactly like a standard quadrotor.

---

## 5. Summary diagram: MC vs FW mode

| Mode | Rotor thrust direction | Active control effectors |
|---|---|---|
| MC (hover) | Vertical (tilts at 0°) | 4 rotors (roll/pitch/yaw/throttle via differential thrust) |
| Transition | Rotors tilting from vertical → horizontal | Blend of rotors + control surfaces |
| FW (cruise) | Horizontal (tilts at ~90°) | 4 rotors for forward thrust + ailerons/elevator/rudder for roll/pitch/yaw |

---

## Related files

- `src/modules/vtol_att_control/tiltrotor.cpp` / `tiltrotor.h` — tiltrotor
  transition logic, tilt angle scheduling.
- `src/modules/control_allocator/module.yaml` — full parameter reference for
  `CA_AIRFRAME`, `CA_ROTOR*`, `CA_SV_CS*`, `CA_SV_TL_COUNT`.
- `ROMFS/px4fmu_common/init.d/rc.vtol_defaults` — common VTOL parameter
  defaults applied before this airframe's own settings.
