# STS3215 servo identification (real rig → Isaac Sim)

The three platform axes are Feetech STS3215 servos driven by `ballbal` ([control code](../src/ballbal/control),
formerly `ball_balancer_v3`). This folder holds the measurements that pin down how they
move, the fitted model, and how that model is used in Isaac Sim so the simulated servos
behave like the real ones.

## Result

Measured on 2026-09-10 at the rig's operating point: 5.4 V supply, goal speed 2650, goal
acceleration 100 (the `rig.toml` defaults every `ballbal` command uses), P/D/I = 32/32/0,
torque limit 1000, factory torque caps.

| quantity | value | in degrees |
| --- | ---: | ---: |
| dead time (goal write → axis model reacts) | **15.5 ms** | |
| acceleration | **10 142 counts/s²** | 891 °/s² |
| deceleration | **13 976 counts/s²** | 1228 °/s² |
| max speed | **2481 counts/s** (up 2437, down 2528) | 218 °/s |
| position-loop lag (first order) | **34.7 ms** | |
| time to first 3 counts of motion (`ballbal latency` definition) | 69 ms | |
| final error | −3 … +2 counts, depends on approach direction | ±0.3° |

At the servo's own maximum (speed 0, acceleration 0) the axes accelerate at ~27 500
counts/s² but top out at the same ~2450 counts/s: at 5.4 V the motor, not the speed register,
sets the top speed. The 2650 in `rig.toml` is never reached under the platform's load.

Acceleration register 100 → 10 142 counts/s² confirms the register unit of 100 counts/s².

Machine-readable runtime values: [params.json](../src/ballbal/simulation/params.json).
Per-step fits and the global model produced by `analyze.py`:
`models/servo/fit.json` (generated locally from a step recording).

## Model

```
goal written ─► dead time 15.5 ms ─► trapezoidal profile ─► first-order lag 34.7 ms ─► position
                                     accel 10142 / decel 13976 counts/s², v_max 2481 counts/s,
                                     re-planned on every new goal (like the servo firmware)
```

Implemented in [servo_model.py](../src/ballbal/simulation/servo_model.py) (plain Python, no Isaac dependency), in counts
and seconds so goals can be fed exactly as `ballbal` writes them.

## Validation

| data | what it is | model error (RMS) |
| --- | --- | ---: |
| `runtime/measurements/servo_model/steps_20260910_121321.csv` | 54 steps, 50–1000 counts, each axis alone + differential | 2.0 counts / 0.18° (median per step) |
| `runtime/measurements/servo_model/tracking_20260910_121906.csv` | 12 s of balance-like goals: 30 Hz, differential, rotating tilt + random jumps, ±200 counts | 0.32° |
| same, replayed in Isaac Sim | the recorded goals fed to the simulated servos, joint angles compared with the real positions | **0.31–0.41°** per axis |
| `runtime/logs/rig/rotate_20260906_*.csv` | older rotation runs (before the 09-08 recalibration) | 0.6–1.0° at ±887 counts |

For comparison on the tracking data: an ideal profile without dead time/lag is off by 1.85°,
an instant servo (position = goal) by 4.5°.

## Isaac Sim

[isaac.py](../src/ballbal/simulation/isaac.py) runs one model per axis on every physics step and
feeds its position (plus velocity feed-forward) to a stiff PhysX drive on the servo joint:

- stiffness 50 Nm/rad, damping 0.15 Nm·s/rad — tracks the model within 0.15°
- torque cap 1.40 Nm: STS3215 stall torque 19.5 kg·cm at 7.4 V scaled to 5.4 V
  (**datasheet-derived, not measured**)
- joint velocity cap 300 °/s, above the servo's 218 °/s so the model sets the speed
- servo joint limits = `calibration.toml` min/max, converted with the mapping below; the lower limit
  is the base contact where min lies below it
- elbows unlimited, as Fusion exported them (`continuous`): with all three axes at max the
  rig folds them past 180°; the URDF's −24° had stopped the platform at +23.7° servo
- goal speed per command like the rig, default `rig.toml`'s `speed`; below saturation the
  model tracks it, above it the motor's 2481 counts/s applies
- acceleration and deceleration scaled with `rig.toml`'s `acceleration` register (identified
  at 100)

Commands use the real rig's units: `servos.goto({1: 1800, 2: 1300, 3: 1400}, speed=1000)`,
`servos.home()`, `servos.positions()`.

### Single source of truth: profiles/platform-v2/

This profile is a snapshot of the repository author's physical platform. It is
used here to reproduce the recorded identification and sim-to-real comparison;
it must not be used to control another build. New hardware needs a profile
copied from `profiles/example` and its own calibration.

Nothing about the rig is duplicated for the simulation. [profile.py](../src/ballbal/simulation/profile.py) reads
`profiles/platform-v2/rig.toml` and `calibration.toml` — limits, home, `rest_position`, speed, acceleration — and adds only what exists
in the simulation alone, from [params.json](../src/ballbal/simulation/params.json): the servo dynamics, which
sim joint each axis drives (`sim_joints`), and the CAD angle of the rest pose. `ballbal
calibrate`, `trim`, `neutral` and `rest` all write `calibration.toml`, and `isaac.py` re-reads the profile
on the first physics step of every run, so their results apply to the next Play. If home
changed, the armed servos print a note: run `tools/simulation/set_home_pose.py` once to make the new home the
paused pose.

### Mapping

The anchor is the rest pose: the platform seated on the base, every lower link resting on it,
which in the CAD is **−21.45°** for every leg (mesh contact test). `ballbal rest` measures it on
the rig (three seatings averaged: 964/962/968, 488/488/476, 534/535/541 → 965/484/537 on
2026-09-10) and stores it as `rest_position`:

sim angle = (counts − zero) · 360/4096, zero = rest_position + 21.45° · 4096/360

and larger counts raise a leg on the rig (`balance.py`) and in the sim. axis_1 is the leg under the
camera, the others follow clockwise from above (confirmed on the rig), which in the model is
servo_1, servo_3, servo_2. With today's `platform-v2` profile:

| rig axis | ID | sim joint | rest | home | min | max | home / min / max in the sim |
| --- | ---: | --- | ---: | ---: | ---: | ---: | --- |
| axis_1 | 1 | servo_1 | 965 | 1345 | 949 | 2741 | +11.9° / −22.9° / +134.6° |
| axis_2 | 2 | servo_3 | 484 | 865 | 473 | 2257 | +12.0° / −22.4° / +134.4° |
| axis_3 | 3 | servo_2 | 537 | 920 | 489 | 2351 | +12.2° / −25.7° / +138.0° |

Home comes out level (12.0 ± 0.1°). The calibrated minima sit below the contact because
`ballbal calibrate` pushes each axis there alone; the sim's servo joints stop at the contact
(lower limit max(min, −21.45°)), so commanded to min an axis lands on the base, as on the rig.

Earlier anchors, for the record: home as the CAD pose put every pose ~13° too low (min drove
the links into the base); the calibrated `observed_min` as the contact left axis_3 3.9° too
high. Releasing torque does not find the rest pose either — the gearbox does not backdrive,
so the platform stays where it was.

### Home as the stage pose

[set_home_pose.py](../tools/simulation/set_home_pose.py) plays the simulation, drives the servos to home,
and authors the settled link transforms and joint angles as the stage pose. The platform then
sits at home while paused or stopped, and Play starts from there without a jump; the servo
models initialise from the authored joint angles.

### Real vs sim, side by side

[compare_real.py](../tools/simulation/compare_real.py) writes the same goals — home, min, max, home, all
axes together at 1000 counts/s — to the rig and to the simulation, and records both
(`runtime/measurements/servo_model/sim_vs_real_20260910_130133.csv`):

| leg | real end (counts) | sim end | real arrives | sim arrives |
| --- | --- | --- | ---: | ---: |
| home | 1342 / 862 / 917 | 1345 / 865 / 920 | — | — |
| min | 960 / 483 / 504 (pressing on the base) | 965 / 484 / 537 (on the base) | — | — |
| max | 2733 / 2249 / 2333 (stalls) | 2741 / 2257 / 2351 | — | 1.98 s |
| home | 1346 / 867 / 924 | 1345 / 865 / 922 | 1.530 s | 1.550 s |

During the moves the traces lie within ~1–2° of each other. The simulation runs slower than
real time; the script waits for it to cover each leg before the next goal.

## Assumptions and open points

- **Binding at max.** With all three axes together the rig stalls 8–18 counts (0.7–1.6°)
  short of max; the sim reaches it (no friction or part compliance modelled).
- **At min** the rig presses on past the seated pose while torque is on (axis_3 by ~33 counts,
  2.9°) and springs back when released — compliance under full servo torque, which the rigid
  sim does not have. Seatings scatter by ±6 counts.
- **Not modelled:** the ±3-count settling deadband, supply sag when three axes accelerate
  hard together (the rig browns out above ~1800 counts/s with all three moving), and torque
  saturation beyond the datasheet estimate.

## Re-running

Take the ball off the plate, activate the project environment, then run from the repo root:

```bash
python tools/simulation/identify.py
```

```bash
python tools/simulation/identify_tracking.py
```

```bash
python tools/simulation/analyze.py runtime/measurements/servo_model/steps_<timestamp>.csv
```

Update `src/ballbal/simulation/params.json` with the new fit;
`servo_model.py` and `isaac.py` read it.

Side-by-side check (Isaac Sim with `models/usd/scene.usda` open and the Python Server extension):

```bash
python tools/simulation/compare_real.py
```
