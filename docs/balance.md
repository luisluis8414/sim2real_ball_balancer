# Balancing: dead time, limits and tuning

`ballbal balance` closes the loop camera → controller → servos → plate → ball.
This page records what limits its speed on this rig, measured on 2026-09-10
with the guardrail fitted and a 40 mm ball, and the settings that came out of
it. The tuned values live in the profile's `[balance]` table
([`profiles/my-platform/rig.toml`](../profiles/my-platform/rig.toml)); another
platform should repeat the measurements (see [Retuning](#retuning)).

## Result

Same rig, same ball, before and after:

| Test | Before (built-in defaults) | After (`[balance]` in the profile) |
| --- | --- | --- |
| 80 mm target jumps: 90% of the way | 1.10 s | **0.62-0.65 s** |
| overshoot | 1 mm | 5 mm |
| jumps that end within 5 mm | 3 of 12 | 10 of 12 (from 1.6-2.0 s) |
| error at the end of each jump | 9.8 mm | **2.9-3.2 mm** |
| holding the centre for 15 s: mean distance | 7.2 mm (ball parked off centre) | **1.2 mm** (max 3.3 mm) |
| catching from the rim (95 mm) | never settles within 6 s, ends 10-16 mm out | stays within 5 mm after 3.2-4.8 s |

The jumps come from [`tools/balance/step_benchmark.py`](../tools/balance/step_benchmark.py),
two runs of 12 jumps each. The tuned rise time is within 10% of what the tilt
cap physically allows (see [Limits](#limits)).

## Where the dead time comes from

A lean written now moves the ball only after the whole chain has answered:

| Stage | acceleration 30 (rig default) | acceleration 254 |
| --- | ---: | ---: |
| servo: goal written → first 3 counts of motion | 90 ms | 54 ms |
| servo: goal written → half of a 20-count step | 122 ms | 75 ms |
| camera: plate moves → frame handed to the program | 0-15 ms | 0-15 ms |
| frame sampling: a new frame every 32 ms, on average half old | 16 ms | 16 ms |
| detection and control | ~2 ms | ~2 ms |
| **whole loop, fitted from balance logs** | **136 ms** | **103 ms** (small leans), **118 ms** (tuned, larger leans) |

The servo is by far the largest part, and its acceleration setting is the one
number that moves it. Goal speed (600, 1500 or unlimited) only matters from
about 100 counts of step upwards, where the move is slew-limited.

How each number was measured:

- **Servo**: [`tools/balance/servo_steps.py`](../tools/balance/servo_steps.py)
  steps one axis by 20/60/120 counts at each speed and acceleration and polls
  its position over the bus.
- **Camera**: [`tools/balance/camera_lag.py`](../tools/balance/camera_lag.py)
  lifts the whole plate while the ball rests against the guardrail. The ball
  only moves in the image by perspective, instantly with the plate, so its image
  trace can be laid over the servo trace from the bus. The camera is fast
  because it streams rows while reading them out: by the time `read()` returns,
  a row in the middle of the image is only a few milliseconds old. The driver's
  own timestamps agree: `read()` returns 32.7 ms after a frame's first USB
  packet, about one frame period, which fits a frame that is sent while it is
  being read out. The camera number also contains any lag of the servo's own
  position report, so treat it as an upper bound on how fast the camera is.
- **Whole loop**: [`tools/balance/fit_delay.py`](../tools/balance/fit_delay.py)
  fits `dv = G * integral(tilt(t - T))` to a balance log with a continuous
  delay. The fits explain 96-98% of the ball's velocity changes. `ballbal tune`
  fits the same law but in whole frames, which is why it prints 102 or 135 ms.

## The plant

- **Gain**: 3.9-4.2 mm/s² of ball acceleration per count of leg swing, the same
  at every tilt cap. Kept per count (`plant_gain`), because a gain per unit of
  tilt changes with `max_tilt`.
- **Breakaway**: from rest the ball does not roll until about 10-25 counts of
  lean, depending on where on the plate it sits. Below that the plate can lean
  and nothing happens. This is what parked the ball 10-18 mm off centre with the
  old settings: 5 mm of error asked for 5 counts, and the integral took tens of
  seconds to add the rest.

## Limits

**Tilt authority.** The ball can accelerate at most `G * reach`. A move of
distance `d` takes at least `2 * sqrt(d / a)`:

| Tilt cap | reach | max acceleration | fastest 80 mm move |
| ---: | ---: | ---: | ---: |
| 25% (old default) | 100 counts | 410 mm/s² | 0.88 s |
| 62.5% (tuned) | 250 counts | 1030 mm/s² | 0.56 s |
| 100% | 400 counts | 1650 mm/s² | 0.44 s |

With the old cap, the loop sat at full tilt during every jump and could not be
faster than it was. The tuned 0.62 s is close to its 0.56 s bound. More needs
more reach: home sits 400 counts above the minimum here, and `ballbal neutral
--mid` would allow 605.

**Dead time.** A PID loop with 118 ms of dead time crosses over at about
0.45 / 0.118 = 3.8 rad/s (0.6 Hz) before it runs out of phase margin. Faster
gains without compensation oscillate. Measured: 2.4 counts/mm with 1.6
counts/(mm/s) at a 50% cap overshot 23 mm.

**Supply.** Three axes accelerating at 254 together did not trip a voltage
fault in any of about 30 runs (`ballbal status` clean afterwards). The
rig-wide `acceleration = 30` stays for ordinary moves; only balancing uses 254.

## What changed in the controller

- **Servo settings while balancing**: acceleration 254 and unlimited goal speed.
  This saves about 30 ms of dead time for small leans.
- **Tilt cap 62.5%** (250 counts), gains scaled to match.
- **Prediction** (`predict_ms`): the loop controls on where the ball will be
  once the leans already sent take effect. That is the present position, plus
  velocity times the horizon, plus the committed leans pushed through
  `plant_gain`. 60 ms ahead it misses the ball's real position by 0.9 mm rms,
  against 3.4 mm for simply using the present position. At the same gains it
  cut overshoot from 4.9 to 2.6 mm. Without it, about 2 counts/mm was the
  most kp that stayed well damped. With it, 3.9 counts/mm overshoots 5 mm. A
  horizon below the full dead time (85 of 118 ms) is the robust choice, because
  the velocity term multiplies camera noise.
- **Integral only near the target** (`integral_zone` 12 mm) and 25 times
  stronger than before. It exists to overcome breakaway and unevenness in the
  last millimetres. Left running through a jump, it collected the whole transit
  and pushed the ball past the target.
- **Less derivative smoothing** (0.6 instead of 0.35 per sample). The raw camera
  mode is quieter, so less filtering is needed, and filtering is phase lag.

In leg counts, the tuned gains are kp 3.9 counts/mm, ki 2.0 counts/(mm·s) and kd
1.4 counts/(mm/s). The old ones were 1.0, 0.08 and 0.75.

## Camera mode

New calibrations capture **640×480 raw (YUYV)** instead of 640×360 MJPG. All
modes run at 29.7 fps with the same lag. They differ in resolution and noise
(ball at rest, position scatter):

| Mode | mm per pixel | position noise |
| --- | ---: | ---: |
| 640×360 MJPG (old) | 0.63 | 0.079 mm |
| 640×360 YUYV | 0.63 | 0.051 mm |
| 640×480 MJPG | 0.47 | 0.065 mm |
| **640×480 YUYV** | **0.47-0.50** | **0.042 mm** |

The 4:3 modes see the full sensor height and only give up the sides (x 238-1678
of 1920), which the square crop does not use. JPEG compression makes the ball's
edge wobble. `camera.json` now records the capture size (`capture`); files
without it are treated as 640×360, so old calibrations keep working. Re-run
`ballbal cam-setup` to switch.

In `cam-setup` the platform centre is the centre of the crop. If the plate does
not sit centred vertically, shrink the crop with `,` until the arrow keys can
move it up or down.

## Retuning

For another plate, ball or servo setting:

1. **Record motion.** Run a step benchmark with the current settings:

   ```bash
   python tools/balance/step_benchmark.py --log runtime/measurements/balance/base.csv
   ```

2. **Fit dead time and gain.** Use the reach that `balance` prints
   ("-> N counts per leg"):

   ```bash
   python tools/balance/fit_delay.py runtime/measurements/balance/base.csv --reach 250
   ```

   Put the gain per count into `plant_gain`, and set `predict_ms` to about 70%
   of the dead time.
3. **Compare candidates.** Change one or two flags at a time, for example
   `--kp`, `--ki`, `--predict` or `--max-tilt`. Compare the medians of two runs
   each, because single runs scatter by 10-20%.
4. **Store the winner** in the profile:

   ```toml
   [balance]
   kp = 0.0156            # tilt fraction per mm (x reach = counts per mm)
   ki = 0.008
   kd = 0.0056
   max_tilt = 62.5        # percent of each axis's headroom
   acceleration = 254     # servo ramp while balancing only
   speed = 3400           # counts/s while balancing only
   derivative_smoothing = 0.6
   integral_zone = 12.0   # mm
   predict_ms = 85.0
   plant_gain = 4.1       # mm/s^2 per count, from fit_delay.py
   ```

Every key is optional, and a flag on `ballbal balance` (`--kp`, `--accel`,
`--speed`, `--max-tilt`, `--smoothing`, `--izone`, `--predict`,
`--plant-gain`, ...) overrides it for one run. `ballbal config show` lists what
the profile sets.

Raw logs of the runs behind this page are in `runtime/measurements/balance/`
(not versioned).
