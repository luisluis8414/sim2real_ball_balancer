# Balancing: dead time, limits and tuning

`ballbal balance` closes the loop camera → controller → servos → plate → ball.
This page records what limits its speed on this rig, measured on 2026-09-10
and 2026-09-11 with the guardrail fitted and a 40 mm ball, and the settings
that came out of it. The tuned values live in the profile's `[balance]` table
([`profiles/my-platform/rig.toml`](../profiles/my-platform/rig.toml)); another
platform should repeat the measurements (see [Retuning](#retuning)).

## Result

Same rig, same ball. The first tuning (2026-09-10) made the loop fast; the
retune (2026-09-11, [below](#retune-of-2026-09-11)) took most of the ringing out
of it. Both profiles were measured on 2026-09-11, in the same session:

| Test | Built-in defaults (2026-09-10) | 2026-09-10 profile | **Profile now** (2026-09-11) |
| --- | --- | --- | --- |
| 80 mm target jumps: 90% of the way | 1.10 s | 0.59 s | 0.67 s |
| overshoot (median, middle half of jumps) | 1 mm | 7.2 mm (4.4-9.0) | **2.9 mm** (0.3-5.2) |
| jumps that end within 5 mm | 3 of 12 | 18 of 24, from 1.94 s | 25 of 36, from **1.30 s** |
| error at the end of each jump | 9.8 mm | 4.5 mm | 4.4 mm |
| error from 1 s after each jump (rms) | – | 7.4 mm | **4.6 mm** |
| holding the centre: mean / max distance | 7.2 mm (ball parked off centre) | 1.9 / 6.3 mm | 2.0 / **3.9 mm** |
| holding: leg movement per frame | 0.4 counts | 1.4 counts (commands on 41% of frames) | 1.2 counts (38%) |
| catching from the rim (86 mm), one run each | never settles within 6 s | within 5 mm after 11.4 s | within 5 mm after **1.9 s** |

The retune trades 0.08 s of rise time for much less ringing: after the first
arrival the ball no longer falls back 20 mm and swings on at about 1.1 Hz.
The error at the end of a jump did not improve. The ball still parks a few
millimetres off target (see [Open problems](#open-problems)).

The jumps come from [`tools/balance/step_benchmark.py`](../tools/balance/step_benchmark.py)
and [`tools/balance/servo_trace.py`](../tools/balance/servo_trace.py), runs of
12 jumps each: two for the 2026-09-10 profile, three for the current one.
Holding is 20 s with the ball released at the rim, counted from 5 s on. The
rise time is still within 20% of what the tilt cap physically allows (see
[Limits](#limits)).

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

### The split under balancing

The table above times single steps from rest. While balancing, the goals
change on every frame, so
[`tools/balance/servo_trace.py`](../tools/balance/servo_trace.py) runs the real
loop and polls every servo's position, speed, load and supply voltage at about
350 Hz beside it, on the same clock as the frames and the goal writes. The
same `fit_delay` law, once with the commanded tilt and once with the tilt
computed from the measured leg positions, splits the loop (2026-09-10 profile,
80 mm jumps):

| Stage | delay |
| --- | ---: |
| goal written → legs there (best-matching pure delay) | 84-102 ms, per axis |
| measured leg position → ball motion in the frame the program receives | 22 ms |
| **commanded tilt → ball** | **106 ms** |

So the servos are about 90 of the 106 ms. The measured tilt reaches only 89%
of the commanded one, because the goals move on before the legs arrive. The
servo model of [`servo_model.py`](../src/ballbal/simulation/servo_model.py),
refitted to these closed-loop traces, matches them to 2-4 counts rms:

| Axis | dead time | acceleration | top speed | lag |
| --- | ---: | ---: | ---: | ---: |
| 1 | 32 ms | 23 300 counts/s² | 2270 counts/s | 17 ms |
| 2 | 26 ms | 26 600 counts/s² | 2740 counts/s | 25 ms |
| 3 | 35 ms | 28 700 counts/s² | 2370 counts/s | 16 ms |

The acceleration is the firmware's own limit: register 254 is 25 400
counts/s², and 0 (no ramp) measured 27 500 on the bench. The legs never reach
the 3400 counts/s written as goal speed; they peak at 1700-2000 counts/s under
the load of the plate.

## The plant

- **Gain**: 3.9-4.2 mm/s² of ball acceleration per count of *commanded* leg
  swing, the same at every tilt cap. Kept per count (`plant_gain`), because a
  gain per unit of tilt changes with `max_tilt`. Per count of *measured* leg
  swing it is 4.4-5.2 mm/s², because the legs fall short of their goals.
- **Breakaway**: from rest the ball does not roll until about 10-25 counts of
  lean, depending on where on the plate it sits. Below that the plate can lean
  and nothing happens. This is what parked the ball 10-18 mm off centre with the
  old settings: 5 mm of error asked for 5 counts, and the integral took tens of
  seconds to add the rest. Where the ball parks is repeatable: after a jump to
  (+40, 0) mm it stopped near (−4, −4) mm off target in most runs, after a jump to
  (0, −40) mm near (+3, +3). That points at unevenness of the plate rather than
  at random stiction.
- **Tilting moves the ball in the image at once.** The ball's centre sits
  above the plate, so a tilting plate carries it sideways before it has rolled
  at all. Between 3.5 and 8 Hz the ball's image moves in phase with the
  measured tilt, about 6 mm per unit of tilt (0.025 mm per count), 22 ms late.
  That explains 97% of the ball's fast motion. The double integrator alone would
  give 1.1 mm at 5.5 Hz, in antiphase. The derivative reads this as ball
  velocity, which closes a fast loop from the legs straight back to the legs;
  see [Retune of 2026-09-11](#retune-of-2026-09-11).
- **Rolling friction** while the ball moves is not measurable against the
  noise, below about 20 mm/s².

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
The servo trace shows how close that is: the 5.4 V supply sags to 4.4-4.9 V
at the servos during jumps, below the profile's `min_voltage = 4.5`, with the
PWM load at up to 90%. This STS3215 is a 7.4 V servo (it accepts 4-8 V). A
stiffer supply nearer 7.4 V would remove the sag and raise the top speed. It would not shorten
the ~30 ms firmware dead time or the acceleration limit, so it helps large
moves more than the small leans that holding needs.

**Frame rate.** Every mode of this camera is capped at 30 fps
(`v4l2-ctl --list-formats-ext`). With the servos at 90 ms, a faster camera would
save at most the ~16 ms of sampling delay.

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
  most kp that stayed well damped. With it, 3.9 counts/mm overshoots 5 mm.
  The first tuning kept the horizon below the full dead time (85 of 118 ms),
  because the velocity term multiplies camera noise. The retune went to 173 ms;
  see [Retune of 2026-09-11](#retune-of-2026-09-11).
- **Integral only near the target** (`integral_zone` 12 mm) and 25 times
  stronger than before. It exists to overcome breakaway and unevenness in the
  last millimetres. Left running through a jump, it collected the whole transit
  and pushed the ball past the target.
- **Derivative from the predicted velocity** (with prediction on), smoothed
  at 0.45 per sample (0.2885 since the retune). Commands are only sent when a leg would move at least 4
  counts (`quiet`). See [Holding still](#holding-still).

In leg counts, the gains of this first tuning were kp 3.9 counts/mm, ki 2.0
counts/(mm·s) and kd 1.4 counts/(mm/s). The built-in defaults are 1.0, 0.08 and
0.75.

## Retune of 2026-09-11

The full report with plots, including the variants that were not adopted, is
[pid-optimisation.md](pid-optimisation.md).

**What was wrong.** With the 2026-09-10 profile the ball reached the target in
0.6 s, then fell back about 20 mm and swung on at about 1.1 Hz, ±10 mm along
the jump and ±7 mm across it. Most of the "overshoot" was this ringing, not
the first pass. The braking lean, driven by the derivative, peaked when the
ball had already stopped and sent it back. The loop sat close to its stability
edge: in a model of the rig, 10 ms more latency made the 80 mm jumps overshoot
17 mm.

**How the candidates were found.** The servo traces gave a model of the whole
rig: the closed-loop servo model above, ball acceleration from the measured
tilt (1500 mm/s² per unit of tilt, rotated −3°), 26 ms to the frame, the
image shift from tilting (6 mm per unit), 0.12 mm of position noise, and the
controller code itself. It reproduced the recorded jumps of the 2026-09-10
profile to about 2 mm rms. The gains were then optimised in the model against
four cases (nominal, +10 ms latency, plant gain ±10%), with plate jitter while
holding capped at the old level. The winners were checked on the rig.

**What came out.**

| Key | 2026-09-10 | now |
| --- | ---: | ---: |
| `kp` | 0.0156 (3.9 counts/mm) | 0.01639 (4.1 counts/mm) |
| `kd` | 0.0056 (1.4 counts/(mm/s)) | 0.00401 (1.0 counts/(mm/s)) |
| `derivative_smoothing` | 0.45 | 0.2885 |
| `predict_ms` | 85 | 173 |
| `plant_gain` | 4.1 | 5.94 |

`ki`, `integral_zone`, `quiet`, the tilt cap and the servo settings are
unchanged. `predict_ms` and `plant_gain` are no longer the measured dead time
and gain. The predictor uses the horizon twice: as how far ahead it looks and
as how late it assumes a sent lean acts. The two values were tuned together
with the gains. A longer horizon moves the braking earlier. The heavier
derivative smoothing keeps the noise that the longer horizon multiplies off
the plate.

**What did not work.** Taking the smoothing out entirely (0.93, with
kd 0.0051, horizon 128 ms) removed the overshoot on the rig completely (0 mm,
24 jumps). While holding, though, the plate buzzed at 5.5 Hz: 12 counts of leg
movement per frame, commands on 94% of frames. That is the image shift from
tilting: the derivative sees the plate carry the ball sideways and answers it,
22 ms later, through a servo that is already 90 ms behind. The first model
lacked that effect and predicted less than half the buzz. With the effect
added, it matched, and the current values were optimised in that model.

## Holding still

The first tuned version held the ball 1.2 mm from the centre, but the plate
jittered hard around it: the legs moved 18 counts from one frame to the next
(95th percentile 43, max 69), commands went out on 99% of frames, and the
shaking plate moved the ball by 0.5 mm per frame.

The derivative caused it. It differenced the *predicted* position frame by
frame, and the prediction contains the controller's own last commands. Every
change of command therefore jumped the derivative, which changed the command
again: a loop running at the frame rate, inside the balance loop. On top of
that, differencing at 30 Hz turns 0.5 mm of movement per frame into 15 mm/s,
which is 21 counts of lean at kd 1.4.

The predictor already knows the velocity the ball will have once the committed
leans act: the present velocity plus their effect over the horizon. The
derivative now uses that directly instead of differencing the prediction.
Holding the centre for 15 s at the same gains:

| Derivative | ball distance mean / max | leg movement per frame | frames with a command |
| --- | ---: | ---: | ---: |
| difference of the prediction (before) | 1.2 / 3.3 mm | 18.3 counts | 99% |
| predicted velocity, smoothing 0.6, quiet 3 | 1.6 / 6.4 mm | 2.2 counts | 74% |
| predicted velocity, smoothing 0.6, quiet 4 | 1.9 / 4.5 mm | 1.7 counts | 50% |
| predicted velocity, smoothing 0.45, quiet 4 (2026-09-10 profile) | 1.2 / 3.6 mm | 1.3 counts | 38% |
| quiet 6 | 1.9 / 8.6 mm | 2.1 counts | 40% |

A higher `quiet` than 4 makes it worse: the legs then move in larger, rarer
steps, and each one shakes the ball more.

The retune was measured over 20 s with the ball released at the rim, counted
from 5 s on (2026-09-11, same session):

| Settings | ball distance mean / max | leg movement per frame | frames with a command |
| --- | ---: | ---: | ---: |
| 2026-09-10 profile | 1.9 / 6.3 mm | 1.4 counts | 41% |
| **profile now** | **2.0 / 3.9 mm** | **1.2 counts** | **38%** |
| smoothing 0.93, horizon 128 ms (rejected) | 2.2 / 5.2 mm | 12.3 counts | 94% |

## Open problems

**The ball parks off target.** In about 30% of the frames late in a jump, the
ball sits still 4-5 mm from the target while the plate leans only 10-13 counts
towards it, below breakaway. The predictor causes this. It assumes every
committed lean accelerates the ball, so it predicts the ball rolling towards
the target at 10-16 mm/s while it stands still (0.3 mm/s). The derivative then
takes back 8-17 counts of the lean. The integral cannot make up for it: it
integrates the *predicted* error, which the same assumption shrinks. More gain
does not fix this. It needs an estimator that notices when the ball does not
respond as predicted.

**A Kalman estimator, tried as a prototype.** It tracks position, velocity and
an unknown acceleration (plate slope, unevenness, stiction) from the camera
and the commanded leans. The lean it adds against that acceleration is fed
forward. The camera position is corrected by 6 mm × the measured tilt of 22 ms
earlier, read from the servos. On the rig it cut the error at the end of a
jump to 2.1-2.8 mm (4.4 mm with the profile now), and 21 of 24 jumps ended
within 5 mm. Its rise time was slower (0.80-0.93 s), and while holding it
jittered at 8.7 counts per frame. Without the image correction, the end error
rose to 6.9 mm. It is not in the controller yet. Its derivative smoothing and
`quiet` need tuning against holding jitter on the rig, because the model
underestimates fast effects like this one.

**Hardware.** A stiffer supply nearer 7.4 V (see [Limits](#limits)) and a flatter
plate surface where the ball parks are the physical levers. In the model,
faster servos or a 60 fps camera each improved the jumps only a little
compared with the retune. The servo firmware's dead time and acceleration
limit stay either way.

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

1. **Record motion.** Run a step benchmark with the current settings. The
   servo trace runs the same jumps and also records what the legs really do:

   ```bash
   python tools/balance/step_benchmark.py --log runtime/measurements/balance/base.csv
   python tools/balance/servo_trace.py steps --out runtime/measurements/trace/base
   python tools/balance/servo_trace.py hold --seconds 20 --out runtime/measurements/trace/base_hold
   ```

2. **Fit dead time and gain.** Use the reach that `balance` prints
   ("-> N counts per leg"):

   ```bash
   python tools/balance/fit_delay.py runtime/measurements/balance/base.csv --reach 250
   ```

   As a starting point, put the gain per count into `plant_gain` and set
   `predict_ms` to about 70% of the dead time. On this rig the best values
   ended up well above both (5.94 and 173 ms), so treat them as tuning knobs
   from here on.
3. **Compare candidates.** Change one or two flags at a time, for example
   `--kp`, `--kd`, `--smoothing`, `--predict` or `--plant-gain`. Compare the
   medians of two or three runs each, because single runs scatter by 10-20%.
   The time to stay within 5 mm scatters more, since it depends on where the
   ball parks. Check every candidate with a hold run as well: less derivative
   smoothing or a longer horizon can look better on jumps and still make the
   plate buzz while holding (leg movement per frame, share of frames with a
   command).
4. **Store the winner** in the profile:

   ```toml
   [balance]
   kp = 0.01639           # tilt fraction per mm (x reach = counts per mm)
   ki = 0.008
   kd = 0.00401
   max_tilt = 62.5        # percent of each axis's headroom
   acceleration = 254     # servo ramp while balancing only
   speed = 3400           # counts/s while balancing only
   derivative_smoothing = 0.2885
   integral_zone = 12.0   # mm
   predict_ms = 173.0     # tuned together with the gains
   plant_gain = 5.94      # mm/s^2 per count; start from fit_delay.py
   quiet = 4              # counts a leg must move before a command is sent
   ```

In the standard `ballbal balance` preview, click anywhere inside the plate to
move the target there. The cyan cross and the `target x/y` line show the active
setpoint; clicking also stops a running circle or square path. For a remote
live view with the same target control, add `--ui` and open
[Lichtblick](lichtblick.md).

Every key is optional, and a flag on `ballbal balance` (`--kp`, `--accel`,
`--speed`, `--max-tilt`, `--smoothing`, `--izone`, `--predict`,
`--plant-gain`, ...) overrides it for one run. `ballbal config show` lists what
the profile sets.

Raw logs of the runs behind this page are in `runtime/measurements/balance/`
(2026-09-10) and `runtime/measurements/trace/` (2026-09-11, with servo traces;
`base_*` is the 2026-09-10 profile, `p2_*` and `profile_*` the one now, `opt_*`
the rejected unsmoothed candidate, `k*_*` the Kalman prototype). They are not
versioned.
