# Platform control

`ballbal` controls three STS3215 servos on one half-duplex serial bus. Commands
use the active directory under `profiles/`. Select it persistently with
`ballbal config set <name>`; without a name that command offers an interactive
list. Every invocation logs the active profile before doing work.

```
ballbal ports      list attached serial adapters
ballbal scan       ping IDs 1..20                       (read-only)
ballbal status     full telemetry per servo             (read-only)
ballbal check      run preflight without moving         (read-only)
ballbal home       ramp servos to their home positions
ballbal goto A 2100    ramp one servo to a position
ballbal jog        interactive keyboard control
ballbal release    torque off, now
```

## First-time setup

```
ballbal setup
```

Runs the whole procedure in order, prompting before every step and moving
nothing without asking. Each step is also its own command, so a single one can
be repeated without starting over:

| step | command | why it is here |
| --- | --- | --- |
| 1 | `ballbal set-ids` | servos ship with the same ID, so several on one bus answer to the same address and cannot be told apart |
| 2 | `ballbal prepare` | a servo reused from another robot carries its offsets, angle limits and torque caps |
| 3 | `ballbal center` | **nothing bolted on** -- this is the reference the mechanism is built around |
| | *assemble the complete platform* | arms, upper links **and** the plate |
| 4 | `ballbal calibrate` | travel can only be measured once the linkage is on |
| 5 | `ballbal sweep` | drive the measured limits under power to confirm they hold |

The order matters because each step invalidates the one before it. Fitting a
linkage before centring leaves an axis short of travel on one side, and no
amount of software fixes that.

**Calibrate only with the whole platform assembled** -- arms, upper links and the
plate. On a parallel mechanism the three legs constrain each other, so an axis
measured with the links or the plate still off swings far further than the
finished machine ever will. What calibration records becomes the hard travel
limit everything afterwards is clamped to, so measuring a partly built rig locks
in limits it cannot reach. Both `ballbal setup` and `ballbal calibrate` say so
and ask before starting.

### Assigning IDs

`ballbal set-ids` walks the servos in the order they appear in the profile's `rig.toml`, asking
for **one servo connected at a time**. Which physical servo becomes 1, 2 or 3 is
decided by which one is plugged in when it asks.

**Assign them clockwise around the platform.** The three servos sit in a circle.
Any of them can be axis 1; the rest must follow clockwise, viewed from above.
The geometry that turns platform tilt into three servo angles assumes that
order, and an axis numbered out of order makes the platform lean the wrong way
with nothing in the readings to explain it.

It refuses to write an ID while more than one servo answers -- that would leave
two servos sharing an address, and neither could be corrected afterwards without
unplugging one. The change also moves the address mid-operation: a servo answers
to its new ID the instant the write lands, so the read-back and the EPROM
re-lock are addressed to the new ID, and success is confirmed by pinging both
addresses rather than by the write's acknowledgement.

### Centring, then mounting

`ballbal center` puts every servo at encoder mid-scale (2048) with nothing
attached, so each axis has equal travel in both directions once the horns go on.
The horn splines give about 0.3 deg of resolution; anything finer is corrected
afterwards through `home_position` in `calibration.toml`, not by refitting.

## Supply voltage

These are 7.4 V STS3215 servos, but this platform is deliberately operated from
a regulated **5 V** supply. A reading around 5.0-5.5 V at the servo is therefore
the expected operating point. The servos' own firmware accepts 4.0-8.0 V
(readable per servo in `ballbal status`), and preflight enforces that range plus
the `min_voltage` floor in the profile's `rig.toml`.

Running below the 7.4 V nominal rating costs torque and top speed -- these
motors are noticeably softer at 5 V than at 7.4 V -- but it is the documented
configuration, not a fault. If an axis stalls under load, insufficient torque
headroom is worth ruling out before assuming a software problem: watch whether
`supply` in `ballbal status` sags while the motor is driving.

## The platform profile

`rig.toml` holds connection settings, safety policy, motion defaults and axis
identity. Generated safe travel and reference positions live beside it in
`calibration.toml` (0..4095 encoder counts = 360°).

Those travel limits are deliberately **not** read from the servos' EPROM angle
limits at runtime. The EPROM limits describe what the servo can survive; they
say nothing about where the platform's linkages collide. On this rig servo 5
reports `0..4095` — using that as a safe range would let a "go to the middle of
your travel" command swing it ~95° in one move.

Use `ballbal calibrate` to measure and store real limits with a safety margin.

## Minimum-only calibration

Once the complete linkage is attached, run:

```
ballbal calibrate
```

Torque is released on every axis. Move the complete platform into its common
lower pose and keep it there. After one `yes` confirmation, all three encoder
positions are read together immediately. The configured margin is added to each
reading and only those three `min_position` values are stored in
`calibration.toml`.

Home and maximum are not independently calibrated:

```text
home_position = min_position + home_offset
max_position  = min_position + max_offset
rest_position = min_position
```

The offsets are profile-specific. `my-platform` currently uses 400 counts for
Home and 1210 counts for Max. This makes all axes share the same geometry while
retaining their individual encoder origins. Changing either offset is an
explicit edit to `rig.toml`, not hidden generated state.

`Rig.goto` clamps every goal to the derived min/max window. The simulation uses
the same derivation in `src/ballbal/simulation/profile.py`, so there is no copied home or max
value to drift out of sync. Calibration logs go under `runtime/logs/`; a backup
of the previous calibration is kept beside it as `calibration.toml.bak`.

## Verifying the limits

```
ballbal sweep
```

Three phases, in this order:

1. **All axes to home**, so the run starts from a known pose rather than
   wherever the last command left things.
2. **Each axis alone**, min then max then home. A failure here names one axis.
3. **All axes together**, the same legs, at a reduced speed. `--no-together`
   skips it; it is also skipped automatically if any axis failed on its own.

The combined phase is the riskiest step, for two reasons. On a parallel
mechanism those poses can bind even when each axis reaches its limit alone. And
three axes accelerating at once draw roughly three times the current: measured on
a 5 V supply, three together hold up at 1800 counts/s (5.2 V minimum) and
collapse at 2650, far enough for the servos to raise a voltage fault. **A faulted
servo drops torque, and the mechanism falls** -- on this rig it settled 14 to 45
counts below three limits at once, which then blocks every normal command.
Recovery needs nothing special -- every goal is clamped into travel, so the only
move available from outside it is one heading back in, and any command will do
it while printing a note. `ballbal recover` exists for when that is the whole
intention: one axis at a time, at reduced speed, reporting what it found first.

The sag is faster than software can sample, so the servo's own protection is what
trips; no polling loop catches it first. The phase therefore runs at 1500
counts/s by default, printed in the plan rather than applied silently, and
`--together-speed` overrides it.

```
-- axis_1 (id 1) alone --
  -> min  523    reached 525  (error +2, peak load 1000)
  -> max  3142   reached 3141 (error -1, peak load 1000)
```

Torque is scoped to the axes actually moving, so a finished axis is not left
energised. Load is sampled while moving rather than only after arrival, so a
tight spot partway along shows up -- though during hard acceleration it sits near
the servo's torque limit by design, so a **stall** is the signal to watch, not
the load number. An axis that stalls short usually means the limit is optimistic:
by hand you could push past a tight spot the servo cannot.

### Speed and what limits it

The axes saturate at the motor: **2650 counts/s (233 deg/s)** on a 5 V supply,
3150 (277 deg/s) on 7.4 V. Below saturation the commanded speed is tracked
closely -- 1000 -> 1000, 2000 -> 2000.

`move_to` sends **one** goal write per move and lets the servo run its own
trapezoidal profile; verified by counting writes over a 2000-count move. Nothing
interpolates on top by default. `--lead` switches to interpolation, which bounds
the position error -- and so the torque driven into an obstruction -- but caps
speed near `sqrt(acceleration * lead)` however high `speed` is set.

A full three-axis sweep takes about 9 seconds.

### 7.4 V buys almost nothing here

Measured end to end, a full sweep takes **9.4s at 5 V against 9.0s at 7.4 V** --
4%, despite 7.4 V raising top speed by 19%. Sweeping is nine short legs, and
short moves are governed by acceleration, not top speed. Top speed only pays on
the long ones.

And acceleration is *worse* at 7.4 V, because there it is capped by regenerative
braking rather than by the motor. A decelerating motor pushes the rail up, and
only 0.6 V separates a 7.4 V supply from the servos' 8.0 V ceiling. Measured peak
voltage over a 1500-count move:

| acc | time | peak V | headroom |
| ---: | ---: | ---: | ---: |
| 100 | 0.77s | 7.4 | 0.6 |
| 120 | 0.72s | 7.6 | 0.4 |
| 140 | 0.69s | 8.0 | 0.0 |
| 160 | 0.66s | 8.2 | over |
| 180 | — | — | voltage fault, torque dropped |

At 5 V the ceiling is 2.6 V away, none of this applies, and acceleration can run
to 254 if wanted. **When a servo faults it releases torque and the axis falls
wherever gravity takes it** -- the servo's own protection, not something software
can get ahead of.

5 V is the better operating point for this rig. Most of the speed that was
recovered came from restoring persistent settings left by a previous
application, not from voltage.

### Settings inherited from another application

A reused servo carries its previous application's configuration in EPROM, and
some of it can cost performance here. `ballbal factory-reset` restores the
expected settings, printing each register it changes.

A previous load-limited role may store reduced values such as
`Max_Torque_Limit` 500, `Protection_Current` 250 and `Overload_Torque` 25, all
roughly half of the values used here. On this rig one reused servo retained that
configuration, and its 50% torque cap made it visibly the slowest of the three:
1250 counts/s peak against 1450-1500 for the others. Restored, all three match.

Previous mechanisms may also lower `P_Coefficient` from Feetech's default 32 to
16 to avoid shakiness under a long lever or compliant load. Measured here over a
2000-count move that costs 15% -- 1.19s at P=16 against 1.01s at P=32 -- and
settles 2 counts short. P=48 and P=64 buy under 2% more and only risk oscillation
under load, so the factory 32 is the right value for this platform.

### Torque is implicit

Writing a goal position sets `TORQUE_ENABLE` on these servos by itself: the
register reads 0 before a goal write and 1 after. `ServoBus` therefore treats
any servo it has sent a goal to as energised, so closing the bus releases it.
Tracking only explicit torque-enable writes would leave a servo holding torque
after a move that never called one.

Under hard acceleration the 5 V supply sags to about 4.5-4.9 V at the servo,
against their 4.0 V floor. There is headroom, but it is not large.

## Adjusting the shared motion window

If Home or Max should move relative to every calibrated minimum, edit
`home_offset` or `max_offset` in the profile's `rig.toml`, then run a slow
`ballbal sweep`. Per-axis trim and neutral writes are intentionally disabled for
derived profiles: otherwise Home and Max would stop being common geometry.

## Rotating the platform

```
ballbal rotate
```

Tilts the platform in a travelling wave: one side down while the opposite side
is up, with the high point going round. Each axis follows the same sinusoid a
third of a turn apart, and because IDs are assigned clockwise, stepping the
phase in rig order sends the high point clockwise too. `--counter-clockwise`
reverses it.

```
  axis_1: 981..2735 around 1858 (+/-877 counts, 77.1 deg), phase 0 deg
  axis_2: 502..2224 around 1363 (+/-861 counts, 75.7 deg), phase 120 deg
  axis_3: 518..2292 around 1405 (+/-887 counts, 78.0 deg), phase 240 deg
```

### Why the servo angle is not a sine

A horn converts servo angle into leg height as `h = L·sin(α)`, so driving α as a
sine gives a height of `sin(sin(...))`. Two things go wrong. The tilt no longer
sweeps evenly, and three legs distorted that way no longer sum to a constant --
so the platform **rises and falls** as the tilt goes round instead of simply
leaning.

Inverting the horn geometry fixes both:

```
α(θ) = arcsin( sin(α_max) · sin(θ) )
```

Measured over a revolution at this rig's 77° amplitude, summed leg height swings
by **0.565 horn lengths** on a raw sine and **0.016** linearised -- the residual
being rounding to whole counts. The extremes are identical either way (both
reach 981 and 2735 on axis 1); what changes is the path between them, which is
where the wobble lives. At small amplitudes the two nearly coincide.

`--raw-sine` restores the old behaviour for comparison.

Amplitude comes off the **tighter** side of each axis's travel around its own
midpoint, so no axis ever leans on a limit. An axis clamped at its stop quietly
stops following the sinusoid, and the tilt goes lopsided in a way the positions
alone do not obviously explain. `--amplitude` scales it down as a percentage.

Peak speed follows from amplitude and period alone, so it is checked before
anything moves rather than found out when three axes accelerating together sag
the supply:

```
error: this rotation needs 2787 counts/s, above the 1500 counts/s that three
axes moving together can draw without sagging the supply. Use --period 3.7 or
longer, or reduce --amplitude.
```

The run eases into the starting pose first -- dropping straight onto the
sinusoid from wherever the axes sit is a step change, and a step change is what
draws the current spike -- then returns to home at the end. Every update is
logged to `logs/rotate_<timestamp>.csv` in the same one-row-per-sample format as
jogging.

`--period` seconds per revolution, `--revolutions` how many, `--yes` to skip the
prompt. `--amplitude` defaults to 100, which does reach each axis's calibrated
min and max; a smaller value keeps the motion near mid-travel.

## Driving the servos by hand

```
ballbal jog
```

| key | |
| --- | --- |
| `q`/`a`, `w`/`s`, `e`/`d` | one axis at a time, in rig order |
| `-` / `+` | **every axis together** |
| `[` / `]` | finer / coarser step, live |
| `h` | home  ·  `p` observed range  ·  `?` help  ·  `x` quit |
| **space** | emergency stop, torque off |

Goals are clamped twice over. Each axis's own travel bounds it, so holding a key
walks up to the limit and stops there rather than pushing past -- and `ServoBus`
refuses an out-of-range goal outright, so no path can exceed it, including a
script driving the bus directly. Goals are also held to `max_lag` counts ahead of
the measured position: keyboard auto-repeat fires far faster than a servo
travels, and an unclamped target accumulates into one large step the moment the
key is released.

Every session writes `logs/jog_<timestamp>.csv`, one row per sample with a
column pair per axis:

```
timestamp, elapsed_s, event, key, step, axis_1_target, axis_1_measured, ...
```

Rows are written on every commanded move and once a second while idle, so the
log shows where the rig sat, not only where it was pushed. A row per servo per
event -- the earlier layout -- turned "where were all three axes at time T" into
a join instead of a lookup.

## One process at a time

Opening the bus takes an exclusive lock on the serial device. A second command
is refused rather than queued:

```
error: /dev/serial/by-id/usb-... is already in use by another ballbal process.
```

Nothing in the OS prevents two processes opening the same tty, and the SDK does
not lock either. Two of them on a half-duplex bus interleave packets so replies
land in the wrong reader -- and a second process can command motion while the
first sits at an interactive prompt. That is not hypothetical: a `recover` run
had its servos moved out from under it by a concurrent `home`, and reported
positions from before the move.

Interactive sessions (`jog`, `calibrate`, `setup`) hold the lock for as long as
they are open.

## What preflight blocks

`ballbal check` and every moving command run the same checks first, and they
block on what is actually unsafe: supply voltage outside the servo's own window,
fault flags set, the wrong control mode, or overheating.

Sitting **outside the configured travel does not block**. It prints a note and
continues. Every goal is clamped into travel before it is sent, so the only move
available from out there is one heading back in -- which is exactly what an axis
nudged by hand, or dropped when a servo faulted, needs. Blocking it meant
refusing the fix:

```
note: axis_2 (id 2) is at 489, 18 counts outside its travel 507..3084.
      The first move will bring it back in.
```

## Positioning accuracy

An STS3215 does not stop exactly on its commanded position. Its deadband
registers are 1 count each, and stiction in the linkage absorbs the rest, so the
residual error depends on the direction the axis approached from. Measured on
this rig: axis_b settles within +/-3 counts, axis_a within +/-6.

`tolerance` in `rig.toml` is what counts as "arrived" (default 8 counts,
0.70 deg). Set below the worst axis's residual and moves will time out waiting
for precision the servo cannot deliver. `ballbal home` reports the residual so
you can tune it:

```
axis_a: 2112 -> 2108 (home 2100, error +8; moved -4)
axis_b: 3132 -> 3132 (home 3130, error +2; already within 8, not commanded)
```

Steady-state error at this scale is normal and is what an outer control loop
compensates for. It is not something to chase with a tighter tolerance.

## Interactive jogging

`ballbal jog` binds `q/a`, `w/s`, `e/d`, … to the servos in rig order.

- `h` home, `p` show observed travel, `?` help, `x` quit
- **space** = emergency stop (torque off immediately)

Keyboard auto-repeat fires far faster than a servo can travel, so the commanded
position is clamped to at most `max_lag` counts ahead of the measured one.
Without that clamp the target accumulates while the servo lags, and releasing
the key leaves a large queued step — which is exactly what makes a stalled or
underpowered servo look like a runaway.

## Layout

| Module | Responsibility |
| --- | --- |
| `src/ballbal/hardware/` | registers, serial adapter, bus and vendored SDK |
| `src/ballbal/setup/` | first-time procedure, calibration and rest reference |
| `src/ballbal/control/` | preflight, safe motion, jogging, tuning and balancing |
| `src/ballbal/vision/` | camera calibration and ball tracking |
| `src/ballbal/config.py` | profile loading, merge and validation |
| `src/ballbal/cli.py` | argument parsing and command dispatch |

### Why the SDK is vendored

Two different libraries both import as `scservo_sdk`:

- the **official** [FTServo_Python](https://github.com/ftservo/FTServo_Python),
  which binds the port handler at construction and has the correct
  `setPacketTimeout`;
- **`feetech-servo-sdk`** on PyPI, which passes the port into every packet call
  and carries a packet-timeout bug that requires a downstream workaround.

The official one ships no `setup.py`, so it cannot be pip-installed from git.
Vendoring it under `ballbal.hardware.vendor` pins the version and prevents the
two implementations from shadowing each other if another robotics package
installs the PyPI SDK in the same environment. `sys.path` is never modified.

### Reliability notes

The SDK's `clearPort()` calls pyserial's `flush()`, which drains the *write*
buffer and leaves stale input bytes in place — so one late reply desyncs every
packet after it. `ServoBus` rebinds it to `reset_input_buffer()`, and retries
each transaction, so a single dropped packet is recoverable instead of fatal.

Writes are confirmed by read-back when the acknowledgement is lost: at this
layer a lost *status* packet is indistinguishable from a lost *instruction*
packet, and the servo may have applied the write either way. That matters most
for torque enable, where retrying blindly can leave torque on while the caller
believes the write failed.

Measured on this rig: sync read of 3 servos ≈ 1900 Hz, sync write ≈ 3100 Hz.
