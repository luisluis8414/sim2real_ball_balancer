"""Keep the ball on the middle of the plate.

The plant is a double integrator: plate tilt sets the ball's *acceleration*, not
its velocity. That has -180 degrees of phase at every frequency, so proportional
control alone cannot stabilise it at any gain -- it oscillates, and adding
integral action makes it worse. The derivative term is not tuning here, it is
the thing that closes the loop at all.

Two properties of this rig decide the design.

The camera is capped at 30 fps in firmware, so the loop runs at 32 ms per step
while the servo bus answers in 0.52 ms. Everything is paced by the camera, and
the useful crossover is somewhere near 1-1.5 Hz: at 30 Hz sampling that is 20x
oversampled, which is comfortable, but the total dead time -- exposure, readout,
transport, decode -- eats phase margin directly, so the loop is written to add
none of its own.

And the legs constrain each other. Driving all three to their individually
measured minima binds the mechanism: measured here, two axes stalled 9 and 14
counts short of the goal and sprang back 33 and 31 counts the moment torque was
released. Every command this module issues is therefore *differential* -- the
three leg offsets sum to zero, so the platform leans without ever translating
towards a pose where the linkage locks.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Sequence

from ..hardware import registers as reg
from ..config import ServoConfig

logger = logging.getLogger(__name__)

LEG_SPACING = 2.0 * math.pi / 3.0
"""Three legs, evenly spaced. IDs run clockwise, so leg order does too."""

DEFAULT_MAX_TILT_PCT = 25.0
"""How much of each axis's usable travel a full-scale tilt may use.

Deliberately small. The plate only needs a few degrees to move the ball -- at 5
degrees a ping-pong ball accelerates at 0.51 m/s^2, which crosses a 200 mm plate
in well under a second -- and a controller that can command the full range will
use it during the first overshoot and throw the ball off.
"""


@dataclass(frozen=True)
class Kinematics:
    """Turns a tilt into three servo positions.

    ``bearing`` is where axis 1 sits, as an angle in the *image*, measured from
    the image's +x axis towards +y (which points down, as it does in every frame
    OpenCV hands over). Getting it wrong by more than 90 degrees turns the
    correction into positive feedback -- the plate pushes the ball the way it was
    already going.

    It is asked for, in `ballbal cam-setup`, rather than measured. Inferring it
    from how a ball accelerates sounds more rigorous and is not: it puts linkage
    backlash, stiction and a 30 Hz velocity estimate between the question and
    the answer, and on this rig it returned 307.9 degrees where the truth was
    about 180. `balance --check-tilt` confirms it the way the quantity actually
    presents itself -- the plate leans, and a person says whether the named edge
    is the one that went down.
    """

    servos: tuple[ServoConfig, ...]
    bearing: float = 0.0
    max_tilt_pct: float = DEFAULT_MAX_TILT_PCT
    clockwise: bool = True

    @property
    def centres(self) -> tuple[int, ...]:
        """The pose leaned about: each axis's ``home_position``.

        Mid-travel was the obvious choice and the wrong one. It is the middle in
        *counts*, and counts are not height: a horn converts them through
        ``h = L*sin(alpha)``, so where in its swing the horn sits decides how
        much height a count is worth. Sitting near the top of that sine gives a
        plate that is high, cannot usefully go higher, and leans less per count
        than it would nearer the middle.

        Reading the neutral from the rig file instead makes that adjustable, and
        makes it the same pose `ballbal home` parks at -- so what is seen when
        the platform is idle is what the controller leans about.
        """
        return tuple(servo.home_position for servo in self.servos)

    @property
    def reach(self) -> int:
        """Counts any leg may swing either way -- one figure for all three.

        Taken from the tighter half of the tightest axis. Tighter half, so a
        lean never rests on a limit: an axis held at its stop stops following
        the command, and the plate then leans differently from what was asked
        with nothing in the positions to say so.

        Shared, because that is what makes the command exactly differential.
        With a per-axis reach the three offsets only nearly cancel -- measured
        on this rig, up to 10 counts of leftover common mode, which is the plate
        creeping towards the pose where the legs bind instead of just leaning.
        """
        return min(
            round(
                min(centre - servo.min_position, servo.max_position - centre)
                * self.max_tilt_pct
                / 100.0
            )
            for servo, centre in zip(self.servos, self.centres)
        )

    @property
    def headroom(self) -> int:
        """Counts a leg could still swing if the tilt limit were lifted."""
        return min(
            min(centre - servo.min_position, servo.max_position - centre)
            for servo, centre in zip(self.servos, self.centres)
        )

    def leg_bearings(self) -> tuple[float, ...]:
        """Each leg's direction in image coordinates."""
        sign = 1.0 if self.clockwise else -1.0
        return tuple(
            self.bearing + sign * index * LEG_SPACING
            for index in range(len(self.servos))
        )

    def goals(self, tilt_x: float, tilt_y: float) -> dict[int, int]:
        """Servo positions for a tilt, given as a vector in image coordinates.

        The vector points *downhill*: (1, 0) drops the plate towards image +x,
        so a ball sitting there rolls further that way -- which is why the
        controller, seeing a ball at +x, asks for a lean towards -x. Its length
        is a fraction of full scale and is clipped at 1, so a controller that
        winds up asks for a big lean rather than an undefined one.

        ``bearing`` names where a leg physically *is*, not where the plate tips
        when it moves. Those are opposite, and keeping the stored angle on the
        thing a person can point at is what makes it checkable by eye.
        """
        magnitude = math.hypot(tilt_x, tilt_y)
        if magnitude > 1.0:
            tilt_x, tilt_y = tilt_x / magnitude, tilt_y / magnitude

        goals: dict[int, int] = {}
        reach = self.reach
        for servo, centre, bearing in zip(
            self.servos, self.centres, self.leg_bearings()
        ):
            # Project the tilt onto this leg, and negate. Larger counts raise
            # a leg -- the rig file parks the platform at the bottom of its
            # travel by homing every axis to its *lower* limit -- so the leg a
            # downhill vector points at is the one that has to come DOWN, and
            # the one opposite has to rise. Without the minus the plate leans
            # away from where it was asked to, which is not a lean in the wrong
            # place but positive feedback: the ball is accelerated the way it
            # was already going.
            #
            # The three projections still sum to zero, so the command stays
            # differential either way; only the direction flips.
            share = -(tilt_x * math.cos(bearing) + tilt_y * math.sin(bearing))
            goals[servo.id] = servo.clamp(centre + round(reach * share))
        return goals

    def level(self) -> dict[int, int]:
        """Every leg at its own mid-travel: the plate as flat as it gets."""
        return {
            servo.id: centre for servo, centre in zip(self.servos, self.centres)
        }


@dataclass
class PID:
    """One axis of the controller, in millimetres in and tilt fraction out.

    The derivative runs on the *measurement*, not on the error. They differ only
    when the setpoint moves, and there the error form produces a spike
    proportional to the step -- a kick that a plate full of rolling ball does not
    need.

    The integral is small but not optional, which is the opposite of what this
    said before. A PD loop on a double integrator leaves a standing error of
    ``disturbance / (G*kp)``, and with the gains the dead time permits that is
    not a rounding error: half a degree of plate tilt parks the ball 13.6 mm off
    centre and leaves it there. It also carries the loop past the mechanism's
    own deadband -- a 5 mm error asks for 4 counts of leg movement, and the axes
    settle to within 3 to 6, so without something that accumulates, small errors
    command motion the servos do not make.
    """

    kp: float
    ki: float
    kd: float
    integral_limit: float = 0.30
    """Most of the tilt the integral alone may ask for, as a fraction.

    A limit on what the integral *contributes*, not on the error it has piled
    up. Bounding the pile directly is the mistake this replaced: the accumulator
    holds millimetre-seconds, so at a 10 mm offset one 32 ms step adds 0.32 and
    a bound of 0.25 was already exceeded by the first sample. The integral sat
    pinned from then on, worth 0.0005 of tilt -- present in the sum, absent from
    the plate.
    """
    output_limit: float = 1.0
    derivative_smoothing: float = 0.35
    """Low-pass on the derivative, as the weight given to each new sample.

    At 30 Hz the raw difference of two positions is the noisiest term in the
    loop: 0.17 mm of centroid noise over a 32 ms step is 7.5 mm/s of velocity
    noise, which the derivative gain then multiplies. Smoothing costs a little
    phase, which is why it is not smaller, and buys a D term that does not chase
    pixel noise.
    """
    integral: float = field(default=0.0, init=False)
    _last_measurement: float | None = field(default=None, init=False)
    _derivative: float = field(default=0.0, init=False)

    def reset(self) -> None:
        self.integral = 0.0
        self._last_measurement = None
        self._derivative = 0.0

    def update(
        self, setpoint: float, measurement: float, dt: float, *,
        scale: float = 1.0,
    ) -> float:
        """``scale`` raises the loop's speed without changing its damping.

        Bandwidth goes as ``sqrt(G*kp)`` and damping as ``kd*sqrt(G/kp)``, so
        scaling kp by the square and kd by the first power moves the former and
        leaves the latter alone. The integral is deliberately left out: it is
        there to trim a plate that is slightly off level, and a trim that grows
        teeth the further out the ball goes is not a trim.
        """
        if dt <= 0:
            raise ValueError("dt must be positive")
        error = setpoint - measurement

        self.integral += error * dt
        if self.ki:
            bound = self.integral_limit / self.ki
            self.integral = max(-bound, min(bound, self.integral))

        if self._last_measurement is None:
            raw = 0.0
        else:
            # Negated because this is d(measurement)/dt standing in for
            # -d(error)/dt: the ball moving away has to oppose, not add.
            raw = -(measurement - self._last_measurement) / dt
        self._last_measurement = measurement
        alpha = self.derivative_smoothing
        self._derivative = alpha * raw + (1.0 - alpha) * self._derivative

        output = (
            self.kp * scale * scale * error
            + self.ki * self.integral
            + self.kd * scale * self._derivative
        )
        return max(-self.output_limit, min(self.output_limit, output))


def plan(
    servos: Sequence[ServoConfig],
    *,
    bearing_deg: float = 0.0,
    max_tilt_pct: float = DEFAULT_MAX_TILT_PCT,
    clockwise: bool = True,
) -> Kinematics:
    if len(servos) != 3:
        raise ValueError(f"balancing needs exactly 3 axes, got {len(servos)}")
    if not 0 < max_tilt_pct <= 100:
        raise ValueError("max tilt must be greater than 0 and at most 100 percent")
    return Kinematics(
        servos=tuple(servos),
        bearing=math.radians(bearing_deg),
        max_tilt_pct=max_tilt_pct,
        clockwise=clockwise,
    )


# -- the loop ---------------------------------------------------------------

DEFAULT_GAINS = (0.010, 0.0008, 0.0075)
"""kp, ki, kd, in tilt fraction per millimetre (and per mm*s, mm/s).

A starting point, not a tuned answer. Sized so that a ball 60 mm off centre and
still asks for 0.6 of full lean, and so the derivative dominates near the
setpoint -- which it must, since a double integrator has no phase margin of its
own. Expect to move kd first: too little and the ball oscillates across the
plate, too much and it jitters in place chasing measurement noise.
"""

QUIET_COUNTS = 3
"""Smallest change in a leg's goal worth sending.

The plate twitches at rest because the loop keeps re-aiming it at camera noise.
Measured here: 0.17 mm of centroid noise becomes 7.5 mm/s of raw velocity, and
after the derivative filter that is still about 3 counts of commanded leg
movement -- the same size as the deadband the axes settle within. Thirty
re-aims a second inside their own resolution is motion with no information in
it.

Smoothing the derivative harder would remove it and cost too much: at a filter
slow enough to matter, 38 degrees of the 44 degrees of phase margin goes with
it. Declining to send the command costs nothing instead. Real ball motion is
nowhere near this small -- a ball at 50 mm/s already asks for 43 counts -- so
the threshold only ever suppresses noise.
"""

LOST_BALL_GRACE = 0.4
"""Seconds the ball may go missing before the plate is returned to level.

Long enough to ride out a frame or two where the blob failed the roundness test,
short enough that a ball which has actually left the plate does not leave the
platform leaning.
"""


@dataclass
class Sample:
    """One pass of the loop, for the log."""

    t: float
    found: bool
    x_mm: float
    y_mm: float
    tilt_x: float
    tilt_y: float
    goals: dict[int, int]
    dt: float


def gain_scale(
    distance: float, radius: float, aggression: float, shape: float = 1.0
) -> float:
    """How much harder to push, given how far out the ball is.

    A single loop cannot be both things at once. Near the middle it has to be
    gentle: the mechanism's own deadband -- the axes settle to within 3 to 6
    counts -- turns a hard loop into a limit cycle, and simulated at 4 counts
    the residual wander went from 0.4 mm to 12.6 mm when kp was merely
    quadrupled. Far out none of that applies, because the ball is not being
    settled there, it is being caught, and a loop sized for settling takes
    twice as long to catch it.

    So the gain grows with distance. Simulated from the rim: recovery went from
    1.15 s to 0.54 s at an aggression of 1, with the residual unchanged. Past
    about 4 it stops helping and starts oscillating.

    ``shape`` bends the curve between the two ends without moving either. At 1
    the gain rises in step with the distance; above it the rise is held back
    near the middle and catches up towards the rim, so the loop goes quiet
    sooner as the ball arrives. The rim is untouched either way -- at the full
    radius the factor is ``1 + aggression`` whatever the shape.

    It matters for the orbit a ball settles into near the middle, which is the
    one thing a linear profile handles worst: simulated from 25 mm with a
    tangential push, the wobble took 2.8 s to die at shape 1 and 1.8 s at 3,
    with the same residual either way.
    """
    if aggression <= 0 or radius <= 0:
        return 1.0
    return 1.0 + aggression * (min(distance / radius, 1.0) ** shape)


def _tilt_from_controllers(
    controllers: tuple["PID", "PID"],
    target: tuple[float, float],
    position: tuple[float, float],
    dt: float,
    scale: float = 1.0,
) -> tuple[float, float]:
    """Both axes, into a downhill tilt vector.

    The sign is the whole game, and it is not the one instinct suggests. A ball
    60 mm along +x gives an error of -60, so the output is negative and already
    points at -x -- which is exactly where the plate has to drop to send the ball
    back. The controller output *is* the downhill direction; negating it "so the
    plate pushes the ball back" turns the loop into positive feedback, and that
    looks like a badly tuned gain right up until the ball leaves the plate.
    """
    # One scale for both axes, from the radial distance. Scaling each axis by
    # its own error would make the correction point somewhere other than back
    # at the setpoint -- a ball out at 45 degrees would be pushed along an axis
    # rather than towards the middle.
    out_x = controllers[0].update(target[0], position[0], dt, scale=scale)
    out_y = controllers[1].update(target[1], position[1], dt, scale=scale)
    return out_x, out_y


def balance(
    rig,  # noqa: ANN001 - Rig, imported lazily to keep cv2 out of the CLI path
    servos: Sequence[ServoConfig],
    *,
    calibration,  # noqa: ANN001 - CameraCalibration
    device: str,
    size: tuple[int, int],
    gains: tuple[float, float, float] = DEFAULT_GAINS,
    bearing_deg: float = 0.0,
    max_tilt_pct: float = DEFAULT_MAX_TILT_PCT,
    target_mm: tuple[float, float] = (0.0, 0.0),
    path: "Path | None" = None,
    aggression: float = 0.0,
    shape: float = 1.0,
    quiet_counts: int = QUIET_COUNTS,
    seconds: float | None = None,
    acceleration: int | None = None,
    square: bool = True,
    offset: tuple[int, int] = (0, 0),
    side: int | None = None,
    dry_run: bool = True,
    show: bool = True,
    log_path=None,  # noqa: ANN001 - Path
    mirror=None,  # noqa: ANN001 - simulation.mirror.Mirror
) -> int:
    """Close the loop: see the ball, lean the plate, repeat.

    ``dry_run`` is the default on purpose. It runs the whole chain -- camera,
    detection, controller, kinematics -- and prints the positions it would send,
    with torque never enabled. That is the only way to check the sign of the
    correction and the size of the gains without a mechanism that can throw a
    ball across the room while it is being checked.

    ``mirror`` copies every frame's ball and the platform's measured pose into
    Isaac Sim. It costs the loop one sync read of the positions, about a
    millisecond on this bus; the simulation itself is fed from another thread.
    """
    import csv
    import time

    import cv2

    from ..hardware.bus import BusError, ServoError
    from ..vision import find_ball_by_colour, lock_camera, open_camera

    if not calibration.has_colour:
        raise RuntimeError(
            "the camera calibration has no ball colour; run `ballbal cam-setup` "
            "or `ballbal cam-calibrate` first"
        )

    kinematics = plan(
        servos, bearing_deg=bearing_deg, max_tilt_pct=max_tilt_pct,
        clockwise=True,
    )
    platform_hint = calibration.platform_radius_px * calibration.mm_per_px
    controllers = (
        PID(kp=gains[0], ki=gains[1], kd=gains[2]),
        PID(kp=gains[0], ki=gains[1], kd=gains[2]),
    )
    print(
        f"  gains kp={gains[0]:.4f} ki={gains[1]:.5f} kd={gains[2]:.4f}\n"
        f"  bearing {bearing_deg:.1f} deg\n"
        f"  tilt limited to {max_tilt_pct:.0f}% -> {kinematics.reach} counts per "
        f"leg, of {kinematics.headroom} available\n"
        f"  neutral pose {kinematics.level()} (from home_position)"
        + (
            f"\n  aggression {aggression:.1f} shape {shape:.1f}: gain rises to "
            f"{1 + aggression:.1f}x at the rim, "
            f"{gain_scale(platform_hint / 2, platform_hint, aggression, shape):.2f}x "
            "at half way"
            if aggression > 0 else ""
        )
        + (
            f"\n  holding still below {quiet_counts} counts of change"
            if quiet_counts > 0 else ""
        )
        + (
            f"\n  acceleration {acceleration} (rig default overridden)"
            if acceleration is not None
            else ""
        )
    )
    # Moving the neutral away from mid-travel silently rescales the plant: the
    # swing is capped by the tighter side, so the same tilt fraction becomes
    # fewer counts and every gain derived from an earlier measurement is now
    # too small by that ratio. It is invisible in the numbers a person types,
    # so it gets said out loud.
    midpoints = [
        (servo.min_position + servo.max_position) // 2 for servo in servos
    ]
    widest = min(
        min(mid - servo.min_position, servo.max_position - mid)
        for servo, mid in zip(servos, midpoints)
    )
    if kinematics.headroom < 0.9 * widest:
        shortfall = widest / max(kinematics.headroom, 1)
        print(
            f"  NOTE: home sits off mid-travel, so the swing is "
            f"{kinematics.headroom} counts where mid-travel would give "
            f"{widest}.\n"
            f"        The plate leans {shortfall:.1f}x less per unit of tilt "
            "than it would\n"
            f"        there, so gains measured at mid-travel are "
            f"{shortfall:.1f}x too small here.\n"
            "        Either `ballbal neutral --mid`, or re-run `ballbal tune` "
            "on a fresh log."
        )
    if path is not None:
        bandwidth = 0.45 / 0.136 / (2 * math.pi)
        print(
            f"  path: {type(path).__name__.lower()}, one lap every "
            f"{path.period:.1f} s ({path.frequency:.2f} Hz), target moving at "
            f"{path.speed:.0f} mm/s"
        )
        for note in check_path(path, bandwidth, platform_hint):
            print(f"  NOTE: {note}")
    if dry_run:
        print("  DRY RUN -- torque stays off, nothing moves")
    if mirror is not None:
        print("  mirroring ball and platform into Isaac Sim")

    lock_camera(device)
    capture = open_camera(device, size, square=square, offset=offset, side=side)
    centre = calibration.centre
    roi_radius = int(calibration.platform_radius_px * 0.97)
    mask = None
    writer = None
    log_file = None
    samples = 0
    lost = 0
    last_goals: dict[int, int] = {}
    applied_tilt = (0.0, 0.0)

    try:
        ok, frame = capture.read()
        if not ok:
            raise RuntimeError(f"{device} opened but returned no frame")
        height, width = frame.shape[:2]
        # The same guard tracking has. Without it a calibration from a wider
        # frame puts the platform centre off to one side, and every reported
        # position is wrong by that much with nothing to show for it.
        calibration.check_size(width, height)
        import numpy as np

        mask = np.zeros((height, width), np.uint8)
        cv2.circle(mask, centre, roi_radius, 255, -1)
        platform_mm = calibration.platform_radius_px * calibration.mm_per_px

        if log_path is not None:
            log_file = open(log_path, "w", newline="")
            writer = csv.writer(log_file)
            writer.writerow(
                ["t_s", "dt_s", "found", "x_mm", "y_mm", "tilt_x", "tilt_y",
                 "goal_1", "goal_2", "goal_3", "sent",
                 "target_x", "target_y"]
            )

        for _ in range(15):  # the 268 ms opening transient
            capture.read()

        started = time.perf_counter()
        previous = started
        last_seen = started

        with rig.bus.torque([s.id for s in servos], hold=False) if not dry_run \
                else _NullGuard():
            if not dry_run:
                rig.goto(kinematics.level(), acceleration=acceleration,
                         settle=0.6)
            while True:
                ok, frame = capture.read()
                if not ok:
                    print("camera stopped returning frames")
                    return 1
                now = time.perf_counter()
                dt = now - previous
                previous = now
                elapsed = now - started
                if dt <= 0:
                    continue

                detection = find_ball_by_colour(
                    frame,
                    hsv_lo=calibration.hsv_lo, hsv_hi=calibration.hsv_hi,
                    mask=mask, min_radius=3.0, max_radius=width * 0.25,
                )
                if detection is None:
                    lost += 1
                    if now - last_seen > LOST_BALL_GRACE:
                        goals = kinematics.level()
                        for controller in controllers:
                            controller.reset()
                        tilt = (0.0, 0.0)
                        x_mm = y_mm = float("nan")
                    else:
                        goals = None
                        tilt = (0.0, 0.0)
                        x_mm = y_mm = float("nan")
                else:
                    last_seen = now
                    samples += 1
                    if path is not None:
                        target_mm = path.at(elapsed)
                    x_mm = (detection.x - centre[0]) * calibration.mm_per_px
                    y_mm = (detection.y - centre[1]) * calibration.mm_per_px
                    distance = math.hypot(x_mm - target_mm[0], y_mm - target_mm[1])
                    scale = gain_scale(distance, platform_mm, aggression, shape)
                    tilt = _tilt_from_controllers(
                        controllers, target_mm, (x_mm, y_mm), dt, scale
                    )
                    goals = kinematics.goals(*tilt)

                sent = goals is not None
                if sent and quiet_counts > 0 and last_goals:
                    if all(
                        abs(goals[key] - last_goals[key]) < quiet_counts
                        for key in goals
                    ):
                        sent = False  # nothing worth moving for
                if sent and not dry_run:
                    rig.goto(goals, acceleration=acceleration)
                if sent:
                    last_goals = dict(goals)
                    applied_tilt = tilt

                if mirror is not None:
                    # The measured pose, not the goal: what the plate really
                    # holds, servo lag and all, and in a dry run the plate that
                    # is not moving. A failed read skips one frame's pose
                    # rather than stopping a loop that is holding a ball.
                    try:
                        pose = rig.positions(servos)
                    except (BusError, ServoError):
                        pose = None
                    seen = detection is not None
                    mirror.send(x_mm if seen else None, y_mm if seen else None, pose)

                # Every frame gets a row, whether or not a command went out, and
                # the tilt recorded is the one the plate is actually holding.
                # Logging only the frames that produced a command was the first
                # version and it corrupts the very thing the log is read for:
                # `tune` takes the loop's timestep from these timestamps, so
                # suppressed frames inflated it and with it the dead time.
                if writer is not None:
                    ordered = (
                        [last_goals[s.id] for s in servos] if last_goals
                        else ["", "", ""]
                    )
                    writer.writerow(
                        [f"{elapsed:.4f}", f"{dt:.4f}", int(detection is not None),
                         "" if detection is None else f"{x_mm:.2f}",
                         "" if detection is None else f"{y_mm:.2f}",
                         f"{applied_tilt[0]:.4f}", f"{applied_tilt[1]:.4f}",
                         *ordered, int(sent),
                         f"{target_mm[0]:.2f}", f"{target_mm[1]:.2f}"]
                    )

                if show:
                    marker = None
                    if path is not None and calibration.mm_per_px:
                        marker = (
                            int(centre[0] + target_mm[0] / calibration.mm_per_px),
                            int(centre[1] + target_mm[1] / calibration.mm_per_px),
                        )
                    _draw_balance(
                        frame, detection, centre, roi_radius, tilt, goals,
                        x_mm, y_mm, calibration.mm_per_px, dry_run, marker,
                    )
                    cv2.imshow("ballbal -- balance", frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        break
                if seconds is not None and elapsed >= seconds:
                    break
    finally:
        capture.release()
        if show:
            cv2.destroyAllWindows()
        if log_file is not None:
            log_file.close()
        if not dry_run:
            # The torque guard already does this; saying so again is cheap and
            # this is the one place where failing to would drop the platform.
            rig.bus.set_torque([s.id for s in servos], False)

    print(f"\n  {samples} frames with a ball, {lost} without")
    if mirror is not None:
        print(f"  {mirror.sent} updates mirrored into Isaac Sim ({mirror.rate:.1f}/s)")
    if log_path is not None:
        print(f"  log written to {log_path}")
    return 0


class _NullGuard:
    """Stands in for the torque guard when nothing is allowed to move."""

    def __enter__(self):  # noqa: ANN204
        return self

    def __exit__(self, *_exc) -> bool:  # noqa: ANN002
        return False


def _draw_balance(
    frame, detection, centre, roi_radius, tilt, goals, x_mm, y_mm,
    mm_per_px, dry_run, target_px=None,
) -> None:  # noqa: ANN001
    """Show the ball, the target, and which way the plate is being leaned."""
    import cv2

    cv2.circle(frame, centre, roi_radius, (200, 200, 60), 1)
    cv2.drawMarker(frame, centre, (0, 200, 255), cv2.MARKER_CROSS, 18, 1)
    if target_px is not None:
        # The moving setpoint, so it is obvious what the ball is chasing.
        cv2.drawMarker(frame, target_px, (255, 255, 0), cv2.MARKER_TILTED_CROSS,
                       20, 2)
    if detection is not None:
        point = (int(detection.x), int(detection.y))
        radius = int(detection.radius)
        cv2.rectangle(
            frame, (point[0] - radius, point[1] - radius),
            (point[0] + radius, point[1] + radius), (0, 255, 0), 2,
        )
        cv2.drawMarker(frame, point, (0, 0, 255), cv2.MARKER_CROSS, 14, 2)
    # The tilt arrow points downhill -- the way the ball is being invited to go.
    tip = (
        int(centre[0] + tilt[0] * roi_radius),
        int(centre[1] + tilt[1] * roi_radius),
    )
    cv2.arrowedLine(frame, centre, tip, (255, 120, 0), 2, tipLength=0.2)

    lines = ["DRY RUN -- nothing moves" if dry_run else "LIVE"]
    if detection is not None:
        lines.append(f"ball  x{x_mm:+7.1f}  y{y_mm:+7.1f} mm")
    else:
        lines.append("ball  not found")
    lines.append(f"tilt  x{tilt[0]:+6.2f}  y{tilt[1]:+6.2f}")
    if goals:
        lines.append("goals " + " ".join(f"{v}" for v in goals.values()))

    height = 12 + 18 * len(lines)
    strip = frame[0:height, 0 : 8 + 22 * 9].copy()
    frame[0:height, 0 : 8 + 22 * 9] = cv2.addWeighted(
        strip, 0.25, strip * 0, 0.75, 0
    )
    for index, line in enumerate(lines):
        cv2.putText(
            frame, line, (6, 20 + 18 * index), cv2.FONT_HERSHEY_SIMPLEX,
            0.5, (255, 255, 255), 1, cv2.LINE_AA,
        )


def check_tilt(
    rig,  # noqa: ANN001
    servos: Sequence[ServoConfig],
    *,
    bearing_deg: float,
    amplitude: float = 0.7,
    max_tilt_pct: float = DEFAULT_MAX_TILT_PCT,
) -> int:
    """Lean the plate four ways and name each one, so a person can confirm it.

    This is the check the ball-rolling experiment was pretending to be. The
    bearing is one number and the thing it controls is one visible fact -- which
    side of the plate goes down -- so the reliable test is to state the
    prediction and let someone look. No stiction, no velocity estimate, no
    threshold: the plate either drops towards the named edge or it does not.
    """
    import time

    from ..prompt import confirm

    kinematics = plan(servos, bearing_deg=bearing_deg, max_tilt_pct=max_tilt_pct)
    named = (
        (0.0, "RIGHT of the image"),
        (90.0, "BOTTOM of the image"),
        (180.0, "LEFT of the image"),
        (270.0, "TOP of the image"),
    )
    print(
        f"\n  Checking the stored bearing of {bearing_deg:.1f} deg.\n"
        "  The plate leans four ways. Each time, the side named is the one that\n"
        "  should end up LOWEST -- the side a ball would roll towards.\n"
        "  Watch the plate, not the screen."
    )
    if not confirm("Ready? The platform will move"):
        return 1

    wrong = 0
    with rig.bus.torque([s.id for s in servos], hold=False):
        rig.goto(kinematics.level(), settle=0.8)
        for degrees, where in named:
            radians = math.radians(degrees)
            rig.goto(
                kinematics.goals(
                    amplitude * math.cos(radians), amplitude * math.sin(radians)
                ),
                settle=0.2,
            )
            print(f"\n  Leaning down towards the {where}.")
            time.sleep(0.8)
            if not confirm("    Is that the side that went down?"):
                wrong += 1
            rig.goto(kinematics.level(), settle=0.5)
        rig.goto(kinematics.level(), settle=0.5)

    if not wrong:
        print("\n  All four match. The bearing is right; balancing can use it.")
        return 0
    print(
        f"\n  {wrong} of 4 were wrong. The bearing is off. If the plate leaned\n"
        "  the exact opposite way each time, add 180 to it; if it was a quarter\n"
        "  turn out, add or subtract 90. Re-run `ballbal cam-setup` and click\n"
        "  the direction of axis_1 more carefully."
    )
    return 1


# -- moving setpoints -------------------------------------------------------


@dataclass(frozen=True)
class Path:
    """Where the ball is meant to be, as a function of time.

    A moving target rather than a fixed one, which the loop already handles: the
    derivative runs on the measurement rather than on the error, so a setpoint
    that moves does not produce the derivative kick that the error form would.
    That choice was made for a different reason and pays off here.
    """

    period: float

    def at(self, elapsed: float) -> tuple[float, float]:
        raise NotImplementedError

    @property
    def speed(self) -> float:
        """Millimetres per second the target itself travels."""
        raise NotImplementedError

    @property
    def frequency(self) -> float:
        """How fast the loop is being asked to work, in Hz."""
        return 1.0 / self.period if self.period > 0 else float("inf")


@dataclass(frozen=True)
class Circle(Path):
    """Round and round, at constant speed.

    The kindest path to ask for: the target's speed never changes and its
    direction turns smoothly, so there is no moment the loop has to react to a
    step. A ball can follow it well below the bandwidth that a square needs.
    """

    radius: float = 35.0

    def at(self, elapsed: float) -> tuple[float, float]:
        angle = 2.0 * math.pi * elapsed / self.period
        return (self.radius * math.cos(angle), self.radius * math.sin(angle))

    @property
    def speed(self) -> float:
        return 2.0 * math.pi * self.radius / self.period


@dataclass(frozen=True)
class Square(Path):
    """Round the outline of a square, at constant speed.

    Harder than it looks, and worth saying why. A corner is a step change in the
    target's direction, and a step contains every frequency -- including all the
    ones above the loop's bandwidth, which it cannot follow by definition. The
    ball will round every corner off. That is not a tuning fault; a corner is
    simply not a shape a bandwidth-limited system can trace.
    """

    size: float = 35.0
    """Half the side length: the square runs from -size to +size on each axis."""

    def at(self, elapsed: float) -> tuple[float, float]:
        # Distance travelled around a perimeter of eight half-sides.
        fraction = (elapsed % self.period) / self.period
        leg, along = divmod(fraction * 4.0, 1.0)
        span = self.size * (2.0 * along - 1.0)
        return {
            0: (span, -self.size),
            1: (self.size, span),
            2: (-span, self.size),
            3: (-self.size, -span),
        }[int(leg)]

    @property
    def speed(self) -> float:
        return 8.0 * self.size / self.period


def check_path(path: Path, bandwidth_hz: float, platform_mm: float) -> list[str]:
    """Complaints about a path the rig cannot actually trace.

    Asking for a path the loop cannot follow does not fail, it just tracks
    badly -- and from the outside that is indistinguishable from bad tuning. So
    the arithmetic is done before anything moves.
    """
    notes: list[str] = []
    reach = getattr(path, "radius", None) or getattr(path, "size", 0.0)
    if reach > 0.8 * platform_mm:
        notes.append(
            f"the path reaches {reach:.0f} mm on a platform of "
            f"{platform_mm:.0f} mm; keep it under {0.8 * platform_mm:.0f} mm or "
            "the ball will be asked to leave the plate"
        )
    if path.frequency > bandwidth_hz:
        notes.append(
            f"one lap takes {path.period:.1f} s, which is {path.frequency:.2f} Hz "
            f"against a bandwidth near {bandwidth_hz:.2f} Hz -- the ball will "
            "lag the target badly. Slow it down"
        )
    elif path.frequency > bandwidth_hz / 3:
        notes.append(
            f"{path.frequency:.2f} Hz is within a third of the bandwidth; expect "
            "the ball to trail the target noticeably"
        )
    if isinstance(path, Square):
        notes.append(
            "corners are step changes in direction and contain frequencies no "
            "bandwidth-limited loop can follow -- expect them rounded off"
        )
    return notes
