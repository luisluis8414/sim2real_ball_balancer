"""Rotate the platform's tilt: one side down, the opposite side up, going round.

Each axis follows a sinusoid of the same period, offset by a third of a turn
from its neighbours. At any instant one axis sits near its low limit while the
axis a third of the way round sits near its high one, and the high point travels
around the platform. Because IDs are assigned clockwise, stepping the phase
offset in rig order makes that high point travel clockwise too.

The peak speed a rotation demands is fixed by geometry -- amplitude and period
give it directly -- so it can be checked before anything moves rather than
discovered when three axes accelerating together brown out the supply.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

from ..hardware import registers as reg
from ..hardware.bus import BusError
from ..config import ServoConfig
from .interactive import PositionLog
from ..prompt import confirm
from ..paths import runtime_dir
from .rig import Rig
from .sweep import MAX_COMBINED_SPEED

UPDATE_HZ = 50.0
"""Goal updates per second. The servos interpolate between them."""


@dataclass(frozen=True)
class AxisMotion:
    """One axis's share of the rotation."""

    servo: ServoConfig
    centre: int
    amplitude: int
    phase: float
    linearise: bool = True

    @property
    def max_angle(self) -> float:
        """Half the axis's swing, in radians."""
        return math.radians(reg.counts_to_degrees(self.amplitude))

    def position(self, angle: float) -> int:
        """Commanded position at `angle` radians into the rotation.

        What has to move sinusoidally is the leg's *height*, not the servo's
        angle. A horn converts one into the other as ``h = L*sin(alpha)``, so
        driving alpha as a sine gives a height of ``sin(sin(...))`` -- and three
        such legs no longer sum to a constant, which makes the platform rise and
        fall as the tilt goes round instead of simply leaning. Inverting the
        horn geometry removes both distortions:

            alpha(theta) = arcsin( sin(alpha_max) * sin(theta) )

        At small amplitudes the two are nearly identical; at the 77 degrees this
        rig actually uses, the raw version is 9 degrees off at a third of a turn
        and heaves by over half a horn length per revolution.
        """
        wave = math.sin(angle - self.phase)
        if self.linearise:
            reach = min(self.max_angle, math.pi / 2)
            alpha = math.asin(math.sin(reach) * wave)
            offset = reg.degrees_to_counts(math.degrees(alpha))
        else:
            offset = round(self.amplitude * wave)
        return self.servo.clamp(self.centre + offset)

    @property
    def peak_counts_per_radian(self) -> float:
        """How fast the axis moves at its fastest, per radian of rotation.

        The linearised profile peaks as it crosses the centre, where its slope
        is ``sin(alpha_max)`` rather than ``alpha_max`` -- slower than the raw
        sine, so a rotation that passes the speed check with this figure passes
        with either.
        """
        if not self.linearise:
            return float(self.amplitude)
        reach = min(self.max_angle, math.pi / 2)
        return reg.COUNTS_PER_REV / (2 * math.pi) * math.sin(reach)


def plan_rotation(
    servos: Sequence[ServoConfig],
    amplitude_pct: float,
    clockwise: bool,
    linearise: bool = True,
) -> list[AxisMotion]:
    """Lay the axes out around one turn, sized to fit inside every travel.

    Amplitude is taken from the *tighter* side of each axis's travel around its
    own midpoint, so a rotation never leans on a limit -- an axis clamped at its
    stop stops following the sinusoid, and the tilt goes lopsided in a way the
    positions alone do not obviously explain.
    """
    if not 0 < amplitude_pct <= 100:
        raise ValueError("amplitude must be greater than 0 and at most 100 percent")

    count = len(servos)
    motions = []
    for index, servo in enumerate(servos):
        centre = (servo.min_position + servo.max_position) // 2
        reach = min(centre - servo.min_position, servo.max_position - centre)
        phase = 2 * math.pi * index / count
        motions.append(
            AxisMotion(
                servo=servo,
                centre=centre,
                amplitude=round(reach * amplitude_pct / 100),
                phase=phase if clockwise else -phase,
                linearise=linearise,
            )
        )
    return motions


def peak_speed(motions: Sequence[AxisMotion], period: float) -> int:
    """Fastest counts/s any axis reaches, from amplitude and period alone."""
    omega = 2 * math.pi / period
    return round(max(m.peak_counts_per_radian for m in motions) * omega)


def rotate(
    rig: Rig,
    servos: Sequence[ServoConfig],
    *,
    period: float = 6.0,
    revolutions: float = 3.0,
    amplitude_pct: float = 100.0,
    clockwise: bool = True,
    linearise: bool = True,
    speed_cap: int = MAX_COMBINED_SPEED,
    assume_yes: bool = False,
    log_path: Path | None = None,
) -> int:
    """Drive a rotating tilt for a number of revolutions."""
    if period <= 0:
        raise ValueError("period must be positive")
    if revolutions <= 0:
        raise ValueError("revolutions must be positive")

    motions = plan_rotation(servos, amplitude_pct, clockwise, linearise)
    demanded = peak_speed(motions, period)
    direction = "clockwise" if clockwise else "counter-clockwise"

    profile = (
        "linearised through the horn geometry, so the platform leans without "
        "rising"
        if linearise
        else "raw sine on the servo angle (the platform will heave)"
    )
    print(f"Rotation plan ({direction}, seen from above; {profile}):")
    for index, motion in enumerate(motions):
        servo = motion.servo
        low = motion.centre - motion.amplitude
        high = motion.centre + motion.amplitude
        print(
            f"  {servo.name}: {low}..{high} around {motion.centre} "
            f"(+/-{motion.amplitude} counts, "
            f"{reg.counts_to_degrees(motion.amplitude):.1f} deg), "
            f"phase {index * 360 // len(motions)} deg"
        )
    print(
        f"\n{revolutions:g} revolutions at {period:g}s each "
        f"({revolutions * period:.1f}s total), goals updated at {UPDATE_HZ:g} Hz."
    )
    print(f"Peak axis speed this demands: {demanded} counts/s.")

    if demanded > speed_cap:
        slowest = (
            2 * math.pi * max(m.peak_counts_per_radian for m in motions) / speed_cap
        )
        raise BusError(
            f"this rotation needs {demanded} counts/s, above the {speed_cap} "
            "counts/s that three axes moving together can draw without sagging "
            f"the supply. Use --period {slowest:.1f} or longer, or reduce "
            "--amplitude."
        )

    if not assume_yes and not confirm("Start the rotation?"):
        print("Aborted; nothing moved.")
        return 1

    rig.preflight(servos)
    ids = [s.id for s in servos]
    log = PositionLog(
        log_path or runtime_dir("logs", f"rotate_{time.strftime('%Y%m%d_%H%M%S')}.csv"),
        servos,
    )
    print(f"Logging to {log.path}")

    try:
        with rig.bus.torque(ids):
            # Ease into the starting pose first: dropping straight onto the
            # sinusoid from wherever the axes happen to sit is a step change,
            # and a step change is what draws the current spike.
            start = {m.servo.id: m.position(0.0) for m in motions}
            print("Moving to the start of the rotation...")
            rig.move_to(start, speed=min(speed_cap, rig.config.speed))

            print(f"Rotating {direction}. Ctrl-C releases torque.")
            period_s = 1.0 / UPDATE_HZ
            began = time.monotonic()
            total = revolutions * period
            turn = 0

            while True:
                elapsed = time.monotonic() - began
                if elapsed >= total:
                    break
                angle = 2 * math.pi * elapsed / period
                goals = {m.servo.id: m.position(angle) for m in motions}
                rig.goto(goals)
                time.sleep(period_s)
                measured = rig.bus.read_positions(ids)
                log.record("rotate", "", 0, goals, measured)

                if int(elapsed / period) > turn:
                    turn = int(elapsed / period)
                    print(f"  revolution {turn}/{revolutions:g}")

            print("Returning to home...")
            rig.home(servos)
    finally:
        log.close()

    print(f"Done. Log: {log.path}")
    return 0
