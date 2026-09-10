"""Slow end-to-end travel check.

Calibration records where the mechanism stops when *you* move it by hand.
Sweeping is the other half: driving each axis to those recorded limits under
power, slowly, to confirm the numbers are actually reachable and that nothing
binds on the way. It is the first time a motor visits its own end stops, so it
runs one axis at a time, at a fraction of the normal speed, and watches load
while it moves rather than only after it has arrived.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Sequence

from ..hardware import registers as reg
from ..hardware.bus import BusError
from ..config import ServoConfig
from .rig import Rig


# Speed here is the servo's own speed setting, which it tracks closely: 500 ->
# 500, 1000 -> 1000, 2000 -> 1900 counts/s measured on axis_a, saturating near
# 2650 counts/s (233 deg/s) at 5 V. Passing --lead switches to interpolation,
# which caps speed near sqrt(acceleration * lead) but bounds the position error
# and so the torque driven into an obstruction.


MAX_COMBINED_SPEED = 1500
"""Ceiling for the all-axes-together phase, in counts/s.

Measured on a 5 V supply: three axes together hold up at 1800 counts/s (5.2 V
minimum) and fault at 2650. 1500 keeps a margin under that. The sag is faster
than this software can sample, so the servo's own protection is what actually
trips -- there is no polling loop that catches it first.
"""


@dataclass
class LegResult:
    label: str
    target: int
    reached: int | None = None
    peak_load: int = 0
    failure: str | None = None
    errors: dict[int, int] = field(default_factory=dict)

    @property
    def error(self) -> int | None:
        return None if self.reached is None else self.reached - self.target

    @property
    def ok(self) -> bool:
        return self.failure is None


@dataclass
class AxisResult:
    servo: ServoConfig
    legs: list[LegResult] = field(default_factory=list)
    label: str | None = None

    @property
    def name(self) -> str:
        return self.label or self.servo.name

    @property
    def ok(self) -> bool:
        return all(leg.ok for leg in self.legs)


def sweep(
    rig: Rig,
    servos: Sequence[ServoConfig],
    *,
    speed: int | None = None,
    lead: int | None = None,
    dwell: float = 0.8,
    assume_yes: bool = False,
    together: bool = True,
    together_speed: int | None = None,
) -> int:
    """Drive the calibrated limits under power, in three phases.

    First every axis goes to its home position, so the run starts from a known
    pose rather than wherever the last command left things. Then each axis is
    swept alone, which isolates a problem to one axis. Only then are they swept
    together, because that is the case a parallel mechanism can bind in: three
    positions each reachable alone are not necessarily reachable at once.
    """
    config = rig.config
    # Full rig speed by default. Halving it here made the sweep quietly slower
    # than the same move made any other way, which is the sort of hidden
    # governor this command exists to expose, not to add. Pass --speed to be
    # gentler while limits are still unproven.
    slow = speed if speed is not None else config.speed
    # Three axes accelerating at once draw roughly three times the current, and
    # measured on this rig a 5 V supply cannot hold up: three together at 1800
    # counts/s sags to 5.2 V, at 2650 it collapses far enough for the servos to
    # raise a voltage fault -- and a faulted servo drops torque, so the
    # mechanism falls. The combined phase therefore runs slower by default. It
    # is printed in the plan rather than applied silently.
    combined = (
        together_speed
        if together_speed is not None
        else min(slow, MAX_COMBINED_SPEED)
    )

    print("Sweep plan:")
    print(f"  1. all axes to home  ({', '.join(str(s.home_position) for s in servos)})")
    for index, servo in enumerate(servos, start=2):
        print(
            f"  {index}. {servo.name} alone: min {servo.min_position} -> "
            f"max {servo.max_position} -> home {servo.home_position}"
        )
    if together:
        print(
            f"  {len(servos) + 2}. all {len(servos)} axes together, same legs, "
            f"at {combined} counts/s"
        )

    lead_text = "derived from speed" if lead is None else f"{lead} counts"
    print(
        f"\nSpeed {slow} counts/s ({slow / config.speed:.0%} of the rig setting), "
        f"lead {lead_text}, {dwell:.1f}s dwell at each end."
    )
    print("Ctrl-C releases torque at any point.")
    if together:
        print(
            "\nNote: the final phase drives every axis onto its own limits at the\n"
            "same time. Two things make it the riskiest step. On a parallel\n"
            "mechanism those poses can bind even when each axis reaches its\n"
            "limit alone. And three axes accelerating together draw around three\n"
            "times the current, which on a 5 V supply is enough to brown the\n"
            "servos out -- a servo that faults drops torque, and the mechanism\n"
            f"falls. Hence the reduced {combined} counts/s. Pass --no-together\n"
            "to skip the phase entirely."
        )

    if not assume_yes and input("\nStart the sweep? [y/N] ").strip().lower() not in (
        "y",
        "yes",
    ):
        print("Aborted; nothing moved.")
        return 1

    rig.preflight(servos)

    print("\n-- phase 1: all axes to home --")
    ids = [s.id for s in servos]
    with rig.bus.torque(ids):
        final = rig.move_to({s.id: s.home_position for s in servos}, speed=slow)
    for servo in servos:
        print(f"  {servo.name}: {final[servo.id]} (home {servo.home_position})")

    results: list[AxisResult] = []
    for servo in servos:
        print(f"\n-- {servo.name} (id {servo.id}) alone --")
        result = AxisResult(servo=servo)
        # Torque is taken per axis and released again by the context manager, so
        # an axis that has finished is never left energised while the next one
        # moves.
        with rig.bus.torque([servo.id]):
            for label, target in _legs(servo):
                result.legs.append(
                    _run_leg(rig, [servo], label, {servo.id: target}, slow, lead, dwell)
                )
                if not result.legs[-1].ok:
                    break
        results.append(result)

    if together and all(r.ok for r in results):
        print(f"\n-- all {len(servos)} axes together --")
        combined_result = AxisResult(servo=servos[0], label=f"all {len(servos)}")
        with rig.bus.torque(ids):
            for label, _ in _legs(servos[0]):
                targets = {s.id: dict(_legs(s))[label] for s in servos}
                combined_result.legs.append(
                    _run_leg(rig, servos, label, targets, combined, lead, dwell)
                )
                if not combined_result.legs[-1].ok:
                    break
        results.append(combined_result)
    elif together:
        print("\nSkipping the combined phase: an axis did not pass on its own.")

    return _report(results)


def _legs(servo: ServoConfig) -> list[tuple[str, int]]:
    return [
        ("min", servo.min_position),
        ("max", servo.max_position),
        ("home", servo.home_position),
    ]


def _run_leg(
    rig: Rig,
    servos: Sequence[ServoConfig],
    label: str,
    targets: dict[int, int],
    speed: int,
    lead: int | None,
    dwell: float,
) -> LegResult:
    leg = LegResult(label=label, target=next(iter(targets.values())))
    peak = 0

    def watch(_positions: dict[int, int]) -> None:
        nonlocal peak
        for servo_id in targets:
            peak = max(peak, abs(rig.bus.read_signed(servo_id, reg.PRESENT_LOAD)))

    shown = ", ".join(str(t) for t in targets.values())
    print(f"  -> {label:<4} {shown:<20} ", end="", flush=True)
    try:
        rig.move_to(targets, step=lead, speed=speed, timeout=60.0, on_tick=watch)
    except BusError as exc:
        leg.peak_load = peak
        leg.failure = str(exc)
        print("FAILED")
        return leg

    time.sleep(dwell)
    reached = rig.bus.read_positions(list(targets))
    leg.reached = next(iter(reached.values()))
    leg.errors = {sid: reached[sid] - targets[sid] for sid in targets}
    leg.peak_load = max(
        peak, max(abs(rig.bus.read_signed(sid, reg.PRESENT_LOAD)) for sid in targets)
    )
    errors = ", ".join(f"{e:+d}" for e in leg.errors.values())
    print(f"reached {', '.join(str(reached[s]) for s in targets)} "
          f"(error {errors}, peak load {leg.peak_load})")
    return leg


def _report(results: Sequence[AxisResult]) -> int:
    print("\n" + "=" * 66)
    print(f"{'AXIS':<10}{'LEG':<6}{'TARGET':>8}{'REACHED':>9}{'ERROR':>7}{'PEAK LOAD':>11}")
    for result in results:
        for leg in result.legs:
            reached = "-" if leg.reached is None else str(leg.reached)
            error = "-" if leg.error is None else f"{leg.error:+d}"
            print(
                f"{result.name:<10}{leg.label:<6}{leg.target:>8}"
                f"{reached:>9}{error:>7}{leg.peak_load:>11}"
            )
    print("=" * 66)

    failed = [r for r in results if not r.ok]
    if not failed:
        print("All axes reached both limits.")
        return 0

    print("\nDid not complete:")
    for result in failed:
        for leg in result.legs:
            if not leg.ok:
                print(f"  {result.name} @ {leg.label} {leg.target}: {leg.failure}")
    print(
        "\nAn axis that stalls short of its limit usually means the limit is "
        "optimistic: it was recorded by hand, where you could push past a tight "
        "spot the servo cannot. Re-run `ballbal calibrate` for that axis, or "
        "widen the margin."
    )
    return 1
