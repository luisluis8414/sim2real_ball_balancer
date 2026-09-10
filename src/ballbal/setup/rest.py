"""Record where every axis reads with the lower links resting on the base.

That pose needs no judgement to reproduce and exists identically in the CAD, which
makes it the reference the simulation hangs its joint angles on: sim angle =
(counts - rest) in degrees + the CAD contact angle. The value lands in the profile's
``calibration.toml`` as
``rest_position`` beside the limits, so the simulation reads it from the same file
as everything else calibration and trim produce.

Releasing torque does not get there: the STS3215 gearbox does not backdrive, so the
platform stays wherever it was. The axes are therefore driven down together, slowly,
until the links land on the base and the move stalls; then torque is released so the
printed parts relax off the servo's push before the positions are read. Driving all
three together matters -- one leg alone would tilt the plate instead of seating it.
"""

from __future__ import annotations

import statistics
import time
from typing import Sequence

from ..hardware.bus import BusError
from .calibrate import write_servo_fields
from ..config import ServoConfig
from ..prompt import confirm
from ..control.rig import Rig

DOWN_SPEED = 600  # counts/s; three axes together, well inside what the supply holds
CYCLES = 3
"""Seatings averaged. The legs bind against each other on the way down, so one seating lands
anywhere within about +/-10 counts; measured on this rig: 960/487/523, 971/478/535."""
SETTLE_S = 1.0
SAMPLES = 20
INTERVAL_S = 0.05


def measure_rest(
    rig: Rig,
    servos: Sequence[ServoConfig],
    *,
    settle: float | None = None,
    samples: int = SAMPLES,
    interval: float | None = None,
    cycles: int | None = None,
) -> tuple[dict[int, int], dict[int, list[int]], list[str]]:
    """Seat the platform on the base ``cycles`` times; return the mean seating per axis, every
    seating, and warnings."""
    every = rig.config.servos
    wanted = [s.id for s in servos]
    seatings: dict[int, list[int]] = {sid: [] for sid in wanted}
    for _ in range(CYCLES if cycles is None else cycles):
        with rig.bus.torque([s.id for s in every]):
            # lift to home first, so every seating starts from the same pose
            rig.move_to({s.id: s.home_position for s in every}, speed=DOWN_SPEED)
            try:
                rig.move_to({s.id: s.min_position for s in every}, speed=DOWN_SPEED)
            except BusError:
                pass  # the expected ending: the links reach the base above min and the move stalls
        time.sleep(SETTLE_S if settle is None else settle)
        readings: dict[int, list[int]] = {sid: [] for sid in wanted}
        for _ in range(samples):
            for sid, position in rig.bus.read_positions(wanted).items():
                readings[sid].append(position)
            time.sleep(INTERVAL_S if interval is None else interval)
        for sid, values in readings.items():
            seatings[sid].append(round(statistics.median(values)))
    rest = {sid: round(statistics.fmean(values)) for sid, values in seatings.items()}
    warnings = [
        f"{s.name} came to rest on its min_position ({s.min_position}), so its link may not be on the "
        "base: rest_position is then only an upper bound."
        for s in servos
        if rest[s.id] - s.min_position <= rig.config.tolerance
    ]
    return rest, seatings, warnings


def record_rest(rig: Rig, servos: Sequence[ServoConfig], *, assume_yes: bool = False) -> int:
    if rig.config.home_offset is not None:
        raise RuntimeError(
            "rest is derived from min_position in this profile; run `ballbal calibrate`"
        )
    source = rig.config.calibration_source or rig.config.source
    if source is None:
        raise RuntimeError("cannot write rest positions back: the rig was not loaded from a file")
    print(f"{CYCLES} times: lift to home, lower every axis at {DOWN_SPEED} counts/s until the links sit on\n"
          "the base, release torque, read.")
    if not assume_yes and not confirm("The platform will move. Go?"):
        return 1
    rest, seatings, warnings = measure_rest(rig, servos)
    print(f"\n  {'axis':10}{'rest':>6}{'stored':>8}{'min':>6}   seatings")
    for servo in servos:
        stored = "-" if servo.rest_position is None else str(servo.rest_position)
        print(f"  {servo.name:10}{rest[servo.id]:>6}{stored:>8}{servo.min_position:>6}   "
              + " ".join(str(v) for v in seatings[servo.id]))
    for warning in warnings:
        print(f"  note: {warning}")
    if not assume_yes and not confirm(f"Write rest_position to {source}?"):
        print("Not written.")
        return 1
    write_servo_fields(source, {sid: {"rest_position": value} for sid, value in rest.items()})
    print(f"Wrote {source}.")
    return 0
