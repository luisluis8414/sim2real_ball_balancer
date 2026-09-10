"""Nudge the travel limits onto the real ones, under power, all axes at once.

Calibration is done by hand with torque off, which is the only way to find where
a mechanism stops -- but hands are coarse. A limit landed a few degrees past
where the platform actually tops out leaves every later command believing in
travel the rig does not have.

All three axes move together here, because on a parallel platform the height at
which the mechanism tops out is a property of the whole linkage, not of one leg.
Driving one axis alone to its limit puts the platform in a completely different
pose from the one where all three are up, and the limit found that way is not
the limit that matters. Per-axis keys are there as well, for levelling the
platform once it is at the stop.

Trimming only ever moves a limit *inward*: it makes the recorded travel smaller,
never larger. So it needs no exemption from the travel limits the bus already
enforces -- the whole operation happens inside ground the axes have been driven
through before, and a goal that would leave it is refused here exactly as it
would be anywhere else.
"""

from __future__ import annotations

import select
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

from ..hardware import registers as reg
from ..setup.calibrate import write_servo_fields
from ..config import ServoConfig
from .interactive import raw_keys
from ..prompt import confirm
from .rig import Rig

DOWN_KEY, UP_KEY = "-", "+"
FINER_KEY, COARSER_KEY = "[", "]"
ACCEPT_KEYS = ("\n", "\r")
ABORT_KEY = "x"
AXIS_KEYS = ("qa", "ws", "ed", "rf", "tg", "yh")
"""Per-axis (down, up) pairs, for levelling once the platform is at the stop."""

TICK_HZ = 30.0


@dataclass
class Trim:
    servo: ServoConfig
    edge: str
    before: int
    after: int

    @property
    def changed(self) -> bool:
        return self.before != self.after

    @property
    def delta(self) -> int:
        return self.after - self.before


def trim(
    rig: Rig,
    servos: Sequence[ServoConfig],
    *,
    edge: str = "max",
    step: int = 5,
    assume_yes: bool = False,
) -> int:
    """Walk each axis, let the operator place its limit, then store the lot."""
    if rig.config.home_offset is not None:
        raise RuntimeError(
            "per-axis trim is disabled: home and max are fixed offsets above min"
        )
    if edge not in ("min", "max"):
        raise ValueError("edge must be 'min' or 'max'")
    if not sys.stdin.isatty():
        raise RuntimeError("trimming is interactive and needs a terminal")
    if not 1 <= step <= 100:
        raise ValueError("step must be 1..100 counts")

    source = rig.config.calibration_source or rig.config.source
    if source is None:
        raise RuntimeError("the rig was not loaded from a file, so it cannot be updated")

    print(
        f"\nTrimming the {edge.upper()} limits of {len(servos)} axes together.\n\n"
        f"  All axes are driven to their stored {edge} limits at once, then you\n"
        "  move them as a group to where the platform actually stops. On a\n"
        "  parallel platform that height comes from all three legs together --\n"
        "  one leg alone at its limit is a different pose entirely.\n\n"
        f"  [{DOWN_KEY}] / [{UP_KEY}]  move all      "
        f"[{FINER_KEY}] / [{COARSER_KEY}]  step size\n"
        + "".join(
            f"  [{pair[0]}] / [{pair[1]}]  {servo.name} alone\n"
            for servo, pair in zip(servos, AXIS_KEYS)
        )
        + f"  [Enter] accept all      [{ABORT_KEY}] abort\n\n"
        "  Trimming only moves a limit inward -- it makes the recorded travel\n"
        "  smaller, never larger -- so movement stays inside the range already\n"
        "  calibrated. A '|' in the table marks an axis against the far end of\n"
        "  that range, which will not go further."
    )
    if not assume_yes and not confirm("Start trimming?"):
        print("Aborted; nothing moved.")
        return 1

    rig.preflight(servos)
    stored = {
        s.id: (s.max_position if edge == "max" else s.min_position) for s in servos
    }
    # The existing travel is the whole permitted range: nothing is relaxed, and
    # the bus would refuse a goal beyond it in any case.
    bounds = {s.id: (s.min_position, s.max_position) for s in servos}
    placed = _place(rig, servos, stored, bounds, step)

    if placed is None:
        print("\nAborted; nothing was written.")
        return 1

    results = [
        Trim(servo, edge, stored[servo.id], placed[servo.id]) for servo in servos
    ]
    return _store(rig, source, results, edge, assume_yes)


def _place(
    rig: Rig,
    servos: Sequence[ServoConfig],
    stored: dict[int, int],
    window: dict[int, tuple[int, int]],
    step: int,
) -> dict[int, int] | None:
    """Drive to the stored limits, then let the operator place them together."""
    ids = [s.id for s in servos]
    bindings = {
        key: (servo, direction)
        for servo, pair in zip(servos, AXIS_KEYS)
        for key, direction in ((pair[0], -1), (pair[1], +1))
    }

    with rig.bus.torque(ids):
        rig.move_to(dict(stored), speed=min(600, rig.config.speed))
        targets = rig.bus.read_positions(ids)
        period = 1.0 / TICK_HZ
        header = "  " + "".join(f"{s.name:>10}" for s in servos)
        print(f"\n{header}   step")

        with raw_keys(sys.stdin):
            while True:
                for key in _drain(sys.stdin):
                    if key in ACCEPT_KEYS:
                        print()
                        return rig.bus.read_positions(ids)
                    if key == ABORT_KEY:
                        print()
                        return None
                    if key in (FINER_KEY, COARSER_KEY):
                        step = max(1, min(100, step + (1 if key == COARSER_KEY else -1)))
                    else:
                        apply_key(key, servos, bindings, targets, window, step)

                rig.goto(targets)
                time.sleep(period)
                measured = rig.bus.read_positions(ids)
                row = ""
                for servo in servos:
                    low, high = window[servo.id]
                    pinned = (
                        "|" if targets[servo.id] in (low, high) else " "
                    )
                    row += (
                        f"{measured[servo.id]:>6}"
                        f"{measured[servo.id] - stored[servo.id]:>+4}{pinned}"
                    )
                sys.stdout.write(f"\r  {row}   {step:>4} ")
                sys.stdout.flush()


def apply_key(
    key: str,
    servos: Sequence[ServoConfig],
    bindings: dict[str, tuple[ServoConfig, int]],
    targets: dict[int, int],
    window: dict[int, tuple[int, int]],
    step: int,
) -> None:
    """Fold one keypress into the targets, in place.

    ``-`` and ``+`` move every axis, which is the point: the height a parallel
    platform tops out at comes from all three legs together. The per-axis keys
    are for levelling once it is there. Either way each axis stays inside its
    own window, so a group move stops per-leg rather than dragging one past its
    bound to keep the others company.
    """
    if key in (DOWN_KEY, UP_KEY):
        direction = -1 if key == DOWN_KEY else 1
        moving = [(servo, direction) for servo in servos]
    elif key in bindings:
        moving = [bindings[key]]
    else:
        return

    for servo, direction in moving:
        low, high = window[servo.id]
        targets[servo.id] = max(
            low, min(high, targets[servo.id] + direction * step)
        )


def _drain(stream) -> list[str]:  # noqa: ANN001
    keys: list[str] = []
    while select.select([stream], [], [], 0)[0]:
        char = stream.read(1)
        if not char:
            break
        keys.append(char.lower())
    return keys


def _store(
    rig: Rig, source: Path, results: Sequence[Trim], edge: str, assume_yes: bool
) -> int:
    changed = [r for r in results if r.changed]
    print("\n" + "=" * 60)
    for result in results:
        mark = f"{result.delta:+d}" if result.changed else "unchanged"
        print(
            f"  {result.servo.name:<10} {edge} {result.before} -> "
            f"{result.after}   {mark}"
        )
    print("=" * 60)

    if not changed:
        print("Nothing changed; rig file left alone.")
        return 0

    field = f"{edge}_position"
    updates: dict[int, dict[str, int]] = {}
    for result in changed:
        servo = result.servo
        low = result.after if edge == "min" else servo.min_position
        high = servo.max_position if edge == "min" else result.after
        if low >= high:
            print(
                f"error: {servo.name} would end up with {low}..{high}, which "
                "leaves no travel. Nothing was written."
            )
            return 1
        fields = {field: result.after}
        # home sits on the lower limit after calibration; move it along rather
        # than leaving it outside the travel it belongs to.
        if edge == "min" and servo.home_position == servo.min_position:
            fields["home_position"] = result.after
        elif not low <= servo.home_position <= high:
            fields["home_position"] = min(max(servo.home_position, low), high)
        updates[servo.id] = fields

    if not assume_yes and not confirm(f"Write these {edge} limits to {source}?"):
        print("Not written.")
        return 1

    write_servo_fields(source, updates)
    print(f"Wrote {source} (previous version kept as {source.name}.bak).")
    return 0
