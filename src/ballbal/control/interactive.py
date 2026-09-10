"""Interactive keyboard control.

Two things make this different from a naive read-a-key-and-write-a-goal loop.

The commanded position is never allowed to run more than ``max_lag`` counts
ahead of the measured one. Keyboard auto-repeat fires far faster than a servo
can travel, so an unclamped target accumulates into a large step the moment the
key is released -- exactly the runaway that makes a stalled or underpowered
servo look like a software bug.

And keys are drained non-blockingly and collapsed into one goal per tick, so a
held key produces smooth motion at a fixed rate rather than a backlog of queued
writes.

Travel limits are applied here and again in ``ServoBus``, which refuses a goal
outside them outright. This layer clamps so that holding a key simply stops at
the limit instead of raising.
"""

from __future__ import annotations

import csv
import select
import sys
import termios
import time
import tty
from contextlib import contextmanager
from datetime import datetime
from pathlib import Path
from typing import Iterator, Sequence, TextIO

from ..hardware import registers as reg
from ..config import ServoConfig
from ..paths import runtime_dir
from .rig import Rig

AXIS_KEYS = ("qa", "ws", "ed", "rf", "tg", "yh")
"""Per-axis (decrease, increase) key pairs, assigned in rig order."""

ALL_DOWN, ALL_UP = "-", "+"
STEP_DOWN, STEP_UP = "[", "]"

TICK_HZ = 40.0
HEARTBEAT_S = 1.0
"""How often to log a sample when nothing is being commanded."""


@contextmanager
def raw_keys(stream: TextIO) -> Iterator[None]:
    """Put the terminal in cbreak mode, restoring it whatever happens."""
    if not stream.isatty():
        raise RuntimeError("interactive control needs a terminal")
    fd = stream.fileno()
    saved = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)


def _drain_keys(stream: TextIO) -> list[str]:
    """Return every key currently buffered, without blocking."""
    keys: list[str] = []
    while select.select([stream], [], [], 0)[0]:
        char = stream.read(1)
        if not char:
            break
        keys.append(char.lower())
    return keys


class PositionLog:
    """One row per sample, one column pair per axis.

    A row per servo per event, which is what this used to write, makes the
    common question -- where were all three axes at time T -- a join instead of
    a lookup.
    """

    def __init__(self, path: Path, servos: Sequence[ServoConfig]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self.servos = servos
        self._file = path.open("w", newline="", encoding="utf-8")
        fields = ["timestamp", "elapsed_s", "event", "key", "step"]
        for servo in servos:
            fields += [f"{servo.name}_target", f"{servo.name}_measured"]
        self._writer = csv.DictWriter(self._file, fieldnames=fields)
        self._writer.writeheader()
        self._file.flush()
        self._start = time.monotonic()

    def record(
        self,
        event: str,
        key: str,
        step: int,
        targets: dict[int, int],
        measured: dict[int, int],
    ) -> None:
        row = {
            "timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "elapsed_s": f"{time.monotonic() - self._start:.3f}",
            "event": event,
            "key": key,
            "step": step,
        }
        for servo in self.servos:
            row[f"{servo.name}_target"] = targets[servo.id]
            row[f"{servo.name}_measured"] = measured[servo.id]
        self._writer.writerow(row)
        self._file.flush()

    def close(self) -> None:
        self._file.close()


def default_log_path(root: Path) -> Path:
    stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    return runtime_dir("logs", f"jog_{stamp}.csv")


def interactive_jog(
    rig: Rig,
    servos: Sequence[ServoConfig],
    *,
    step: int | None = None,
    log_path: Path | None = None,
) -> int:
    """Drive servos from the keyboard until the operator quits."""
    if len(servos) > len(AXIS_KEYS):
        raise ValueError(
            f"interactive control handles at most {len(AXIS_KEYS)} axes at once"
        )
    if not sys.stdin.isatty():
        raise RuntimeError("interactive control must run in a terminal")

    config = rig.config
    step = step if step is not None else config.step
    if not 1 <= step <= 200:
        raise ValueError("step must be 1..200 counts")

    rig.preflight(servos)

    bindings = {
        key: (servo, direction)
        for servo, pair in zip(servos, AXIS_KEYS)
        for key, direction in ((pair[0], -1), (pair[1], +1))
    }

    ids = [s.id for s in servos]
    measured = rig.bus.read_positions(ids)
    targets = dict(measured)
    observed = {sid: [measured[sid], measured[sid]] for sid in ids}

    root = config.source.parent if config.source else Path.cwd()
    log = PositionLog(log_path or default_log_path(root), servos)
    print(f"Logging to {log.path}")
    _print_help(servos, step, config.max_lag)

    period = 1.0 / TICK_HZ
    last_log = 0.0

    try:
        with rig.bus.torque(ids):
            log.record("start", "", step, targets, measured)
            with raw_keys(sys.stdin):
                while True:
                    keys = _drain_keys(sys.stdin)
                    event, pressed = "", ""

                    if " " in keys:
                        log.record("emergency_stop", " ", step, targets, measured)
                        print("\r\nEMERGENCY STOP: releasing torque.")
                        return 0
                    if "x" in keys:
                        print("\r\nExiting.")
                        break
                    if "?" in keys:
                        _print_help(servos, step, config.max_lag)
                        keys = [k for k in keys if k != "?"]
                    if "p" in keys:
                        _print_ranges(servos, observed)
                        keys = [k for k in keys if k != "p"]
                    if "h" in keys:
                        print("\r\nHoming...")
                        measured = rig.home(servos)
                        targets = dict(measured)
                        log.record("home", "h", step, targets, measured)
                        keys = [k for k in keys if k != "h"]
                    for key in [k for k in keys if k in (STEP_DOWN, STEP_UP)]:
                        step = max(1, min(200, step + (1 if key == STEP_UP else -1)))
                        print(f"\r\nStep: {step} counts")
                    keys = [k for k in keys if k not in (STEP_DOWN, STEP_UP)]

                    for key in keys:
                        if key in (ALL_DOWN, ALL_UP):
                            direction = -1 if key == ALL_DOWN else +1
                            for servo in servos:
                                targets[servo.id] = servo.clamp(
                                    targets[servo.id] + direction * step
                                )
                            event, pressed = "move_all", key
                        elif key in bindings:
                            servo, direction = bindings[key]
                            targets[servo.id] = servo.clamp(
                                targets[servo.id] + direction * step
                            )
                            event, pressed = "move", key

                    # Clamp the commanded position to a bounded lead over the
                    # measured one. Without this, auto-repeat lets the target
                    # sprint away from a servo that is stalled or too slow.
                    for servo in servos:
                        low = measured[servo.id] - config.max_lag
                        high = measured[servo.id] + config.max_lag
                        targets[servo.id] = servo.clamp(
                            max(low, min(high, targets[servo.id]))
                        )

                    rig.goto(targets)
                    time.sleep(period)
                    measured = rig.bus.read_positions(ids)

                    for servo in servos:
                        seen = observed[servo.id]
                        seen[0] = min(seen[0], measured[servo.id])
                        seen[1] = max(seen[1], measured[servo.id])

                    now = time.monotonic()
                    if event:
                        log.record(event, pressed, step, targets, measured)
                        last_log = now
                        print("\r" + _status_line(servos, targets, measured), end="")
                        sys.stdout.flush()
                    elif now - last_log >= HEARTBEAT_S:
                        # Keep sampling while idle: the log should show where the
                        # rig sat, not only where it was pushed.
                        log.record("sample", "", step, targets, measured)
                        last_log = now
    finally:
        log.close()

    _print_ranges(servos, observed)
    print(f"Log saved: {log.path}")
    return 0


def _status_line(
    servos: Sequence[ServoConfig],
    targets: dict[int, int],
    measured: dict[int, int],
) -> str:
    parts = []
    for servo in servos:
        at_limit = (
            "<" if measured[servo.id] <= servo.min_position + 2
            else ">" if measured[servo.id] >= servo.max_position - 2
            else " "
        )
        parts.append(f"{servo.name}: {measured[servo.id]:>4}{at_limit}")
    return "  ".join(parts) + "   "


def _print_help(servos: Sequence[ServoConfig], step: int, max_lag: int) -> None:
    print("\r\nControls:")
    for servo, pair in zip(servos, AXIS_KEYS):
        print(
            f"\r  {servo.name:<10} [{pair[0]}] -   [{pair[1]}] +      "
            f"travel {servo.min_position}..{servo.max_position} "
            f"({reg.counts_to_degrees(servo.max_position - servo.min_position):.0f} deg)"
        )
    print(f"\r  {'all axes':<10} [{ALL_DOWN}] -   [{ALL_UP}] +")
    print(f"\r  step {step:<5}  [{STEP_DOWN}] finer  [{STEP_UP}] coarser")
    print("\r  [h] home   [p] observed range   [?] help   [x] quit")
    print("\r  [space] EMERGENCY STOP (torque off)")
    print(
        f"\r  goals are clamped to each axis's travel and to {max_lag} counts "
        "ahead of measured\r"
    )


def _print_ranges(
    servos: Sequence[ServoConfig], observed: dict[int, list[int]]
) -> None:
    print("\r\nObserved travel this session:")
    for servo in servos:
        low, high = observed[servo.id]
        print(
            f"\r  {servo.name:<10} {low}..{high}  (span {high - low} counts, "
            f"{reg.counts_to_degrees(high - low):.1f} deg)"
        )
