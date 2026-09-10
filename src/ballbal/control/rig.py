"""The rig: a configured set of servos, with preflight checks and safe motion."""

from __future__ import annotations

import logging
import math
import time
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Callable, Iterator, Sequence

from ..hardware import registers as reg
from ..hardware.bus import BusError, ServoBus, Telemetry
from ..config import RigConfig, ServoConfig
from ..hardware.port import find_port

logger = logging.getLogger(__name__)

RAMP_HEADROOM = 1.5
"""How far ahead of one tick's travel the ramp is allowed to lead.

Below 1.0 the ramp throttles the servo below its own speed setting. Well above
it, the commanded position runs far enough ahead that a collision no longer
stalls the motion promptly -- the lead is the safety margin.
"""


def ramp_step(speed: int, rate_hz: float, tolerance: int) -> int:
    """Counts per ramp tick: fast enough not to throttle, big enough to move.

    Two bounds, and the floor is the one that bites. A step at or below the
    servo's settling band is a command it treats as "already there": it does not
    move, so the measured position does not change, so the next step is computed
    from the same place and is identical. The ramp deadlocks and the move times
    out having never started. ``tolerance`` is exactly that band, measured, so
    the floor is derived from it rather than guessed.
    """
    return max(2 * tolerance, math.ceil(speed / rate_hz * RAMP_HEADROOM))


class PreflightError(RuntimeError):
    """The rig is not in a state where it is safe to apply torque."""


@dataclass(frozen=True)
class Reading:
    servo: ServoConfig
    telemetry: Telemetry

    @property
    def in_travel(self) -> bool:
        return (
            self.servo.min_position
            <= self.telemetry.position
            <= self.servo.max_position
        )


class Rig:
    """Configured servos on one bus.

    Use it as a context manager; the bus closes and releases torque on exit.
    """

    def __init__(self, config: RigConfig, bus: ServoBus) -> None:
        self.config = config
        self.bus = bus

    @classmethod
    @contextmanager
    def open(
        cls, config: RigConfig, port_override: str | None = None
    ) -> Iterator["Rig"]:
        port = find_port(port_override or config.port)
        logger.info("opening %s at %d baud", port, config.baudrate)
        with ServoBus(port, config.baudrate, retries=config.retries) as bus:
            # Arm the bus with the rig's limits, so they hold even for code that
            # bypasses Rig and writes goals directly.
            bus.set_limits(
                {s.id: (s.min_position, s.max_position) for s in config.servos}
            )
            yield cls(config, bus)

    # -- inspection ---------------------------------------------------------

    def survey(self, servos: Sequence[ServoConfig] | None = None) -> list[Reading]:
        """Read full telemetry for the given servos (default: all of them)."""
        return [
            Reading(servo, self.bus.telemetry(servo.id))
            for servo in (servos if servos is not None else self.config.servos)
        ]

    def positions(
        self, servos: Sequence[ServoConfig] | None = None
    ) -> dict[int, int]:
        chosen = servos if servos is not None else self.config.servos
        return self.bus.read_positions([s.id for s in chosen])

    # -- preflight ----------------------------------------------------------

    def preflight(self, servos: Sequence[ServoConfig]) -> list[Reading]:
        """Refuse to energise servos that are not ready to move.

        Checked in the order that matters: presence, supply voltage, fault
        flags, control mode, temperature, and finally whether the servo is
        parked somewhere the configured travel allows.

        Sitting outside the configured travel is reported but does not block.
        Every goal is clamped into travel before it is sent, so the only move
        available from out there is one heading back in -- and that is exactly
        what an axis nudged by hand, or dropped when a servo faulted, needs.
        Refusing it meant refusing the fix.
        """
        readings = self.survey(servos)
        problems: list[str] = []

        for reading in readings:
            servo, telemetry = reading.servo, reading.telemetry
            label = f"{servo.name} (id {servo.id})"

            # The servo's own limit is authoritative; the rig file can only
            # raise the bar, never lower it below what the firmware accepts.
            floor = max(self.config.min_voltage, telemetry.min_voltage_limit)
            if telemetry.voltage < floor:
                problems.append(
                    f"{label}: supply is {telemetry.voltage:.1f} V, below the "
                    f"{floor:.1f} V minimum (servo accepts "
                    f"{telemetry.min_voltage_limit:.1f}-"
                    f"{telemetry.max_voltage_limit:.1f} V). Check the DC input "
                    "on the driver board and the supply's current rating."
                )
            # The servo's own ceiling, enforced the same way as the floor. These
            # are 7.4 V parts reading 4.0-8.0 V; the driver board's silkscreen
            # says 9-12.6 V because it is normally sold with the 12 V variant,
            # and it passes the input straight through. Following the board
            # would take the servos past their own limit.
            if telemetry.voltage > telemetry.max_voltage_limit:
                problems.append(
                    f"{label}: supply is {telemetry.voltage:.1f} V, above the "
                    f"servo's {telemetry.max_voltage_limit:.1f} V ceiling. Turn "
                    "the supply down before running anything -- ignore the "
                    "board's printed range, it is for the 12 V servo variant."
                )
            if telemetry.status:
                problems.append(f"{label}: fault flags set ({telemetry.status_text})")
            if telemetry.mode != reg.POSITION_MODE:
                problems.append(
                    f"{label}: in mode {telemetry.mode}, not position mode "
                    f"{reg.POSITION_MODE}"
                )
            if telemetry.temperature > self.config.max_temperature:
                problems.append(
                    f"{label}: {telemetry.temperature} C exceeds the "
                    f"{self.config.max_temperature} C limit"
                )
            if not reading.in_travel:
                outside = (
                    servo.min_position - telemetry.position
                    if telemetry.position < servo.min_position
                    else telemetry.position - servo.max_position
                )
                print(
                    f"note: {label} is at {telemetry.position}, {outside} counts "
                    f"outside its travel {servo.min_position}.."
                    f"{servo.max_position}. The first move will bring it back in."
                )

        if problems:
            raise PreflightError("\n".join(f"  - {p}" for p in problems))
        return readings

    # -- motion -------------------------------------------------------------

    def goto(
        self,
        goals: dict[int, int],
        *,
        speed: int | None = None,
        acceleration: int | None = None,
        settle: float = 0.0,
    ) -> None:
        """Command goal positions, clamped to each servo's configured travel."""
        clamped = {
            servo_id: self.config.by_id(servo_id).clamp(position)
            for servo_id, position in goals.items()
        }
        for servo_id, position in goals.items():
            if clamped[servo_id] != position:
                servo = self.config.by_id(servo_id)
                logger.warning(
                    "%s: goal %d clamped to %d (travel %d..%d)",
                    servo.name,
                    position,
                    clamped[servo_id],
                    servo.min_position,
                    servo.max_position,
                )
        self.bus.write_goals(
            clamped,
            speed if speed is not None else self.config.speed,
            acceleration if acceleration is not None else self.config.acceleration,
        )
        if settle:
            time.sleep(settle)

    def move_to(
        self,
        goals: dict[int, int],
        *,
        step: int | None = None,
        rate_hz: float = 50.0,
        timeout: float = 15.0,
        tolerance: int | None = None,
        speed: int | None = None,
        acceleration: int | None = None,
        stall_seconds: float = 0.4,
        on_tick: Callable[[dict[int, int]], None] | None = None,
    ) -> dict[int, int]:
        """Move to the goals and wait for arrival, watching for a stall.

        By default the goal is commanded once and the servo runs its own
        trapezoidal profile, bounded by ``speed`` and ``acceleration``. That is
        what lets it reach cruise speed: re-aiming the goal a short distance
        ahead on every tick, as ``step`` does, forces it to decelerate for each
        segment, capping the achievable speed near ``sqrt(acceleration * step)``
        however high ``speed`` is set.

        Progress is polled regardless, so an axis that stops short of its goal
        is caught within ``stall_seconds`` rather than at the timeout.

        Args:
            step: Interpolate instead, keeping the commanded position at most
                this many counts ahead of the measured one. Slower, but it caps
                the position error and so the torque driven into an obstruction.
                Worth it when the travel limits are not yet trusted.
            stall_seconds: How long the position may fail to change by more than
                ``tolerance`` before the move is called stalled.
            on_tick: Called with the measured positions after every poll, for
                callers watching load or voltage while the axis is moving.

        Returns:
            The final measured positions.
        """
        if tolerance is None:
            tolerance = self.config.tolerance
        commanded_speed = speed if speed is not None else self.config.speed
        servo_ids = list(goals)
        current = self.bus.read_positions(servo_ids)
        period = 1.0 / rate_hz
        deadline = time.monotonic() + timeout

        if step is None:
            self.goto(goals, speed=speed, acceleration=acceleration)

        last_progress = time.monotonic()
        reference = dict(current)

        while time.monotonic() < deadline:
            remaining = {sid: goals[sid] - current[sid] for sid in servo_ids}
            if all(abs(d) <= tolerance for d in remaining.values()):
                return current

            if step is not None:
                wave = {
                    sid: current[sid] + max(-step, min(step, remaining[sid]))
                    for sid in servo_ids
                }
                self.goto(wave, speed=speed, acceleration=acceleration)

            time.sleep(period)
            current = self.bus.read_positions(servo_ids)
            if on_tick is not None:
                on_tick(current)

            if any(
                abs(current[sid] - reference[sid]) > tolerance for sid in servo_ids
            ):
                reference = dict(current)
                last_progress = time.monotonic()
            elif time.monotonic() - last_progress > stall_seconds:
                self._stop_where_they_are(servo_ids, current)
                raise BusError(
                    f"stalled: no axis moved more than {tolerance} counts in "
                    f"{stall_seconds:.1f}s. "
                    f"{self._describe_shortfall(goals, current)}. Goals were "
                    "reset to the measured positions so nothing keeps pushing."
                )

        self._stop_where_they_are(servo_ids, current)
        raise BusError(
            f"move timed out after {timeout:.0f}s. "
            f"{self._describe_shortfall(goals, current)}"
        )

    def _stop_where_they_are(
        self, servo_ids: Sequence[int], current: dict[int, int]
    ) -> None:
        """Re-aim each goal at the measured position.

        A servo holds its last goal. Abandoning a move without doing this leaves
        it straining toward a point it could not reach, which is the worst place
        to leave a stalled axis.
        """
        try:
            self.goto({sid: current[sid] for sid in servo_ids})
        except BusError:
            logger.warning("could not neutralise goals after a failed move")

    def _describe_shortfall(
        self, goals: dict[int, int], current: dict[int, int]
    ) -> str:
        parts = [
            f"{self.config.by_id(sid).name}: at {current[sid]}, wanted {goal}"
            for sid, goal in goals.items()
            if abs(goal - current[sid]) > self.config.tolerance
        ]
        return "; ".join(parts) if parts else "all axes arrived"

    def home(self, servos: Sequence[ServoConfig], **kwargs: object) -> dict[int, int]:
        """Ramp the given servos to their configured home positions."""
        return self.move_to(
            {s.id: s.home_position for s in servos},
            **kwargs,  # type: ignore[arg-type]
        )
