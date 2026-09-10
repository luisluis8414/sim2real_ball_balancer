"""Behavioural model of one Feetech STS3215 axis as driven by the live system (ballbal).

goal written  ->  dead time  ->  trapezoidal profile (re-planned on every new goal,
like the servo firmware)  ->  first-order lag of the servo's position loop  ->  position

Units are encoder counts (4096 per turn) and seconds, so goals can be fed exactly as
``ballbal`` writes them. Parameters come from this package's ``params.json``, identified
on the real rig; see ``docs/servo-model.md``.
"""

from __future__ import annotations

import json
import math
from collections import deque
from dataclasses import dataclass
from pathlib import Path

COUNTS_PER_REV = 4096
PARAMS_FILE = Path(__file__).resolve().parent / "params.json"


@dataclass(frozen=True)
class STS3215Params:
    delay_s: float
    accel: float  # counts/s^2 while speeding up
    decel: float  # counts/s^2 while slowing down
    v_max: float  # counts/s
    tau_s: float  # position-loop lag

    @classmethod
    def load(cls, path: Path = PARAMS_FILE) -> "STS3215Params":
        m = json.loads(Path(path).read_text())["model"]
        return cls(m["delay_s"], m["accel_counts_s2"], m["decel_counts_s2"], m["v_max_counts_s"], m["tau_s"])


class STS3215:
    def __init__(self, params: STS3215Params, position: float) -> None:
        self.p = params
        self.goal = float(position)
        self.speed = params.v_max  # speed limit of the goal in effect
        self._pending: deque[tuple[float, float, float]] = deque()  # (time it takes effect, goal, speed)
        self._q = float(position)  # profile position
        self._v = 0.0  # profile velocity
        self.position = float(position)  # output after the lag

    def reset(self, position: float) -> None:
        self.__init__(self.p, position)

    def set_goal(self, goal: float, now: float, speed: float | None = None) -> None:
        """Goal written on the bus at time ``now``.

        ``speed`` is the goal-speed register in counts/s (0 or None = as fast as the motor goes).
        Below saturation the servo tracks it closely, above it the motor caps it at v_max.
        """
        v = self.p.v_max if not speed else min(float(speed), self.p.v_max)
        self._pending.append((now + self.p.delay_s, float(goal), v))

    def step(self, now: float, dt: float) -> float:
        while self._pending and self._pending[0][0] <= now:
            _, self.goal, self.speed = self._pending.popleft()
        e = self.goal - self._q
        # fastest speed from which the goal can still be reached at the decel limit
        v_target = math.copysign(min(self.speed, math.sqrt(2.0 * self.p.decel * abs(e))), e) if e else 0.0
        speeding_up = abs(v_target) > abs(self._v) and v_target * self._v >= 0.0
        limit = (self.p.accel if speeding_up else self.p.decel) * dt
        self._v += max(-limit, min(limit, v_target - self._v))
        q_next = self._q + self._v * dt
        if (self.goal - q_next) * e < 0.0:  # would pass the goal inside this step
            q_next, self._v = self.goal, 0.0
        self._q = q_next
        alpha = 1.0 if self.p.tau_s <= 0 else 1.0 - math.exp(-dt / self.p.tau_s)
        self.position += (self._q - self.position) * alpha
        return self.position


def counts_to_rad(counts: float, zero: float) -> float:
    """Sim joint angle for an encoder reading; ``zero`` is the reading at the CAD pose."""
    return (counts - zero) * 2.0 * math.pi / COUNTS_PER_REV


def rad_to_counts(rad: float, zero: float) -> float:
    return zero + rad * COUNTS_PER_REV / (2.0 * math.pi)
