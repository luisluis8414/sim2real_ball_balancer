"""Mirror the real rig -- ball and platform -- into the running Isaac Sim.

Used by ``ballbal balance --mirror`` and ``tools/simulation/ball_sync.py``. The ball arrives as
an offset from the plate centre in image millimetres, the platform as measured servo positions;
Isaac Sim shows the ball as a marker on the simulated plate (``isaac_ball.py``) and holds the
simulated axes at the measured positions (``SimServos.follow``).

Built for loops that must not wait. Isaac Sim usually takes fewer updates per second than the
camera delivers, so ``send`` only leaves the newest sample for a sender thread and drops the one
before it. A failing simulation is reported once and then ignored: mirroring never stops the
loop it watches.
"""

from __future__ import annotations

import contextlib
import math
import sys
import threading
import time

from .remote import PACKAGE, IsaacError, IsaacSim

SYNC = "ball.place(x, y) if x is not None else ball.hide()\nservos.follow(positions) if positions else None"


def plate_mm(dx_mm: float, dy_mm: float, bearing_deg: float) -> tuple[float, float]:
    """Image offset from the plate centre (mm, image +y down) -> plate frame (mm).

    The camera image is a top view whose y axis points down, so its angles grow clockwise;
    ``bearing_deg`` is axis_1's angle in it, as in ``balance.Kinematics``. The plate frame, shared
    with the simulation, has +x towards axis_1 and +y counter-clockwise seen from above.
    """
    b = math.radians(bearing_deg)
    along = dx_mm * math.cos(b) + dy_mm * math.sin(b)
    clockwise = -dx_mm * math.sin(b) + dy_mm * math.cos(b)
    return along, -clockwise


class _Latest:
    """One-slot mailbox: a new sample replaces one not yet taken."""

    def __init__(self) -> None:
        self._cond = threading.Condition()
        self._item = None
        self._closed = False

    def put(self, item) -> None:  # noqa: ANN001
        with self._cond:
            self._item = item
            self._cond.notify()

    def take(self):  # noqa: ANN201
        """The newest sample, or None once closed."""
        with self._cond:
            while self._item is None and not self._closed:
                self._cond.wait()
            item, self._item = self._item, None
            return item

    def close(self) -> None:
        with self._cond:
            self._closed = True
            self._cond.notify()


class Mirror:
    """A running mirror; use it as a context manager."""

    def __init__(self, *, bearing_deg: float, ball_mm: float, sim: IsaacSim | None = None) -> None:
        self.bearing_deg = bearing_deg
        self.ball_mm = ball_mm
        self.sim = sim or IsaacSim()
        self.sent = 0
        self.error: Exception | None = None
        self._box = _Latest()
        self._thread = threading.Thread(target=self._forward, name="isaac-mirror", daemon=True)
        self._started = 0.0

    def __enter__(self) -> "Mirror":
        """Scene open, servos armed, timeline playing, ball marker ready. Raises IsaacError."""
        self.sim.start()
        self.sim.run_file(PACKAGE / "isaac_ball.py", ball_mm=self.ball_mm)
        self._started = time.perf_counter()
        self._thread.start()
        return self

    def __exit__(self, *_exc) -> None:  # noqa: ANN002
        self._box.close()
        self._thread.join(timeout=5.0)
        with contextlib.suppress(IsaacError):
            self.sim.run("ball.hide()", timeout=5.0, echo=False)

    def send(self, dx_mm: float | None, dy_mm: float | None,
             positions: dict[int, int] | None = None) -> None:
        """Queue one sample: the ball's image offset from the plate centre in mm (None: not seen)
        and the measured servo positions in counts (None: leave the simulated platform alone)."""
        if self.error is not None:
            return
        x = y = None
        if dx_mm is not None and dy_mm is not None:
            x, y = plate_mm(dx_mm, dy_mm, self.bearing_deg)
        self._box.put((x, y, positions))

    @property
    def rate(self) -> float:
        """Updates Isaac Sim has taken per second so far."""
        elapsed = time.perf_counter() - self._started
        return self.sent / elapsed if self._started and elapsed > 0 else 0.0

    def _forward(self) -> None:
        try:
            while (sample := self._box.take()) is not None:
                x, y, positions = sample
                self.sim.run(SYNC, timeout=5.0, echo=False, x=x, y=y, positions=positions)
                self.sent += 1
        except Exception as exc:  # noqa: BLE001 - reported, never raised into the loop
            self.error = exc
            print(f"\n  Isaac Sim mirror stopped: {exc}", file=sys.stderr)
