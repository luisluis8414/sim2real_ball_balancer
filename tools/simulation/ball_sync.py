"""Track the real ball with the rig's camera and mirror it onto the simulated plate.

    python tools/simulation/ball_sync.py                  # ball only
    python tools/simulation/ball_sync.py --rig            # also mirror the real platform pose
    python tools/simulation/ball_sync.py --no-window --seconds 30

Every camera frame the ball is found the way ``ballbal balance`` finds it -- the profile's
camera.json supplies crop, plate centre, mm/px, ball colour and axis_1's bearing -- and converted
into the plate frame shared with the simulation: origin at the plate centre, +x towards the
axis_1 leg, +y a quarter turn counter-clockwise seen from above. Isaac Sim shows it as an orange
marker on the simulated plate (``src/ballbal/simulation/isaac_ball.py``).

With --rig the real servo positions are read every frame as well (torque is not touched, nothing
moves) and the simulated axes are held there, so the simulated plate leans like the real one.
The servo bus serves one process at a time: --rig does not run alongside ``ballbal balance`` or
``ballbal jog``.

Isaac Sim usually renders slower than the camera delivers. A sender thread always forwards the
newest sample and drops older ones, so the camera loop never waits for the simulation.
Needs Isaac Sim launched with its Python Server (docs/simulation.md).
"""

from __future__ import annotations

import argparse
import contextlib
import math
import sys
import threading
import time
from pathlib import Path

from ballbal.config import RigConfig
from ballbal.paths import active_profile_dir
from ballbal.simulation.remote import PACKAGE, IsaacError, IsaacSim

SYNC = "ball.place(x, y) if x is not None else ball.hide()\nservos.follow(positions) if positions else None"


def plate_mm(dx_mm: float, dy_mm: float, bearing_deg: float) -> tuple[float, float]:
    """Image offset from the plate centre (mm, image +y down) -> plate frame (mm).

    The camera image is a top view whose y axis points down, so its angles grow clockwise;
    ``bearing_deg`` is axis_1's angle in it, as in ``balance.Kinematics``. The plate frame has
    +x towards axis_1 and +y counter-clockwise seen from above.
    """
    b = math.radians(bearing_deg)
    along = dx_mm * math.cos(b) + dy_mm * math.sin(b)
    clockwise = -dx_mm * math.sin(b) + dy_mm * math.cos(b)
    return along, -clockwise


class Latest:
    """One-slot mailbox: a new sample replaces one not yet taken."""

    def __init__(self) -> None:
        self._cond = threading.Condition()
        self._item = None
        self._closed = False

    def put(self, item) -> None:
        with self._cond:
            self._item = item
            self._cond.notify()

    def take(self):
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


class Forwarder(threading.Thread):
    """Sends the newest (x, y, positions) sample to Isaac Sim, one request at a time."""

    def __init__(self, sim: IsaacSim) -> None:
        super().__init__(daemon=True)
        self.sim, self.box = sim, Latest()
        self.sent = 0
        self.error: Exception | None = None

    def run(self) -> None:
        try:
            while (sample := self.box.take()) is not None:
                x, y, positions = sample
                self.sim.run(SYNC, timeout=5.0, echo=False, x=x, y=y, positions=positions)
                self.sent += 1
        except Exception as exc:  # reported by the camera loop
            self.error = exc


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--rig", action="store_true",
                    help="also read the real servo positions and hold the simulated axes there")
    ap.add_argument("--device", default=None, help="camera ID or device path (default: `ballbal camera set`)")
    ap.add_argument("--calibration", type=Path, default=None,
                    help="camera calibration (default: the active profile's camera.json)")
    ap.add_argument("--seconds", type=float, default=None, help="stop after this long")
    ap.add_argument("--no-window", action="store_true", help="open no camera window")
    ap.add_argument("--every", type=int, default=15, help="print one line every N frames (0: none)")
    args = ap.parse_args()
    if args.no_window and args.seconds is None:
        print("note: without a window stop with Ctrl+C")

    import cv2
    import numpy as np

    from ballbal.control.rig import Rig
    from ballbal.vision import (AXIS_1_BEARING_DEG, DEFAULT_SIZE, WARMUP_FRAMES, CameraCalibration,
                                find_ball_by_colour, find_camera, lock_camera, open_camera)

    path = args.calibration or active_profile_dir() / "camera.json"
    if not path.exists():
        print(f"no camera calibration at {path}; run `ballbal cam-setup`", file=sys.stderr)
        return 1
    cal = CameraCalibration.load(path)
    if not cal.has_colour:
        print("the camera calibration has no ball colour; run `ballbal cam-setup`", file=sys.stderr)
        return 1
    bearing = cal.bearing_deg if cal.bearing_deg is not None else AXIS_1_BEARING_DEG
    device = find_camera(args.device)

    sim = IsaacSim()
    try:
        sim.start()
        sim.run_file(PACKAGE / "isaac_ball.py", ball_mm=cal.ball_mm)
    except IsaacError as exc:
        print(exc, file=sys.stderr)
        return 1

    forwarder = Forwarder(sim)
    with contextlib.ExitStack() as stack:
        rig = None
        if args.rig:
            rig = stack.enter_context(Rig.open(RigConfig.load(active_profile_dir() / "rig.toml")))
        lock_camera(device)
        capture = open_camera(device, DEFAULT_SIZE, square=True,
                              offset=tuple(cal.crop_offset or (0, 0)), side=cal.crop_side)
        stack.callback(capture.release)
        ok, frame = capture.read()
        if not ok:
            print(f"{device} opened but returned no frame", file=sys.stderr)
            return 1
        height, width = frame.shape[:2]
        cal.check_size(width, height)
        centre = cal.centre
        roi = int(cal.platform_radius_px * 0.97)
        mask = np.zeros((height, width), np.uint8)
        cv2.circle(mask, centre, roi, 255, -1)
        for _ in range(WARMUP_FRAMES):
            capture.read()

        forwarder.start()
        stack.callback(forwarder.join, 5.0)
        stack.callback(forwarder.box.close)
        if not args.no_window:
            stack.callback(cv2.destroyAllWindows)
        print(f"{device}: {width}x{height}, axis_1 bearing {bearing:.0f} deg, "
              + ("mirroring ball and platform" if rig else "mirroring the ball") + "  (q quits)")
        started = time.perf_counter()
        frames = seen = 0
        try:
            while True:
                ok, frame = capture.read()
                if not ok:
                    print("camera stopped returning frames", file=sys.stderr)
                    return 1
                frames += 1
                elapsed = time.perf_counter() - started
                d = find_ball_by_colour(frame, hsv_lo=cal.hsv_lo, hsv_hi=cal.hsv_hi, mask=mask,
                                        min_radius=3.0, max_radius=width * 0.25)
                x = y = None
                if d is not None:
                    seen += 1
                    x, y = plate_mm((d.x - centre[0]) * cal.mm_per_px, (d.y - centre[1]) * cal.mm_per_px, bearing)
                positions = rig.positions() if rig else None
                forwarder.box.put((x, y, positions))
                if forwarder.error is not None:
                    raise forwarder.error

                rate = forwarder.sent / elapsed if elapsed > 0 else 0.0
                where = "no ball" if x is None else f"plate x {x:+6.1f} y {y:+6.1f} mm"
                if args.every and frames % args.every == 0:
                    print(f"  t={elapsed:6.1f}s  {where}   sim {rate:4.1f} updates/s"
                          + (f"   rig {sorted(positions.values())}" if positions else ""))
                if not args.no_window:
                    cv2.circle(frame, centre, roi, (80, 80, 80), 1)
                    cv2.drawMarker(frame, centre, (80, 80, 80), cv2.MARKER_CROSS, 12)
                    if d is not None:
                        cv2.circle(frame, (int(d.x), int(d.y)), int(d.radius), (0, 255, 0), 2)
                    cv2.putText(frame, where, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
                    cv2.putText(frame, f"sim {rate:.1f}/s", (6, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
                    cv2.imshow("ballbal -- ball sync", frame)
                    if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                        break
                if args.seconds is not None and elapsed >= args.seconds:
                    break
        except KeyboardInterrupt:
            pass
        except IsaacError as exc:
            print(exc, file=sys.stderr)
            return 1

    elapsed = time.perf_counter() - started
    print(f"\n{frames} frames in {elapsed:.1f} s, ball in {seen}; {forwarder.sent} sent to Isaac Sim "
          f"({forwarder.sent / max(elapsed, 1e-9):.1f}/s)")
    with contextlib.suppress(IsaacError):
        sim.run("ball.hide()", echo=False)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
