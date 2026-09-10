"""How old the ball's position is when `read()` hands a frame over, in the active camera mode.

    python tools/balance/camera_lag.py [--steps 8]

The plate is leaned until the ball rests against the guardrail, then all three legs step up
and down together by 150 counts. The plate rises without tilting, and the ball -- now closer to
the camera -- appears further out and larger: a pure perspective shift that moves with the
plate, no rolling involved. Axis 1's position is polled over the bus meanwhile. For each step the
times at which bus and image have covered 30, 50 and 70% of their move are compared; the
difference is the camera's lag for the ball's image row: exposure, readout, transport, decode.
"""

from __future__ import annotations

import argparse
import statistics
import threading
import time

import cv2
import numpy as np

from ballbal.config import RigConfig
from ballbal.control.balance import plan
from ballbal.control.rig import Rig
from ballbal.hardware import registers as reg
from ballbal.paths import active_profile_dir
from ballbal.vision import (AXIS_1_BEARING_DEG, CameraCalibration, find_ball_by_colour, find_camera,
                            lock_camera, open_camera)

LIFT = 150


def crossing(times: np.ndarray, values: np.ndarray, level: float) -> float:
    for i in range(1, len(values)):
        if values[i] >= level > values[i - 1] or values[i - 1] < level <= values[i]:
            return times[i - 1] + (level - values[i - 1]) / (values[i] - values[i - 1]) * (times[i] - times[i - 1])
    return float("nan")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--steps", type=int, default=8)
    args = ap.parse_args()
    cfg = RigConfig.load(active_profile_dir() / "rig.toml")
    cal = CameraCalibration.load(active_profile_dir() / "camera.json")
    kin = plan(cfg.servos, bearing_deg=cal.bearing_deg or AXIS_1_BEARING_DEG, max_tilt_pct=100.0)
    device = find_camera(None)
    lock_camera(device)
    capture = open_camera(device, cal.capture_size, square=True, offset=tuple(cal.crop_offset or (0, 0)),
                          side=cal.crop_side)
    ok, frame = capture.read()
    height, width = frame.shape[:2]
    seen: list[tuple[float, tuple[float, float] | None]] = []
    stop = threading.Event()

    def grab() -> None:
        while not stop.is_set():
            ok, image = capture.read()
            t = time.perf_counter()
            if ok:
                d = find_ball_by_colour(image, hsv_lo=cal.hsv_lo, hsv_hi=cal.hsv_hi, mask=None,
                                        min_radius=3.0, max_radius=width * 0.25)
                seen.append((t, None if d is None else (d.x, d.y)))

    reader = threading.Thread(target=grab, daemon=True)
    reader.start()
    lags = []
    servo = cfg.servos[0]
    try:
        with Rig.open(cfg) as rig:
            rig.preflight(cfg.servos)
            with rig.bus.torque([s.id for s in cfg.servos], hold=False):
                lean = kin.goals(-0.12, 0.0)
                rig.goto(lean, settle=2.5)
                for n in range(args.steps):
                    up = n % 2 == 0
                    goal = {i: lean[i] + (LIFT if up else 0) for i in lean}
                    start = rig.bus.read(servo.id, reg.PRESENT_POSITION)
                    time.sleep(0.5)
                    seen.clear()
                    time.sleep(0.3)
                    t0 = time.perf_counter()
                    rig.goto(goal, speed=1500, acceleration=100)
                    trace = []
                    while time.perf_counter() - t0 < 0.7:
                        trace.append((time.perf_counter(), rig.bus.read(servo.id, reg.PRESENT_POSITION)))
                    time.sleep(0.2)
                    points = [(t, p) for t, p in seen if p is not None]
                    before = [p for t, p in points if t < t0]
                    after = [p for t, p in points if t > t0 + 0.55]
                    if len(before) < 3 or len(after) < 3:
                        print(f"  step {n}: ball not seen")
                        continue
                    p0, p1 = np.mean(before, axis=0), np.mean(after, axis=0)
                    span = float(np.linalg.norm(p1 - p0))
                    if span < 2.0:
                        print(f"  step {n}: image moved only {span:.1f} px")
                        continue
                    unit = (p1 - p0) / span
                    ti = np.array([t for t, _ in points])
                    image = np.array([float(np.dot(np.array(p) - p0, unit)) / span for _, p in points])
                    tb = np.array([t for t, _ in trace])
                    bus = (np.array([p for _, p in trace], float) - start) / (goal[servo.id] - start)
                    lag = statistics.median(crossing(ti, image, f) - crossing(tb, bus, f) for f in (0.3, 0.5, 0.7))
                    lags.append(lag)
                    print(f"  step {n}: ball moved {span:.1f} px, image {lag * 1000:5.1f} ms behind the bus", flush=True)
                rig.goto(kin.level(), settle=0.6)
    finally:
        stop.set()
        reader.join(2.0)
        capture.release()
    lags = [x for x in lags if x == x]
    if not lags:
        return 1
    print(f"{cal.capture_size[0]}x{cal.capture_size[1]}: camera lag {statistics.median(lags) * 1000:.0f} ms "
          f"(median of {len(lags)}, spread {min(lags) * 1000:.0f}..{max(lags) * 1000:.0f} ms)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
