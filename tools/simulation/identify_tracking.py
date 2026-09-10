"""Record the real axes following balance-like goals (30 Hz, differential) for model validation.

    python tools/simulation/identify_tracking.py

A rotating tilt plus random jumps, +/-AMPLITUDE counts about home, offsets summing to zero,
written with the rig's own speed/acceleration exactly like ``ballbal balance``.
Output: runtime/measurements/servo_model/tracking_<timestamp>.csv.
"""

from __future__ import annotations

import csv
import math
import random
import time
from datetime import datetime
from pathlib import Path

from ballbal.config import RigConfig, active_profile_name
from ballbal.control.rig import Rig

PROFILE_NAME = active_profile_name()
RIG_FILE = Path(__file__).resolve().parents[2] / "profiles" / PROFILE_NAME / "rig.toml"
OUT_DIR = Path(__file__).resolve().parents[2] / "runtime" / "measurements" / "servo_model"
AMPLITUDE = 200
RATE_HZ = 30.0
DURATION_S = 12.0


def goals_at(t, home, jump):
    """Differential offsets: a tilt vector turning once per 2 s, plus a random tilt that changes every 0.5 s."""
    ang = 2 * math.pi * t / 2.0
    tx, ty = 0.6 * math.cos(ang) + jump[0], 0.6 * math.sin(ang) + jump[1]
    mag = math.hypot(tx, ty)
    if mag > 1:
        tx, ty = tx / mag, ty / mag
    return [round(h - AMPLITUDE * (tx * math.cos(b) + ty * math.sin(b))) for h, b in zip(home, (0.0, 2.0944, 4.1888))]


def main() -> int:
    cfg = RigConfig.load(RIG_FILE)
    servos = cfg.servos
    ids = [s.id for s in servos]
    home = [s.home_position for s in servos]
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out = OUT_DIR / f"tracking_{datetime.now():%Y%m%d_%H%M%S}.csv"
    rnd = random.Random(1)
    with Rig.open(cfg) as rig, out.open("w", newline="") as fh:
        rig.preflight(servos)
        w = csv.writer(fh)
        w.writerow(["t_s", "kind", "v1", "v2", "v3"])
        with rig.bus.torque(ids):
            rig.move_to(dict(zip(ids, home)), speed=1000)
            time.sleep(0.5)
            t0 = time.perf_counter()
            next_goal, jump, next_jump = 0.0, (0.0, 0.0), 0.0
            while (t := time.perf_counter() - t0) < DURATION_S:
                if t >= next_jump:
                    jump = (rnd.uniform(-0.5, 0.5), rnd.uniform(-0.5, 0.5))
                    next_jump += 0.5
                if t >= next_goal:
                    g = [s.clamp(v) for s, v in zip(servos, goals_at(t, home, jump))]
                    rig.goto(dict(zip(ids, g)))
                    w.writerow([f"{time.perf_counter() - t0:.5f}", "goal", *g])
                    next_goal += 1.0 / RATE_HZ
                pos = rig.bus.read_positions(ids)
                w.writerow([f"{time.perf_counter() - t0:.5f}", "pos", *(pos[i] for i in ids)])
            rig.move_to(dict(zip(ids, home)), speed=1000)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
