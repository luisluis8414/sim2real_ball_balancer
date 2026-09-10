"""Time one axis's answer to small steps at several acceleration and speed settings.

    python tools/balance/servo_steps.py [--axis 1] [--out runtime/measurements/balance/servo_steps.csv]

Steps of 20, 60 and 120 counts, both ways, around home, polling the position over the bus
(~0.6 ms per read). Reports the time to the first 3 counts of motion ("dead"), to half and to
90% of the step. Balancing lives on small steps, so the 20-count row is the one that decides.
Only one axis moves at a time; the ball may stay on the plate.
"""

from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path

from ballbal.config import RigConfig
from ballbal.control.rig import Rig
from ballbal.hardware import registers as reg
from ballbal.paths import active_profile_dir

SPEEDS = (600, 1500, 3400)
ACCELERATIONS = (30, 100, 254)
STEPS = (20, 60, 120)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--axis", type=int, default=1, help="servo id")
    ap.add_argument("--out", type=Path, default=Path("runtime/measurements/balance/servo_steps.csv"))
    args = ap.parse_args()
    cfg = RigConfig.load(active_profile_dir() / "rig.toml")
    servo = cfg.by_id(args.axis)
    rows = []
    with Rig.open(cfg) as rig:
        rig.preflight(cfg.servos)
        with rig.bus.torque([s.id for s in cfg.servos], hold=False):
            rig.goto({s.id: s.home_position for s in cfg.servos}, settle=1.0)
            for speed in SPEEDS:
                for acceleration in ACCELERATIONS:
                    for size in STEPS:
                        for direction in (1, -1):
                            start = rig.bus.read(servo.id, reg.PRESENT_POSITION)
                            target = servo.clamp(start + direction * size)
                            step = target - start
                            trace = []
                            t0 = time.perf_counter()
                            rig.goto({servo.id: target}, speed=speed, acceleration=acceleration)
                            while time.perf_counter() - t0 < 0.6:
                                trace.append((time.perf_counter() - t0,
                                              rig.bus.read(servo.id, reg.PRESENT_POSITION)))

                            def reached(fraction: float) -> float:
                                return next((t for t, p in trace
                                             if (p - start) * step >= fraction * step * step), float("nan"))

                            dead = next((t for t, p in trace if abs(p - start) > 3), float("nan"))
                            rows.append(dict(speed=speed, acceleration=acceleration, step=step,
                                             dead_ms=round(dead * 1000, 1), half_ms=round(reached(0.5) * 1000, 1),
                                             ninety_ms=round(reached(0.9) * 1000, 1)))
                            r = rows[-1]
                            print(f"speed {speed:4} acc {acceleration:3} step {step:+4}: first motion "
                                  f"{r['dead_ms']:5.1f} ms, half {r['half_ms']:6.1f} ms, 90% {r['ninety_ms']:6.1f} ms",
                                  flush=True)
                            time.sleep(0.15)
            rig.goto({s.id: s.home_position for s in cfg.servos}, settle=0.8)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
