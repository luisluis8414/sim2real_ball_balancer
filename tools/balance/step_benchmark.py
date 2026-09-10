"""Score `ballbal balance` on target jumps: rise time, overshoot, settling time.

    python tools/balance/step_benchmark.py --log runtime/measurements/balance/run.csv
    python tools/balance/step_benchmark.py --kp 0.02 --predict 60 --log run.csv   # try a setting
    python tools/balance/step_benchmark.py --score-only --log run.csv             # re-score a log

The target jumps between (+A,0), (-A,0), (0,+A), (0,-A) every DWELL seconds and the real balance
loop follows it, live, with the active profile's [balance] settings unless flags override them.
Each jump is scored: time to cover 90% of it, overshoot past the target, time from which the
ball stays within 5 mm, and the error over the last 0.6 s. Runs scatter a little -- compare
medians of two runs of 12 jumps each (``--cycles 3``), not single jumps.

Guardrail on, ball on the plate: the platform moves.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

from ballbal.control import balance as bal


@dataclass(frozen=True)
class Jumps(bal.Path):
    size: float = 40.0
    dwell: float = 2.5

    def at(self, elapsed: float) -> tuple[float, float]:
        a = self.size
        return [(a, 0.0), (-a, 0.0), (0.0, a), (0.0, -a)][int(elapsed // self.dwell) % 4]

    @property
    def speed(self) -> float:
        return 0.0


def score(log: Path, dwell: float, band: float = 5.0) -> list[dict]:
    rows = [r for r in csv.DictReader(log.open()) if r["found"] == "1"]
    jumps = []
    for k in range(1, 1000):
        t0 = k * dwell
        seg = [r for r in rows if t0 <= float(r["t_s"]) < t0 + dwell]
        before = [r for r in rows if t0 - 0.2 <= float(r["t_s"]) < t0]
        if len(seg) < 20 or not before:
            break
        tx, ty = float(seg[0]["target_x"]), float(seg[0]["target_y"])
        px, py = float(before[-1]["target_x"]), float(before[-1]["target_y"])
        dist = math.hypot(tx - px, ty - py)
        ux, uy = (tx - px) / dist, (ty - py) / dist
        prog = [(float(r["t_s"]) - t0,
                 (float(r["x_mm"]) - px) * ux + (float(r["y_mm"]) - py) * uy,
                 math.hypot(float(r["x_mm"]) - tx, float(r["y_mm"]) - ty)) for r in seg]
        rise = next((t for t, s, _ in prog if s >= 0.9 * dist), math.nan)
        settle = next((prog[i][0] for i in range(len(prog)) if max(e for *_, e in prog[i:]) <= band),
                      math.nan)
        jumps.append(dict(rise=rise, over=max(s for _, s, _ in prog) - dist, settle=settle,
                          final=statistics.median(e for t, _, e in prog if t > dwell - 0.6)))
    return jumps


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    for flag in ("--kp", "--ki", "--kd", "--max-tilt", "--smoothing", "--izone", "--predict",
                 "--plant-gain", "--aggression", "--shape"):
        ap.add_argument(flag, type=float, default=None)
    for flag in ("--accel", "--speed", "--quiet"):
        ap.add_argument(flag, type=int, default=None)
    ap.add_argument("--size", type=float, default=40.0, help="mm; jumps are twice this")
    ap.add_argument("--dwell", type=float, default=2.5, help="seconds per target")
    ap.add_argument("--cycles", type=int, default=3, help="rounds of four jumps")
    ap.add_argument("--log", type=Path, required=True)
    ap.add_argument("--score-only", action="store_true")
    args = ap.parse_args()

    if not args.score_only:
        from ballbal.cli import _balance_settings
        from ballbal.config import RigConfig
        from ballbal.control.rig import Rig
        from ballbal.paths import active_profile_dir
        from ballbal.vision import AXIS_1_BEARING_DEG, CameraCalibration, find_camera

        cfg = RigConfig.load(active_profile_dir() / "rig.toml")
        cal = CameraCalibration.load(active_profile_dir() / "camera.json")
        args.log.parent.mkdir(parents=True, exist_ok=True)
        with Rig.open(cfg) as rig:
            rig.preflight(cfg.servos)
            bal.balance(
                rig, cfg.servos, calibration=cal, device=find_camera(None), size=cal.capture_size,
                bearing_deg=cal.bearing_deg if cal.bearing_deg is not None else AXIS_1_BEARING_DEG,
                path=Jumps(period=4 * args.dwell, size=args.size, dwell=args.dwell),
                seconds=args.dwell * (4 * args.cycles + 1) + 0.2,
                offset=tuple(cal.crop_offset or (0, 0)), side=cal.crop_side,
                dry_run=False, show=False, log_path=args.log,
                **_balance_settings(args, cfg),
            )

    jumps = score(args.log, args.dwell)
    if not jumps:
        print("no scorable jumps")
        return 1

    def median(key: str) -> float:
        values = [j[key] for j in jumps if not math.isnan(j[key])]
        return statistics.median(values) if values else math.nan

    settled = sum(not math.isnan(j["settle"]) for j in jumps)
    print(f"{args.log.name}: {len(jumps)} jumps of {2 * args.size:.0f} mm | 90% after {median('rise'):.2f} s"
          f" | overshoot {median('over'):.1f} mm | within 5 mm from {median('settle'):.2f} s"
          f" ({settled}/{len(jumps)}) | final error {median('final'):.1f} mm")
    return 0


if __name__ == "__main__":
    sys.exit(main())
