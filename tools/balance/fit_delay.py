"""Loop dead time and plant gain from balance logs, to the millisecond.

    python tools/balance/fit_delay.py LOG.csv [LOG2.csv ...] [--reach 250]

`ballbal tune` fits the same law but shifts the command by whole frames, so its dead time
comes out in 34 ms steps (102, 135, ...). Here the command is replayed as the plate held it --
each lean from the moment it was written until the next -- and shifted by a continuous delay:

    dv = G * integral of tilt(t - T) dt     over 8-frame windows, both axes as one complex gain

The delay that explains the most velocity change is the whole loop's dead time as the loop
sees it: servo, plate, exposure, readout and frame timing. ``--reach`` (counts of leg swing at
full tilt, printed by `balance`) turns the gain into mm/s^2 per count.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np


def fit(path: Path) -> tuple[float, complex, float]:
    rows = list(csv.DictReader(path.open()))
    seen = [r for r in rows if r["found"] == "1"]
    t = np.array([float(r["t_s"]) for r in seen])
    x = np.array([float(r["x_mm"]) for r in seen])
    y = np.array([float(r["y_mm"]) for r in seen])
    written = np.array([float(r["t_s"]) for r in rows])
    tilt = np.array([complex(float(r["tilt_x"]), float(r["tilt_y"])) for r in rows])

    def velocity(p: np.ndarray) -> np.ndarray:
        v = np.full(len(p), np.nan)
        for i in range(3, len(p) - 3):
            v[i] = np.polyfit(t[i - 3:i + 4], p[i - 3:i + 4], 1)[0]
        return v

    vx, vy = velocity(x), velocity(y)
    radius = np.hypot(x, y)
    rim = np.quantile(radius, 0.97) * 0.85  # the guardrail is not part of the plant
    best = (-math.inf, 0.0, 0j)
    for delay in np.arange(0.04, 0.20, 0.002):
        area, change = [], []
        for i in range(3, len(t) - 12):
            j = i + 8
            if np.isnan(vx[i]) or np.isnan(vx[j]) or radius[i:j + 1].max() > rim:
                continue
            ts = np.linspace(t[i], t[j], 40)
            held = np.clip(np.searchsorted(written, ts - delay, side="right") - 1, 0, len(rows) - 1)
            area.append(tilt[held].sum() * (t[j] - t[i]) / 40)
            change.append(complex(vx[j] - vx[i], vy[j] - vy[i]))
        a, d = np.array(area), np.array(change)
        gain = np.vdot(a, d) / np.vdot(a, a)
        explained = 1 - np.sum(abs(d - gain * a) ** 2) / np.sum(abs(d) ** 2)
        if explained > best[0]:
            best = (explained, delay, gain)
    explained, delay, gain = best
    return delay, gain, explained


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("logs", type=Path, nargs="+")
    ap.add_argument("--reach", type=float, default=None, help="counts per unit of tilt")
    args = ap.parse_args()
    for log in args.logs:
        delay, gain, explained = fit(log)
        per_count = f", {abs(gain) / args.reach:.2f} mm/s^2 per count" if args.reach else ""
        print(f"{log.name}: dead time {delay * 1000:.0f} ms, gain {abs(gain):.0f} mm/s^2 per unit tilt"
              f"{per_count}, rotation {math.degrees(np.angle(gain)):+.1f} deg, explains {explained * 100:.0f}%")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
