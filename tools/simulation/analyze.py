"""Fit a trapezoidal motion model to recorded STS3215 step responses.

    python tools/simulation/analyze.py runtime/measurements/servo_model/steps_<timestamp>.csv

Per step the measured position is fitted with

    dead time d  ->  accelerate at a_acc  ->  cruise at v_max  ->  decelerate at a_dec

(least squares over the moving part of the trace). Also reported: the dead time
as ``ballbal latency`` defines it (first sample more than 3 counts from the
start), overshoot and the final error. Finally one parameter set with an added
first-order lag is fitted across all operating-point steps -- that is the model
the simulation uses (params.json). Everything goes to fit.json.
"""

from __future__ import annotations

import csv
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

MOVED_COUNTS = 3  # same threshold as ballbal.latency.measure_axis


def trapezoid(t, d, a1, v, a2, dist):
    """Displacement of a trapezoidal (or triangular) profile covering ``dist``."""
    if dist <= v * v / (2 * a1) + v * v / (2 * a2):
        v = np.sqrt(2 * dist * a1 * a2 / (a1 + a2))
        t2 = 0.0
    else:
        t2 = (dist - v * v / (2 * a1) - v * v / (2 * a2)) / v
    t1, t3 = v / a1, v / a2
    s = np.clip(t - d, 0.0, None)
    x1 = 0.5 * a1 * np.minimum(s, t1) ** 2
    x2 = v * np.clip(s - t1, 0.0, t2)
    s3 = np.clip(s - t1 - t2, 0.0, t3)
    x3 = v * s3 - 0.5 * a2 * s3 ** 2
    return np.minimum(x1 + x2 + x3, dist)


def fit_step(t, p, start, target):
    sign = 1.0 if target >= start else -1.0
    dist = abs(target - start)
    x = sign * (p - start)  # displacement toward the target
    moved = np.nonzero(np.abs(p - start) > MOVED_COUNTS)[0]
    dead = float(t[moved[0]]) if len(moved) else float("nan")
    arrive = np.nonzero(x >= dist - MOVED_COUNTS)[0]
    t_end = float(t[arrive[0]]) + 0.05 if len(arrive) else float(t[-1])
    m = t <= t_end

    def resid(q):
        return trapezoid(t[m], q[0], q[1], q[2], q[3], dist) - x[m]

    sol = least_squares(resid, x0=[0.02, 8000.0, 2500.0, 8000.0],
                        bounds=([0.0, 500.0, 100.0, 500.0], [0.2, 2e5, 6000.0, 2e5]), x_scale=[0.01, 1e3, 1e3, 1e3])
    d, a1, v, a2 = sol.x
    cruising = dist > v * v / (2 * a1) + v * v / (2 * a2)
    settled = p[t > t[-1] - 0.2]
    return {
        "dist": dist, "dir": "up" if sign > 0 else "down",
        "dead_moved3_s": dead, "delay_s": d, "a_acc": a1, "v_max": v if cruising else float("nan"), "a_dec": a2,
        "peak_v": float(np.max(np.gradient(x[m], t[m]))) if m.sum() > 5 else float("nan"),
        "overshoot": float(np.max(x) - dist), "final_error": float(np.median(settled) - target),
        "move_time_s": float(t[arrive[0]]) if len(arrive) else float("nan"),
        "rms_fit": float(np.sqrt(np.mean(sol.fun ** 2))),
    }


def lagged(t, d, a1, v, a2, tau, dist, grid=5e-4):
    """Trapezoid followed by a first-order lag (the servo's position loop)."""
    tt = np.arange(0.0, t[-1] + grid, grid)
    x = trapezoid(tt, d, a1, v, a2, dist)
    if tau > 1e-4:
        k, y, acc = grid / tau, np.empty_like(x), 0.0
        for i, xi in enumerate(x):
            acc += (xi - acc) * k
            y[i] = acc
        x = y
    return np.interp(t, tt, x)


def global_fit(runs):
    """One parameter set (delay, accel, v_max, decel, lag) for every operating-point step."""
    steps = []
    for rows in runs.values():
        if int(rows[0]["speed"]) != 2650:
            continue
        t = np.array([float(r["t_s"]) for r in rows])
        for sid in (1, 2, 3):
            if not rows[0][f"pos_{sid}"]:
                continue
            s0, tg = int(rows[0][f"start_{sid}"]), int(rows[0][f"target_{sid}"])
            if abs(tg - s0) < 20:
                continue
            sign = 1 if tg > s0 else -1
            x = sign * (np.array([float(r[f"pos_{sid}"]) for r in rows]) - s0)
            sel = np.linspace(0, len(t) - 1, 400).astype(int)
            steps.append((t[sel], x[sel], abs(tg - s0)))

    def resid(q):
        return np.concatenate([lagged(t, q[0], q[1], q[2], q[3], q[4], dist) - x for t, x, dist in steps])

    sol = least_squares(resid, x0=[0.03, 9500, 2440, 9500, 0.02], bounds=([0, 1000, 500, 1000, 0], [0.15, 1e5, 5000, 1e5, 0.2]),
                        x_scale=[0.01, 1e3, 1e3, 1e3, 0.01])
    rms = float(np.sqrt(np.mean(sol.fun ** 2)))
    d, a1, v, a2, tau = (float(v) for v in sol.x)
    return {"delay_s": round(d, 4), "accel_counts_s2": round(a1), "decel_counts_s2": round(a2), "v_max_counts_s": round(v),
            "tau_s": round(tau, 4), "fit_rms_counts": round(rms, 2), "fit_rms_deg": round(rms * 360 / 4096, 3), "steps": len(steps)}


def main(path: Path) -> None:
    runs = defaultdict(list)
    for row in csv.DictReader(path.open()):
        runs[int(row["run"])].append(row)
    results = []
    for run, rows in sorted(runs.items()):
        kind = rows[0]["kind"]
        t = np.array([float(r["t_s"]) for r in rows])
        for sid in (1, 2, 3):
            if not rows[0][f"pos_{sid}"]:
                continue
            start, target = int(rows[0][f"start_{sid}"]), int(rows[0][f"target_{sid}"])
            if abs(target - start) < 20:
                continue
            p = np.array([float(r[f"pos_{sid}"]) for r in rows])
            r = fit_step(t, p, start, target)
            r.update(run=run, kind=kind, axis=sid, speed=int(rows[0]["speed"]), acc=int(rows[0]["acc"]))
            results.append(r)

    print(f"{'run':>3} {'kind':28} ax {'dir':4} {'dist':>5} {'dead3':>6} {'delay':>6} {'a_acc':>7} {'v_max':>6} {'a_dec':>7} "
          f"{'peak_v':>6} {'ovs':>4} {'err':>4} {'t_move':>6} {'rms':>4}")
    for r in results:
        print(f"{r['run']:3d} {r['kind']:28} {r['axis']:2d} {r['dir']:4} {r['dist']:5d} {r['dead_moved3_s']*1e3:5.1f}ms "
              f"{r['delay_s']*1e3:5.1f}ms {r['a_acc']:7.0f} {r['v_max']:6.0f} {r['a_dec']:7.0f} {r['peak_v']:6.0f} "
              f"{r['overshoot']:4.0f} {r['final_error']:4.0f} {r['move_time_s']*1e3:5.0f}ms {r['rms_fit']:4.1f}")

    def agg(sel, key):
        vals = np.array([r[key] for r in sel if np.isfinite(r[key])])
        return {"median": float(np.median(vals)), "min": float(vals.min()), "max": float(vals.max()), "n": int(len(vals))} if len(vals) else None

    summary = {}
    for label, cond in (("operating_speed2650_acc100", lambda r: r["speed"] == 2650 and r["kind"].startswith(("step", "return"))),
                        ("servo_max_speed0_acc0", lambda r: r["speed"] == 0),
                        ("differential_speed2650_acc100", lambda r: r["kind"].startswith("diff"))):
        sel = [r for r in results if cond(r)]
        summary[label] = {k: agg(sel, k) for k in ("dead_moved3_s", "delay_s", "a_acc", "v_max", "a_dec", "overshoot", "final_error")}
        summary[label]["by_direction"] = {
            d: {k: agg([r for r in sel if r["dir"] == d], k) for k in ("delay_s", "a_acc", "v_max", "a_dec")} for d in ("up", "down")
        }
    model = global_fit(runs)
    out = Path(__file__).resolve().parents[2] / "models" / "servo" / "fit.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({"source": path.name, "model": model, "summary": summary, "steps": results}, indent=2))
    print(f"\nglobal model (copy into params.json): {model}")
    print(f"wrote {out}")
    for label, s in summary.items():
        print(label, {k: (round(v['median'], 4) if isinstance(v, dict) and v and 'median' in v else None) for k, v in s.items() if k != 'by_direction'})


if __name__ == "__main__":
    main(Path(sys.argv[1]))
