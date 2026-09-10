"""Drive the real platform and the Isaac Sim platform through the same goals, side by side.

    python tools/simulation/compare_real.py [--speed 1000] [--hold 1.5]

Needs Isaac Sim with ``models/usd/scene.usda`` open and the Python Server extension
(port 8226). Every leg -- home, min, max, home -- is written to both at the same moment:
to the rig through ballbal (``Rig.move_to``, with its stall detection) and to the simulated
STS3215 models (``src/ballbal/simulation/isaac.py``) with the same speed. Both are recorded; a table compares
where each axis ended up and how long it took. Take the ball off the plate first.

All three axes move together, so the speed defaults to 1000 counts/s: the rig's supply
browns out when three axes accelerate at full speed (ballbal's own sweep uses 1500).
"""

from __future__ import annotations

import argparse
import csv
import json
import socket
import time
from datetime import datetime
from pathlib import Path

from ballbal.hardware.bus import BusError
from ballbal.config import RigConfig, active_profile_name
from ballbal.control.rig import Rig

PROFILE_NAME = active_profile_name()
REPO = Path(__file__).resolve().parents[2]
RIG_FILE = REPO / "profiles" / PROFILE_NAME / "rig.toml"
ISAAC = ("127.0.0.1", 8226)
SERVOS_PY = REPO / "src" / "ballbal" / "simulation" / "isaac.py"
OUT_DIR = REPO / "runtime" / "measurements" / "servo_model"
ARRIVED = 8  # counts, rig.toml tolerance


def isaac(code: str, timeout: float = 30.0):
    """Run code in Isaac Sim's python_server. Returns a value only when ``code`` is a single expression."""
    envelope = {"code": code, "context": "sim_vs_real", "timeout": timeout}
    with socket.create_connection(ISAAC, timeout=timeout + 5) as s:
        s.sendall(json.dumps(envelope).encode())
        s.shutdown(socket.SHUT_WR)
        buf = b""
        while chunk := s.recv(1 << 16):
            buf += chunk
    reply = json.loads(buf.decode())
    if reply.get("status") != "ok":
        raise RuntimeError("Isaac: " + "".join(reply.get("traceback") or [str(reply.get("evalue"))]))
    return reply.get("result")


SETUP = f"""
import builtins, omni.kit.app, omni.physx, omni.timeline
exec(open({str(SERVOS_PY)!r}).read())
servos = builtins._sim_servos
_tl = omni.timeline.get_timeline_interface()
_tl.stop()
await omni.kit.app.get_app().next_update_async()
_tl.play()
for _ in range(5):
    await omni.kit.app.get_app().next_update_async()
builtins._svr_log = []
def _svr_rec(dt):
    p = servos.positions()
    builtins._svr_log.append((servos.time, p[1], p[2], p[3]))
builtins._svr_sub = omni.physx.get_physx_interface().subscribe_physics_step_events(_svr_rec)
"""


def arrival(samples, goal, t_from=0.0):
    """First time from which every axis stays within ARRIVED counts of its goal."""
    for k, (t, pos) in enumerate(samples):
        if t >= t_from and all(all(abs(p[i] - goal[i]) <= ARRIVED for i in goal) for _, p in samples[k:]):
            return t
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--speed", type=int, default=1000, help="goal speed in counts/s for both")
    ap.add_argument("--hold", type=float, default=1.5, help="seconds to wait after each leg")
    args = ap.parse_args()

    cfg = RigConfig.load(RIG_FILE)
    ids = [s.id for s in cfg.servos]
    legs = [("home", {s.id: s.home_position for s in cfg.servos}),
            ("min", {s.id: s.min_position for s in cfg.servos}),
            ("max", {s.id: s.max_position for s in cfg.servos}),
            ("home", {s.id: s.home_position for s in cfg.servos})]

    isaac(SETUP, timeout=60)
    if not isaac("_tl.is_playing()"):
        raise RuntimeError("Isaac timeline did not start")
    real_log: list[tuple[float, str, dict]] = []  # (seconds since this leg's goal, leg, positions)
    sim_start: list[float] = []
    notes: dict[int, str] = {}

    with Rig.open(cfg) as rig:
        rig.preflight(cfg.servos)
        with rig.bus.torque(ids):
            for n, (name, goal) in enumerate(legs):
                sim_start.append(isaac(f"servos.goto({goal!r}, speed={args.speed})"))
                t0 = time.perf_counter()
                tick = lambda pos, n=n, t0=t0: real_log.append((time.perf_counter() - t0, n, dict(pos)))
                try:
                    rig.move_to(goal, speed=args.speed, on_tick=tick)
                except BusError as exc:
                    notes[n] = str(exc).split(". Goals")[0]
                while (t := time.perf_counter() - t0) < args.hold + (1.0 if n == 0 else 0.0):
                    real_log.append((t, n, rig.positions()))
                    time.sleep(0.02)
                # the simulation usually runs slower than real time: let it cover the same duration
                # before both get the next goal
                while (sim_t := isaac("servos.time") - sim_start[-1]) < t:
                    time.sleep(0.05)
                print(f"leg {n} {name}: done" + (f"  ({notes[n]})" if n in notes else "")
                      + f"   sim took {time.perf_counter() - t0:.1f} s wall for {sim_t:.1f} s sim time")

    isaac("builtins._svr_sub = None")
    sim = isaac("[list(r) for r in builtins._svr_log]", timeout=60)
    # split the sim trace into legs by the sim time each goal was written
    sim_legs = {n: [] for n in range(len(legs))}
    for t, *p in sim:
        n = max((k for k, t0 in enumerate(sim_start) if t >= t0), default=None)
        if n is not None:
            sim_legs[n].append((t - sim_start[n], dict(zip(ids, p))))
    real_legs = {n: [(t, p) for t, k, p in real_log if k == n] for n in range(len(legs))}

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out = OUT_DIR / f"sim_vs_real_{datetime.now():%Y%m%d_%H%M%S}.csv"
    with out.open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["source", "leg", "t_s", "axis_1", "axis_2", "axis_3"])
        for src, data in (("real", real_legs), ("sim", sim_legs)):
            for n, samples in data.items():
                for t, p in samples:
                    w.writerow([src, legs[n][0], f"{t:.4f}", *(round(p[i], 1) for i in ids)])

    print(f"\n speed {args.speed} counts/s   (axis values in counts; 'end' = last sample of the leg)")
    print(f" {'leg':5} {'axis':6} {'goal':>5} {'real end':>9} {'sim end':>8} {'sim-real':>8}   {'real arrives':>12} {'sim arrives':>11}")
    for n, (name, goal) in enumerate(legs):
        r_end, s_end = real_legs[n][-1][1], sim_legs[n][-1][1] if sim_legs[n] else {}
        ra, sa = arrival(real_legs[n], goal), arrival(sim_legs[n], goal)
        for k, i in enumerate(ids):
            diff = s_end.get(i, float("nan")) - r_end[i]
            print(f" {name if k == 0 else '':5} axis_{i} {goal[i]:5d} {r_end[i]:9.0f} {s_end.get(i, float('nan')):8.0f} {diff:+8.0f}   "
                  + (f"{ra * 1000:10.0f}ms" if (k == 0 and ra is not None) else ("   not within 8" if k == 0 else " " * 12))
                  + (f" {sa * 1000:9.0f}ms" if (k == 0 and sa is not None) else ("  not within 8" if k == 0 else "")))
        if n in notes:
            print(f"        rig: {notes[n]}")
    print(f"\nwrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
