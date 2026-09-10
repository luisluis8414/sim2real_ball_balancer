"""Record step responses of the real STS3215 axes for system identification.

Run with the live system's environment (it uses the ``ballbal`` package):

    python tools/simulation/identify.py

Every move starts from each axis's ``home_position`` and stays inside the
min/max travel from rig.toml (Rig.goto clamps anyway). Single-axis steps run with
only that axis energised, as ``ballbal sweep`` does; the differential steps
energise all three and keep the three offsets summing to zero, the way the
balance loop leans the plate. Take the ball off the plate first.

Output: ``runtime/measurements/servo_model/steps_<timestamp>.csv``, one row per position sample.
"""

from __future__ import annotations

import argparse
import csv
import time
from datetime import datetime
from pathlib import Path

from ballbal.hardware import registers as reg
from ballbal.config import RigConfig, active_profile_name
from ballbal.control.rig import Rig

PROFILE_NAME = active_profile_name()
RIG_FILE = Path(__file__).resolve().parents[2] / "profiles" / PROFILE_NAME / "rig.toml"
OUT_DIR = Path(__file__).resolve().parents[2] / "runtime" / "measurements" / "servo_model"

# (offset from home in counts, speed, acceleration); every step is followed by the return to home
SINGLE_STEPS = [
    (+50, 2650, 100), (+150, 2650, 100), (+400, 2650, 100), (+1000, 2650, 100),
    (-150, 2650, 100), (-300, 2650, 100),
    (+400, 0, 0), (+1000, 0, 0),  # 0 = the servo's own maximum
]
# per-axis offsets (axis_1, axis_2, axis_3), summing to zero
DIFFERENTIAL_STEPS = [(+120, -60, -60), (-60, +120, -60), (+240, -120, -120)]

RECORD_S = 1.2  # per move; longest move (1000 counts at 2650/s) takes ~0.65 s
SETTLE_S = 0.4


def record(rig, writer, run, kind, ids, targets, speed, acc):
    """Write one goal and poll positions as fast as the bus allows."""
    start = rig.bus.read_positions(ids)
    t0 = time.perf_counter()
    rig.goto(targets, speed=speed, acceleration=acc)
    while (t := time.perf_counter() - t0) < RECORD_S:
        pos = rig.bus.read_positions(ids) if len(ids) > 1 else {ids[0]: rig.bus.read(ids[0], reg.PRESENT_POSITION)}
        writer.writerow([run, kind, speed, acc, f"{t:.5f}"] + [
            v for sid in (1, 2, 3) for v in ((start[sid], targets[sid], pos[sid]) if sid in ids else ("", "", ""))
        ])
    time.sleep(SETTLE_S)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--yes", action="store_true", help="do not ask before moving")
    args = parser.parse_args()

    cfg = RigConfig.load(RIG_FILE)
    servos = cfg.servos
    home = {s.id: s.home_position for s in servos}
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out = OUT_DIR / f"steps_{datetime.now():%Y%m%d_%H%M%S}.csv"
    if not args.yes and input(f"Platform will move (ball off the plate?). Log to {out}. Go? [y/N] ").lower() != "y":
        return 1

    with Rig.open(cfg) as rig, out.open("w", newline="") as fh:
        rig.preflight(servos)
        writer = csv.writer(fh)
        writer.writerow(["run", "kind", "speed", "acc", "t_s"] + [
            f"{c}_{sid}" for sid in (1, 2, 3) for c in ("start", "target", "pos")
        ])
        run = 0
        for servo in servos:
            with rig.bus.torque([servo.id]):
                rig.move_to({servo.id: home[servo.id]}, speed=1000)
                time.sleep(SETTLE_S)
                for offset, speed, acc in SINGLE_STEPS:
                    for kind, target in (("step", home[servo.id] + offset), ("return", home[servo.id])):
                        run += 1
                        record(rig, writer, run, f"{kind}:{servo.name}:{offset:+d}", [servo.id],
                               {servo.id: servo.clamp(target)}, speed, acc)
                print(f"{servo.name} done")
        ids = [s.id for s in servos]
        with rig.bus.torque(ids):
            rig.move_to(home, speed=1000)
            time.sleep(SETTLE_S)
            for offsets in DIFFERENTIAL_STEPS:
                tag = "/".join(f"{o:+d}" for o in offsets)
                for kind, goal in (("diff", {s.id: s.clamp(home[s.id] + o) for s, o in zip(servos, offsets)}),
                                   ("diff_return", home)):
                    run += 1
                    record(rig, writer, run, f"{kind}:{tag}", ids, goal, 2650, 100)
            rig.move_to(home, speed=1000)
        print("differential done")
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
