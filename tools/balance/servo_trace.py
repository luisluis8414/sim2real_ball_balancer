"""Run `ballbal balance` and record what the servos really do, next to the ball.

    python tools/balance/servo_trace.py steps --out runtime/measurements/trace/base
    python tools/balance/servo_trace.py hold --seconds 20 --out runtime/measurements/trace/base_hold
    python tools/balance/servo_trace.py steps --kp 0.012 --out ...      # any balance flag

The balance log only has the goals the loop *sent*. This runs the same loop and, beside it, polls
every servo's present position, speed, load and supply voltage over the bus at ~300 Hz from a
second thread (bus access is serialised with a lock, a goal write waits at most one read). It
also timestamps every camera frame and every goal write on the same clock, so ball, command and
real leg motion can be laid on one time axis. Writes, next to the balance log ``OUT.csv``:

    OUT.servo.csv    t_s, then per axis pos/speed/load/volt   (bus poll)
    OUT.writes.csv   t_s, goal per axis, speed, acceleration    (every goal write)
    OUT.frames.csv   t_s of every frame the loop received       (row i <-> balance log row i)

All t_s share the balance log's zero. Guardrail on, ball on the plate: the platform moves.
"""

from __future__ import annotations

import argparse
import csv
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

from ballbal.control import balance as bal
from ballbal.hardware import registers as reg
from ballbal.hardware.vendor.scservo_sdk import COMM_SUCCESS, GroupSyncRead  # type: ignore

WARMUP_READS = 16
"""Frames `balance` reads before its first logged row: one to size the image, 15 to settle."""

BLOCK = (reg.PRESENT_POSITION.address, 8)
"""Position, speed, load, voltage, temperature: one contiguous block from address 56."""


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


def signed(value: int, bit: int) -> int:
    return -(value & ((1 << bit) - 1)) if value & (1 << bit) else value


class Recorder:
    def __init__(self, rig, ids: list[int]) -> None:  # noqa: ANN001
        self.bus = rig.bus
        self.ids = ids
        # Reentrant: set_torque and telemetry call read/write, which are guarded too.
        self.lock = threading.RLock()
        self.samples: list[tuple] = []
        self.writes: list[tuple] = []
        self.frames: list[float] = []
        self.stop = threading.Event()
        self._reader = GroupSyncRead(self.bus._sdk, *BLOCK)
        for name in ("write_goals", "read_positions", "set_torque", "read", "write", "telemetry"):
            self._guard(name)
        original = self.bus.write_goals

        def write_goals(goals, speed, acceleration):  # noqa: ANN001, ANN202
            original(goals, speed, acceleration)
            self.writes.append((time.perf_counter(), dict(goals), speed, acceleration))

        self.bus.write_goals = write_goals

    def _guard(self, name: str) -> None:
        method = getattr(self.bus, name)

        def locked(*args, **kwargs):  # noqa: ANN002, ANN003, ANN202
            with self.lock:
                return method(*args, **kwargs)

        setattr(self.bus, name, locked)

    def poll(self) -> None:
        r = self._reader
        while not self.stop.is_set():
            with self.lock:
                r.clearParam()
                for sid in self.ids:
                    r.addParam(sid)
                ok = r.txRxPacket() == COMM_SUCCESS
                t = time.perf_counter()
                if ok and all(r.isAvailable(sid, BLOCK[0], BLOCK[1])[0] for sid in self.ids):
                    row = [t]
                    for sid in self.ids:
                        row += [
                            r.getData(sid, 56, 2),
                            signed(r.getData(sid, 58, 2), 15),
                            signed(r.getData(sid, 60, 2), 10),
                            r.getData(sid, 62, 1) / 10.0,
                        ]
                    self.samples.append(tuple(row))
                else:
                    self.bus._handler.ser.reset_input_buffer()
            time.sleep(0.002)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("mode", choices=("steps", "hold"))
    for flag in ("--kp", "--ki", "--kd", "--max-tilt", "--smoothing", "--izone", "--predict",
                 "--plant-gain", "--aggression", "--shape"):
        ap.add_argument(flag, type=float, default=None)
    for flag in ("--accel", "--speed", "--quiet"):
        ap.add_argument(flag, type=int, default=None)
    ap.add_argument("--size", type=float, default=40.0, help="steps: mm; jumps are twice this")
    ap.add_argument("--dwell", type=float, default=2.5, help="steps: seconds per target")
    ap.add_argument("--cycles", type=int, default=3, help="steps: rounds of four jumps")
    ap.add_argument("--seconds", type=float, default=20.0, help="hold: duration")
    ap.add_argument("--out", type=Path, required=True, help="path prefix, no extension")
    args = ap.parse_args()

    import ballbal.vision as vision
    from ballbal.cli import _balance_settings
    from ballbal.config import RigConfig
    from ballbal.control.rig import Rig
    from ballbal.paths import active_profile_dir
    from ballbal.vision import AXIS_1_BEARING_DEG, CameraCalibration, find_camera

    cfg = RigConfig.load(active_profile_dir() / "rig.toml")
    cal = CameraCalibration.load(active_profile_dir() / "camera.json")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    log = args.out.with_suffix(".csv")

    if args.mode == "steps":
        path = Jumps(period=4 * args.dwell, size=args.size, dwell=args.dwell)
        seconds = args.dwell * (4 * args.cycles + 1) + 0.2
    else:
        path, seconds = None, args.seconds

    frames: list[float] = []
    open_camera = vision.open_camera

    class Timed:
        def __init__(self, capture) -> None:  # noqa: ANN001
            self.capture = capture

        def read(self):  # noqa: ANN202
            result = self.capture.read()
            frames.append(time.perf_counter())
            return result

        def __getattr__(self, name: str):  # noqa: ANN204
            return getattr(self.capture, name)

    vision.open_camera = lambda *a, **k: Timed(open_camera(*a, **k))

    with Rig.open(cfg) as rig:
        rig.preflight(cfg.servos)
        ids = [s.id for s in cfg.servos]
        rec = Recorder(rig, ids)
        poller = threading.Thread(target=rec.poll, daemon=True)
        poller.start()
        try:
            bal.balance(
                rig, cfg.servos, calibration=cal, device=find_camera(None), size=cal.capture_size,
                bearing_deg=cal.bearing_deg if cal.bearing_deg is not None else AXIS_1_BEARING_DEG,
                path=path, seconds=seconds,
                offset=tuple(cal.crop_offset or (0, 0)), side=cal.crop_side,
                dry_run=False, show=False, log_path=log,
                **_balance_settings(args, cfg),
            )
        finally:
            rec.stop.set()
            poller.join(timeout=1.0)

    rows = list(csv.DictReader(log.open()))
    zero = frames[WARMUP_READS] - float(rows[0]["t_s"])
    with args.out.with_suffix(".servo.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s"] + [f"{k}_{i}" for i in ids for k in ("pos", "speed", "load", "volt")])
        for s in rec.samples:
            w.writerow([f"{s[0] - zero:.5f}", *s[1:]])
    with args.out.with_suffix(".writes.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s"] + [f"goal_{i}" for i in ids] + ["speed", "acceleration"])
        for t, goals, speed, acc in rec.writes:
            w.writerow([f"{t - zero:.5f}", *[goals.get(i, "") for i in ids], speed, acc])
    with args.out.with_suffix(".frames.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s"])
        for t in frames[WARMUP_READS:]:
            w.writerow([f"{t - zero:.5f}"])
    span = rec.samples[-1][0] - rec.samples[0][0] if rec.samples else 0.0
    print(f"  {len(rec.samples)} servo samples ({len(rec.samples) / max(span, 1e-9):.0f} Hz), "
          f"{len(rec.writes)} goal writes, {len(frames) - WARMUP_READS} frames -> {args.out}.*")
    return 0


if __name__ == "__main__":
    sys.exit(main())
