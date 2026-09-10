"""Drive the simulated platform between its lowest and highest position, then home.

    python tools/simulation/platform_sweep.py [--cycles 3] [--hold 1.5]
    python tools/simulation/platform_sweep.py --stop

The goals are the ones the real rig would get from ``ballbal goto``: all axes to min, then all to
max, HOLD seconds apart. They are queued in the driver's schedule in simulation time, so the
timing holds however fast Isaac Sim runs; how fast the platform gets there is the identified
servo behaviour, not a scripted trajectory. Ctrl+C or --stop cancels the rest.
"""

from __future__ import annotations

import argparse
import sys
import time

from ballbal.simulation.remote import IsaacError, IsaacSim


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--cycles", type=int, default=3, help="min/max round trips")
    ap.add_argument("--hold", type=float, default=1.5, help="simulation seconds between goals")
    ap.add_argument("--stop", action="store_true", help="cancel a running sweep")
    args = ap.parse_args()

    sim = IsaacSim()
    try:
        sim.start()
        if args.stop:
            sim.run("servos.schedule = []", echo=False)
            print("sweep cancelled")
            return 0
        limits = {int(k): v for k, v in sim.run("{i: (a.min, a.max, a.home) for i, a in servos.axes.items()}",
                                                echo=False).items()}
        t0 = sim.time() + 0.1
        legs = []
        for k in range(2 * args.cycles):
            legs.append((t0 + k * args.hold, {i: lim[k % 2] for i, lim in limits.items()}))
        legs.append((t0 + len(legs) * args.hold, {i: lim[2] for i, lim in limits.items()}))
        sim.run(f"servos.schedule = {legs!r}", echo=False)
        end = legs[-1][0] + args.hold
        print(f"sweep queued: {args.cycles} x min <-> max, {args.hold:.1f} s apart, then home")
        try:
            while (now := sim.time()) < end:
                if now < t0 - 0.1:
                    print("\ntimeline stopped; sweep cancelled")
                    return 1
                done = sum(t <= now for t, _ in legs)
                print(f"\r  sim {now - t0:6.2f} s  goal {done}/{len(legs)}  "
                      + "  ".join(f"{p:6.0f}" for _, p in sorted(sim.positions().items())), end="", flush=True)
                time.sleep(0.1)
        except KeyboardInterrupt:
            sim.run("servos.schedule = []", echo=False)
            print("\nsweep cancelled")
            return 130
        print()
    except IsaacError as exc:
        print(exc, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
