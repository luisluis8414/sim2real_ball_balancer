"""Control the running Isaac Sim platform from the repository.

    python tools/simulation/sim.py start                 # open scene, arm servos, press Play
    python tools/simulation/sim.py status
    python tools/simulation/sim.py home [--speed 1000]
    python tools/simulation/sim.py goto 1=1800 axis_2=1300 [--speed 1000]
    python tools/simulation/sim.py positions
    python tools/simulation/sim.py play | pause | stop
    python tools/simulation/sim.py arm                   # re-read profile and driver code
    python tools/simulation/sim.py exec "servos.time"    # any Python inside Isaac Sim
    python tools/simulation/sim.py run some_isaac_script.py

Needs Isaac Sim launched with its Python Server (docs/simulation.md):

    isaacsim isaacsim.exp.full --enable isaacsim.code_editor.python_server
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from ballbal.simulation.remote import IsaacError, IsaacSim


def _goals(sim: IsaacSim, items: list[str]) -> dict[int, int]:
    names = {v: int(k) for k, v in sim.run("{i: a.name for i, a in servos.axes.items()}", echo=False).items()}
    goals = {}
    for item in items:
        key, sep, value = item.partition("=")
        if not sep or not value.lstrip("-").isdigit():
            raise SystemExit(f"goal {item!r}: expected ID=COUNTS or NAME=COUNTS, e.g. 1=1800 or axis_1=1800")
        if key.isdigit() and int(key) in names.values():
            goals[int(key)] = int(value)
        elif key in names:
            goals[names[key]] = int(value)
        else:
            raise SystemExit(f"unknown axis {key!r}; axes: {', '.join(f'{i}={n}' for n, i in names.items())}")
    return goals


def _print_positions(sim: IsaacSim) -> None:
    names = sim.run("{i: a.name for i, a in servos.axes.items()}", echo=False)
    for i, counts in sorted(sim.positions().items()):
        print(f"  {names[str(i)]:8} {counts:7.1f}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="command", required=True)
    sub.add_parser("status", help="server, scene, timeline and servo state")
    sub.add_parser("start", help="open the scene, arm the servos and press Play")
    sub.add_parser("arm", help="(re-)arm the servos: re-reads the profile and the driver code")
    sub.add_parser("play")
    sub.add_parser("pause")
    sub.add_parser("stop", help="stop the timeline; the platform returns to the stage pose")
    home = sub.add_parser("home", help="drive every axis to the profile's home")
    home.add_argument("--speed", type=int, default=None, help="counts/s, default rig.toml's speed")
    goto = sub.add_parser("goto", help="drive axes to goal counts, like `ballbal goto`")
    goto.add_argument("goals", nargs="+", help="ID=COUNTS or NAME=COUNTS")
    goto.add_argument("--speed", type=int, default=None, help="counts/s, default rig.toml's speed")
    sub.add_parser("positions", help="simulated joint positions in counts")
    exe = sub.add_parser("exec", help="run Python inside Isaac Sim (context 'ballbal'), print the value")
    exe.add_argument("code")
    run = sub.add_parser("run", help="run an Isaac-side Python file inside Isaac Sim")
    run.add_argument("file", type=Path)
    args = ap.parse_args()

    sim = IsaacSim()
    try:
        if args.command == "status":
            if not sim.reachable():
                sim.run("1")  # raises with the launch hint
            print(f"Isaac Sim  {sim.host}:{sim.port}")
            print(f"scene      {sim.scene() or '(none)'}")
            print(f"timeline   {sim.state()}")
            if not sim.armed():
                print("servos     not armed (python tools/simulation/sim.py start)")
                return 0
            print(f"servos     armed from {sim.run('str(servos.rig.source)', echo=False)}")
            if sim.state() != "stopped":
                print(f"sim time   {sim.time():.2f} s")
                _print_positions(sim)
        elif args.command == "start":
            sim.start()
            print("ready: timeline playing, servos armed")
        elif args.command == "arm":
            sim.arm()
        elif args.command in ("play", "pause", "stop"):
            getattr(sim, args.command)()
            print(f"timeline {sim.state()}")
        elif args.command == "home":
            sim.start()
            sim.home(args.speed)
        elif args.command == "goto":
            sim.start()
            sim.goto(_goals(sim, args.goals), args.speed)
        elif args.command == "positions":
            sim.start()
            _print_positions(sim)
        elif args.command in ("exec", "run"):
            sim.armed()  # binds `servos` when the driver runs
            if args.command == "run":
                sim.run_file(args.file)
            elif (result := sim.run(args.code, timeout=120.0)) is not None:
                print(result)
    except IsaacError as exc:
        print(exc, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
