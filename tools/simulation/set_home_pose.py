"""Make the active profile's home pose the authored stage pose of the Isaac Sim scene.

    python tools/simulation/set_home_pose.py

Run it after a recalibration moved home (arming then prints "the stage pose is not the
profile's home"). Through Isaac Sim's Python Server it plays the simulation, drives the servo
models to home, lets the linkage settle, authors every link transform and joint angle as the
stage pose and saves ``models/usd/scene.usda`` -- together with any other unsaved change in that
stage. Isaac-side part: ``src/ballbal/simulation/isaac_home_pose.py``.
"""

from __future__ import annotations

import sys

from ballbal.simulation.remote import PACKAGE, IsaacError, IsaacSim


def main() -> int:
    sim = IsaacSim()
    try:
        sim.ensure_armed()
        sim.run_file(PACKAGE / "isaac_home_pose.py")
        sim.run("await set_home_pose()", timeout=120.0)
    except IsaacError as exc:
        print(exc, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
