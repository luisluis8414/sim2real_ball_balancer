"""The simulation's view of the real rig.

Everything calibration, trim, ``neutral`` and ``rest`` produce lives in the selected
platform profile. This module only reads it (with tomllib, so it
also works inside Isaac Sim's Python, which has no pyserial for the ballbal package) and adds
what exists only in the simulation: which joint each axis drives, the CAD angle of the rest
pose, and the identified servo dynamics (``params.json``).
"""

from __future__ import annotations

import json
import os
import tomllib
from dataclasses import dataclass, replace
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
ACTIVE_PROFILE_FILE = REPO / "runtime" / "state" / "active-profile"
PROFILE_NAME = (
    ACTIVE_PROFILE_FILE.read_text().strip()
    if ACTIVE_PROFILE_FILE.is_file()
    else os.environ.get("BALLBAL_PROFILE", "platform-v2")
)
PROFILE_DIR = REPO / "profiles" / PROFILE_NAME
RIG_FILE = PROFILE_DIR / "rig.toml"
CALIBRATION_FILE = PROFILE_DIR / "calibration.toml"
PARAMS_FILE = Path(__file__).resolve().parent / "params.json"
COUNTS_PER_REV = 4096


def profile_mtime() -> float:
    return max(RIG_FILE.stat().st_mtime, CALIBRATION_FILE.stat().st_mtime)


@dataclass(frozen=True)
class Axis:
    id: int
    name: str
    sim_joint: str
    min: int
    max: int
    home: int
    rest: int
    zero: float
    """Counts at the CAD pose (sim angle 0)."""

    def deg(self, counts: float) -> float:
        return (counts - self.zero) * 360.0 / COUNTS_PER_REV


@dataclass(frozen=True)
class RigView:
    axes: dict[int, Axis]
    speed: int  # rig.toml goal speed, used when a command gives none (as Rig.goto does)
    acceleration: int  # rig.toml goal acceleration register
    contact_deg: float
    source: Path
    mtime: float


def load(
    rig_file: Path = RIG_FILE,
    calibration_file: Path = CALIBRATION_FILE,
    params_file: Path = PARAMS_FILE,
) -> RigView:
    with open(rig_file, "rb") as fh:
        rig = tomllib.load(fh)
    with open(calibration_file, "rb") as fh:
        calibration = tomllib.load(fh)["axis"]
    params = json.loads(Path(params_file).read_text())
    joints = {int(k): v for k, v in params["sim_joints"].items()}
    contact = float(params["cad_lower_link_base_contact_deg"])
    axes = {}
    home_offset = int(rig["home_offset"])
    max_offset = int(rig["max_offset"])
    for identity in rig["servo"]:
        minimum = int(calibration[identity["name"]]["min_position"])
        s = {
            **identity,
            "min_position": minimum,
            "home_position": minimum + home_offset,
            "max_position": minimum + max_offset,
        }
        rest = minimum
        axes[int(s["id"])] = Axis(
            id=int(s["id"]), name=s["name"], sim_joint=joints[int(s["id"])],
            min=int(s["min_position"]), max=int(s["max_position"]), home=int(s["home_position"]),
            rest=rest, zero=rest - contact * COUNTS_PER_REV / 360.0,
        )
    return RigView(axes, int(rig.get("speed", 600)), int(rig.get("acceleration", 30)), contact,
                   Path(calibration_file), max(Path(rig_file).stat().st_mtime,
                                               Path(calibration_file).stat().st_mtime))


def scaled_params(params, rig: RigView):
    """Servo dynamics for the rig's acceleration register.

    The model was identified at register 100 (``identified_at``); accel and decel are scaled with
    the register (unit 100 counts/s^2), which the identification confirmed for accel. v_max is the
    motor's own limit and the goal speed is applied per command.
    """
    identified = json.loads(PARAMS_FILE.read_text())["identified_at"]["goal_acceleration"]
    k = rig.acceleration / identified
    return replace(params, accel=params.accel * k, decel=params.decel * k)
