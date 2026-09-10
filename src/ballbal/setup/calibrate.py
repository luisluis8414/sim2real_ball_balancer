"""Manual common-pose lower-reference calibration.

The operator moves the complete platform to its lower pose with torque disabled.
All encoder positions are then read together. Home and maximum travel are fixed
geometry offsets above those measured encoder values.
"""

from __future__ import annotations

import json
import sys
import tomllib
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Sequence

import tomlkit

from ..hardware import registers as reg
from ..config import ServoConfig
from ..prompt import confirm as _confirm
from ..control.rig import Rig
from ..paths import runtime_dir

@dataclass(frozen=True)
class AxisCalibration:
    """One axis's measured lower contact and the limits derived from it."""

    name: str
    id: int
    observed_min: int
    observed_max: int
    margin: int
    min_position: int
    max_position: int
    home_position: int
    rest_position: int | None = None
    """Reading with torque off and the whole platform settled; see ballbal.rest."""

    @property
    def span(self) -> int:
        return self.max_position - self.min_position

    @property
    def degrees(self) -> float:
        return reg.counts_to_degrees(self.span)


def calibrate(
    rig: Rig,
    servos: Sequence[ServoConfig],
    *,
    margin: int = 5,
    apply: bool = True,
    log_dir: Path | None = None,
) -> int:
    """Measure one lower reference per axis and derive home/max from it."""
    if not sys.stdin.isatty():
        raise RuntimeError("calibration is interactive and needs a terminal")
    if margin < 0:
        raise ValueError("margin must not be negative")

    config = rig.config
    target = config.calibration_source or config.source
    if target is None:
        raise RuntimeError("cannot write limits back: the rig was not loaded from a file")

    if config.home_offset is None or config.max_offset is None:
        raise RuntimeError(
            "this profile has no fixed home_offset/max_offset; migrate it before calibrating"
        )

    selected_ids = {servo.id for servo in servos}
    configured_ids = {servo.id for servo in config.servos}
    if selected_ids != configured_ids:
        raise ValueError(
            "minimum calibration records the complete platform at one pose; "
            "do not select individual axes"
        )

    print(f"Calibrating {len(servos)} axis/axes; minima will be stored in {target}.")
    rig.bus.set_torque([s.id for s in config.servos], False)
    print(
        "\n  THE COMPLETE PLATFORM MUST BE ASSEMBLED FOR THIS.\n\n"
        "  With torque OFF, move the COMPLETE platform by hand into its common\n"
        "  lower pose. All three axes must be resting at their lower mechanical\n"
        "  reference at the same time. Keep the platform there while answering\n"
        "  yes; all encoder positions are then read together immediately.\n"
    )
    if not _confirm(
        "Is the complete platform now in its lower pose?", expected="yes"
    ):
        print("Aborted; nothing was written.")
        return 1

    measured = rig.positions(servos)
    print("\nRead all axis positions with torque off.")

    results: list[AxisCalibration] = []
    for servo in servos:
        low = measured[servo.id]
        safe_low = low + margin
        home = safe_low + config.home_offset
        safe_high = safe_low + config.max_offset
        if safe_high >= reg.COUNTS_PER_REV:
            print(
                f"\n  {servo.name}: min {safe_low} + max_offset "
                f"{config.max_offset} exceeds encoder range."
            )
            return 1

        results.append(
            AxisCalibration(
                name=servo.name,
                id=servo.id,
                observed_min=low,
                observed_max=low,
                margin=margin,
                min_position=safe_low,
                max_position=safe_high,
                home_position=home,
            )
        )
        print(
            f"  {servo.name}: measured {low} -> safe min {safe_low}, "
            f"home {home}, max {safe_high}"
        )

    _print_summary(results)

    log_path = _write_log(results, log_dir or runtime_dir("logs", "calibration"))
    print(f"\nCalibration log: {log_path}")

    if not apply:
        print("Dry run: rig file not modified.")
        return 0

    answer = input(f"\nWrite these limits to {target}? [y/N] ").strip().lower()
    if answer not in ("y", "yes"):
        print("Not written. The log above still has the values.")
        return 1

    _apply_to_rig_file(target, results)
    print(f"Wrote {target}.")
    print(
        "These are now hard limits: Rig.goto clamps every goal to them, so no "
        "script can drive past them."
    )
    print(
        f"home and max are derived for every axis: min + {config.home_offset} "
        f"and min + {config.max_offset}."
    )
    return 0


def _print_summary(results: Sequence[AxisCalibration]) -> None:
    print("=" * 58)
    print(
        f"{'AXIS':<10}{'ID':>4}{'CONTACT':>10}{'MIN':>8}"
        f"{'HOME':>8}{'MAX':>8}{'SPAN':>10}"
    )
    for r in results:
        print(
            f"{r.name:<10}{r.id:>4}"
            f"{r.observed_min:>10}{r.min_position:>8}"
            f"{r.home_position:>8}{r.max_position:>8}"
            f"{f'{r.span} ({r.degrees:.0f}d)':>10}"
        )
    print("=" * 58)


def _write_log(results: Sequence[AxisCalibration], log_dir: Path) -> Path:
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().astimezone()
    path = log_dir / f"calibration_{stamp.strftime('%Y%m%d_%H%M%S')}.json"
    path.write_text(
        json.dumps(
            {
                "recorded_at": stamp.isoformat(timespec="seconds"),
                "axes": [asdict(r) for r in results],
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    return path


def write_servo_fields(
    path: Path, updates: dict[int, dict[str, int]]
) -> None:
    """Set named fields on servo tables in place, keeping comments and layout.

    Rewriting the file wholesale would lose the comments that explain what every
    limit is for, which is most of what makes the rig file reviewable.
    """
    document = tomlkit.parse(path.read_text(encoding="utf-8"))
    tables = document.get("servo")
    profile_axes = document.get("axis")
    if tables is None and profile_axes is None:
        raise ValueError(f"{path} has neither [[servo]] nor [axis.*] tables to update")

    if tables is None:
        rig_path = path.parent / "rig.toml"
        with rig_path.open("rb") as handle:
            identities = tomllib.load(handle).get("servo", [])
        names = {int(item["id"]): str(item["name"]) for item in identities}
        missing = set(updates) - set(names)
        if missing:
            raise ValueError(f"{rig_path} has no servo with id {sorted(missing)}")
        for servo_id, fields in updates.items():
            table = profile_axes.get(names[servo_id])
            if table is None:
                raise ValueError(f"{path} has no axis {names[servo_id]!r}")
            for name, value in fields.items():
                table[name] = value
        _write_with_backup(path, document)
        return

    seen: set[int] = set()
    for table in tables:
        servo_id = int(table["id"])
        fields = updates.get(servo_id)
        if fields is None:
            continue
        seen.add(servo_id)
        for name, value in fields.items():
            table[name] = value

    missing = set(updates) - seen
    if missing:
        raise ValueError(f"{path} has no servo with id {sorted(missing)}")

    _write_with_backup(path, document)


def _write_with_backup(path: Path, document) -> None:  # noqa: ANN001
    backup = path.with_suffix(path.suffix + ".bak")
    backup.write_text(path.read_text(encoding="utf-8"), encoding="utf-8")
    path.write_text(tomlkit.dumps(document), encoding="utf-8")


def _apply_to_rig_file(path: Path, results: Sequence[AxisCalibration]) -> None:
    """Update the servo tables from a completed calibration."""
    document = tomlkit.parse(path.read_text(encoding="utf-8"))
    derived_profile = document.get("axis") is not None
    write_servo_fields(
        path,
        {
            r.id: (
                {"min_position": r.min_position}
                if derived_profile
                else {
                    "min_position": r.min_position,
                    "max_position": r.max_position,
                    "home_position": r.home_position,
                    **({"rest_position": r.rest_position} if r.rest_position is not None else {}),
                }
            )
            for r in results
        },
    )
