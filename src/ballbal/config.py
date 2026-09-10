"""Rig configuration.

Every limit that protects the mechanism lives here, in a file the operator can
read and edit, rather than being inferred at runtime from the servos' own EPROM
angle limits. Those EPROM limits describe what the *servo* can survive; they say
nothing about where the platform's linkages collide.
"""

from __future__ import annotations

import os
import tomllib
from dataclasses import dataclass, field
from pathlib import Path

from .hardware import registers as reg

DEFAULT_PROFILE = "platform-v2"
PROFILE_ENV = "BALLBAL_PROFILE"
ACTIVE_PROFILE_RELATIVE = Path("runtime/state/active-profile")

# A floor below the servos' own configured minimum (4.0 V from the factory on
# the 7.4 V STS3215), meant only to catch a supply that has actually collapsed.
#
# It is deliberately NOT set to the servo's 7.4 V nominal rating: this platform
# is operated from a regulated 5 V supply, so a reading around 5.0-5.5 V is the
# expected operating point, not a fault. Preflight also enforces each servo's own
# MIN_VOLTAGE_LIMIT register, which is the authoritative value.
DEFAULT_MIN_VOLTAGE = 4.5


@dataclass(frozen=True)
class ServoConfig:
    """One servo's identity and its mechanically safe travel."""

    name: str
    id: int
    min_position: int
    max_position: int
    home_position: int
    invert: bool = False
    rest_position: int | None = None
    """Where the axis reads with torque off and the platform settled -- every
    lower link lying on the base. It is the one pose that is the same in the CAD
    and on the rig, so the simulation anchors counts to joint angles on it
    (simulation/isaac.py). Recorded by ``ballbal rest`` and ``ballbal calibrate``.
    It may lie below ``min_position``: calibration pushes each axis further
    alone than it settles with the whole platform on it."""

    def __post_init__(self) -> None:
        if not 1 <= self.id <= 253:
            raise ValueError(f"{self.name}: id {self.id} is outside 1..253")
        if self.rest_position is not None and not 0 <= self.rest_position <= reg.COUNTS_PER_REV - 1:
            raise ValueError(
                f"{self.name}: rest_position {self.rest_position} is outside "
                f"0..{reg.COUNTS_PER_REV - 1}"
            )
        if not 0 <= self.min_position < self.max_position <= reg.COUNTS_PER_REV - 1:
            raise ValueError(
                f"{self.name}: needs 0 <= min_position < max_position <= "
                f"{reg.COUNTS_PER_REV - 1}, got "
                f"{self.min_position}..{self.max_position}"
            )
        if not self.min_position <= self.home_position <= self.max_position:
            raise ValueError(
                f"{self.name}: home_position {self.home_position} is outside "
                f"its own travel {self.min_position}..{self.max_position}"
            )

    def clamp(self, position: int) -> int:
        return max(self.min_position, min(self.max_position, position))

    @property
    def span(self) -> int:
        return self.max_position - self.min_position


@dataclass(frozen=True)
class RigConfig:
    """The whole rig: which servos, on which port, moving how fast."""

    servos: tuple[ServoConfig, ...]
    port: str | None = None
    baudrate: int = 1_000_000
    speed: int = 600
    acceleration: int = 30
    min_voltage: float = DEFAULT_MIN_VOLTAGE
    max_temperature: int = 65
    retries: int = 3
    step: int = 20
    max_lag: int = 60
    tolerance: int = 8
    home_offset: int | None = None
    max_offset: int | None = None
    source: Path | None = field(default=None, compare=False)
    calibration_source: Path | None = field(default=None, compare=False)

    def __post_init__(self) -> None:
        if not self.servos:
            raise ValueError("the rig defines no servos")
        ids = [s.id for s in self.servos]
        if len(set(ids)) != len(ids):
            raise ValueError(f"duplicate servo ids in the rig: {sorted(ids)}")
        names = [s.name for s in self.servos]
        if len(set(names)) != len(names):
            raise ValueError(f"duplicate servo names in the rig: {sorted(names)}")
        if not 0 < self.speed <= 3400:
            raise ValueError(f"speed {self.speed} is outside 1..3400 counts/s")
        if not 0 < self.acceleration <= 254:
            raise ValueError(f"acceleration {self.acceleration} is outside 1..254")
        if self.max_lag < 1:
            raise ValueError("max_lag must be at least 1 count")
        if not 1 <= self.tolerance <= 100:
            raise ValueError("tolerance must be 1..100 counts")
        if (self.home_offset is None) != (self.max_offset is None):
            raise ValueError("home_offset and max_offset must be configured together")
        if self.home_offset is not None:
            if not 0 < self.home_offset < self.max_offset:  # type: ignore[operator]
                raise ValueError("needs 0 < home_offset < max_offset")

    @property
    def ids(self) -> tuple[int, ...]:
        return tuple(s.id for s in self.servos)

    def by_id(self, servo_id: int) -> ServoConfig:
        for servo in self.servos:
            if servo.id == servo_id:
                return servo
        raise KeyError(f"no servo with id {servo_id} in the rig")

    def select(self, names_or_ids: list[str] | None) -> tuple[ServoConfig, ...]:
        """Resolve a user-supplied subset, accepting either names or ids."""
        if not names_or_ids:
            return self.servos
        chosen: list[ServoConfig] = []
        for token in names_or_ids:
            match = next(
                (
                    s
                    for s in self.servos
                    if s.name == token or (token.isdigit() and s.id == int(token))
                ),
                None,
            )
            if match is None:
                known = ", ".join(f"{s.name}({s.id})" for s in self.servos)
                raise KeyError(f"unknown servo {token!r}; the rig has {known}")
            if match not in chosen:
                chosen.append(match)
        return tuple(chosen)

    @classmethod
    def load(cls, path: str | Path | None = None) -> "RigConfig":
        """Load a platform profile or a legacy combined rig file."""
        resolved = resolve_profile(path)
        if not resolved.is_file():
            raise FileNotFoundError(
                f"No rig configuration at {resolved}. Copy profiles/example "
                "to profiles/<name> and calibrate that profile."
            )
        with resolved.open("rb") as handle:
            raw = tomllib.load(handle)

        servo_tables = raw.pop("servo", None)
        if not servo_tables:
            raise ValueError(f"{resolved} defines no [[servo]] entries")
        raw_servo_tables = list(servo_tables)

        calibration_source: Path | None = None
        if any(
            "min_position" not in entry
            or "max_position" not in entry
            or "home_position" not in entry
            for entry in servo_tables
        ):
            calibration_source = resolved.parent / "calibration.toml"
            if not calibration_source.is_file():
                raise FileNotFoundError(
                    f"No calibration at {calibration_source}; run `ballbal setup` "
                    "for this profile."
                )
            with calibration_source.open("rb") as handle:
                calibration_document = tomllib.load(handle)
            version = int(calibration_document.get("format_version", 1))
            if version != 1:
                raise ValueError(
                    f"{calibration_source} uses unsupported format_version {version}"
                )
            calibration = calibration_document.get("axis", {})
            identities = {str(entry["name"]) for entry in servo_tables}
            missing = identities - set(calibration)
            if missing:
                raise ValueError(
                    f"{calibration_source} has no calibration for: "
                    f"{', '.join(sorted(missing))}"
                )
            extra = set(calibration) - identities
            if extra:
                raise ValueError(
                    f"{calibration_source} has unknown axes: "
                    f"{', '.join(sorted(extra))}"
                )
            home_offset = raw.get("home_offset")
            max_offset = raw.get("max_offset")
            if home_offset is None or max_offset is None:
                raise ValueError(
                    f"{resolved} must define home_offset and max_offset when "
                    "using a separate calibration.toml"
                )
            servo_tables = []
            for identity in raw_servo_tables:
                measured = calibration[str(identity["name"])]
                if set(measured) != {"min_position"}:
                    raise ValueError(
                        f"{calibration_source}: {identity['name']} must contain "
                        "only min_position"
                    )
                minimum = int(measured["min_position"])
                servo_tables.append(
                    {
                        **identity,
                        "min_position": minimum,
                        "home_position": minimum + int(home_offset),
                        "max_position": minimum + int(max_offset),
                        "rest_position": minimum,
                    }
                )

        servos = tuple(
            ServoConfig(
                name=str(entry["name"]),
                id=int(entry["id"]),
                min_position=int(entry["min_position"]),
                max_position=int(entry["max_position"]),
                home_position=int(entry["home_position"]),
                invert=bool(entry.get("invert", False)),
                rest_position=int(entry["rest_position"]) if "rest_position" in entry else None,
            )
            for entry in servo_tables
        )
        known = {f.name for f in cls.__dataclass_fields__.values()} - {
            "servos",
            "source",
            "calibration_source",
        }
        unknown = set(raw) - known
        if unknown:
            raise ValueError(
                f"{resolved} has unknown keys: {', '.join(sorted(unknown))}"
            )
        return cls(
            servos=servos,
            source=resolved,
            calibration_source=calibration_source or resolved,
            **raw,
        )


def _repo_root() -> Path:
    working = Path.cwd()
    if (working / "profiles").is_dir():
        return working
    source_checkout = Path(__file__).resolve().parents[2]
    if (source_checkout / "profiles").is_dir():
        return source_checkout
    return working


def available_profiles() -> tuple[str, ...]:
    """Return every usable profile directory in stable display order."""
    profiles = _repo_root() / "profiles"
    if not profiles.is_dir():
        return ()
    return tuple(
        path.name
        for path in sorted(profiles.iterdir(), key=lambda item: item.name.casefold())
        if path.is_dir() and (path / "rig.toml").is_file()
    )


def active_profile_file() -> Path:
    """Machine-local file containing the persistent default profile name."""
    return _repo_root() / ACTIVE_PROFILE_RELATIVE


def active_profile_name() -> str:
    """Resolve the persistent default, with the old env var as a fallback."""
    state = active_profile_file()
    if state.is_file():
        selected = state.read_text(encoding="utf-8").strip()
        if not selected or Path(selected).name != selected:
            raise ValueError(f"invalid active profile stored in {state}: {selected!r}")
        return selected
    environment = os.environ.get(PROFILE_ENV)
    if environment:
        return environment
    return DEFAULT_PROFILE


def set_active_profile(name: str) -> Path:
    """Persist one named profile as the local default and return the state file."""
    known = available_profiles()
    if name not in known:
        detail = ", ".join(known) if known else "none"
        raise ValueError(f"unknown profile {name!r}; available profiles: {detail}")

    # Validate the complete profile before making it the default.
    RigConfig.load(name)
    state = active_profile_file()
    state.parent.mkdir(parents=True, exist_ok=True)
    temporary = state.with_suffix(".tmp")
    temporary.write_text(name + "\n", encoding="utf-8")
    temporary.replace(state)
    return state


def resolve_profile(path: str | Path | None = None) -> Path:
    """Resolve a rig file, profile directory, or profile name."""
    if path is not None:
        candidate = Path(path)
        if candidate.is_dir():
            return candidate / "rig.toml"
        if candidate.parent == Path(".") and candidate.suffix == "":
            named = _repo_root() / "profiles" / str(candidate) / "rig.toml"
            if named.exists():
                return named
        return candidate

    selected = active_profile_name()
    return _repo_root() / "profiles" / selected / "rig.toml"
