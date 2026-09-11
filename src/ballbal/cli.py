"""Command-line entry point: ``ballbal <command>``."""

from __future__ import annotations

import argparse
import contextlib
import logging
import sys
from pathlib import Path

from .hardware import registers as reg
from .hardware.bus import ServoBus
from .config import (
    RigConfig,
    active_profile_name,
    available_profiles,
    resolve_profile,
    set_active_profile,
)
from .setup.calibrate import calibrate
from .control.interactive import interactive_jog
from .hardware.port import find_port, list_ports
from .control.rig import PreflightError, Rig
from .control.rotate import rotate
from .setup import (
    SETUP_STEPS,
    assign_ids,
    centre_bare,
    guided_setup,
    open_travel,
    prepare_servos,
    recover,
)
from .control.sweep import sweep
from .control.trim import trim

logger = logging.getLogger(__name__)
REFERENCE_PROFILE = "platform-v2"


def _warn_if_reference_profile(name: str) -> None:
    if name == REFERENCE_PROFILE:
        logger.warning(
            "platform-v2 is calibrated for the repository author's physical "
            "hardware and is kept only for reference/comparison; create and "
            "calibrate your own profile before controlling another platform"
        )


def _add_profile(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--profile",
        "--rig",
        dest="rig",
        type=Path,
        default=None,
        help="temporary profile override (persistent default: ballbal config set)",
    )


def _add_common(parser: argparse.ArgumentParser) -> None:
    _add_profile(parser)
    parser.add_argument("--port", default=None, help="override the serial port")
    parser.add_argument("-v", "--verbose", action="store_true")


def _add_balance_tuning(parser: argparse.ArgumentParser) -> None:
    """Loop settings beyond the gains; each falls back to the profile's [balance]."""
    parser.add_argument(
        "--speed", type=int, default=None,
        help="servo goal speed while balancing, counts/s",
    )
    parser.add_argument(
        "--smoothing", type=float, default=None,
        help="weight of each new sample in the derivative filter, 0..1; "
        "higher is less lag and more noise",
    )
    parser.add_argument(
        "--izone", type=float, default=None,
        help="mm; the integral only accumulates this close to the target",
    )
    parser.add_argument(
        "--predict", type=float, default=None,
        help="ms to look ahead, cancelling the loop's dead time; 0 is off",
    )
    parser.add_argument(
        "--plant-gain", type=float, default=None,
        help="ball acceleration per count of leg swing, mm/s^2, for --predict",
    )


def _live_view(args: argparse.Namespace, calibration, servos):  # noqa: ANN001, ANN202
    """The Lichtblick server when --ui was given, else a context that yields None."""
    if not getattr(args, "ui", False):
        return contextlib.nullcontext()
    try:
        from .ui import LiveView
    except ImportError as exc:
        raise RuntimeError(
            f"--ui needs the ui extra ({exc.name} is missing): pip install -e \".[ui]\""
        ) from exc
    from .vision import find_camera, find_side_camera

    side = find_side_camera(args.side_camera, find_camera(args.device))
    return LiveView(calibration, servos, side_camera=side)


def _balance_settings(args: argparse.Namespace, config: RigConfig) -> dict:
    """Flag, else the profile's [balance], else the built-in default."""
    from .control.balance import DEFAULT_GAINS, DEFAULT_MAX_TILT_PCT, QUIET_COUNTS

    tuned = config.balance

    def pick(flag, stored, default):  # noqa: ANN001, ANN202
        return flag if flag is not None else stored if stored is not None else default

    predict_ms = pick(args.predict, tuned.predict_ms, 0.0)
    return dict(
        gains=(
            pick(args.kp, tuned.kp, DEFAULT_GAINS[0]),
            pick(args.ki, tuned.ki, DEFAULT_GAINS[1]),
            pick(args.kd, tuned.kd, DEFAULT_GAINS[2]),
        ),
        max_tilt_pct=pick(args.max_tilt, tuned.max_tilt, DEFAULT_MAX_TILT_PCT),
        acceleration=pick(args.accel, tuned.acceleration, None),
        speed=pick(args.speed, tuned.speed, None),
        derivative_smoothing=pick(args.smoothing, tuned.derivative_smoothing, None),
        integral_zone=pick(args.izone, tuned.integral_zone, None),
        predict=predict_ms / 1000.0,
        plant_gain=pick(args.plant_gain, tuned.plant_gain, None),
        quiet_counts=pick(args.quiet, tuned.quiet, QUIET_COUNTS),
        aggression=pick(args.aggression, tuned.aggression, 0.0),
        shape=pick(args.shape, tuned.shape, 1.0),
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="ballbal", description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    config_parser = subparsers.add_parser(
        "config", help="show or validate the selected platform profile"
    )
    config_parser.add_argument(
        "action",
        choices=("show", "validate", "paths", "set"),
        nargs="?",
        default="show",
    )
    config_parser.add_argument(
        "profile_name",
        nargs="?",
        help="profile to make the persistent default; omit for interactive selection",
    )
    config_parser.add_argument(
        "--profile",
        "--rig",
        dest="rig",
        type=Path,
        default=None,
        help="temporary override for show/validate/paths",
    )
    config_parser.add_argument("-v", "--verbose", action="store_true")

    camera_parser = subparsers.add_parser(
        "camera", help="show or persistently select the camera"
    )
    camera_parser.add_argument(
        "action", choices=("show", "set"), nargs="?", default="show"
    )
    camera_parser.add_argument(
        "camera_id",
        nargs="?",
        help="one-based camera ID; omit with set for interactive selection",
    )
    camera_parser.add_argument(
        "--side", action="store_true",
        help="with set: choose the side view camera for --ui instead of the tracked one",
    )
    camera_parser.add_argument("-v", "--verbose", action="store_true")

    ports = subparsers.add_parser("ports", help="list attached serial adapters")
    ports.add_argument("-v", "--verbose", action="store_true")

    scan = subparsers.add_parser("scan", help="ping an ID range (read-only)")
    _add_common(scan)
    scan.add_argument("--first", type=int, default=1)
    scan.add_argument("--last", type=int, default=20)

    status = subparsers.add_parser("status", help="read telemetry (read-only)")
    _add_common(status)
    status.add_argument("servos", nargs="*", help="names or ids; default all")

    check = subparsers.add_parser(
        "check", help="run the preflight checks without moving anything"
    )
    _add_common(check)
    check.add_argument("servos", nargs="*")

    home = subparsers.add_parser("home", help="ramp servos to their home positions")
    _add_common(home)
    home.add_argument("servos", nargs="*")
    home.add_argument("--hold", action="store_true", help="leave torque enabled")

    goto = subparsers.add_parser("goto", help="ramp one servo to a position")
    _add_common(goto)
    goto.add_argument("servo", help="name or id")
    goto.add_argument("position", type=int, help="target in encoder counts")
    goto.add_argument("--hold", action="store_true")

    jog = subparsers.add_parser("jog", help="interactive keyboard control")
    _add_common(jog)
    jog.add_argument("servos", nargs="*")
    jog.add_argument("--step", type=int, default=None, help="counts per keypress")
    jog.add_argument("--log", type=Path, default=None, help="CSV of every move")

    center = subparsers.add_parser(
        "center",
        help="ramp servos to encoder mid-scale (2048), ignoring rig home values",
    )
    _add_common(center)
    center.add_argument("servos", nargs="*")
    center.add_argument(
        "--position",
        type=int,
        default=reg.CENTER_POSITION,
        help=f"target counts (default {reg.CENTER_POSITION})",
    )
    center.add_argument("--hold", action="store_true")

    factory = subparsers.add_parser(
        "factory-reset",
        help="clear homing offset and open angle limits to a full turn (EPROM)",
    )
    _add_common(factory)
    factory.add_argument("servos", nargs="*")
    factory.add_argument(
        "--yes", action="store_true", help="skip the confirmation prompt"
    )

    setup_parser = subparsers.add_parser(
        "setup", help="guided first-time setup: ids, prepare, centre, calibrate, sweep"
    )
    _add_common(setup_parser)
    setup_parser.add_argument("servos", nargs="*")
    setup_parser.add_argument(
        "--skip-ids", action="store_true", help="servo IDs are already assigned"
    )
    setup_parser.add_argument(
        "--from",
        dest="start",
        choices=SETUP_STEPS,
        default=None,
        help="resume the procedure at this step, assuming the ones before it "
        "are done (overrides --skip-ids)",
    )

    ids_parser = subparsers.add_parser(
        "set-ids", help="give each servo its own ID, one servo at a time"
    )
    _add_common(ids_parser)
    ids_parser.add_argument("servos", nargs="*")

    prepare_parser = subparsers.add_parser(
        "prepare",
        help="clear a previous robot's offsets, angle limits, torque caps and gains",
    )
    _add_common(prepare_parser)
    prepare_parser.add_argument("servos", nargs="*")
    prepare_parser.add_argument(
        "--keep-travel",
        action="store_true",
        help="leave the rig file's travel alone instead of opening it fully",
    )

    cal = subparsers.add_parser(
        "calibrate",
        help="read all minima at one hand-placed pose; derive home/max",
    )
    _add_common(cal)
    cal.add_argument(
        "--margin",
        type=int,
        default=5,
        help="counts to place min above the measured lower reference "
        "(default 5, about 0.44 deg)",
    )
    cal.add_argument(
        "--dry-run",
        action="store_true",
        help="record and log, but leave the rig file untouched",
    )

    sweep_parser = subparsers.add_parser(
        "sweep",
        help="slowly drive each axis to its min, then max, to verify the limits",
    )
    _add_common(sweep_parser)
    sweep_parser.add_argument("servos", nargs="*")
    sweep_parser.add_argument(
        "--speed",
        type=int,
        default=None,
        help=f"counts/s (default: a quarter of the rig speed)",
    )
    sweep_parser.add_argument(
        "--lead",
        type=int,
        default=None,
        help="ramp lead in counts; the main speed control (default: derived). "
        "Bigger is faster but drives harder into a collision",
    )
    sweep_parser.add_argument(
        "--dwell", type=float, default=0.8, help="seconds to pause at each end"
    )
    sweep_parser.add_argument(
        "--together-speed",
        type=int,
        default=None,
        help="counts/s for the all-axes-together phase (default: capped, since "
        "three axes at once can brown out the supply)",
    )
    sweep_parser.add_argument(
        "--no-together",
        action="store_true",
        help="skip the final phase that moves every axis at once",
    )
    sweep_parser.add_argument("--yes", action="store_true", help="skip the prompt")

    recover_parser = subparsers.add_parser(
        "recover",
        help="return axes that settled outside their travel, slowly, one at a time",
    )
    _add_common(recover_parser)
    recover_parser.add_argument("servos", nargs="*")
    recover_parser.add_argument("--speed", type=int, default=400)

    rotate_parser = subparsers.add_parser(
        "rotate",
        help="tilt the platform in a rotating wave: one side down, the opposite up",
    )
    _add_common(rotate_parser)
    rotate_parser.add_argument("servos", nargs="*")
    rotate_parser.add_argument(
        "--period", type=float, default=6.0, help="seconds per revolution"
    )
    rotate_parser.add_argument(
        "--revolutions", type=float, default=3.0, help="how many turns to run"
    )
    rotate_parser.add_argument(
        "--amplitude",
        type=float,
        default=100.0,
        help="percent of each axis's usable travel (default 100)",
    )
    rotate_parser.add_argument(
        "--counter-clockwise",
        action="store_true",
        help="reverse the direction (default is clockwise, seen from above)",
    )
    rotate_parser.add_argument(
        "--raw-sine",
        action="store_true",
        help="drive a plain sine on the servo angle instead of linearising "
        "through the horn geometry (the platform will heave as it turns)",
    )
    rotate_parser.add_argument("--log", type=Path, default=None)
    rotate_parser.add_argument("--yes", action="store_true", help="skip the prompt")

    trim_parser = subparsers.add_parser(
        "trim",
        help="trim independently stored limits in legacy profiles",
    )
    _add_common(trim_parser)
    trim_parser.add_argument("servos", nargs="*")
    trim_parser.add_argument(
        "--min",
        dest="edge",
        action="store_const",
        const="min",
        default="max",
        help="trim the lower limit instead of the upper one",
    )
    trim_parser.add_argument(
        "--step", type=int, default=5, help="counts per keypress (default 5)"
    )
    trim_parser.add_argument("--yes", action="store_true", help="skip the prompts")

    cam_setup = subparsers.add_parser(
        "cam-setup",
        help="guided camera calibration: frame the platform, click the ball",
    )
    _add_profile(cam_setup)
    cam_setup.add_argument(
        "--no-square", action="store_true",
        help="keep the camera's full 16:9 frame instead of cropping it square",
    )
    cam_setup.add_argument(
        "--device", default=None, help="camera ID from the displayed list, or path"
    )
    cam_setup.add_argument("--width", type=int, default=None)
    cam_setup.add_argument("--height", type=int, default=None)
    cam_setup.add_argument("--ball-mm", type=float, default=None)
    cam_setup.add_argument("--platform-mm", type=float, default=None)
    cam_setup.add_argument("--out", type=Path, default=None)
    cam_setup.add_argument("--no-lock", action="store_true")
    cam_setup.add_argument("-v", "--verbose", action="store_true")

    cam = subparsers.add_parser(
        "cam-calibrate",
        help="measure where the platform sits in the image and the mm/px scale",
    )
    _add_profile(cam)
    cam.add_argument(
        "--no-square", action="store_true",
        help="keep the camera's full 16:9 frame instead of cropping it square",
    )
    cam.add_argument("--device", default=None, help="camera ID or device path")
    cam.add_argument("--width", type=int, default=None)
    cam.add_argument("--height", type=int, default=None)
    cam.add_argument("--threshold", type=int, default=None)
    cam.add_argument(
        "--ball-mm", type=float, default=None, help="the ball's real diameter"
    )
    cam.add_argument(
        "--platform-mm", type=float, default=None,
        help="the platform's real diameter; used as the scale when given, and "
        "cross-checked against the ball",
    )
    cam.add_argument("--frames", type=int, default=40)
    cam.add_argument("--out", type=Path, default=None, help="where to write it")
    cam.add_argument("--no-lock", action="store_true")
    cam.add_argument("-v", "--verbose", action="store_true")

    track_parser = subparsers.add_parser(
        "track", help="show the camera feed and report the ball's position"
    )
    _add_profile(track_parser)
    track_parser.add_argument(
        "--no-square", action="store_true",
        help="keep the camera's full 16:9 frame instead of cropping it square",
    )
    track_parser.add_argument("--device", default=None, help="camera ID or device path")
    track_parser.add_argument("--width", type=int, default=None)
    track_parser.add_argument("--height", type=int, default=None)
    track_parser.add_argument(
        "--threshold", type=int, default=None,
        help="brightness above which a pixel may be ball",
    )
    track_parser.add_argument(
        "--ball-mm", type=float, default=None,
        help="the ball's real diameter, used to measure the mm/px scale",
    )
    track_parser.add_argument(
        "--mm-per-px", type=float, default=None,
        help="skip the scale measurement and use this instead",
    )
    track_parser.add_argument(
        "--roi", type=float, default=None,
        help="search-circle radius as a fraction of frame width",
    )
    track_parser.add_argument(
        "--no-window", action="store_true", help="log only, open no window"
    )
    track_parser.add_argument(
        "--no-lock", action="store_true",
        help="leave the camera's automatic exposure and white balance alone",
    )
    track_parser.add_argument("--log", type=Path, default=None, help="CSV of every frame")
    track_parser.add_argument(
        "--every", type=int, default=10,
        help="print one line every N frames (0 to print none)",
    )
    track_parser.add_argument(
        "--calibration", type=Path, default=None,
        help="camera calibration to use (default: selected profile's camera.json)",
    )
    track_parser.add_argument(
        "--no-calibration", action="store_true",
        help="ignore any stored calibration",
    )
    track_parser.add_argument(
        "--seconds", type=float, default=None,
        help="stop after this long (required with --no-window, which has no key "
        "to press)",
    )
    track_parser.add_argument("-v", "--verbose", action="store_true")

    bal = subparsers.add_parser(
        "balance",
        help="close the loop: track the ball and lean the plate to centre it",
    )
    _add_common(bal)
    bal.add_argument("servos", nargs="*")
    bal.add_argument(
        "--no-square", action="store_true",
        help="keep the camera's full 16:9 frame instead of cropping it square",
    )
    bal.add_argument("--device", default=None, help="camera ID or device path")
    bal.add_argument("--width", type=int, default=None)
    bal.add_argument("--height", type=int, default=None)
    bal.add_argument("--calibration", type=Path, default=None)
    bal.add_argument("--kp", type=float, default=None)
    bal.add_argument("--ki", type=float, default=None)
    bal.add_argument("--kd", type=float, default=None)
    bal.add_argument(
        "--bearing", type=float, default=None,
        help="where axis_1 sits in the image, in degrees; taken from the "
        "camera calibration when not given",
    )
    bal.add_argument(
        "--shape", type=float, default=None,
        help="how the aggression is spread between middle and rim; 1 rises in "
        "step with distance, higher holds it back near the middle so the loop "
        "goes quiet sooner as the ball arrives",
    )
    bal.add_argument(
        "--quiet", type=int, default=None,
        help="smallest change in leg counts worth sending; 0 sends every frame "
        "and lets the plate twitch on camera noise",
    )
    bal.add_argument(
        "--aggression", type=float, default=None,
        help="how much harder to push the further out the ball is; 0 is a "
        "plain loop, 1 to 3 catches a ball about twice as fast, past 4 it "
        "oscillates",
    )
    bal.add_argument(
        "--accel", type=int, default=None,
        help="servo ramp for this run only, 1..254; higher cuts the mechanism's "
        "dead time but jerks the platform harder",
    )
    bal.add_argument(
        "--check-tilt", action="store_true",
        help="lean the plate to four known image directions and name each one, "
        "so the stored bearing can be confirmed by eye (moves the servos)",
    )
    bal.add_argument(
        "--max-tilt", type=float, default=None,
        help="percent of each axis's travel a full lean may use",
    )
    bal.add_argument("--seconds", type=float, default=None)
    bal.add_argument("--no-window", action="store_true")
    bal.add_argument("--log", type=Path, default=None)
    bal.add_argument(
        "--live", action="store_true",
        help="actually drive the servos; without it nothing moves",
    )
    _add_balance_tuning(bal)
    bal.add_argument(
        "--ui", action="store_true",
        help="stream cameras, tracking and servos to Lichtblick and take targets "
        "clicked there; balancing waits for Start in Lichtblick (docs/lichtblick.md)",
    )
    bal.add_argument(
        "--side-camera", default=None,
        help="with --ui: camera ID or path shown as the side view, 'none' for no side "
        "view (default: `ballbal camera set --side`, else the other attached camera)",
    )
    bal.add_argument(
        "--mirror", action="store_true",
        help="mirror ball and platform into the running Isaac Sim "
        "(its Python Server must be up, see docs/simulation.md)",
    )

    tune_parser = subparsers.add_parser(
        "tune",
        help="read plant gain, dead time and gains out of a balance log",
    )
    tune_parser.add_argument("log", type=Path, help="CSV from `balance --log`")
    tune_parser.add_argument(
        "--zeta", type=float, default=None,
        help="target damping; 0.7 overshoots ~5%%, 1.0 not at all but is slower",
    )
    tune_parser.add_argument("-v", "--verbose", action="store_true")

    neutral = subparsers.add_parser(
        "neutral",
        help="show or move the pose the platform rests and leans about",
    )
    _add_common(neutral)
    neutral.add_argument("servos", nargs="*")
    neutral.add_argument(
        "--lower", type=int, default=None,
        help="counts to move every axis DOWN from its current home",
    )
    neutral.add_argument(
        "--raise", dest="raise_by", type=int, default=None,
        help="counts to move every axis UP from its current home",
    )
    neutral.add_argument(
        "--mid", action="store_true", help="put home back at mid-travel"
    )
    neutral.add_argument("--yes", action="store_true", help="skip the prompt")

    lat = subparsers.add_parser(
        "latency",
        help="step each axis and time it, to split the loop delay from the camera's",
    )
    _add_common(lat)
    lat.add_argument("servos", nargs="*")
    lat.add_argument("--device", default=None, help="camera ID or device path")
    lat.add_argument("--step", type=int, default=120, help="counts per step")
    lat.add_argument("--repeats", type=int, default=4)
    lat.add_argument(
        "--step-sweep", action="store_true",
        help="time several step sizes, to see whether the axis is limited by "
        "acceleration or by speed",
    )
    lat.add_argument(
        "--camera", action="store_true",
        help="measure the camera pipeline by timing a servo step against the bus",
    )
    lat.add_argument(
        "--accel-sweep", action="store_true",
        help="time the dead time at several acceleration settings instead",
    )
    lat.add_argument(
        "--total-ms", type=float, default=None,
        help="the loop's total dead time from `ballbal tune`, to subtract from",
    )

    for name, help_text, size_help in (
        ("circle", "drive the ball round a circle", "radius in mm"),
        ("square", "drive the ball round a square", "half the side, in mm"),
    ):
        path_parser = subparsers.add_parser(name, help=help_text)
        _add_common(path_parser)
        path_parser.add_argument("servos", nargs="*")
        path_parser.add_argument("--size", type=float, default=35.0, help=size_help)
        path_parser.add_argument(
            "--period", type=float, default=12.0, help="seconds for one lap"
        )
        path_parser.add_argument("--no-square", action="store_true")
        path_parser.add_argument("--device", default=None, help="camera ID or device path")
        path_parser.add_argument("--calibration", type=Path, default=None)
        path_parser.add_argument("--kp", type=float, default=None)
        path_parser.add_argument("--ki", type=float, default=None)
        path_parser.add_argument("--kd", type=float, default=None)
        path_parser.add_argument("--bearing", type=float, default=None)
        path_parser.add_argument("--aggression", type=float, default=None)
        path_parser.add_argument("--shape", type=float, default=None)
        path_parser.add_argument("--ui", action="store_true", help="live view in Lichtblick")
        path_parser.add_argument(
            "--side-camera", default=None,
            help="with --ui: side view camera ID or path, 'none' for no side view",
        )
        _add_balance_tuning(path_parser)
        path_parser.add_argument("--quiet", type=int, default=None)
        path_parser.add_argument("--accel", type=int, default=None)
        path_parser.add_argument("--max-tilt", type=float, default=None)
        path_parser.add_argument("--seconds", type=float, default=None)
        path_parser.add_argument("--no-window", action="store_true")
        path_parser.add_argument("--log", type=Path, default=None)
        path_parser.add_argument("--live", action="store_true")

    rest_parser = subparsers.add_parser(
        "rest",
        help="measure rest_position for a legacy profile",
    )
    _add_common(rest_parser)
    rest_parser.add_argument("servos", nargs="*")
    rest_parser.add_argument("--yes", action="store_true", help="skip the prompt")

    release = subparsers.add_parser("release", help="disable torque immediately")
    _add_common(release)
    release.add_argument("servos", nargs="*")

    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(levelname)s %(message)s",
    )

    try:
        return _dispatch(args)
    except PreflightError as exc:
        # Checked before RuntimeError below, which it subclasses.
        print(f"preflight failed:\n{exc}", file=sys.stderr)
        return 2
    except (RuntimeError, FileNotFoundError, KeyError, ValueError) as exc:
        # RuntimeError covers BusError and ServoError.
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\ninterrupted; torque released", file=sys.stderr)
        return 130


def _cmd_cam_calibrate(args: argparse.Namespace) -> int:
    from .vision import DEFAULT_BALL_MM, DEFAULT_SIZE
    from .vision import calibrate_camera, find_camera

    width, height = DEFAULT_SIZE
    return calibrate_camera(
        device=find_camera(args.device),
        size=(args.width or width, args.height or height),
        threshold=args.threshold,  # None -> chosen per frame by Otsu
        ball_mm=args.ball_mm if args.ball_mm is not None else DEFAULT_BALL_MM,
        platform_mm=args.platform_mm,
        frames=args.frames,
        lock=not args.no_lock,
        square=not args.no_square,
        output=args.out or RigConfig.load(args.rig).source.parent / "camera.json",
    )


def _cmd_track(args: argparse.Namespace) -> int:
    """Tracking needs the camera only; opening the servo bus would be noise."""
    from .vision import DEFAULT_BALL_MM, DEFAULT_ROI_FRACTION
    from .vision import DEFAULT_SIZE, find_camera, track

    from .vision import CALIBRATION_NAME, CameraCalibration

    calibration = None
    if not args.no_calibration:
        path = args.calibration or RigConfig.load(args.rig).source.parent / "camera.json"
        if path.exists():
            calibration = CameraCalibration.load(path)
        elif args.calibration is not None:
            raise FileNotFoundError(f"no calibration at {path}")

    width, height = calibration.capture_size if calibration is not None else DEFAULT_SIZE
    if calibration is not None and calibration.crop_offset:
        offset = tuple(calibration.crop_offset)
    else:
        offset = (0, 0)
    return track(
        device=find_camera(args.device),
        size=(args.width or width, args.height or height),
        threshold=args.threshold,  # None -> chosen per frame by Otsu
        ball_mm=args.ball_mm if args.ball_mm is not None else DEFAULT_BALL_MM,
        mm_per_px=args.mm_per_px,
        roi_fraction=args.roi if args.roi is not None else DEFAULT_ROI_FRACTION,
        show=not args.no_window,
        log_path=args.log,
        every=args.every,
        lock=not args.no_lock,
        square=not args.no_square,
        offset=offset,
        side=(calibration.crop_side if calibration else None),
        seconds=args.seconds,
        calibration=calibration,
    )


def _dispatch(args: argparse.Namespace) -> int:
    if args.command == "config" and args.action == "set":
        if args.rig is not None:
            raise ValueError("use `ballbal config set PROFILE`, without --profile")
        selected = args.profile_name or _select_profile_interactively()
        state = set_active_profile(selected)
        source = resolve_profile(selected)
        logger.info("active profile: %s (%s)", selected, source)
        _warn_if_reference_profile(selected)
        print(f"Default profile set to {selected}.")
        print(f"Stored in {state}.")
        return 0

    if getattr(args, "profile_name", None) is not None:
        raise ValueError("a profile name is only accepted by `ballbal config set`")

    profile_source = resolve_profile(getattr(args, "rig", None))
    logger.info(
        "active profile: %s (%s)", profile_source.parent.name, profile_source
    )
    _warn_if_reference_profile(profile_source.parent.name)

    if args.command == "camera":
        return _cmd_camera(args)

    if args.command == "config":
        config = RigConfig.load(args.rig)
        if args.action == "validate":
            print(f"valid: {config.source}")
            print(f"valid: {config.calibration_source}")
            return 0
        if args.action == "paths":
            print(f"configuration: {config.source}")
            print(f"calibration:   {config.calibration_source}")
            print(f"camera:        {config.source.parent / 'camera.json'}")
            from .paths import runtime_dir

            print(f"runtime:       {runtime_dir()}")
            return 0
        print(f"profile: {config.source.parent.name}")
        print(f"configuration: {config.source}")
        print(f"calibration: {config.calibration_source}")
        print(
            f"motion: speed={config.speed} acceleration={config.acceleration} "
            f"tolerance={config.tolerance}"
        )
        tuned = {
            key: value for key, value in vars(config.balance).items() if value is not None
        }
        if tuned:
            print("balance: " + " ".join(f"{key}={value}" for key, value in tuned.items()))
        print("axes:")
        for servo in config.servos:
            rest = "-" if servo.rest_position is None else servo.rest_position
            print(
                f"  {servo.name}: id={servo.id} travel={servo.min_position}.."
                f"{servo.max_position} home={servo.home_position} rest={rest}"
            )
        return 0

    if args.command == "ports":
        return _cmd_ports()

    if args.command == "tune":
        from .control.tune import DEFAULT_ZETA, report

        return report(
            args.log,
            zeta=args.zeta if args.zeta is not None else DEFAULT_ZETA,
        )

    if args.command == "track":
        return _cmd_track(args)

    if args.command == "cam-calibrate":
        return _cmd_cam_calibrate(args)

    if args.command == "cam-setup":
        from .vision import DEFAULT_BALL_MM, DEFAULT_SIZE
        from .vision import find_camera, guided_camera_setup

        width, height = DEFAULT_SIZE
        return guided_camera_setup(
            device=find_camera(args.device),
            size=(args.width or width, args.height or height),
            ball_mm=args.ball_mm if args.ball_mm is not None else DEFAULT_BALL_MM,
            platform_mm=args.platform_mm,
            output=args.out or RigConfig.load(args.rig).source.parent / "camera.json",
            lock=not args.no_lock,
            square=not args.no_square,
        )

    config = RigConfig.load(args.rig)

    if args.command == "scan":
        port = find_port(args.port or config.port)
        with ServoBus(port, config.baudrate, retries=config.retries) as bus:
            found = bus.scan(range(args.first, args.last + 1))
        if not found:
            print("No servos answered. Check power, wiring and baud rate.")
            return 1
        for servo_id, model in sorted(found.items()):
            name = reg.MODEL_NUMBERS.get(model, f"model {model}")
            print(f"  id {servo_id:>3}  {name}")
        return 0

    with Rig.open(config, args.port) as rig:
        servos = config.select(getattr(args, "servos", None) or None)

        if args.command == "status":
            for reading in rig.survey(servos):
                _print_reading(reading)
            return 0

        if args.command == "check":
            rig.preflight(servos)
            print(f"preflight ok for {', '.join(s.name for s in servos)}")
            return 0

        if args.command == "home":
            rig.preflight(servos)
            start = rig.positions(servos)
            with rig.bus.torque([s.id for s in servos], hold=args.hold):
                final = rig.home(servos)
            for servo in servos:
                moved = final[servo.id] - start[servo.id]
                error = final[servo.id] - servo.home_position
                note = (
                    f"moved {moved:+d}"
                    if moved
                    else f"already within {config.tolerance}, not commanded"
                )
                print(
                    f"  {servo.name}: {start[servo.id]} -> {final[servo.id]} "
                    f"(home {servo.home_position}, error {error:+d}; {note})"
                )
            return 0

        if args.command == "goto":
            servo = config.select([args.servo])[0]
            rig.preflight([servo])
            with rig.bus.torque([servo.id], hold=args.hold):
                final = rig.move_to({servo.id: servo.clamp(args.position)})
            print(f"  {servo.name}: {final[servo.id]}")
            return 0

        if args.command == "setup":
            return guided_setup(
                rig, servos, skip_ids=args.skip_ids, start=args.start
            )

        if args.command == "set-ids":
            return assign_ids(rig, servos)

        if args.command == "prepare":
            outcome = prepare_servos(rig, servos)
            if outcome == 0 and not args.keep_travel:
                open_travel(rig, servos)
            return outcome

        if args.command == "center":
            target = args.position
            if not 0 <= target <= reg.COUNTS_PER_REV - 1:
                raise ValueError(f"position must be 0..{reg.COUNTS_PER_REV - 1}")
            blocked = [
                s for s in servos if not s.min_position <= target <= s.max_position
            ]
            if blocked:
                names = ", ".join(
                    f"{s.name} ({s.min_position}..{s.max_position})" for s in blocked
                )
                raise ValueError(
                    f"{target} is outside the configured travel of: {names}. "
                    "Widen those ranges in the rig file, or run factory-reset "
                    "first if the servo still carries another project's limits."
                )
            start = rig.positions(servos)
            with rig.bus.torque([s.id for s in servos], hold=args.hold):
                final = rig.move_to({s.id: target for s in servos})
            for servo in servos:
                print(
                    f"  {servo.name}: {start[servo.id]} -> {final[servo.id]} "
                    f"(target {target}, error {final[servo.id] - target:+d})"
                )
            return 0

        if args.command == "factory-reset":
            return _cmd_factory_reset(rig, servos, assume_yes=args.yes)

        if args.command == "jog":
            return interactive_jog(rig, servos, step=args.step, log_path=args.log)

        if args.command == "calibrate":
            return calibrate(
                rig, servos, margin=args.margin, apply=not args.dry_run
            )

        if args.command == "sweep":
            return sweep(
                rig,
                servos,
                speed=args.speed,
                lead=args.lead,
                dwell=args.dwell,
                together=not args.no_together,
                together_speed=args.together_speed,
                assume_yes=args.yes,
            )

        if args.command == "recover":
            return recover(rig, servos, speed=args.speed)

        if args.command == "rotate":
            return rotate(
                rig,
                servos,
                period=args.period,
                revolutions=args.revolutions,
                amplitude_pct=args.amplitude,
                clockwise=not args.counter_clockwise,
                linearise=not args.raw_sine,
                assume_yes=args.yes,
                log_path=args.log,
            )

        if args.command == "trim":
            return trim(
                rig,
                servos,
                edge=args.edge,
                step=args.step,
                assume_yes=args.yes,
            )

        if args.command in ("circle", "square"):
            from .control.balance import Circle, Square, balance
            from .vision import AXIS_1_BEARING_DEG, CALIBRATION_NAME, DEFAULT_SIZE
            from .vision import CameraCalibration, find_camera

            where = args.calibration or config.source.parent / "camera.json"
            if not where.exists():
                raise FileNotFoundError(
                    f"no camera calibration at {where}; run `ballbal cam-setup`"
                )
            cal = CameraCalibration.load(where)
            heading = (
                args.bearing
                if args.bearing is not None
                else (
                    cal.bearing_deg
                    if cal.bearing_deg is not None
                    else AXIS_1_BEARING_DEG
                )
            )
            route = (
                Circle(period=args.period, radius=args.size)
                if args.command == "circle"
                else Square(period=args.period, size=args.size)
            )
            if args.live:
                rig.preflight(servos)
            width, height = cal.capture_size
            with _live_view(args, cal, servos) as view:
                return balance(
                    rig, servos,
                    view=view,
                    wait_for_start=view is not None,
                    calibration=cal,
                    device=find_camera(args.device),
                    size=(width, height),
                    bearing_deg=heading,
                    path=route,
                    square=not args.no_square,
                    offset=tuple(cal.crop_offset or (0, 0)),
                    side=cal.crop_side,
                    seconds=args.seconds,
                    dry_run=not args.live,
                    show=not args.no_window,
                    log_path=args.log,
                    **_balance_settings(args, config),
                )

        if args.command == "balance":
            from .control.balance import balance
            from .vision import AXIS_1_BEARING_DEG, CALIBRATION_NAME, DEFAULT_SIZE
            from .vision import CameraCalibration, find_camera

            path = args.calibration or config.source.parent / "camera.json"
            if not path.exists():
                raise FileNotFoundError(
                    f"no camera calibration at {path}; run `ballbal cam-setup`"
                )
            calibration = CameraCalibration.load(path)
            width, height = calibration.capture_size
            bearing = args.bearing
            if bearing is None:
                bearing = (
                    calibration.bearing_deg
                    if calibration.bearing_deg is not None
                    else AXIS_1_BEARING_DEG
                )
            if args.live or args.check_tilt:
                rig.preflight(servos)
            if args.check_tilt:
                from .control.balance import check_tilt

                return check_tilt(rig, servos, bearing_deg=bearing)
            mirror = contextlib.nullcontext()
            if args.mirror:
                # Connected before anything moves: an unreachable Isaac Sim ends
                # the run here, not halfway through a balance.
                from .simulation.mirror import Mirror

                mirror = Mirror(bearing_deg=bearing, ball_mm=calibration.ball_mm)
            with mirror as mirroring, _live_view(args, calibration, servos) as view:
                return balance(
                    rig, servos,
                    calibration=calibration,
                    device=find_camera(args.device),
                    size=(args.width or width, args.height or height),
                    bearing_deg=bearing,
                    seconds=args.seconds,
                    **_balance_settings(args, config),
                    square=not args.no_square,
                    offset=tuple(calibration.crop_offset or (0, 0)),
                    side=calibration.crop_side,
                    dry_run=not args.live,
                    show=not args.no_window,
                    log_path=args.log,
                    mirror=mirroring,
                    view=view,
                    wait_for_start=view is not None,
                )

        if args.command == "neutral":
            from .control.balance import plan
            from .setup.calibrate import write_servo_fields
            from .prompt import confirm

            if config.home_offset is not None:
                raise RuntimeError(
                    "home is derived from min_position; edit home_offset in the profile"
                )

            move = 0
            if args.mid:
                move = None
            elif args.lower is not None:
                move = -abs(args.lower)
            elif args.raise_by is not None:
                move = abs(args.raise_by)

            proposed = {}
            for servo in servos:
                if move is None:
                    target = (servo.min_position + servo.max_position) // 2
                else:
                    target = servo.home_position + move
                proposed[servo.id] = max(
                    servo.min_position, min(servo.max_position, target)
                )

            print("axis        travel        home -> new     room down/up")
            for servo in servos:
                new = proposed[servo.id]
                print(
                    f"  {servo.name:8} {servo.min_position:5}..{servo.max_position:<5} "
                    f"{servo.home_position:5} -> {new:<5}   "
                    f"{new - servo.min_position:5} / {servo.max_position - new:5}"
                )
            before = plan(servos).reach
            after = min(
                min(proposed[s.id] - s.min_position,
                    s.max_position - proposed[s.id])
                for s in servos
            )
            print(
                f"\n  tilt swing at 25%: {before} -> "
                f"{round(after * 0.25)} counts per leg"
            )
            if move == 0:
                return 0
            mid_gap = min(
                abs(proposed[s.id] - (s.min_position + s.max_position) // 2)
                for s in servos
            )
            direction = "Lowering" if (move or 0) < 0 else "Raising"
            print(
                f"\n  {direction} trades tilt range for height. The swing is capped\n"
                "  by the tighter side of the neutral, so every count away from\n"
                "  mid-travel is a count of swing given up. The new neutral sits\n"
                f"  {mid_gap} counts off mid-travel."
            )
            if after < before:
                print(
                    f"  This gives up {before - after} counts of swing per leg. "
                    "Worth it\n  only if the horns convert counts to height better "
                    "down there --\n  which `ballbal tune` measures as the plant gain."
                )
            if not (args.yes or confirm("Write this to the rig file?")):
                print("Nothing written.")
                return 1
            write_servo_fields(
                config.calibration_source or config.source,
                {s.id: {"home_position": proposed[s.id]} for s in servos},
            )
            print(f"Wrote {config.calibration_source or config.source}. Run `ballbal home` to move there.")
            return 0

        if args.command == "latency":
            from .control.latency import measure, sweep_acceleration

            if args.step_sweep:
                from .control.latency import sweep_step

                return sweep_step(rig, servos)

            if args.accel_sweep:
                return sweep_acceleration(rig, servos, step=args.step)

            if args.camera:
                from .control.latency import measure_camera
                from .vision import DEFAULT_SIZE, find_camera

                return measure_camera(
                    rig, servos, device=find_camera(args.device),
                    size=DEFAULT_SIZE, repeats=args.repeats,
                )

            return measure(
                rig, servos, step=args.step, repeats=args.repeats,
                total_delay=(
                    None if args.total_ms is None else args.total_ms / 1000.0
                ),
            )

        if args.command == "rest":
            from .setup.rest import record_rest

            return record_rest(rig, servos, assume_yes=args.yes)

        if args.command == "release":
            rig.bus.set_torque([s.id for s in servos], False)
            print(f"torque off: {', '.join(s.name for s in servos)}")
            return 0

    raise AssertionError(f"unhandled command {args.command}")


def _select_profile_interactively() -> str:
    profiles = available_profiles()
    if not profiles:
        raise FileNotFoundError("no profiles with a rig.toml found under profiles/")

    active = active_profile_name()
    print("Available profiles:")
    for index, name in enumerate(profiles, start=1):
        marker = " (active)" if name == active else ""
        reference = " [author hardware; reference only]" if name == REFERENCE_PROFILE else ""
        print(f"  {index}. {name}{marker}{reference}")

    while True:
        try:
            answer = input("Select profile by number or name: ").strip()
        except EOFError as exc:
            raise RuntimeError("profile selection needs interactive input") from exc
        if answer.isdigit():
            index = int(answer)
            if 1 <= index <= len(profiles):
                return profiles[index - 1]
        elif answer in profiles:
            return answer
        print("Invalid selection; choose one of the listed profiles.")


def _cmd_camera(args: argparse.Namespace) -> int:
    from .vision import (
        active_camera_file, list_cameras, set_active_camera, set_side_camera,
        side_camera_file,
    )

    cameras = list_cameras()
    state = active_camera_file()
    active = state.read_text(encoding="utf-8").strip() if state.is_file() else None
    side_state = side_camera_file()
    side = side_state.read_text(encoding="utf-8").strip() if side_state.is_file() else None

    if args.action == "show":
        if args.camera_id is not None:
            raise ValueError("a camera ID is only accepted by `ballbal camera set`")
        if not cameras:
            raise FileNotFoundError("no cameras found under /dev/v4l/by-id")
        print("Available cameras:")
        for index, path in enumerate(cameras, start=1):
            marker = " (active)" if str(path) == active else ""
            marker += " (side view)" if str(path) == side else ""
            print(f"  {index}. {path}{marker}")
        if active and not Path(active).exists():
            print(f"Selected but not attached: {active}")
        elif active is None:
            print("No camera selected. Run `ballbal camera set`.")
        return 0

    selector = args.camera_id or _select_camera_interactively(
        cameras, side if args.side else active
    )
    if args.side:
        selected, saved_at = set_side_camera(selector)
        print(f"Side view camera set to {selected}.")
    else:
        selected, saved_at = set_active_camera(selector)
        logger.info("active camera: %s", selected)
        print(f"Default camera set to {selected}.")
    print(f"Stored in {saved_at}.")
    return 0


def _select_camera_interactively(cameras: list[Path], active: str | None) -> str:
    if not cameras:
        raise FileNotFoundError("no cameras found under /dev/v4l/by-id")
    print("Available cameras:")
    for index, path in enumerate(cameras, start=1):
        marker = " (active)" if str(path) == active else ""
        print(f"  {index}. {path}{marker}")
    while True:
        try:
            answer = input("Select camera by ID: ").strip()
        except EOFError as exc:
            raise RuntimeError("camera selection needs interactive input") from exc
        if answer.isdigit() and 1 <= int(answer) <= len(cameras):
            return answer
        print("Invalid selection; choose one of the listed camera IDs.")


def _cmd_factory_reset(rig: Rig, servos, assume_yes: bool) -> int:  # noqa: ANN001
    """Clear per-servo calibration held in EPROM, after showing what changes."""
    print("This rewrites persistent (EPROM) registers on:")
    for servo in servos:
        offset = rig.bus.read_signed(servo.id, reg.OFFSET)
        low = rig.bus.read(servo.id, reg.MIN_ANGLE_LIMIT)
        high = rig.bus.read(servo.id, reg.MAX_ANGLE_LIMIT)
        print(
            f"  {servo.name} (id {servo.id}): "
            f"offset {offset} -> 0, limits {low}..{high} -> "
            f"{reg.FACTORY_MIN_ANGLE_LIMIT}..{reg.FACTORY_MAX_ANGLE_LIMIT}"
        )
        for register, factory in reg.FACTORY_SETTINGS:
            present = rig.bus.read(servo.id, register)
            if present != factory:
                print(f"      {register.name}: {present} -> {factory}")
    print(
        "\nAny calibration another project stored on these servos is lost, and\n"
        "reported positions will shift by the offset being cleared."
    )
    if not assume_yes:
        if input("Type 'reset' to continue: ").strip() != "reset":
            print("Aborted; nothing was written.")
            return 1
    for servo in servos:
        rig.bus.factory_reset_travel(servo.id)
        for name, was, now in rig.bus.restore_factory_settings(servo.id):
            print(f"  {servo.name}: {name} {was} -> {now}")
        print(
            f"  {servo.name}: offset 0, limits "
            f"{reg.FACTORY_MIN_ANGLE_LIMIT}..{reg.FACTORY_MAX_ANGLE_LIMIT}, "
            f"now reading {rig.bus.read(servo.id, reg.PRESENT_POSITION)}"
        )
    print("\nPositions above are in the new frame. Re-check `ballbal status`.")
    return 0


def _cmd_ports() -> int:
    paths = list_ports()
    if not paths:
        print("No serial adapters under /dev/serial/by-id.")
        return 1
    for path in paths:
        print(f"  {path}  ->  {path.resolve()}")
    return 0


def _print_reading(reading) -> None:  # noqa: ANN001 - local formatting helper
    servo, t = reading.servo, reading.telemetry
    model = reg.MODEL_NUMBERS.get(t.model, f"model {t.model}")
    flag = "" if reading.in_travel else "  <-- OUTSIDE CONFIGURED TRAVEL"
    print(f"{servo.name} (id {servo.id}, {model})")
    print(
        f"  position   {t.position:>5}  ({t.degrees:6.1f} deg)  "
        f"travel {servo.min_position}..{servo.max_position}{flag}"
    )
    print(
        f"  supply     {t.voltage:>5.1f} V   "
        f"(servo accepts {t.min_voltage_limit:.1f}-{t.max_voltage_limit:.1f} V)   "
        f"temperature {t.temperature} C"
    )
    print(
        f"  torque     {'on' if t.torque_enabled else 'off':>5}   "
        f"mode {t.mode}   status: {t.status_text}"
    )
    print(f"  speed {t.speed}   load {t.load}   current {t.current}")


if __name__ == "__main__":
    raise SystemExit(main())
