"""Guided first-time setup.

The order here exists because each step invalidates the one before it. IDs must
be unique before anything can address a servo individually; a servo reused from
another robot carries settings that make it behave differently from its
neighbours, so those go before any measurement; centring has to happen with
nothing bolted on, because a linkage fitted at the wrong angle limits travel in
one direction for good; and travel can only be measured once the linkage is on.

Every step is also a command of its own, so a single step can be repeated
without starting over.
"""

from __future__ import annotations

from dataclasses import replace
from typing import Sequence

from ..hardware import registers as reg
from ..hardware.bus import BusError
from .calibrate import _apply_to_rig_file, calibrate
from ..config import ServoConfig
from ..prompt import RULE, confirm as _confirm
from ..control.rig import Rig
from ..control.sweep import sweep

def _heading(step: int, total: int, title: str) -> None:
    print(f"\n{RULE}\nSTEP {step}/{total}  {title}\n{RULE}")


def assign_ids(rig: Rig, servos: Sequence[ServoConfig]) -> int:
    """Give each servo its ID, one at a time, with only that servo connected.

    Servos ship with the same default ID, so several on one bus answer to the
    same address and cannot be told apart. The only way out is to connect them
    one at a time.
    """
    print(
        "\nEach servo needs its own ID. Servos ship with the same one, so with\n"
        "several connected they all answer to the same address and none of them\n"
        "can be addressed individually.\n\n"
        "  >> START AT THE CAMERA ARM, THEN GO CLOCKWISE. <<\n\n"
        "  The three servos sit in a circle. The one directly under the camera\n"
        f"  arm is {servos[0].name}; from there work clockwise, seen from above, for\n"
        "  the rest. Both halves matter. Starting at the camera arm ties the\n"
        "  servo angles to the image the camera sees, so a tilt the vision loop\n"
        "  asks for comes out in the direction it meant. And the geometry that\n"
        "  turns platform tilt into three servo angles assumes the clockwise\n"
        "  order, so an axis numbered out of turn makes the platform lean the\n"
        "  wrong way -- with nothing in the readings to show why.\n\n"
        "You will connect ONE servo at a time. Disconnect all the others from\n"
        "the driver board first -- the servo being programmed must be the only\n"
        "one on the bus."
    )
    for index, servo in enumerate(servos, start=1):
        position = (
            "the one directly under the camera arm"
            if index == 1
            else f"the next one CLOCKWISE from {servos[index - 2].name}"
        )
        print(f"\n--- {index}/{len(servos)}: {servo.name}  ->  ID {servo.id} ---")
        print(
            f"  Connect ONLY the servo that is to become '{servo.name}':\n"
            f"  {position}, viewed from above.\n"
            "  Every other servo must be unplugged from the board."
        )
        while True:
            if not _confirm("Is exactly one servo connected?"):
                print("Aborted; no IDs were changed.")
                return 1

            found = rig.bus.scan(range(1, 254))
            if not found:
                print("  No servo answered. Check the cable and the supply.")
                continue
            if len(found) > 1:
                print(
                    f"  {len(found)} servos answered (IDs {sorted(found)}). "
                    "Unplug all but one -- writing an ID now would give two "
                    "servos the same address."
                )
                continue

            current = next(iter(found))
            if current == servo.id:
                print(f"  Already ID {servo.id}, nothing to do.")
                break
            print(f"  Found ID {current}, setting it to {servo.id}...")
            rig.bus.set_servo_id(current, servo.id)
            check = rig.bus.scan(range(1, 254))
            if list(check) != [servo.id]:
                print(f"  Verification failed: bus now shows {sorted(check)}.")
                return 1
            print(f"  {servo.name} is now ID {servo.id}.")
            break

    print(
        f"\nAll {len(servos)} IDs assigned. Reconnect every servo to the board "
        "now.\n"
    )
    if not _confirm("Are all servos reconnected?"):
        return 1
    found = rig.bus.scan(range(1, 254))
    expected = sorted(s.id for s in servos)
    if sorted(found) != expected:
        print(f"  Expected IDs {expected}, bus shows {sorted(found)}.")
        return 1
    print(f"  All {len(expected)} servos present: {expected}")
    return 0


def prepare_servos(rig: Rig, servos: Sequence[ServoConfig]) -> int:
    """Clear settings inherited from another robot and open travel fully."""
    print(
        "\nA servo reused from another robot keeps that robot's settings in its\n"
        "own memory: a homing offset that shifts every position it reports,\n"
        "angle limits that stop it before your mechanism does, and sometimes a\n"
        "torque cap. Those are cleared now, before anything is measured."
    )
    changes = 0
    for servo in servos:
        offset = rig.bus.read_signed(servo.id, reg.OFFSET)
        low = rig.bus.read(servo.id, reg.MIN_ANGLE_LIMIT)
        high = rig.bus.read(servo.id, reg.MAX_ANGLE_LIMIT)
        pending = [
            f"offset {offset} -> 0" if offset else None,
            (
                f"angle limits {low}..{high} -> "
                f"{reg.FACTORY_MIN_ANGLE_LIMIT}..{reg.FACTORY_MAX_ANGLE_LIMIT}"
                if (low, high)
                != (reg.FACTORY_MIN_ANGLE_LIMIT, reg.FACTORY_MAX_ANGLE_LIMIT)
                else None
            ),
        ]
        for register, factory in reg.FACTORY_SETTINGS:
            present = rig.bus.read(servo.id, register)
            if present != factory:
                pending.append(f"{register.name} {present} -> {factory}")
        pending = [p for p in pending if p]
        print(f"\n  {servo.name} (id {servo.id}):")
        if not pending:
            print("    already at factory settings")
        for line in pending:
            print(f"    {line}")
            changes += 1

    if changes and not _confirm(f"Write these {changes} changes?"):
        print("Aborted; nothing was written.")
        return 1

    for servo in servos:
        rig.bus.factory_reset_travel(servo.id)
        rig.bus.restore_factory_settings(servo.id)
    print("\n  Done. Reported positions have shifted into the new frame.")
    return 0


def open_travel(rig: Rig, servos: Sequence[ServoConfig]) -> int:
    """Widen the rig file's travel to a full turn, ready for centring."""
    source = rig.config.calibration_source or rig.config.source
    if source is None:
        raise BusError("the rig was not loaded from a file, so it cannot be updated")

    from .calibrate import AxisCalibration

    wide = [
        AxisCalibration(
            name=servo.name,
            id=servo.id,
            observed_min=reg.FACTORY_MIN_ANGLE_LIMIT,
            observed_max=reg.FACTORY_MAX_ANGLE_LIMIT,
            margin=0,
            min_position=reg.FACTORY_MIN_ANGLE_LIMIT,
            max_position=reg.FACTORY_MAX_ANGLE_LIMIT,
            home_position=reg.CENTER_POSITION,
        )
        for servo in servos
    ]
    if rig.config.home_offset is None:
        _apply_to_rig_file(source, wide)
    # Writing the file is only half the job. Every later step in this same run
    # clamps its goals against the config already in memory, so widening the
    # file alone left `centre` still refusing to leave the travel that had just
    # been opened -- it stalled against a limit no longer on disk.
    chosen = {servo.id for servo in servos}
    rig.config = replace(
        rig.config,
        servos=tuple(
            replace(
                servo,
                min_position=reg.FACTORY_MIN_ANGLE_LIMIT,
                max_position=reg.FACTORY_MAX_ANGLE_LIMIT,
                home_position=reg.CENTER_POSITION,
            )
            if servo.id in chosen
            else servo
            for servo in rig.config.servos
        ),
    )
    # ServoBus keeps its own guard because setup code and diagnostics can write
    # goals without going through Rig.goto. Keep that guard in sync with the
    # temporary in-memory setup range or it will still reject the 2048 centre
    # goal using the profile limits loaded when the process started.
    rig.bus.set_limits(
        {
            servo.id: (servo.min_position, servo.max_position)
            for servo in rig.config.servos
        }
    )
    print(
        f"  Live setup travel opened to "
        f"{reg.FACTORY_MIN_ANGLE_LIMIT}..{reg.FACTORY_MAX_ANGLE_LIMIT}, "
        f"home {reg.CENTER_POSITION}, on {len(wide)} axes.\n"
        "  This temporary range is not stored; calibration will write the new minimum.\n"
        "  Nothing is bolted on yet, so there is nothing for limits to protect."
    )
    return 0


def centre_bare(rig: Rig, servos: Sequence[ServoConfig]) -> int:
    """Move every servo to encoder mid-scale, with nothing attached."""
    print(
        "\n  !! NOTHING MAY BE BOLTED TO THE SERVO HORNS FOR THIS STEP !!\n\n"
        "  Every servo goes to the centre of its 360 deg range. That is the\n"
        "  reference the mechanism gets built around: with the horns fitted at\n"
        "  centre, each axis has equal travel in both directions. Fitted at a\n"
        "  random angle instead, an axis runs out of travel on one side and\n"
        "  that cannot be corrected in software later.\n\n"
        "  If a horn or linkage is already fitted, remove it now."
    )
    if not _confirm("Are all servo horns free of any linkage?"):
        print("Aborted; nothing moved.")
        return 1

    ids = [s.id for s in servos]
    start = rig.positions(servos)
    rig.preflight(servos)
    with rig.bus.torque(ids):
        final = rig.move_to({s.id: reg.CENTER_POSITION for s in servos})
    for servo in servos:
        print(
            f"  {servo.name}: {start[servo.id]} -> {final[servo.id]} "
            f"(centre {reg.CENTER_POSITION})"
        )
    return 0


def mount_prompt(servos: Sequence[ServoConfig]) -> int:
    """Hold until the whole mechanism is assembled at the centred position.

    Not just the lower arms: the *complete* platform. On a parallel mechanism
    the three legs constrain each other, so an axis with its upper link and the
    platform plate still off swings far further than the assembled machine ever
    could. Calibrating in that state records a range the finished rig cannot
    reach -- and that range becomes the hard limit everything afterwards trusts.
    """
    print(
        "\n  All servos are now at centre and torque is released.\n\n"
        "  ASSEMBLE THE COMPLETE PLATFORM NOW.\n\n"
        "  Everything: servo horns, lower arms, upper links, and the platform\n"
        "  plate. Not just the lower arms -- the next step measures how far each\n"
        "  axis can travel, and on a parallel mechanism the three legs limit one\n"
        "  another. With the upper links or the plate still off, an axis swings\n"
        "  much further than the finished machine ever will, and that wider range\n"
        "  becomes the hard limit every later command is clamped to.\n\n"
        "  Fit everything so the mechanism sits in its neutral pose -- platform\n"
        "  level -- while the servos stay exactly where they are. Do not turn a\n"
        "  horn to make a link reach: adjust the link instead. The horn splines\n"
        "  give about 0.3 deg of resolution; anything finer is corrected\n"
        "  afterwards through home_position in the rig file, not by refitting.\n\n"
        "  Servo positions to keep, for reference:"
    )
    for servo in servos:
        print(f"    {servo.name} (id {servo.id}): {reg.CENTER_POSITION}")
    if not _confirm(
        "Is the platform COMPLETELY assembled -- arms, links and plate -- and level?"
    ):
        print("Stopped. Re-run `ballbal setup` when the mechanism is complete.")
        return 1
    return 0


SETUP_STEPS = ("set-ids", "prepare", "center", "mount", "calibrate", "sweep")
"""The procedure in order, by name, for ``--from``.

Naming the steps is what makes the run resumable. Falling out half way through
-- a dropped serial link, a servo that needed replugging -- used to mean
starting at the addresses again, and re-running `set-ids` on servos that already
have their IDs is not harmless: it walks you back through unplugging all but
one, with a live chance of writing a duplicate address.
"""


def guided_setup(
    rig: Rig,
    servos: Sequence[ServoConfig],
    *,
    skip_ids: bool = False,
    start: str | None = None,
) -> int:
    """Run the whole first-time procedure in order, or resume part way in."""

    def run_ids() -> int:
        return assign_ids(rig, servos)

    def run_prepare() -> int:
        nonlocal servos
        if prepare_servos(rig, servos):
            return 1
        open_travel(rig, servos)
        # Carry the widened servos forward; the ones captured before the call
        # still describe the old travel, and every step after this one is
        # handed them.
        servos = rig.config.select([servo.name for servo in servos])
        return 0

    def run_center() -> int:
        return centre_bare(rig, servos)

    def run_mount() -> int:
        return mount_prompt(servos)

    def run_calibrate() -> int:
        print(
            "  Torque is released. Move the complete platform by hand into its\n"
            "  common lower pose. After one confirmation, all three positions are\n"
            "  read together. Home and maximum are fixed offsets above each minimum."
        )
        return calibrate(rig, servos)

    def run_sweep() -> int:
        print(
            "\n  This is the first time the servos drive the limits you just measured,\n"
            "  and the first movement since the mechanism went on. Each axis runs\n"
            "  alone first, then all three together.\n\n"
            "  THE PLATFORM IS ABOUT TO MOVE THROUGH ITS FULL TRAVEL.\n\n"
            "  Stand clear, keep hands out of the mechanism, and stay near the power\n"
            "  switch. Ctrl-C releases torque at any point -- note that this drops\n"
            "  the platform wherever gravity takes it.\n\n"
            "  The plan is printed next and asks again before anything moves."
        )
        if not _confirm("Ready to start the sweep?"):
            print(
                "\nStopped before the sweep. Everything up to here is saved -- run\n"
                "`ballbal sweep` when you are ready."
            )
            return 1
        reloaded = type(rig.config).load(rig.config.source)
        rig.config = reloaded
        rig.bus.set_limits(
            {s.id: (s.min_position, s.max_position) for s in reloaded.servos}
        )
        # Not assume_yes: sweep prints the leg-by-leg plan and confirms it
        # itself. Suppressing that was how this step started moving with no
        # warning at all.
        return sweep(rig, reloaded.servos, dwell=0.3)

    # name, command behind it (None: no standalone command), heading, blurb
    plan = [
        ("set-ids", "set-ids", "Assign servo IDs", "give each servo its own address", run_ids),
        ("prepare", "prepare", "Clear settings from a previous robot", "clear settings from a previous robot", run_prepare),
        ("center", "center", "Centre the servos -- nothing attached", "centre the servos, nothing attached", run_center),
        ("mount", None, "Fit the mechanism", "... assemble the complete platform ...", run_mount),
        ("calibrate", "calibrate", "Measure the common lower pose", "place the platform at min and read every axis", run_calibrate),
        ("sweep", "sweep", "Verify the limits under power", "drive the measured limits under power", run_sweep),
    ]

    # --skip-ids is the old spelling of starting at `prepare`; --from wins.
    if start is None and skip_ids:
        start = "prepare"
    if start is not None and start not in SETUP_STEPS:
        raise ValueError(
            f"unknown setup step {start!r}; choose one of {', '.join(SETUP_STEPS)}"
        )
    first = SETUP_STEPS.index(start) if start else 0
    todo = plan[first:]

    listing = []
    number = 0
    for index, (name, command, _heading_text, blurb, _runner) in enumerate(plan):
        skipped = index < first
        if command is None:
            listing.append(f"     {blurb}" + ("   (skipped)" if skipped else ""))
        else:
            if not skipped:
                number += 1
            marker = "  -." if skipped else f"  {number}."
            listing.append(
                f"{marker} ballbal {command:<12} -- {blurb}"
                + ("   (skipped)" if skipped else "")
            )

    print(
        f"\n{RULE}\n"
        "BALL BALANCER -- FIRST-TIME SETUP\n"
        f"{RULE}\n"
        "This runs the whole procedure in order. Each step is also its own\n"
        "command, so any single one can be repeated later without starting over.\n\n"
        + "\n".join(listing)
        + "\n\nHave the linkages and whatever tool fits your horn screws to hand.\n"
        "Nothing moves without asking."
    )
    if first:
        done = "step" if first == 1 else "steps"
        print(
            f"\nResuming at '{SETUP_STEPS[first]}'; skipping the {first} "
            f"{done} above it."
        )
    if not _confirm("Begin?"):
        return 1

    steps = len(todo)
    for step, (_name, _command, heading, _blurb, runner) in enumerate(todo, start=1):
        _heading(step, steps, heading)
        if runner():
            return 1

    print(
        f"\n{RULE}\nSETUP COMPLETE\n{RULE}\n"
        f"Travel limits are stored in {getattr(rig.config, 'calibration_source', None) or rig.config.source} and are enforced on\n"
        "every goal. Re-run any single step with its own command, or resume the\n"
        "whole procedure part way in with `ballbal setup --from <step>`\n"
        f"({', '.join(SETUP_STEPS)}).\n"
    )
    return 0


def recover(rig: Rig, servos: Sequence[ServoConfig], *, speed: int) -> int:
    """Bring axes that have settled outside their travel back to home.

    Other commands also recover on their own -- every goal is clamped into
    travel, so the only move available from outside is one heading back in.
    This one exists for when that is the whole intention: it moves one axis at a
    time at reduced speed, because whatever pushed them out may still be there,
    and it says what it found before touching anything.
    """
    readings = rig.preflight(servos)
    outside = [r for r in readings if not r.in_travel]
    print("Axis positions:")
    for reading in readings:
        servo = reading.servo
        mark = "" if reading.in_travel else "   <-- outside travel"
        print(
            f"  {servo.name}: {reading.telemetry.position} "
            f"(travel {servo.min_position}..{servo.max_position}){mark}"
        )
    if not outside:
        print("\nAll axes are already inside their travel; nothing to recover.")
        return 0

    print(
        f"\n{len(outside)} axis/axes will be moved back to home, one at a time,\n"
        f"at {speed} counts/s."
    )
    if not _confirm("Move them back?"):
        print("Aborted; nothing moved.")
        return 1

    for servo in servos:
        with rig.bus.torque([servo.id]):
            final = rig.move_to(
                {servo.id: servo.home_position}, speed=speed, acceleration=40
            )
        print(f"  {servo.name}: -> {final[servo.id]} (home {servo.home_position})")
    return 0
