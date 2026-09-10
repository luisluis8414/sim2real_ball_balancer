"""Split the loop's dead time between the camera and the mechanism.

The whole design is governed by one number. A loop crossing over at ``w`` with a
dead time ``tau`` has spent ``w*tau`` radians of phase before any gain is
chosen, so the usable bandwidth is essentially ``0.45/tau``. Measured on this
rig the total is 136 ms, which caps it at 0.53 Hz -- and at 0.5 m/s the ball
travels 68 mm, two thirds of the plate's radius, between being seen and being
answered.

Cutting that is the only thing that makes a balancer quick. But "buy a faster
camera" and "buy better servos" are different bills, and which one to pay
depends on how the 136 ms divides. This measures the mechanism's share
directly: the servo bus answers in 0.52 ms, three orders of magnitude below the
delay being measured, so a step can be commanded and the reply polled densely
enough to see exactly when the axis starts to move and how fast it gets there.

Whatever is left over after subtracting that is the camera's.
"""

from __future__ import annotations

import math
import statistics
import time
from dataclasses import dataclass
from typing import Sequence

from ..hardware import registers as reg
from ..config import ServoConfig
from .rig import Rig


@dataclass(frozen=True)
class StepResponse:
    """One commanded step, as the axis actually answered it."""

    axis: str
    step: int
    dead_time: float
    """Seconds between the goal being written and the axis moving at all."""
    rise_time: float
    """Seconds from the goal to 63% of the step -- the first-order constant."""
    settle_time: float
    """Seconds to come within tolerance and stay."""
    reached: int


def measure_axis(
    rig: Rig,
    servo: ServoConfig,
    *,
    step: int = 120,
    tolerance: int = 8,
    timeout: float = 1.5,
    moved_threshold: int = 3,
    acceleration: int | None = None,
    speed: int | None = None,
) -> StepResponse | None:
    """Command one step and watch the axis answer it.

    The dead time is taken as the first sample that has moved more than a few
    counts, not the first that has moved at all: the position register dithers
    by a count or two while the servo holds, and treating that as motion would
    report a dead time of zero for a mechanism that has not begun to move.
    """
    start_position = rig.bus.read(servo.id, reg.PRESENT_POSITION)
    target = servo.clamp(start_position + step)
    actual_step = target - start_position
    if abs(actual_step) < abs(step) // 2:
        return None  # too near a limit for this step to say anything

    samples: list[tuple[float, int]] = []
    started = time.perf_counter()
    rig.goto({servo.id: target}, acceleration=acceleration, speed=speed)
    deadline = started + timeout
    while time.perf_counter() < deadline:
        position = rig.bus.read(servo.id, reg.PRESENT_POSITION)
        samples.append((time.perf_counter() - started, position))
        if len(samples) > 20 and abs(position - target) <= tolerance:
            recent = [p for _, p in samples[-15:]]
            if all(abs(p - target) <= tolerance for p in recent):
                break

    dead_time = rise_time = settle_time = float("nan")
    for elapsed, position in samples:
        if abs(position - start_position) > moved_threshold:
            dead_time = elapsed
            break
    threshold = start_position + 0.63 * actual_step
    for elapsed, position in samples:
        if (actual_step > 0 and position >= threshold) or (
            actual_step < 0 and position <= threshold
        ):
            rise_time = elapsed
            break
    for index, (elapsed, position) in enumerate(samples):
        if abs(position - target) <= tolerance and all(
            abs(p - target) <= tolerance for _, p in samples[index:]
        ):
            settle_time = elapsed
            break

    return StepResponse(
        axis=servo.name,
        step=actual_step,
        dead_time=dead_time,
        rise_time=rise_time,
        settle_time=settle_time,
        reached=samples[-1][1] if samples else start_position,
    )


def measure(
    rig: Rig,
    servos: Sequence[ServoConfig],
    *,
    step: int = 120,
    repeats: int = 4,
    total_delay: float | None = None,
) -> int:
    """Step every axis both ways and report where the loop's delay lives."""
    print(
        f"\n  Stepping each axis {step} counts, {repeats} times each way, and\n"
        "  polling the position as fast as the bus allows.\n"
        "  Take the ball off the plate first -- the platform will move."
    )
    rig.preflight(servos)
    results: list[StepResponse] = []

    with rig.bus.torque([s.id for s in servos], hold=False):
        rig.goto({s.id: s.home_position for s in servos}, settle=0.8)
        for servo in servos:
            for repeat in range(repeats):
                direction = 1 if repeat % 2 == 0 else -1
                response = measure_axis(rig, servo, step=step * direction)
                if response is not None:
                    results.append(response)
                time.sleep(0.25)
            rig.goto({servo.id: servo.home_position}, settle=0.4)
        rig.goto({s.id: s.home_position for s in servos}, settle=0.6)

    if not results:
        print("  No axis could take the step; try a smaller --step.")
        return 1

    print("\n  axis      step    dead   rise(63%)   settle")
    for response in results:
        print(
            f"    {response.axis:8} {response.step:+5}  "
            f"{response.dead_time * 1000:5.0f}ms   {response.rise_time * 1000:5.0f}ms   "
            f"{response.settle_time * 1000:5.0f}ms"
        )

    dead = statistics.median(
        r.dead_time for r in results if r.dead_time == r.dead_time
    )
    rise = statistics.median(
        r.rise_time for r in results if r.rise_time == r.rise_time
    )
    # Only the dead time counts as delay. The rise is how long the axis takes
    # to *cover the step*, which is rate limiting, not lag: it scales with the
    # step size, and in the loop the axis is never asked to make a step this
    # big -- a new goal arrives every 32 ms, a few counts from the last. Adding
    # the rise in reported 238 ms of mechanism inside a 136 ms loop, which is
    # how the error announced itself.
    mechanism = dead
    print(
        f"\n  mechanism dead time: {mechanism * 1000:.0f} ms"
        f"   (rise {rise * 1000:.0f} ms over {step} counts is slew, not lag --"
        f" it shrinks with the step)"
    )
    if total_delay is None:
        return 0
    if mechanism >= total_delay:
        print(
            f"  loop total (from `ballbal tune`): {total_delay * 1000:.0f} ms\n"
            "  The mechanism alone accounts for all of it, which cannot be right:\n"
            "  one of the two measurements is off. Re-run `tune` on a log where\n"
            "  the ball actually moved about."
        )
        return 1
    camera = total_delay - mechanism
    print(
        f"  loop total (from `ballbal tune`): {total_delay * 1000:.0f} ms\n"
        f"  -> camera and vision: {camera * 1000:.0f} ms "
        f"({camera / total_delay * 100:.0f}%)\n"
        f"  -> mechanism:         {mechanism * 1000:.0f} ms "
        f"({mechanism / total_delay * 100:.0f}%)"
    )
    if abs(camera - mechanism) < 0.2 * total_delay:
        print(
            "\n  The two halves are close to even, so neither alone decides it:\n"
            "  halving only one caps the gain at about a third."
        )
    else:
        bigger = "camera" if camera > mechanism else "mechanism"
        print(f"\n  The {bigger} is the larger half; spend there first.")
    return 0


def sweep_acceleration(
    rig: Rig,
    servos: Sequence[ServoConfig],
    *,
    values: Sequence[int] = (30, 60, 100, 150, 200, 254),
    step: int = 120,
    repeats: int = 2,
) -> int:
    """Time the dead time at several ramp settings.

    Part of the 67 ms measured here is the servo's own acceleration ramp: it
    starts from rest, so the first few counts take longer the gentler the ramp.
    That part is free to change -- it is one number in the rig file -- and this
    says what it is worth before anything is bought.

    The rise time is reported alongside because the trade runs through it: a
    harsher ramp reaches the goal sooner but jerks the platform, and on a
    parallel mechanism a jerk is a load on all three legs at once.
    """
    print(
        f"\n  Stepping {step} counts at each acceleration setting.\n"
        "  Take the ball off the plate -- the platform will move, harder at the"
        " top end."
    )
    rig.preflight(servos)
    servo = servos[0]
    print(f"\n  using {servo.name}\n")
    print("  acceleration    dead     rise(63%)")
    best: tuple[float, int] | None = None
    with rig.bus.torque([s.id for s in servos], hold=False):
        rig.goto({s.id: s.home_position for s in servos}, settle=0.8)
        for value in values:
            deads: list[float] = []
            rises: list[float] = []
            for repeat in range(repeats * 2):
                direction = 1 if repeat % 2 == 0 else -1
                response = measure_axis(
                    rig, servo, step=step * direction, acceleration=value
                )
                if response and response.dead_time == response.dead_time:
                    deads.append(response.dead_time)
                    rises.append(response.rise_time)
                time.sleep(0.2)
            if not deads:
                print(f"  {value:>10}      -- no movement --")
                continue
            dead = statistics.median(deads)
            rise = statistics.median(rises)
            print(f"  {value:>10}    {dead * 1000:5.0f}ms      {rise * 1000:5.0f}ms")
            if best is None or dead < best[0]:
                best = (dead, value)
        rig.goto({s.id: s.home_position for s in servos}, settle=0.6)

    if best is None:
        return 1
    dead, value = best
    print(
        f"\n  Lowest dead time {dead * 1000:.0f} ms at acceleration {value}."
    )
    print(
        "  Set it in rig.toml, then re-run `ballbal tune` on a fresh log: the\n"
        "  loop's total is what actually decides the bandwidth, and this only\n"
        "  moves one part of it."
    )
    return 0


def measure_camera(
    rig: Rig,
    servos: Sequence[ServoConfig],
    *,
    device: str,
    size: tuple[int, int],
    step: int = 260,
    repeats: int = 4,
) -> int:
    """Time the camera against the bus, using one event both can see.

    Everything else here is estimated: the exposure is known, the readout and
    the queue are guesses, and guesses are what a spending decision should not
    rest on. But a servo step is an event with two independent witnesses. The
    bus reports the axis moving within 0.52 ms of it happening. The camera
    reports the plate moving whenever its pipeline gets round to it. Subtract
    the two and what is left is the pipeline, measured rather than assumed.

    The camera is read on its own thread because ``read()`` blocks for a frame
    period; polling the bus behind it would inherit the very delay being
    measured.
    """
    import threading

    import cv2
    import numpy as np

    from ..vision import lock_camera, open_camera

    lock_camera(device)
    capture = open_camera(device, size)
    frames: list[tuple[float, np.ndarray]] = []
    stop = threading.Event()

    def grab() -> None:
        while not stop.is_set():
            ok, frame = capture.read()
            if not ok:
                break
            frames.append(
                (time.perf_counter(),
                 cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.int16))
            )

    print(
        f"\n  Stepping {step} counts and timing when the bus sees it against\n"
        "  when the camera does. Keep the plate in view and still otherwise."
    )
    rig.preflight(servos)
    servo = servos[0]
    lags: list[float] = []

    reader = threading.Thread(target=grab, daemon=True)
    reader.start()
    try:
        time.sleep(1.0)  # let the opening transient pass
        with rig.bus.torque([s.id for s in servos], hold=False):
            rig.goto({s.id: s.home_position for s in servos}, settle=1.0)
            for repeat in range(repeats):
                direction = 1 if repeat % 2 == 0 else -1
                start = rig.bus.read(servo.id, reg.PRESENT_POSITION)
                target = servo.clamp(start + step * direction)
                if abs(target - start) < abs(step) // 2:
                    continue

                frames.clear()
                time.sleep(0.4)
                if len(frames) < 5:
                    print("  camera is not delivering frames")
                    return 1
                baseline = frames[-1][1]
                mark = len(frames)

                rig.goto({servo.id: target})
                bus_moved = None
                deadline = time.perf_counter() + 1.0
                while time.perf_counter() < deadline:
                    if abs(
                        rig.bus.read(servo.id, reg.PRESENT_POSITION) - start
                    ) > 3:
                        bus_moved = time.perf_counter()
                        break
                if bus_moved is None:
                    continue
                time.sleep(0.5)

                # First frame that differs from the resting plate by more than
                # the sensor noise floor, which the pre-step frames measure.
                noise = max(
                    float(np.mean(np.abs(frames[i][1] - baseline)))
                    for i in range(max(0, mark - 4), mark)
                ) if mark >= 2 else 1.0
                camera_saw = None
                for when, image in frames[mark:]:
                    if float(np.mean(np.abs(image - baseline))) > noise * 3 + 1:
                        camera_saw = when
                        break
                if camera_saw is None:
                    continue
                lag = camera_saw - bus_moved
                lags.append(lag)
                print(
                    f"    step {step * direction:+5}: bus saw it, camera "
                    f"{lag * 1000:5.0f} ms later"
                )
                rig.goto({servo.id: start}, settle=0.6)
            rig.goto({s.id: s.home_position for s in servos}, settle=0.6)
    finally:
        stop.set()
        reader.join(timeout=2.0)
        capture.release()

    if not lags:
        print("  Could not see the plate move. Is it in frame and lit?")
        return 1
    median = statistics.median(lags)
    print(
        f"\n  camera pipeline: {median * 1000:.0f} ms"
        f"   (from {len(lags)} steps, spread "
        f"{(max(lags) - min(lags)) * 1000:.0f} ms)"
    )
    print(
        "  That is exposure, readout, transport and queue together -- everything\n"
        "  between the world changing and the program being able to see it."
    )
    return 0


def sweep_step(
    rig: Rig,
    servos: Sequence[ServoConfig],
    *,
    sizes: Sequence[int] = (30, 60, 120, 240, 480, 800),
    repeats: int = 2,
) -> int:
    """Time steps of different sizes, to see which limit the axis is under.

    It decides which knob matters. Under an acceleration limit the axis is
    ramping the whole way and distance goes as ``t^2``, so the time to cover it
    grows as the square root -- doubling the step costs 1.41x. Under a speed
    limit it spends most of the move at a constant rate and the time doubles.

    If the moves a balancing loop makes never reach top speed -- and they are
    small, a few counts per 32 ms update -- then the ``speed`` setting does
    nothing at all and only ``acceleration`` is worth touching.
    """
    print(
        "\n  Stepping several distances and timing each. Take the ball off --\n"
        "  the larger steps swing the platform a long way."
    )
    rig.preflight(servos)
    servo = servos[0]
    print(f"\n  using {servo.name}\n")
    print("  step    dead    rise(63%)   vs previous   expected if...")
    previous: tuple[int, float] | None = None
    with rig.bus.torque([s.id for s in servos], hold=False):
        rig.goto({s.id: s.home_position for s in servos}, settle=0.8)
        for size in sizes:
            rises: list[float] = []
            deads: list[float] = []
            for repeat in range(repeats * 2):
                direction = 1 if repeat % 2 == 0 else -1
                response = measure_axis(
                    rig, servo, step=size * direction, timeout=3.0
                )
                if response and response.rise_time == response.rise_time:
                    rises.append(response.rise_time)
                    deads.append(response.dead_time)
                time.sleep(0.2)
            if not rises:
                print(f"  {size:5}   -- no usable move (too near a limit?) --")
                continue
            rise = statistics.median(rises)
            dead = statistics.median(deads)
            note = ""
            if previous is not None:
                grew = (rise - dead) / max(previous[1], 1e-6)
                ratio = size / previous[0]
                note = (
                    f"  x{grew:4.2f}      accel: x{math.sqrt(ratio):.2f}"
                    f"   speed: x{ratio:.2f}"
                )
            print(
                f"  {size:5}  {dead * 1000:5.0f}ms   {rise * 1000:6.0f}ms{note}"
            )
            previous = (size, rise - dead)
        rig.goto({s.id: s.home_position for s in servos}, settle=0.6)
    print(
        "\n  Compare the middle column against the two on the right. Matching\n"
        "  the square-root column means the axis never reaches top speed, and\n"
        "  the `speed` setting in rig.toml is doing nothing for balancing."
    )
    return 0
