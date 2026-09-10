"""Read gains out of a balance log instead of guessing at them.

Two numbers decide what the gains should be, and neither can be worked out from
the drawing: how hard the plate accelerates the ball per unit of commanded tilt,
and how long the loop takes to notice. Both are in any log where the ball moved.

The plant is a double integrator, so with proportional and derivative action the
closed loop is a plain second-order system:

    x'' = G*u,   u = -kp*x - kd*x'   =>   x'' + G*kd*x' + G*kp*x = 0

which gives ``omega = sqrt(G*kp)`` and ``zeta = kd/2 * sqrt(G/kp)``. Turn that
around and the gains follow from a chosen frequency and damping:

    kp = omega^2 / G      kd = 2*zeta*omega / G

The only judgement left is how fast to ask for, and that is bounded by the dead
time: a loop crossing over at ``omega`` with a delay ``tau`` has already spent
``omega*tau`` radians of its phase margin before any gain is chosen.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path

DEFAULT_ZETA = 0.8
"""Target damping. Overshoot is roughly exp(-pi*z/sqrt(1-z^2)): 0.7 leaves 5%,
0.8 leaves 1.5%, 1.0 none at all but takes noticeably longer to arrive."""

PHASE_BUDGET = 0.45
"""Radians of phase the dead time may eat at crossover -- about 26 degrees.

The rest of the margin has to cover the derivative filter, the servos' own lag
and the fact that G is measured, not exact.
"""


@dataclass(frozen=True)
class Fit:
    """What one axis of a log says about the mechanism."""

    axis: str
    gain: float
    """mm/s^2 per unit of commanded tilt."""
    delay: float
    """Seconds between a tilt being commanded and the ball answering."""
    correlation: float
    samples: int

    @property
    def trustworthy(self) -> bool:
        return self.samples >= 100 and self.correlation >= 0.3 and self.gain > 0


def _read(path: Path) -> list[dict]:
    with path.open() as handle:
        return [row for row in csv.DictReader(handle) if row.get("found") == "1"]


def _series(rows: list[dict], axis: str) -> tuple[list[float], list[float], list[float]]:
    times, positions, tilts = [], [], []
    for row in rows:
        try:
            times.append(float(row["t_s"]))
            positions.append(float(row[f"{axis}_mm"]))
            tilts.append(float(row[f"tilt_{axis}"]))
        except (KeyError, ValueError):
            continue
    return times, positions, tilts


def _velocity(times: list[float], positions: list[float], half: int) -> list[float | None]:
    """Velocity by straight-line fit over a short span, not a difference.

    Differentiating twice to get acceleration was the first attempt and it does
    not survive contact with the data: it squares the measurement noise, and
    because the commanded tilt is computed from that same noisy position, the
    noise correlates with itself and inflates the fitted gain. Checked against
    simulated plants of known gain, that route returned 681 for a true 400.

    One differentiation, over a window, is well behaved -- and the plant law can
    be written to need only that.
    """
    out: list[float | None] = [None] * len(positions)
    for i in range(half, len(positions) - half):
        window = slice(i - half, i + half + 1)
        ts, xs = times[window], positions[window]
        n = len(ts)
        mean_t = sum(ts) / n
        mean_x = sum(xs) / n
        var = sum((t - mean_t) ** 2 for t in ts)
        if var <= 0:
            continue
        out[i] = sum((t - mean_t) * (x - mean_x) for t, x in zip(ts, xs)) / var
    return out


def _correlate(a: list[float], b: list[float]) -> float:
    n = len(a)
    if n < 2:
        return 0.0
    mean_a, mean_b = sum(a) / n, sum(b) / n
    va = sum((x - mean_a) ** 2 for x in a)
    vb = sum((x - mean_b) ** 2 for x in b)
    if va <= 0 or vb <= 0:
        return 0.0
    cov = sum((x - mean_a) * (y - mean_b) for x, y in zip(a, b))
    return cov / math.sqrt(va * vb)


def _windows(
    times: list[float],
    velocities: list[float | None],
    tilts: list[float],
    *,
    span: int,
    lag: int,
) -> list[tuple[float, float]]:
    """Pairs of (integrated tilt, change in velocity) over each window.

    Integrating the command and differencing the velocity turns ``x'' = G*u``
    into ``dv = G * integral(u)``, which is a straight line through the origin
    with one differentiation instead of two.
    """
    pairs: list[tuple[float, float]] = []
    for start in range(lag, len(times) - span):
        end = start + span
        v0, v1 = velocities[start], velocities[end]
        if v0 is None or v1 is None:
            continue
        area = 0.0
        for i in range(start, end):
            step = times[i + 1] - times[i]
            if step <= 0:
                break
            area += tilts[i - lag] * step
        else:
            pairs.append((area, v1 - v0))
    return pairs


def fit_axis(path: Path, axis: str, *, max_lag: int = 10) -> Fit | None:
    """Estimate plant gain and dead time for one axis.

    The delay is found by sliding the commanded tilt against the response and
    keeping the shift that fits best. It matters as much as the gain: a loop
    tuned as though it were instant is tuned too fast, and the overshoot that
    follows looks exactly like too little damping.
    """
    rows = _read(path)
    if len(rows) < 90:
        return None
    times, positions, tilts = _series(rows, axis)
    if len(times) < 90:
        return None
    velocities = _velocity(times, positions, half=3)
    span = 8

    best: tuple[float, int, float] = (0.0, 0, 0.0)
    for lag in range(max_lag + 1):
        pairs = _windows(times, velocities, tilts, span=span, lag=lag)
        if len(pairs) < 60:
            continue
        score = _correlate([p[0] for p in pairs], [p[1] for p in pairs])
        if abs(score) > abs(best[0]):
            denominator = sum(a * a for a, _ in pairs)
            slope = (
                sum(a * d for a, d in pairs) / denominator
                if denominator > 0
                else 0.0
            )
            best = (score, lag, slope)
    correlation, lag, slope = best
    if slope <= 0:
        return None

    step = (times[-1] - times[0]) / max(len(times) - 1, 1)
    pairs = _windows(times, velocities, tilts, span=span, lag=lag)
    return Fit(
        axis=axis,
        gain=slope,
        delay=lag * step,
        correlation=correlation,
        samples=len(pairs),
    )


def crossover_ratio(zeta: float) -> float:
    """How far above ``omega_n`` a PD-controlled double integrator crosses over.

    Not 1, which is the mistake this exists to stop. Solving
    ``|G(kp + kd*jw)/(jw)^2| = 1`` in terms of ``r = w/omega_n`` gives
    ``r^4 - 4*zeta^2*r^2 - 1 = 0``, so ``r = sqrt(2*zeta^2 + sqrt(4*zeta^4 + 1))``
    -- about 1.70 at zeta 0.8.

    Budgeting the dead time's phase at ``omega_n`` instead of at the real
    crossover under-counts it by that factor. Done that way against a measured
    136 ms of delay, the "improved" gains came out with 26 degrees of phase
    margin where the untuned ones already had 30.
    """
    return math.sqrt(2 * zeta * zeta + math.sqrt(4 * zeta**4 + 1))


def gains_for(gain: float, delay: float, *, zeta: float = DEFAULT_ZETA) -> tuple[float, float, float]:
    """kp, ki, kd for a measured plant, as fast as the dead time allows."""
    ratio = crossover_ratio(zeta)
    crossover = PHASE_BUDGET / delay if delay > 0 else 18.8
    crossover = min(crossover, 18.8)  # 3 Hz; past that 31 fps sampling decides
    omega = crossover / ratio
    kp = omega * omega / gain
    kd = 2.0 * zeta * omega / gain
    # An integral, placed an octave and a half below crossover. This function
    # used to return zero here, on the arithmetic that a plate half a degree off
    # level would hold the ball "within a millimetre" -- which was wrong by an
    # order of magnitude. The standing error of a PD loop is
    # disturbance/(G*kp), and at these gains half a degree of tilt is 51 mm/s^2
    # against a G*kp of 3.8, so the ball parks 13.6 mm out and stays there.
    #
    # At crossover/8 it costs about 7 degrees of phase, leaving the margin
    # near 37, and it stays far inside the stability condition for a double
    # integrator with PID: s^3 + G*kd*s^2 + G*kp*s + G*ki is stable while
    # ki < G*kd*kp, which here is 0.0147 against the 0.0020 returned.
    ki = kp * crossover / 8.0
    return kp, ki, kd


@dataclass(frozen=True)
class PlaneFit:
    """What a log says when both axes are read together.

    Fitting the axes separately hides the one failure that matters most. If the
    stored bearing is wrong, the commanded tilt is not weaker than it should be,
    it is *rotated* -- and a rotation couples the axes rather than changing
    either one's gain. At 90 degrees the correction stands square to the error,
    so it never opposes the ball's displacement, only turns it: the ball keeps a
    constant distance from the middle and circles.
    """

    gain: float
    rotation_deg: float
    delay: float
    correlation: float
    samples: int

    @property
    def bearing_correction(self) -> float:
        """Degrees to add to the stored bearing to line the plate up."""
        return -self.rotation_deg

    @property
    def verdict(self) -> str:
        turn = abs(self.rotation_deg)
        if turn < 20:
            return "aligned"
        if turn > 160:
            return "reversed"
        return "rotated"


def fit_plane(path: Path, *, max_lag: int = 10) -> PlaneFit | None:
    """Fit tilt to response as one complex gain: size and rotation together.

    Writing the plane as a complex number turns ``dv = G * R(phi) * integral(u)``
    into ``dv = c * integral(u)`` with ``c = G*exp(i*phi)``. One least-squares
    solve then yields the plant gain as ``|c|`` and, as ``arg(c)``, exactly how
    far the mechanism is turned from where the bearing says it is.
    """
    rows = _read(path)
    if len(rows) < 90:
        return None
    times, xs, tilt_x = _series(rows, "x")
    _, ys, tilt_y = _series(rows, "y")
    if min(len(times), len(ys)) < 90:
        return None

    vx = _velocity(times, xs, half=3)
    vy = _velocity(times, ys, half=3)
    span = 8

    # The rim is not part of the plant. A ball resting against it, or bouncing
    # off it, changes velocity for reasons that have nothing to do with the
    # commanded tilt, and those samples arrive exactly when the loop is
    # misbehaving most -- so they are the ones that would dominate the fit.
    # Checked against a simulated 90 degree error: including them reported the
    # rotation as 21 degrees.
    radii = sorted(math.hypot(x, y) for x, y in zip(xs, ys))
    rim = radii[int(0.97 * (len(radii) - 1))] * 0.85 if radii else float("inf")

    def pairs_at(lag: int) -> list[tuple[complex, complex]]:
        out: list[tuple[complex, complex]] = []
        for start in range(lag, len(times) - span):
            end = start + span
            if None in (vx[start], vx[end], vy[start], vy[end]):
                continue
            if any(
                math.hypot(xs[i], ys[i]) > rim for i in range(start, end + 1)
            ):
                continue
            area = 0j
            for i in range(start, end):
                step = times[i + 1] - times[i]
                if step <= 0:
                    break
                area += complex(tilt_x[i - lag], tilt_y[i - lag]) * step
            else:
                out.append(
                    (area, complex(vx[end] - vx[start], vy[end] - vy[start]))
                )
        return out

    best: tuple[float, int, complex] = (0.0, 0, 0j)
    for lag in range(max_lag + 1):
        pairs = pairs_at(lag)
        if len(pairs) < 60:
            continue
        denominator = sum(abs(a) ** 2 for a, _ in pairs)
        if denominator <= 0:
            continue
        coefficient = sum(a.conjugate() * d for a, d in pairs) / denominator
        # How much of the response the fit actually explains, as a fraction.
        residual = sum(abs(d - coefficient * a) ** 2 for a, d in pairs)
        total = sum(abs(d) ** 2 for _, d in pairs)
        score = 1.0 - residual / total if total > 0 else 0.0
        if score > best[0]:
            best = (score, lag, coefficient)

    score, lag, coefficient = best
    if coefficient == 0:
        return None
    step = (times[-1] - times[0]) / max(len(times) - 1, 1)
    return PlaneFit(
        gain=abs(coefficient),
        rotation_deg=math.degrees(math.atan2(coefficient.imag, coefficient.real)),
        delay=lag * step,
        correlation=score,
        samples=len(pairs_at(lag)),
    )


def saturation(path: Path) -> tuple[float, int]:
    """How much of a run was spent asking for more tilt than is allowed.

    Worth measuring because a saturated loop is not a badly tuned one -- it is a
    loop that ran out of range, and no gain will fix that. It also separates two
    complaints that feel identical from the operator's chair: a plate that
    barely leans because the gain is small, and one that barely leans because
    the tilt cap will not let it.
    """
    rows = _read(path)
    if not rows:
        return 0.0, 0
    clipped = 0
    for row in rows:
        try:
            magnitude = math.hypot(float(row["tilt_x"]), float(row["tilt_y"]))
        except (KeyError, ValueError):
            continue
        if magnitude >= 0.99:
            clipped += 1
    return clipped / len(rows), len(rows)


def report(path: Path, *, zeta: float = DEFAULT_ZETA) -> int:
    """Print what a balance log says about the mechanism and the gains."""
    plane = fit_plane(path)
    if plane is None:
        print(
            f"{path}: not enough usable motion to fit.\n"
            "  Either the ball barely moved, or it spent the run against the\n"
            "  rim -- which is itself the answer if the loop is diverging."
        )
        return 1

    print(f"{path}: {plane.samples} windows, fit explains "
          f"{plane.correlation * 100:.0f}% of the response\n")
    print(f"  plant gain    {plane.gain:8.0f} mm/s^2 per unit tilt")
    print(f"  dead time     {plane.delay * 1000:8.0f} ms")
    print(f"  rotation      {plane.rotation_deg:+8.1f} deg  ({plane.verdict})")
    clipped, total = saturation(path)
    print(f"  at full tilt  {clipped * 100:7.1f}% of {total} frames")
    rows = _read(path)
    if rows and "sent" in rows[0]:
        sent = sum(1 for row in rows if row.get("sent") == "1")
        span = float(rows[-1]["t_s"]) - float(rows[0]["t_s"])
        if span > 0:
            print(
                f"  loop rate     {len(rows) / span:7.1f} Hz  "
                f"({sent / len(rows) * 100:.0f}% of frames moved the plate)"
            )
    if clipped > 0.15:
        print(
            "\n  The loop spent a good part of the run asking for more lean than\n"
            "  the tilt cap allows. That is a range problem, not a gain one:\n"
            "  raise --max-tilt and scale kp and kd down by the same factor, so\n"
            "  the small-signal loop is unchanged and only the ceiling moves."
        )

    if plane.verdict != "aligned":
        print(
            f"\n  The plate does not lean where it is asked to. Add "
            f"{plane.bearing_correction:+.1f} degrees to the stored bearing.\n"
            "  Near 90 degrees the correction stands square to the error, which\n"
            "  is why the ball holds its distance and circles instead of\n"
            "  settling: no gain will fix that, only the angle will."
        )
        return 1

    kp, ki, kd = gains_for(plane.gain, plane.delay, zeta=zeta)
    omega = math.sqrt(plane.gain * kp)
    crossover = omega * crossover_ratio(zeta)
    print(
        f"\n  For damping {zeta:.2f}, crossing over at "
        f"{crossover / (2 * math.pi):.2f} Hz -- as fast as {plane.delay * 1000:.0f} ms\n"
        f"  of dead time allows while keeping "
        f"{math.degrees(math.atan(2 * zeta * crossover_ratio(zeta)) - crossover * plane.delay):.0f}"
        f" degrees of phase margin:\n"
    )
    print(f"    --kp {kp:.4f} --ki {ki:.5f} --kd {kd:.4f}")
    standing = 51.4 / (plane.gain * kp)
    print(
        f"\n  Without the integral, half a degree of plate tilt would leave the\n"
        f"  ball {standing:.0f} mm off centre and keep it there -- a PD loop's "
        "standing\n  error is disturbance/(G*kp), and kp is small because the "
        "dead time\n  says it must be. The integral also carries small errors "
        "past the\n  mechanism's own deadband: the axes settle to within 3 to 6 "
        "counts,\n  and a 5 mm error alone asks for about 4."
    )
    return 0
