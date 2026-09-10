"""Ball tracking from the overhead camera.

The platform is matte black and the ball is bright, which makes this a
brightness problem rather than a colour one: a plain threshold separates the two
cleanly, and no colour model has to be tuned or re-tuned when the light changes.

Two things about this rig shape the code more than the algorithm does.

The camera sits directly above the platform centre, looking straight down, so
image centre and platform centre coincide and the mapping from pixels to
millimetres is a single scale factor rather than a homography. That holds near
the middle. The lens is a 2.8 mm, 130 deg wide angle, so it barrel-distorts
noticeably towards the rim -- positions out there read closer to the centre than
they are. Correcting that needs camera intrinsics from a chessboard calibration;
until then, treat rim readings as approximate.

The camera is capped at 30 fps in firmware, so the frame period is the slowest
thing in any control loop built on this. Everything here is therefore arranged
to avoid adding to it: MJPG so the USB link is not the limit, a one-frame
capture buffer so a frame is never served stale, and a fixed exposure so the
camera cannot silently drop its own frame rate in dim light.
"""

from __future__ import annotations

import csv
import logging
import math
import os
import shutil
import subprocess
import time
from dataclasses import dataclass, replace
from pathlib import Path

import cv2
import numpy as np

from ..paths import active_profile_dir, runtime_dir

logger = logging.getLogger(__name__)

CAMERA_BY_ID = Path("/dev/v4l/by-id")
DEFAULT_DEVICE = None
"""No fixed device node. ``/dev/videoN`` numbering is not stable.

Replugging the camera, or another one appearing, renumbers them -- and the node
that was the platform camera yesterday may be unopenable today. The serial side
of this project already solved that with ``/dev/serial/by-id``; the video side
has ``/dev/v4l/by-id``, which names devices by what they are rather than by the
order the kernel found them.
"""

AXIS_1_BEARING_DEG = 180.0
"""Axis 1 direction in the image, fixed by the platform camera mount."""


def list_cameras() -> list[Path]:
    """Stable by-id paths for every capture device, first interface only.

    A UVC camera exposes more than one node: the second is a metadata stream
    that returns no images. Only ``index0`` is the picture.
    """
    if not CAMERA_BY_ID.is_dir():
        return []
    return sorted(
        path for path in CAMERA_BY_ID.iterdir() if path.name.endswith("index0")
    )


def active_camera_file() -> Path:
    """Machine-local file containing the selected stable camera path."""
    return runtime_dir("state", "active-camera")


def set_active_camera(preferred: str) -> tuple[str, Path]:
    """Resolve and persist a camera ID/path; return the stable path and state file."""
    selected = find_camera(preferred, use_stored=False)
    state = active_camera_file()
    state.parent.mkdir(parents=True, exist_ok=True)
    temporary = state.with_suffix(".tmp")
    temporary.write_text(selected + "\n", encoding="utf-8")
    temporary.replace(state)
    return selected, state


def find_camera(preferred: str | None = None, *, use_stored: bool = True) -> str:
    """Resolve a camera path/ID, then the persistent selection, then one camera."""
    candidates = list_cameras()
    if preferred:
        if preferred.isdigit():
            camera_id = int(preferred)
            if 1 <= camera_id <= len(candidates):
                return str(candidates[camera_id - 1])
            raise FileNotFoundError(
                f"Camera ID {camera_id} does not exist. Available cameras:\n"
                f"{_format_camera_choices(candidates)}"
            )
        if Path(preferred).exists():
            return preferred
        raise FileNotFoundError(
            f"Camera {preferred!r} does not exist. Available cameras:\n"
            f"{_format_camera_choices(candidates)}"
        )

    state = active_camera_file()
    if use_stored and state.is_file():
        selected = state.read_text(encoding="utf-8").strip()
        if selected and Path(selected).exists():
            logger.info("active camera: %s", selected)
            return selected
        raise FileNotFoundError(
            f"Selected camera {selected!r} is not attached. Run `ballbal camera "
            f"set` to choose again. Available cameras:\n"
            f"{_format_camera_choices(candidates)}"
        )

    if not candidates:
        # No by-id tree (some kernels, some containers): fall back to probing.
        for index in range(8):
            node = Path(f"/dev/video{index}")
            if node.exists() and os.access(node, os.R_OK):
                return str(node)
        raise FileNotFoundError("No camera found under /dev/v4l/by-id or /dev/video*")
    if len(candidates) > 1:
        raise FileNotFoundError(
            f"{len(candidates)} cameras attached; run `ballbal camera set`:\n"
            f"{_format_camera_choices(candidates)}"
        )
    selected = str(candidates[0])
    logger.info("camera: %s", selected)
    return selected


def _format_camera_choices(candidates: list[Path]) -> str:
    if not candidates:
        return "  none"
    return "\n".join(
        f"  {index}. {path}" for index, path in enumerate(candidates, start=1)
    )
DEFAULT_SIZE = (640, 360)
"""16:9, because every 16:9 mode on this camera shares the full field of view.

Measured across 1920x1080, 1280x720 and 640x360: the ball spans an identical
18.0% of the frame width in all three. Dropping resolution costs no field of
view and no frame rate -- only decode and processing time, which is latency the
control loop would otherwise pay for nothing.
"""

DEFAULT_THRESHOLD = 190
DEFAULT_BALL_MM = 40.0
DEFAULT_ROI_FRACTION = 0.45
"""Radius of the search circle, as a fraction of frame width.

The platform does not fill the frame corners, and what shows outside it -- desk,
cabling, whatever is on the bench -- is often brighter than the ball. Restricting
the search to a disc around the platform centre is what stops the tracker
locking onto the background.
"""

WARMUP_FRAMES = 15
"""Frames to discard after opening.

Measured on this camera: the first frame arrives 268 ms late while everything
after it holds 32.02 ms with no drops at all. Throwing the opening transient away
keeps that outlier out of both the log and the scale calibration.
"""

LOCKED_CONTROLS = (
    ("power_line_frequency", 1),  # 50 Hz mains; 60 banded under artificial light
    ("auto_exposure", 1),  # manual
    ("exposure_dynamic_framerate", 0),  # never trade frame rate for brightness
    ("exposure_time_absolute", 80),  # 8 ms: half the auto's blur, still bright
    ("gain", 40),
    ("white_balance_automatic", 0),
    ("white_balance_temperature", 4600),
    ("backlight_compensation", 0),
)
"""Every automatic the camera would otherwise run, pinned.

A control loop needs the same scene to produce the same pixels. Auto-exposure
re-times the shutter as the platform tilts, auto white balance shifts the ball's
brightness, and `exposure_dynamic_framerate` lets the camera quietly drop below
30 fps -- which silently changes the timebase a PID's D term is divided by.

These live in the driver, not the device, so they are lost on every replug and
have to be set again per run.
"""


@dataclass(frozen=True)
class Detection:
    """One ball sighting, in pixels."""

    x: float
    y: float
    radius: float
    area: float
    circularity: float

    @property
    def diameter(self) -> float:
        return self.radius * 2.0


def lock_camera(device: str) -> bool:
    """Pin the camera's automatics. Returns False if v4l2-ctl is unavailable."""
    if shutil.which("v4l2-ctl") is None:
        logger.warning(
            "v4l2-ctl not found; the camera keeps its automatics and the image "
            "will drift with the room light"
        )
        return False
    args = [f"{name}={value}" for name, value in LOCKED_CONTROLS]
    result = subprocess.run(
        ["v4l2-ctl", "-d", device, *sum((["-c", a] for a in args), [])],
        capture_output=True,
        text=True,
    )
    # Controls differ between cameras; a missing one is worth saying out loud
    # rather than failing, since the rest still apply.
    for line in (result.stdout + result.stderr).splitlines():
        if line.strip():
            logger.warning("v4l2-ctl: %s", line.strip())
    return True


class Camera:
    """A capture that hands back a centred square.

    The plate is round and the frame is 16:9, so the left and right thirds hold
    no platform at all -- on this bench they held a lamp, a printed chessboard
    and the desk, and both of those have already been mistaken for the ball. A
    square keeps everything the plate can occupy and drops the rest.

    It is a crop, not a sensor mode: this camera offers only 16:9 and 4:3, so
    the full frame still crosses the USB link and the transport delay is exactly
    what it was. What improves is what can be confused for a ball, not latency.

    Cropping here rather than at each call site means every consumer -- setup,
    tracking, calibration, the balance loop -- sees the same frame, and the
    coordinates written by one are the coordinates read by the next.
    """

    def __init__(
        self, capture, square: bool = True,  # noqa: ANN001
        offset: tuple[int, int] = (0, 0),
        side: int | None = None,
    ) -> None:
        self._capture = capture
        self.square = square
        self.offset = offset
        self.side = side
        """Length of the square, or None for the largest that fits.

        Adjustable because the largest square is as tall as a 16:9 frame, and a
        crop that already spans the full height cannot be moved up or down at
        all -- it is pinned against both edges. Vertical room only exists once
        the square is smaller than the frame.
        """

    def read(self):  # noqa: ANN201
        ok, frame = self._capture.read()
        if not ok or frame is None or not self.square:
            return ok, frame
        height, width = frame.shape[:2]
        side = min(width, height) if self.side is None else self.side
        side = max(64, min(side, width, height))
        # The offset exists because the image centre is not the optical axis.
        # An M12 lens is seldom mounted with the sensor exactly behind its
        # centre -- the principal point sits a few percent off -- and any slight
        # tilt of the camera moves the projected centre again. So a lens hanging
        # squarely over the plate still lands it off-centre in the frame, and
        # the crop has to be movable rather than merely centred.
        left = (width - side) // 2 + self.offset[0]
        top = (height - side) // 2 + self.offset[1]
        left = max(0, min(width - side, left))
        top = max(0, min(height - side, top))
        return ok, frame[top : top + side, left : left + side]

    def release(self) -> None:
        self._capture.release()

    def isOpened(self) -> bool:  # noqa: N802 - matches VideoCapture
        return self._capture.isOpened()


def open_camera(
    device: str, size: tuple[int, int], *, square: bool = True,
    offset: tuple[int, int] = (0, 0), side: int | None = None,
) -> Camera:
    """Open the camera for low latency, not for image quality."""
    # By path, not by index: OpenCV's V4L2 backend accepts either, and the
    # path is what survives a replug.
    capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not capture.isOpened():
        raise RuntimeError(f"cannot open {device}")
    # MJPG first: the default YUYV negotiates 5 fps at 1080p on this camera,
    # and setting the size before the format can leave it there.
    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
    capture.set(cv2.CAP_PROP_FPS, 30)
    # Two, not one. The default queue is four deep and hands the loop a frame
    # captured several periods ago -- dead time a controller cannot recover and
    # that is invisible in the image. But a single buffer is worse than the
    # disease: with nothing to fill while the one frame is held, the driver
    # cannot double-buffer and every read waits a whole extra period. Measured
    # on this camera: depth 1 gives 68.0 ms per read (14.7 fps), depth 2 gives
    # 32.1 ms (31.1 fps), and deeper buys no throughput, only staleness.
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 2)
    return Camera(capture, square=square, offset=offset, side=side)


def otsu_threshold(gray: np.ndarray, mask: np.ndarray | None = None) -> int:
    """Pick the brightness split from the picture, not from a constant.

    A fixed threshold is a claim about the room light. 190 worked under one
    lamp; after the light was changed and the ball swapped for an orange one it
    returned a crescent of the ball at 0.52 circularity -- below the roundness
    filter, so the ball vanished entirely. Otsu reads the histogram each frame
    and puts the split between the two peaks, which on this rig lands near 116
    where 190 was wrong.

    The mask matters: only pixels inside the plate should vote, or a bright
    bench behind it drags the split up.
    """
    values = gray[mask > 0] if mask is not None else gray.reshape(-1)
    if values.size == 0:
        return 128
    level, _ = cv2.threshold(
        values.reshape(-1, 1), 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )
    return int(level)


def _best_blob(
    binary: np.ndarray,
    *,
    min_radius: float,
    max_radius: float,
    min_circularity: float,
    close: int = 0,
) -> Detection | None:
    """The largest round-enough blob in a binary image.

    Shared by the brightness and colour finders so both apply the same shape
    test: size bounds first, then circularity, and only then area. Taking the
    largest blob without the shape test is how a lit plate edge or a stretch of
    cable wins.
    """
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)
    if close:
        # Colour masks freckle where a highlight desaturates the ball; closing
        # fills those before the shape is judged.
        binary = cv2.morphologyEx(
            binary, cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close, close)),
        )

    contours, _ = cv2.findContours(
        binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    best: Detection | None = None
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area <= 0:
            continue
        (_, _), radius = cv2.minEnclosingCircle(contour)
        if not min_radius <= radius <= max_radius:
            continue
        circularity = area / (math.pi * radius * radius)
        if circularity < min_circularity:
            continue
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            continue
        # The intensity-weighted centroid, not the enclosing circle's centre:
        # it averages over every pixel of the blob and so lands sub-pixel, which
        # is what makes a usable velocity estimate possible at only 30 fps.
        candidate = Detection(
            x=moments["m10"] / moments["m00"],
            y=moments["m01"] / moments["m00"],
            radius=float(radius),
            area=area,
            circularity=circularity,
        )
        if best is None or candidate.area > best.area:
            best = candidate
    return best


def find_ball(
    gray: np.ndarray,
    *,
    threshold: int | None,
    mask: np.ndarray | None,
    min_radius: float,
    max_radius: float,
    min_circularity: float = 0.6,
) -> Detection | None:
    """Find the ball by brightness. ``threshold=None`` picks one with Otsu.

    Circularity is what keeps a stray highlight -- a reflection off a linkage, a
    bright edge of the platform -- from being taken for the ball. A real ball
    measures 0.89 to 0.96 here; anything long and thin scores far lower.
    """
    if threshold is None:
        threshold = otsu_threshold(gray, mask)
    _, binary = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)
    if mask is not None:
        binary = cv2.bitwise_and(binary, mask)
    return _best_blob(
        binary,
        min_radius=min_radius,
        max_radius=max_radius,
        min_circularity=min_circularity,
    )


def find_ball_by_colour(
    bgr: np.ndarray,
    *,
    hsv_lo: tuple[int, int, int],
    hsv_hi: tuple[int, int, int],
    mask: np.ndarray | None,
    min_radius: float,
    max_radius: float,
    min_circularity: float = 0.6,
) -> Detection | None:
    """Find the ball by hue and saturation rather than brightness.

    Worth the extra channel because brightness stopped separating things. With
    a printed chessboard in frame for the lens calibration, its white squares
    measure 186 in grey against the orange ball's 183 -- indistinguishable, and
    the board is the larger blob, so it wins. In colour the two are nowhere near
    each other: saturation 36 against 113.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    binary = _hue_range_mask(hsv, hsv_lo, hsv_hi)
    if mask is not None:
        binary = cv2.bitwise_and(binary, mask)
    return _best_blob(
        binary,
        min_radius=min_radius,
        max_radius=max_radius,
        min_circularity=min_circularity,
        close=11,
    )


def _hue_range_mask(
    hsv: np.ndarray, lo: tuple[int, int, int], hi: tuple[int, int, int]
) -> np.ndarray:
    """inRange, but correct for a hue window that crosses the 0/180 seam.

    OpenCV packs hue into 0..179, so red sits at both ends. A single inRange
    over a wrapped window matches nothing, which would make this work for an
    orange ball and fail silently for a red one.
    """
    if lo[0] <= hi[0]:
        return cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    low = cv2.inRange(
        hsv, np.array((0, lo[1], lo[2]), np.uint8),
        np.array((hi[0], hi[1], hi[2]), np.uint8),
    )
    high = cv2.inRange(
        hsv, np.array((lo[0], lo[1], lo[2]), np.uint8),
        np.array((179, hi[1], hi[2]), np.uint8),
    )
    return cv2.bitwise_or(low, high)


def sample_ball_colour(
    bgr: np.ndarray, detection: Detection
) -> tuple[tuple[int, int, int], tuple[int, int, int]]:
    """Derive an HSV window from a ball that has already been found.

    Bounds come from percentiles of the ball's own pixels rather than from
    constants, so a different ball or a different lamp does not need the numbers
    re-guessed. The floors are deliberately generous on saturation and value: a
    lit sphere is shaded on one side, and clipping those pixels out is what
    turned a round blob into a crescent (circularity 0.62 against 0.89).
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    blob = np.zeros(hsv.shape[:2], np.uint8)
    cv2.circle(
        blob,
        (int(round(detection.x)), int(round(detection.y))),
        max(int(detection.radius * 0.7), 2),
        255,
        -1,
    )
    pixels = hsv[blob > 0]
    hue = int(np.median(pixels[:, 0]))
    spread = int(
        np.percentile(pixels[:, 0], 90) - np.percentile(pixels[:, 0], 10)
    )
    # Measured on this ball: a half-width of 12 gave circularity 0.86 and 17
    # gave 0.94, after which it plateaus -- so err on the generous side.
    half = max(14, spread // 2 + 11)
    sat_floor = max(25, int(np.percentile(pixels[:, 1], 10) * 0.6))
    val_floor = max(25, int(np.percentile(pixels[:, 2], 10) * 0.35))
    return (
        ((hue - half) % 180, sat_floor, val_floor),
        ((hue + half) % 180, 255, 255),
    )


def calibrate_scale(
    samples: list[float], ball_mm: float
) -> tuple[float, float]:
    """Turn measured ball diameters into millimetres per pixel.

    The ball is its own ruler: its real diameter is known and it is the one
    object in frame whose size cannot change. Taking the median over a run of
    frames rejects the odd part-occluded or clipped sighting that a mean would
    absorb.

    Calibrate with the ball near the middle. The wide-angle lens shrinks it
    towards the rim, so a scale measured out there is wrong everywhere else.
    """
    if not samples:
        raise RuntimeError(
            "no ball was seen, so the scale could not be measured -- check the "
            "threshold and that the ball is inside the search circle"
        )
    diameter = float(np.median(samples))
    if diameter <= 0:
        raise RuntimeError("measured a ball diameter of zero")
    return ball_mm / diameter, diameter


def _draw(
    frame: np.ndarray,
    detection: Detection | None,
    centre: tuple[int, int],
    roi_radius: int,
    mm_per_px: float | None,
    trail: list[tuple[int, int]],
    fps: float,
    threshold: int,
) -> None:
    """Overlay the search circle, the platform centre, and the ball."""
    cv2.circle(frame, centre, roi_radius, (200, 200, 60), 1)
    # Platform centre: the origin every reported position is measured from.
    cv2.drawMarker(frame, centre, (0, 200, 255), cv2.MARKER_CROSS, 18, 1)

    for index, point in enumerate(trail):
        weight = int(40 + 160 * index / max(len(trail) - 1, 1))
        cv2.circle(frame, point, 1, (weight, weight, 0), -1)

    lines = [f"{fps:5.1f} fps   thr {threshold}"]
    if detection is None:
        lines.append("no ball")
        cv2.putText(
            frame, "NO BALL", (10, frame.shape[0] - 12),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2,
        )
    else:
        x, y, r = detection.x, detection.y, detection.radius
        top_left = (int(x - r), int(y - r))
        bottom_right = (int(x + r), int(y + r))
        cv2.rectangle(frame, top_left, bottom_right, (0, 255, 0), 2)
        cv2.drawMarker(
            frame, (int(round(x)), int(round(y))),
            (0, 0, 255), cv2.MARKER_CROSS, 14, 2,
        )
        # The line from centre to ball is the error a controller would act on.
        cv2.line(frame, centre, (int(round(x)), int(round(y))), (0, 140, 255), 1)

        dx_px, dy_px = x - centre[0], y - centre[1]
        lines.append(f"px  x{dx_px:+8.1f}  y{dy_px:+8.1f}  r {r:5.1f}")
        if mm_per_px is not None:
            lines.append(
                f"mm  x{dx_px * mm_per_px:+8.1f}  y{dy_px * mm_per_px:+8.1f}"
                f"   d {math.hypot(dx_px, dy_px) * mm_per_px:6.1f}"
            )
        lines.append(f"circularity {detection.circularity:.2f}")

    # The readout sits over whatever the bench happens to look like, so it needs
    # its own ground; plain white text on this scene was unreadable.
    panel_h = 12 + 18 * len(lines)
    panel_w = 8 + max(len(line) for line in lines) * 9
    overlay = frame[0:panel_h, 0:panel_w].copy()
    frame[0:panel_h, 0:panel_w] = cv2.addWeighted(
        overlay, 0.25, np.zeros_like(overlay), 0.75, 0
    )
    for index, line in enumerate(lines):
        cv2.putText(
            frame, line, (6, 20 + 18 * index),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA,
        )


def track(
    *,
    device: str = DEFAULT_DEVICE,
    size: tuple[int, int] = DEFAULT_SIZE,
    threshold: int | None = None,
    ball_mm: float = DEFAULT_BALL_MM,
    mm_per_px: float | None = None,
    roi_fraction: float = DEFAULT_ROI_FRACTION,
    show: bool = True,
    log_path: Path | None = None,
    every: int = 10,
    lock: bool = True,
    seconds: float | None = None,
    square: bool = True,
    offset: tuple[int, int] = (0, 0),
    side: int | None = None,
    calibration: "CameraCalibration | None" = None,
) -> int:
    """Track the ball and report where it is, relative to the platform centre."""
    if lock:
        lock_camera(device)
    capture = open_camera(device, size, square=square, offset=offset, side=side)

    writer = None
    log_file = None
    frames = 0
    seen = 0
    scale_samples: list[float] = []
    trail: list[tuple[int, int]] = []
    started = time.perf_counter()
    last = started
    fps = 0.0

    try:
        ok, frame = capture.read()
        if not ok:
            raise RuntimeError(f"{device} opened but returned no frame")
        height, width = frame.shape[:2]
        if calibration is not None:
            calibration.check_size(width, height)
            centre = calibration.centre
            # Pull in slightly: the plate's own rim is an edge, and a highlight
            # sitting exactly on it would otherwise be inside the search area.
            roi_radius = int(calibration.platform_radius_px * 0.97)
            mm_per_px = calibration.mm_per_px
            print(
                f"  calibration: centre {centre}, platform r="
                f"{calibration.platform_radius_px:.1f} px, "
                f"{mm_per_px:.4f} mm/px"
            )
        else:
            centre = (width // 2, height // 2)
            roi_radius = int(roi_fraction * width)
            print(
                "  no calibration: assuming the platform centre is the image "
                "centre, and searching a fixed circle. Run `ballbal "
                "cam-calibrate` -- the camera is hung by hand, and the offset "
                "is reported as a real ball displacement."
            )
        # Sized off the real ball: a fifth of it is too small to be the ball and
        # anything past twice it is the platform edge or a light, not the ball.
        min_radius = 3.0
        max_radius = width * 0.25
        use_colour = calibration is not None and calibration.has_colour
        print(
            "  detecting by colour" if use_colour
            else "  detecting by brightness"
            + (" (Otsu, per frame)" if threshold is None else f" (threshold {threshold})")
        )
        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.circle(mask, centre, roi_radius, 255, -1)

        print(
            f"{device}: {width}x{height}, search circle r={roi_radius} px "
            f"about {centre}"
        )
        if mm_per_px is None:
            print(
                f"  measuring scale from a {ball_mm:.1f} mm ball -- keep it near "
                "the middle for the first second"
            )

        if log_path is not None:
            log_file = log_path.open("w", newline="")
            writer = csv.writer(log_file)
            writer.writerow(
                ["t_s", "frame", "found", "x_px", "y_px", "dx_px", "dy_px",
                 "dx_mm", "dy_mm", "radius_px", "circularity"]
            )

        while True:
            ok, frame = capture.read()
            if not ok:
                print("camera stopped returning frames")
                return 1
            frames += 1
            now = time.perf_counter()
            # Exponential average: a per-frame reciprocal is too jumpy to read.
            if now > last:
                fps = 0.9 * fps + 0.1 / (now - last) if fps else 1 / (now - last)
            last = now

            if frames <= WARMUP_FRAMES:
                continue

            if use_colour:
                detection = find_ball_by_colour(
                    frame,
                    hsv_lo=calibration.hsv_lo,
                    hsv_hi=calibration.hsv_hi,
                    mask=mask,
                    min_radius=min_radius,
                    max_radius=max_radius,
                )
            else:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                detection = find_ball(
                    gray,
                    threshold=threshold,
                    mask=mask,
                    min_radius=min_radius,
                    max_radius=max_radius,
                )

            if detection is not None:
                seen += 1
                if mm_per_px is None and len(scale_samples) < 30:
                    scale_samples.append(detection.diameter)
                    if len(scale_samples) == 30:
                        mm_per_px, median = calibrate_scale(scale_samples, ball_mm)
                        print(
                            f"  scale: ball measures {median:.1f} px across -> "
                            f"{mm_per_px:.4f} mm/px "
                            f"({1 / mm_per_px:.2f} px/mm)"
                        )
                point = (int(round(detection.x)), int(round(detection.y)))
                trail.append(point)
                if len(trail) > 60:
                    trail.pop(0)

            elapsed = now - started
            dx_px = detection.x - centre[0] if detection else float("nan")
            dy_px = detection.y - centre[1] if detection else float("nan")
            dx_mm = dx_px * mm_per_px if (detection and mm_per_px) else float("nan")
            dy_mm = dy_px * mm_per_px if (detection and mm_per_px) else float("nan")

            if writer is not None:
                writer.writerow(
                    [f"{elapsed:.4f}", frames, int(detection is not None),
                     f"{detection.x:.2f}" if detection else "",
                     f"{detection.y:.2f}" if detection else "",
                     f"{dx_px:.2f}" if detection else "",
                     f"{dy_px:.2f}" if detection else "",
                     f"{dx_mm:.2f}" if detection and mm_per_px else "",
                     f"{dy_mm:.2f}" if detection and mm_per_px else "",
                     f"{detection.radius:.2f}" if detection else "",
                     f"{detection.circularity:.3f}" if detection else ""]
                )

            if every and frames % every == 0:
                if detection is None:
                    print(f"  t={elapsed:7.2f}s  no ball")
                elif mm_per_px:
                    print(
                        f"  t={elapsed:7.2f}s  px ({dx_px:+7.1f},{dy_px:+7.1f})"
                        f"   mm ({dx_mm:+7.1f},{dy_mm:+7.1f})"
                        f"   r={detection.radius:5.1f}"
                    )
                else:
                    print(
                        f"  t={elapsed:7.2f}s  px ({dx_px:+7.1f},{dy_px:+7.1f})"
                        f"   r={detection.radius:5.1f}"
                    )

            # Without a window there is no 'q' to press, so a headless run needs
            # its own way to stop; with one it is still useful for fixed-length
            # logging runs.
            if seconds is not None and elapsed >= seconds:
                break

            if show:
                _draw(frame, detection, centre, roi_radius, mm_per_px,
                      trail, fps, threshold)
                cv2.imshow("ballbal -- ball tracking", frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord("c") and detection is not None:
                    # The camera sits over the platform centre by construction,
                    # but it is mounted by hand. Parking the ball at the real
                    # centre and pressing 'c' beats trusting the mount.
                    centre = (int(round(detection.x)), int(round(detection.y)))
                    mask[:] = 0
                    cv2.circle(mask, centre, roi_radius, 255, -1)
                    print(f"  centre set to {centre}")
                if key == ord("+") or key == ord("="):
                    threshold = min(254, threshold + 5)
                    print(f"  threshold {threshold}")
                if key == ord("-"):
                    threshold = max(1, threshold - 5)
                    print(f"  threshold {threshold}")
    finally:
        capture.release()
        if show:
            cv2.destroyAllWindows()
        if log_file is not None:
            log_file.close()

    tracked = frames - WARMUP_FRAMES
    print(
        f"\n{frames} frames, ball found in {seen}/{max(tracked, 1)} "
        f"({100 * seen / max(tracked, 1):.1f}%)"
    )
    if log_path is not None:
        print(f"log written to {log_path}")
    return 0


# -- calibration ------------------------------------------------------------

CALIBRATION_NAME = str(active_profile_dir() / "camera.json")
PLATFORM_R_MIN = 0.15
PLATFORM_R_MAX = 0.45
"""Bounds on the platform's radius as a fraction of frame width.

Wide enough to survive the camera being re-hung at a different height, narrow
enough that a mug or a lamp shade cannot win the vote.
"""


@dataclass(frozen=True)
class CameraCalibration:
    """Where the platform is in the image, and how big a pixel is.

    Worth storing rather than assuming, because both parts are wrong by default.
    The camera is hung over the platform centre by hand, so image centre and
    platform centre differ -- measured on this rig by 23 px, which at the
    calibrated scale is 16 mm of phantom offset reported for a ball sitting
    dead centre. And the scale changes whenever the camera is moved.
    """

    centre_x: float
    centre_y: float
    platform_radius_px: float
    mm_per_px: float
    width: int
    height: int
    ball_mm: float
    platform_mm: float | None = None
    hsv_lo: tuple[int, int, int] | None = None
    hsv_hi: tuple[int, int, int] | None = None
    crop_offset: tuple[int, int] | None = None
    """How far the square crop is moved from centre, in source pixels.

    Stored so every command crops identically. A different crop is a different
    frame, and the platform centre and millimetre scale are both measured in
    frame pixels -- so a run that cropped elsewhere would read the ball as
    displaced by exactly the difference.
    """
    crop_side: int | None = None
    """Length of the square crop, or None for the largest that fits."""
    bearing_deg: float | None = AXIS_1_BEARING_DEG
    """Which way axis 1 lies, as an angle in the image.

    The field remains in the JSON format for explicitness and compatibility,
    but cam-setup writes the 180-degree direction fixed by the mechanical camera
    mount instead of asking the operator to mark it.
    """

    @property
    def has_colour(self) -> bool:
        return self.hsv_lo is not None and self.hsv_hi is not None

    @property
    def centre(self) -> tuple[int, int]:
        return (int(round(self.centre_x)), int(round(self.centre_y)))

    def check_size(self, width: int, height: int) -> None:
        """Refuse a calibration measured at a different frame size.

        Every stored number is in pixels of the frame it was measured on, so a
        frame of another size silently relocates the platform centre and
        rescales the millimetre conversion. Cropping to a square changes the
        size, which is exactly when this bites.
        """
        if (self.width, self.height) == (width, height):
            return
        raise RuntimeError(
            f"the calibration was measured at {self.width}x{self.height} but "
            f"the camera is delivering {width}x{height}. Re-run "
            "`ballbal cam-setup`."
        )

    @property
    def platform_diameter_mm(self) -> float:
        return 2.0 * self.platform_radius_px * self.mm_per_px

    def to_json(self) -> str:
        import json

        payload = dict(self.__dict__)
        for key in ("hsv_lo", "hsv_hi"):
            if payload[key] is not None:
                payload[key] = list(payload[key])
        return json.dumps(payload, indent=2) + "\n"

    @classmethod
    def load(cls, path: Path) -> "CameraCalibration":
        import json

        payload = json.loads(path.read_text())
        for key in ("hsv_lo", "hsv_hi"):
            if payload.get(key) is not None:
                payload[key] = tuple(payload[key])
        return cls(**payload)


def find_platform(gray: np.ndarray) -> tuple[float, float, float] | None:
    """Locate the platform disc. Returns (x, y, radius) in pixels.

    Brightness alone cannot do this: the platform is dark, and so is much of
    what surrounds it -- desk, mat, shadow -- so a dark threshold floods
    straight off the plate and returns a blob filling the frame (measured fill
    against a circle: 0.34). The plate's *edge*, on the other hand, is a clean
    circle, which is what Hough votes on. Measured here it returns the same
    centre and radius for every sensitivity from 30 to 60.
    """
    height, width = gray.shape[:2]
    blurred = cv2.medianBlur(gray, 5)
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1.5,
        minDist=width // 2,
        param1=120,
        param2=50,
        minRadius=int(PLATFORM_R_MIN * width),
        maxRadius=int(PLATFORM_R_MAX * width),
    )
    if circles is None:
        return None
    x, y, radius = circles[0][0]
    return float(x), float(y), float(radius)


def calibrate_camera(
    *,
    device: str = DEFAULT_DEVICE,
    size: tuple[int, int] = DEFAULT_SIZE,
    threshold: int | None = None,
    ball_mm: float = DEFAULT_BALL_MM,
    platform_mm: float | None = None,
    frames: int = 40,
    lock: bool = True,
    square: bool = True,
    offset: tuple[int, int] = (0, 0),
    side: int | None = None,
    output: Path | None = None,
) -> int:
    """Measure the platform's place in the image and the millimetre scale.

    Put the ball on the platform, near the middle, and leave it still. The
    middle matters: the 2.8 mm lens shrinks the ball towards the rim, so a scale
    taken out there is wrong everywhere else.
    """
    if lock:
        lock_camera(device)
    capture = open_camera(device, size, square=square, offset=offset, side=side)
    centres: list[tuple[float, float]] = []
    radii: list[float] = []
    ball_diameters: list[float] = []
    colour_frame: np.ndarray | None = None
    colour_ball: Detection | None = None

    try:
        ok, frame = capture.read()
        if not ok:
            raise RuntimeError(f"{device} opened but returned no frame")
        height, width = frame.shape[:2]
        print(f"{device}: {width}x{height}, averaging {frames} frames")

        for index in range(frames + WARMUP_FRAMES):
            ok, frame = capture.read()
            if not ok:
                raise RuntimeError("camera stopped returning frames")
            if index < WARMUP_FRAMES:
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            found = find_platform(gray)
            if found is not None:
                centres.append((found[0], found[1]))
                radii.append(found[2])
            # Search inside the plate only. Outside it sit the bench, a lamp and
            # -- while the lens is being calibrated -- the chessboard, whose
            # white squares are the same grey as the ball and a far bigger blob.
            roi = None
            if found is not None:
                roi = np.zeros((height, width), np.uint8)
                cv2.circle(
                    roi, (int(found[0]), int(found[1])),
                    int(found[2] * 0.97), 255, -1,
                )
            ball = find_ball(
                gray,
                threshold=threshold,
                mask=roi,
                min_radius=3.0,
                max_radius=width * 0.25,
            )
            if ball is not None:
                ball_diameters.append(ball.diameter)
                colour_frame, colour_ball = frame, ball
    finally:
        capture.release()

    if not radii:
        print(
            "  Could not find the platform. It has to be fully in frame, and "
            "its edge has to be visible against what is behind it."
        )
        return 1
    # Median on each axis: one bad frame moves it not at all, where a mean
    # would drag the origin every later position is measured from.
    centre_x = float(np.median([c[0] for c in centres]))
    centre_y = float(np.median([c[1] for c in centres]))
    radius = float(np.median(radii))
    print(
        f"  platform: centre ({centre_x:.1f}, {centre_y:.1f}), radius "
        f"{radius:.1f} px  [{len(radii)}/{frames} frames]"
    )
    offset = math.hypot(centre_x - width / 2, centre_y - height / 2)
    print(f"  camera is {offset:.1f} px off the platform centre")

    mm_per_px, ball_px = calibrate_scale(ball_diameters, ball_mm)
    # A ball sits in a known size band relative to the plate it is on, and the
    # scale is only as good as the blob that set it. Both ends bite: a speck of
    # sensor noise on an empty plate measured 13.9 px and implied a platform
    # 1249 mm across, while a hand or a sheet of paper comes in oversized. The
    # scale is trusted by everything downstream, so a refusal beats a guess.
    # The band is wide: the real ball here measures 39% of the radius, and even
    # a small ball on a large plate stays above 15%.
    fraction = ball_px / radius
    if not 0.15 <= fraction <= 1.0:
        print(
            f"\n  The blob taken for the ball is {ball_px:.0f} px across, which "
            f"is {fraction * 100:.0f}% of the\n  platform's {radius:.0f} px "
            f"radius -- it implies a platform {2 * radius * mm_per_px:.0f} mm "
            "wide.\n  That is not a ball. Put one on the plate, or run "
            "`ballbal cam-setup`\n  and click it. Nothing was written."
        )
        return 1
    print(
        f"  ball: {ball_px:.1f} px across = {ball_mm:.1f} mm -> "
        f"{mm_per_px:.4f} mm/px  [{len(ball_diameters)}/{frames} frames]"
    )
    print(f"  implies a platform {2 * radius * mm_per_px:.0f} mm across")

    if platform_mm is not None:
        # Two independent rulers for the same scale. They should agree; if they
        # do not, one of the two measurements is wrong and silently averaging
        # them would hide which.
        from_platform = platform_mm / (2 * radius)
        disagreement = abs(from_platform - mm_per_px) / mm_per_px * 100
        print(
            f"  platform as ruler: {from_platform:.4f} mm/px "
            f"({disagreement:.1f}% from the ball's figure)"
        )
        if disagreement > 10:
            print(
                "  These disagree by more than 10%. Check the ball diameter and "
                "the platform diameter you gave; the platform's is used."
            )
        mm_per_px = from_platform

    hsv_lo = hsv_hi = None
    if colour_frame is not None and colour_ball is not None:
        hsv_lo, hsv_hi = sample_ball_colour(colour_frame, colour_ball)
        print(
            f"  ball colour: hue {hsv_lo[0]}..{hsv_hi[0]}, saturation "
            f">= {hsv_lo[1]}, value >= {hsv_lo[2]}"
        )

    calibration = CameraCalibration(
        centre_x=centre_x,
        centre_y=centre_y,
        platform_radius_px=radius,
        mm_per_px=mm_per_px,
        width=width,
        height=height,
        ball_mm=ball_mm,
        platform_mm=platform_mm,
        hsv_lo=hsv_lo,
        hsv_hi=hsv_hi,
    )
    target = output or Path(CALIBRATION_NAME)
    target.write_text(calibration.to_json())
    print(f"\n  written to {target}")
    return 0


# -- lens intrinsics --------------------------------------------------------

INTRINSICS_NAME = "camera_intrinsics.json"
DEFAULT_BOARD = (9, 6)
DEFAULT_SQUARE_MM = 25.0
DEFAULT_VIEWS = 18


@dataclass(frozen=True)
class Intrinsics:
    """The lens, as focal lengths, a principal point and distortion terms.

    Kept apart from :class:`CameraCalibration` on purpose. These two describe
    different things and go stale for different reasons: re-hanging the camera
    invalidates where the platform sits in the image, but not how the lens
    bends light. Storing them together would mean re-shooting a chessboard
    every time the mount is nudged.

    Resolution is recorded because the camera matrix is in pixels and so scales
    with it. The distortion coefficients are not: OpenCV applies them to
    normalised coordinates, so they carry across resolutions untouched. That
    only holds while the field of view does -- measured on this camera, every
    16:9 mode frames identically, so 1280x720 intrinsics are valid at 640x360.
    """

    fx: float
    fy: float
    cx: float
    cy: float
    dist: tuple[float, ...]
    width: int
    height: int
    rms: float
    views: int

    @property
    def camera_matrix(self) -> np.ndarray:
        return np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

    @property
    def dist_coeffs(self) -> np.ndarray:
        return np.array(self.dist, dtype=np.float64).reshape(1, -1)

    def scaled_to(self, width: int, height: int) -> "Intrinsics":
        """Re-express the matrix at another resolution of the same framing."""
        if (width, height) == (self.width, self.height):
            return self
        sx, sy = width / self.width, height / self.height
        if abs(sx - sy) > 0.01:
            raise RuntimeError(
                f"intrinsics were measured at {self.width}x{self.height} and "
                f"cannot be scaled to {width}x{height}: the aspect ratio "
                "differs, so the framing does too"
            )
        return replace(
            self,
            fx=self.fx * sx, fy=self.fy * sy,
            cx=self.cx * sx, cy=self.cy * sy,
            width=width, height=height,
        )

    def undistort_points(self, points: np.ndarray) -> np.ndarray:
        """Move image points to where a pinhole lens would have put them.

        Used instead of undistorting whole frames in the tracking loop: the
        control path needs one corrected point, and remapping every pixel to get
        it would add latency to the one place that cannot afford any.
        """
        source = np.asarray(points, dtype=np.float64).reshape(-1, 1, 2)
        fixed = cv2.undistortPoints(
            source, self.camera_matrix, self.dist_coeffs, P=self.camera_matrix
        )
        return fixed.reshape(-1, 2)

    def undistort_maps(self, width: int, height: int):  # noqa: ANN201
        scaled = self.scaled_to(width, height)
        return cv2.initUndistortRectifyMap(
            scaled.camera_matrix, scaled.dist_coeffs, None,
            scaled.camera_matrix, (width, height), cv2.CV_16SC2,
        )

    def to_json(self) -> str:
        import json

        payload = dict(self.__dict__)
        payload["dist"] = list(self.dist)
        return json.dumps(payload, indent=2) + "\n"

    @classmethod
    def load(cls, path: Path) -> "Intrinsics":
        import json

        payload = json.loads(path.read_text())
        payload["dist"] = tuple(payload["dist"])
        return cls(**payload)


def chessboard_image(
    board: tuple[int, int] = DEFAULT_BOARD,
    square_mm: float = DEFAULT_SQUARE_MM,
    dpi: int = 300,
    margin_mm: float = 10.0,
) -> np.ndarray:
    """Render a printable chessboard at true scale.

    ``board`` counts *inner corners*, which is what OpenCV asks for and is two
    fewer than the squares along each edge -- the usual reason a calibration
    finds nothing.
    """
    cols, rows = board[0] + 1, board[1] + 1
    px_per_mm = dpi / 25.4
    square = int(round(square_mm * px_per_mm))
    margin = int(round(margin_mm * px_per_mm))
    image = np.full(
        (rows * square + 2 * margin, cols * square + 2 * margin), 255, np.uint8
    )
    for row in range(rows):
        for col in range(cols):
            if (row + col) % 2 == 0:
                continue
            y, x = margin + row * square, margin + col * square
            image[y : y + square, x : x + square] = 0
    return image


# -- guided setup -----------------------------------------------------------

_HELP = {
    "platform": [
        "STEP 1/2   How big is the platform?",
        "",
        "The frame is cropped square, and its middle is taken as the middle of",
        "the platform. Move the crop with the arrow keys until the plate sits",
        "centred, then click the rim to set the size. Everything outside the",
        "circle is ignored when looking for the ball.",
        "",
        "The crop needs moving because the middle of the image is not the",
        "optical axis: an M12 lens is rarely mounted with the sensor exactly",
        "behind its centre, so a lens hanging squarely over the plate still",
        "lands it off-centre.",
        "",
        "[click the rim] set the circle   arrows (or h/j/k/l) move the crop",
        "[,] / [.] shrink or grow the crop   [f] fullscreen   [a] accept  [q] quit",
        "",
        "Up and down only move once the crop is smaller than the frame is tall:",
        "the largest square already spans the full height, so it is pinned.",
    ],
    "ball": [
        "STEP 2/2   Which colour is the ball?",
        "",
        "Click on the ball. Its colour is sampled there and everything matching",
        "is tinted green -- the ball should light up, and nothing else should.",
        "A brightness threshold is not enough any more: printed white paper",
        "measures the same grey as an orange ball.",
        "",
        "[click the ball] sample  [w]ider/[n]arrower  [f] fullscreen  [a] accept  [q] quit",
    ],
}


ARROW_KEYS: dict[int, tuple[int, int]] = {
    # X11 keysyms, as the Qt backend reports them through waitKeyEx...
    65361: (-1, 0), 65362: (0, -1), 65363: (1, 0), 65364: (0, 1),
    # ...and the same codes with the high bits masked off, which is what other
    # backends hand back. Both are accepted so the keys work either way.
    81: (-1, 0), 82: (0, -1), 83: (1, 0), 84: (0, 1),
    # Plain letters, which no backend can garble.
    ord("h"): (-1, 0), ord("k"): (0, -1), ord("l"): (1, 0), ord("j"): (0, 1),
}
"""Arrow keys to a one-pixel nudge.

Arrow keys have no single code across OpenCV's window backends, and a nudge
that silently does nothing is worse than no nudge at all -- so every code they
are known to arrive as is mapped, with letters underneath as a guarantee.
"""


def nudge_for(key: int) -> tuple[int, int]:
    """The (dx, dy) a key asks for, or (0, 0) if it is not a nudge."""
    return ARROW_KEYS.get(key, ARROW_KEYS.get(key & 0xFF, (0, 0)))


SETUP_HEIGHT = 1080
"""Height in pixels the setup picture is drawn at, before the window scales it.

Fixed, not taken from the window. Sizing it from `cv2.getWindowImageRect` fed
back on itself: under Qt that reports the area the previous picture occupied,
so a canvas built to fit it shrank frame by frame -- 630 px tall in a 1048 px
window, measured. A canvas of constant shape that the window scales keeps the
picture as tall as the window, and clicks still arrive in canvas pixels.
"""
PANEL_FONT = 0.7
PANEL_LINE = 30
PANEL_MARGIN = 18


def panel_width(lines: list[str]) -> int:
    """Width in canvas pixels of a side panel that fits the longest line."""
    widest = max(
        (cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, PANEL_FONT, 1)[0][0]
         for line in lines),
        default=0,
    )
    return widest + 2 * PANEL_MARGIN


def beside_panel(
    view: np.ndarray, lines: list[str], panel: int,
) -> tuple[np.ndarray, tuple[int, int]]:
    """The picture with the instructions in a panel to its left, never over it.

    Written over the picture, nine lines of help hid the top of the plate --
    exactly where the rim is clicked. The window is wider than the square
    picture anyway, so the text goes beside it. Returns the canvas and where
    the picture's top-left corner sits in it, which is what clicks are
    converted back through.
    """
    height, width = view.shape[:2]
    text_height = 2 * PANEL_MARGIN + PANEL_LINE * len(lines)
    canvas_h = max(height, text_height)
    canvas = np.full((canvas_h, panel + width, 3), 24, np.uint8)
    origin = (panel, (canvas_h - height) // 2)
    canvas[origin[1]:origin[1] + height, panel:panel + width] = view
    for index, line in enumerate(lines):
        cv2.putText(
            canvas, line, (PANEL_MARGIN, PANEL_MARGIN + 20 + PANEL_LINE * index),
            cv2.FONT_HERSHEY_SIMPLEX, PANEL_FONT, (235, 235, 235), 1, cv2.LINE_AA,
        )
    return canvas, origin


def guided_camera_setup(
    *,
    device: str = DEFAULT_DEVICE,
    size: tuple[int, int] = DEFAULT_SIZE,
    ball_mm: float = DEFAULT_BALL_MM,
    platform_mm: float | None = None,
    output: Path | None = None,
    lock: bool = True,
    square: bool = True,
    offset: tuple[int, int] = (0, 0),
    side: int | None = None,
) -> int:
    """Walk through the camera calibration with the live image in front of you.

    The automatic path guesses, and when it guesses wrong it does so silently:
    with a chessboard laid across the plate it locked onto a stray edge, called
    a 265 px radius the platform, and wrote that out without complaint. Here
    every guess is shown before it is accepted, and the ball's colour is taken
    from the ball you point at rather than from a constant that was true under
    one particular lamp.
    """
    print(
        f"\n{'=' * 68}\nCAMERA SETUP\n{'=' * 68}\n"
        "Put the ball on the platform and take everything else off it --\n"
        "especially the chessboard, which belongs to the lens calibration and\n"
        "hides the plate's edge from this one.\n\n"
        "Two questions get answered here: where the platform sits in the image,\n"
        "and what the ball looks like. Both are shown before anything is saved."
    )
    if lock:
        lock_camera(device)
    capture = open_camera(device, size, square=square, offset=offset, side=side)
    window = "ballbal -- camera setup"

    # "scale" is how much bigger the picture is drawn than the frame, "origin"
    # where it sits beside the help panel. Clicks arrive in window pixels, and
    # every one of them sets a calibration value, so they have to be converted
    # back into frame pixels -- otherwise enlarging the window quietly moves
    # the platform edge and the ball. Clicks on the panel are ignored.
    state: dict = {"click": None, "scale": 1.0, "origin": (0, 0), "size": (0, 0)}

    def on_mouse(event, x, y, flags, _param):  # noqa: ANN001, ANN202
        if event == cv2.EVENT_LBUTTONDOWN:
            factor = state["scale"] or 1.0
            fx = (x - state["origin"][0]) / factor
            fy = (y - state["origin"][1]) / factor
            if 0 <= fx < state["size"][0] and 0 <= fy < state["size"][1]:
                state["click"] = (int(round(fx)), int(round(fy)))

    cv2.namedWindow(window, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    cv2.resizeWindow(window, 1280, 720)
    cv2.setMouseCallback(window, on_mouse)

    # One width for both steps, so the picture keeps its size when they change.
    panel = panel_width([line for help_lines in _HELP.values() for line in help_lines])
    step = "platform"
    centre: tuple[int, int] | None = None
    radius: int = 0
    hsv_lo = hsv_hi = None
    widen = 0
    seed: tuple[int, int] | None = None
    diameters: list[float] = []
    bearing_deg = AXIS_1_BEARING_DEG

    try:
        for _ in range(WARMUP_FRAMES):
            capture.read()
        while True:
            ok, frame = capture.read()
            if not ok:
                print("camera stopped returning frames")
                return 1
            height, width = frame.shape[:2]
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            view = frame.copy()
            click = state.pop("click", None)
            state["click"] = None

            if not radius:
                radius = int(0.45 * width)
            # Always the middle of what is being shown: the crop is what moves,
            # so the plate is centred by placing the crop, not by marking a
            # point inside it.
            centre = (width // 2, height // 2)

            if step == "platform":
                # The centre is the image centre, by instruction rather than by
                # detection. Fitting a circle to the plate's edge was the first
                # approach and it wobbled: across this rig's sessions it
                # returned radii of 152, 158, 176 and 218 px for the same plate,
                # depending on how it happened to be tilted, and a tilted disc
                # projects as an ellipse that a circle model fits badly. One
                # click is steadier than that.
                if click:
                    radius = max(20, int(math.dist(centre, click)))
                cv2.circle(view, centre, radius, (0, 255, 255), 2)
                cv2.drawMarker(
                    view, centre, (0, 255, 255), cv2.MARKER_CROSS, 16, 2
                )
                lines = list(_HELP["platform"])
                lines.append(
                    f"    crop {width}px at {capture.offset[0]:+d},"
                    f"{capture.offset[1]:+d}   circle radius {radius} px"
                )

            else:  # step == "ball"
                assert centre is not None and radius is not None
                mask = np.zeros((height, width), np.uint8)
                cv2.circle(mask, centre, int(radius * 0.97), 255, -1)
                if click:
                    seed = click
                    # Refine once: the patch under the cursor is whatever part
                    # of the ball was clicked, usually the lit highlight, and a
                    # floor taken from it cuts the shaded side away (measured
                    # circularity 0.72 against 0.92). Re-sampling from the whole
                    # blob the first guess finds covers the lit and shaded sides
                    # both.
                    rough = _blob_at(gray, mask, seed)
                    if rough is not None:
                        hsv_lo, hsv_hi = sample_ball_colour(frame, rough)
                    else:
                        hsv_lo, hsv_hi = _sample_at(frame, seed, 0)
                    # Keep the window as centre-and-half so [w]/[n] can open
                    # the *hue* range. That is the channel that decides: with
                    # the window 8 wide only 47% of the ball's pixels matched,
                    # at 14 it was 99%, while every saturation and value floor
                    # tried passed 98% or better.
                    span = (hsv_hi[0] - hsv_lo[0]) % 180
                    state["base"] = (
                        (hsv_lo[0] + span // 2) % 180, span // 2,
                        hsv_lo[1], hsv_lo[2],
                    )
                if seed is not None and state.get("base"):
                    hue, half, sat, val = state["base"]
                    half = max(4, half + widen)
                    hsv_lo = ((hue - half) % 180, max(15, sat - widen),
                              max(15, val - widen))
                    hsv_hi = ((hue + half) % 180, 255, 255)
                detection = None
                if hsv_lo is not None:
                    binary = _hue_range_mask(
                        cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), hsv_lo, hsv_hi
                    )
                    binary = cv2.bitwise_and(binary, mask)
                    view[binary > 0] = (0, 255, 0)
                    detection = find_ball_by_colour(
                        frame, hsv_lo=hsv_lo, hsv_hi=hsv_hi, mask=mask,
                        min_radius=3.0, max_radius=width * 0.25,
                    )
                cv2.circle(view, centre, int(radius * 0.97), (0, 255, 255), 1)
                lines = list(_HELP["ball"])
                if detection is not None:
                    diameters.append(detection.diameter)
                    diameters[:] = diameters[-30:]
                    cv2.circle(
                        view, (int(detection.x), int(detection.y)),
                        int(detection.radius), (0, 0, 255), 2,
                    )
                    lines.append(
                        f"    hue {hsv_lo[0]}..{hsv_hi[0]}  sat>={hsv_lo[1]}  "
                        f"val>={hsv_lo[2]}   diameter {detection.diameter:.1f} px  "
                        f"round {detection.circularity:.2f}"
                    )
                elif seed is None:
                    lines.append("    click the ball to sample its colour")
                else:
                    lines.append("    nothing round matches -- try [w]ider, or click again")

            # Draw at frame size, then enlarge to the fixed setup height. Doing
            # it this way round keeps one scale factor for the whole frame,
            # which is what makes the click conversion exact.
            factor = max(1.0, SETUP_HEIGHT / height)
            if factor > 1.0:
                view = cv2.resize(
                    view,
                    (round(width * factor), round(height * factor)),
                    interpolation=cv2.INTER_LINEAR,
                )
            canvas, origin = beside_panel(view, lines, panel)
            state.update(scale=factor, origin=origin, size=(width, height))
            cv2.imshow(window, canvas)
            # waitKeyEx, not waitKey: the arrow keys carry information above the
            # low byte on some backends, and masking it off first throws away
            # which arrow was pressed.
            key = cv2.waitKeyEx(1)
            if step == "platform":
                if key & 0xFF == ord(","):
                    capture.side = max(64, (capture.side or height) - 8)
                if key & 0xFF == ord("."):
                    capture.side = min(
                        min(size), (capture.side or height) + 8
                    )
                dx, dy = nudge_for(key)
                if dx or dy:
                    # Move the crop, not a marker inside it. The platform's
                    # centre is the crop's centre by construction, so one
                    # control does both -- and the crop is the thing that has
                    # to be right, since every later frame comes through it.
                    capture.offset = (
                        capture.offset[0] + dx, capture.offset[1] + dy
                    )
            key &= 0xFF
            if key in (ord("q"), 27):
                print("\nAborted; nothing was written.")
                return 1
            if key == ord("f"):
                full = cv2.getWindowProperty(window, cv2.WND_PROP_FULLSCREEN)
                cv2.setWindowProperty(
                    window, cv2.WND_PROP_FULLSCREEN,
                    cv2.WINDOW_NORMAL if full == cv2.WINDOW_FULLSCREEN
                    else cv2.WINDOW_FULLSCREEN,
                )
            if step == "platform" and key in (ord("+"), ord("=")):
                radius += 3
            if step == "platform" and key == ord("-"):
                radius = max(20, radius - 3)
            if key == ord("a"):
                if step == "platform":
                    print(f"  platform: centre {centre}, radius {radius} px")
                    step = "ball"
                    continue
                if step == "ball":
                    if hsv_lo is None or not diameters:
                        print("  no ball detected yet -- click on it first")
                        continue
                    print(
                        f"  ball colour: hue {hsv_lo[0]}..{hsv_hi[0]}, "
                        f"saturation >= {hsv_lo[1]}, value >= {hsv_lo[2]}"
                    )
                    print(f"  axis_1 bearing: fixed at {bearing_deg:.1f} deg")
                    break
            if step == "ball" and key == ord("w"):
                widen = min(widen + 5, 40)
            if step == "ball" and key == ord("n"):
                widen = max(widen - 5, -20)
    finally:
        capture.release()
        cv2.destroyAllWindows()

    diameter = float(np.median(diameters))
    mm_per_px = ball_mm / diameter
    print(f"\n  ball: {diameter:.1f} px across = {ball_mm:.1f} mm "
          f"-> {mm_per_px:.4f} mm/px")
    print(f"  implies a platform {2 * radius * mm_per_px:.0f} mm across")
    if platform_mm is not None:
        from_platform = platform_mm / (2 * radius)
        off = abs(from_platform - mm_per_px) / mm_per_px * 100
        print(f"  platform as ruler: {from_platform:.4f} mm/px ({off:.1f}% apart)")
        if off > 10:
            print("  Those disagree by more than 10%. Check both diameters.")
        mm_per_px = from_platform

    calibration = CameraCalibration(
        centre_x=float(centre[0]), centre_y=float(centre[1]),
        platform_radius_px=float(radius), mm_per_px=mm_per_px,
        width=width, height=height, ball_mm=ball_mm, platform_mm=platform_mm,
        hsv_lo=hsv_lo, hsv_hi=hsv_hi, bearing_deg=bearing_deg,
        crop_offset=tuple(capture.offset),
        crop_side=capture.side,
    )
    target = output or Path(CALIBRATION_NAME)
    target.write_text(calibration.to_json())
    print(f"\n  written to {target}\n  run `ballbal track` to use it")
    return 0


def _blob_at(
    gray: np.ndarray, mask: np.ndarray, point: tuple[int, int]
) -> Detection | None:
    """Find the bright blob the user pointed at.

    Brightness locates the ball reliably *inside* the plate -- there is nothing
    else bright in there, and Otsu measures 0.90 circularity against colour's
    0.72. Colour is what tracking needs afterwards, because outside the plate
    brightness cannot tell the ball from printed paper. So each method is used
    where it is strong: brightness to find it once, colour to follow it.
    """
    threshold = otsu_threshold(gray, mask)
    _, binary = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)
    binary = cv2.bitwise_and(binary, mask)
    binary = cv2.morphologyEx(
        binary, cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
    )
    contours, _ = cv2.findContours(
        binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    # Otsu has nothing to split on a featureless view -- it returns 0 and the
    # whole mask passes. A "ball" covering most of the plate is not a ball, and
    # sampling its colour would hand tracking the colour of the plate.
    ceiling = 0.6 * float(np.count_nonzero(mask))
    best = None
    best_distance = float("inf")
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < 30 or area > ceiling:
            continue
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            continue
        x = moments["m10"] / moments["m00"]
        y = moments["m01"] / moments["m00"]
        distance = math.dist((x, y), point)
        if distance < best_distance:
            (_, _), radius = cv2.minEnclosingCircle(contour)
            best_distance = distance
            best = Detection(
                x=x, y=y, radius=float(radius), area=area,
                circularity=area / (math.pi * radius * radius),
            )
    return best


def _sample_at(
    bgr: np.ndarray, point: tuple[int, int], widen: int
) -> tuple[tuple[int, int, int], tuple[int, int, int]]:
    """Take a colour window from a small patch under the cursor."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    height, width = hsv.shape[:2]
    x, y = point
    patch = hsv[
        max(y - 4, 0) : min(y + 5, height), max(x - 4, 0) : min(x + 5, width)
    ].reshape(-1, 3)
    hue = int(np.median(patch[:, 0]))
    half = max(8, int(np.std(patch[:, 0])) * 2 + 6) + widen
    sat = max(20, int(np.median(patch[:, 1]) * 0.45) - widen)
    val = max(20, int(np.median(patch[:, 2]) * 0.35) - widen)
    return ((hue - half) % 180, sat, val), ((hue + half) % 180, 255, 255)
