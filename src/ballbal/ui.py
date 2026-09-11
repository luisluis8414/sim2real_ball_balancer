"""Live view of the balance loop for Lichtblick, over the Foxglove WebSocket protocol.

    ballbal balance --live --ui [--mirror]

then in Lichtblick: Open connection -> Foxglove WebSocket -> ws://localhost:8765, and import
the layout ``tools/lichtblick/ballbal_layout.json`` (docs/lichtblick.md).

Topics:

    /camera/real            foxglove.CompressedImage   the real camera's square crop, as tracked
    /camera/real/tracking   foxglove.ImageAnnotations  plate, ball, target, prediction, trail
    /camera/sim/overhead    foxglove.CompressedImage   Isaac Sim's overhead camera (if reachable)
    /camera/sim/view        foxglove.CompressedImage   Isaac Sim's view camera
    /camera/side            foxglove.CompressedImage   a second, side-on camera, untouched (optional)
    /plate                  foxglove.SceneUpdate       top-down plate, frame "plate", centimetres
    /ball                   JSON                       position, target, prediction, tilt, loop rate
    /pid                    JSON                       P, I, D and their sum per axis, in leg counts
    /servos                 JSON                       goal, measured and goal - measured, counts
    /balance/state          JSON                       "ready" or "balancing"
    /clicked_point          <- from Lichtblick          a click on the plate sets the target
    /balance/command        <- from Lichtblick          {"command": "start"} or {"command": "stop"}

The 3D panel's "Publish point" tool sends a click in the display frame ("plate": x to the right
of the camera image, y up the image, z up), which becomes the balance target.

Two libraries, on purpose. Messages are encoded with the foxglove SDK's schemas (protobuf), but
served by ``foxglove-websocket``: Lichtblick's 3D panel only publishes clicks to a server that
announces itself as ROS (``ROS_DISTRO`` in its metadata), and the SDK's server cannot send
metadata.

Nothing here waits on the loop or the other way round. ``publish`` leaves the newest frame for
the server thread, which encodes and sends it; frames it has not taken yet are replaced.
"""

from __future__ import annotations

import asyncio
import base64
import json
import logging
import math
import os
import threading
import time
from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np
from foxglove import messages as fm
from foxglove_websocket.server import FoxgloveServer, FoxgloveServerListener

HOST = os.environ.get("BALLBAL_UI_HOST", "127.0.0.1")
PORT = int(os.environ.get("BALLBAL_UI_PORT", "8765"))
CLICK_TOPIC = "/clicked_point"
COMMAND_TOPIC = "/balance/command"
CONTROL_SCHEMA = "balance_control"
"""Schema of /balance/state. Lichtblick's Publish panel only offers schemas the server announced,
so the Start and Stop buttons publish this one on /balance/command."""
SIM_PERIOD = 0.1
"""Seconds between simulated camera frames: each costs Isaac Sim a JPEG encode on its main loop."""
SIDE_SIZE = (1280, 720)
"""Side camera capture size, MJPG. Its frames are forwarded as the camera sends them."""
TRAIL = 90
SCENE_MM = 10.0
"""Millimetres per unit of the /plate scene: it is drawn in centimetres, not metres.

Lichtblick draws the marker of its Publish point tool as a sphere of 0.25 units and its frame axes
at a similar size, neither adjustable. In metres that is a 25 cm ball over a 23 cm plate. In
centimetres it is a 2.5 mm dot.
"""

GREY, GREEN, ORANGE, RED, WHITE = (
    (0.6, 0.6, 0.6, 1.0), (0.1, 0.9, 0.2, 1.0), (1.0, 0.55, 0.0, 1.0), (1.0, 0.15, 0.15, 1.0),
    (1.0, 1.0, 1.0, 1.0),
)

logger = logging.getLogger(__name__)


@dataclass
class Frame:
    """One pass of the balance loop, as the view shows it. Positions in image mm from the centre."""

    image: np.ndarray
    ball: tuple[float, float] | None
    ball_radius_px: float | None
    target: tuple[float, float]
    predicted: tuple[float, float] | None
    tilt: tuple[float, float]
    goals: dict[int, int] | None
    measured: dict[int, int] | None
    loop_hz: float
    pid: dict[str, tuple[float, float, float]] | None = None
    """P, I, D per image axis ("x", "y"), in leg counts; None while the ball is not seen."""
    state: str = "balancing"
    """"ready" (servos off, waiting for Start) or "balancing"."""


def _stamp(ns: int) -> fm.Timestamp:
    return fm.Timestamp(sec=ns // 1_000_000_000, nsec=ns % 1_000_000_000)


def _colour(rgba: tuple[float, float, float, float]) -> fm.Color:
    return fm.Color(r=rgba[0], g=rgba[1], b=rgba[2], a=rgba[3])


def _json_schema(fields: dict[str, str]) -> str:
    return json.dumps({"type": "object", "properties": {k: {"type": v} for k, v in fields.items()}})


class _Clicks(FoxgloveServerListener):
    def __init__(self, view: "LiveView") -> None:
        self.view = view
        self.topics: dict[int, str] = {}

    async def on_client_advertise(self, server, channel) -> None:  # noqa: ANN001
        self.topics[channel["id"]] = channel["topic"]

    async def on_client_unadvertise(self, server, channel_id) -> None:  # noqa: ANN001
        self.topics.pop(channel_id, None)

    async def on_client_message(self, server, channel_id, payload) -> None:  # noqa: ANN001
        topic = self.topics.get(channel_id)
        if topic == COMMAND_TOPIC:
            try:
                message = json.loads(payload)
                command = str(message.get("command") or message.get("data") or "").strip().lower()
            except (ValueError, AttributeError) as exc:
                logger.warning("ignored a command that was not JSON: %s", exc)
                return
            if command in ("start", "stop"):
                self.view.set_command(command)
            else:
                logger.warning("ignored unknown command %r (start or stop)", command)
            return
        if topic != CLICK_TOPIC:
            return
        try:
            point = json.loads(payload)["point"]
            self.view.set_target((float(point["x"]) * SCENE_MM, -float(point["y"]) * SCENE_MM))
        except (ValueError, KeyError, TypeError) as exc:
            logger.warning("ignored a click that was not a PointStamped: %s", exc)


class SideCamera:
    """A second camera, read on its own thread and only ever shown.

    It never touches the control loop: the loop's camera and this one run on separate threads,
    and a frame the server has not sent yet is replaced by the next. MJPG frames are forwarded
    as the camera encoded them when OpenCV hands them over undecoded; otherwise they are decoded
    and encoded again.
    """

    def __init__(self, device: str, size: tuple[int, int] = SIDE_SIZE) -> None:
        self.device, self.size = device, size
        self._cond = threading.Condition()
        self._jpeg: bytes | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="side-camera", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        with self._cond:
            self._cond.notify_all()
        self._thread.join(timeout=3.0)

    def take(self, timeout: float = 0.5) -> bytes | None:
        """The newest JPEG not yet taken, waiting up to ``timeout``; None if there was none."""
        with self._cond:
            if self._jpeg is None and not self._stop.is_set():
                self._cond.wait(timeout)
            jpeg, self._jpeg = self._jpeg, None
        return jpeg

    def _open(self):  # noqa: ANN202
        capture = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
        capture.set(cv2.CAP_PROP_FPS, 30)
        capture.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        return capture

    def _run(self) -> None:
        capture = self._open()
        if not capture.isOpened():
            logger.warning("side camera %s could not be opened", self.device)
            return
        failures = 0
        try:
            while not self._stop.is_set():
                ok, data = capture.read()
                if not ok or data is None:
                    failures += 1
                    if failures == 30:
                        logger.warning("side camera %s returns no frames", self.device)
                    time.sleep(0.05)
                    continue
                failures = 0
                raw = data.reshape(-1)
                if raw.size > 2 and raw[0] == 0xFF and raw[1] == 0xD8:
                    jpeg = raw.tobytes()
                else:
                    image = data if data.ndim == 3 else cv2.imdecode(raw, cv2.IMREAD_COLOR)
                    if image is None:
                        continue
                    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    if not ok:
                        continue
                    jpeg = encoded.tobytes()
                with self._cond:
                    self._jpeg = jpeg
                    self._cond.notify()
        finally:
            capture.release()


class LiveView:
    """The server, run on its own thread; use it as a context manager around the loop."""

    def __init__(self, calibration, servos, *, host: str = HOST, port: int = PORT,  # noqa: ANN001
                 sim_cameras: bool = True, side_camera: str | None = None) -> None:
        self.cal = calibration
        self.names = {s.id: s.name for s in servos}
        self.plate_mm = calibration.platform_radius_px * calibration.mm_per_px
        self.host, self.port, self.sim_cameras = host, port, sim_cameras
        self.url = f"ws://{'localhost' if host in ('127.0.0.1', '0.0.0.0') else host}:{port}"
        self._cond = threading.Condition()
        self._frame: Frame | None = None
        self._target: tuple[float, float] | None = None
        self._command: str | None = None
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._error: BaseException | None = None
        self._trail: deque[tuple[float, float]] = deque(maxlen=TRAIL)
        self._thread = threading.Thread(target=self._serve, name="lichtblick-view", daemon=True)
        self._side = SideCamera(side_camera) if side_camera else None

    # -- loop side ------------------------------------------------------------
    def __enter__(self) -> "LiveView":
        self._thread.start()
        self._ready.wait(10.0)
        if self._error is not None:
            raise RuntimeError(f"live view could not start on {self.url}: {self._error}")
        if self._side is not None:
            self._side.start()
            print(f"  side camera {self._side.device} on /camera/side")
        return self

    def __exit__(self, *_exc) -> None:  # noqa: ANN002
        self._stop.set()
        with self._cond:
            self._cond.notify()
        if self._side is not None:
            self._side.stop()
        self._thread.join(timeout=5.0)

    def publish(self, frame: Frame) -> None:
        with self._cond:
            self._frame = frame
            self._cond.notify()

    def take_target(self) -> tuple[float, float] | None:
        """A target clicked since the last call, in image mm; None if there was none."""
        with self._cond:
            target, self._target = self._target, None
        return target

    def take_command(self) -> str | None:
        """"start" or "stop" if one arrived since the last call, else None."""
        with self._cond:
            command, self._command = self._command, None
        return command

    def set_command(self, command: str) -> None:
        with self._cond:
            self._command = command

    def set_target(self, target: tuple[float, float]) -> None:
        # Keep clicks on the plate and off the rim: the ball is 40 mm across and the guardrail
        # stands at the edge, so the last fifth of the radius is not a place to balance.
        limit = 0.75 * self.plate_mm
        distance = math.hypot(*target)
        if distance > limit:
            target = (target[0] * limit / distance, target[1] * limit / distance)
        with self._cond:
            self._target = target
        print(f"\n  target set from Lichtblick: ({target[0]:+.0f}, {target[1]:+.0f}) mm")

    # -- server side ----------------------------------------------------------
    def _serve(self) -> None:
        try:
            asyncio.run(self._main())
        except BaseException as exc:  # noqa: BLE001 - reported to the loop on start, logged after
            self._error = exc
            self._ready.set()
            if not self._stop.is_set():
                logger.error("live view stopped: %s", exc)

    def _next_frame(self) -> Frame | None:
        with self._cond:
            if self._frame is None and not self._stop.is_set():
                self._cond.wait(0.5)
            frame, self._frame = self._frame, None
        return frame

    async def _main(self) -> None:
        logging.getLogger("FoxgloveServer").setLevel(logging.WARNING)
        server = FoxgloveServer(
            self.host, self.port, "ballbal balance",
            capabilities=["clientPublish"], supported_encodings=["json"],
            metadata={"ROS_DISTRO": "humble"},
        )
        server.set_listener(_Clicks(self))
        async with server:
            await server.wait_opened()
            self._channels = {}
            for topic, message in (("/camera/real", fm.CompressedImage),
                                   ("/camera/real/tracking", fm.ImageAnnotations),
                                   ("/camera/sim/overhead", fm.CompressedImage),
                                   ("/camera/sim/view", fm.CompressedImage),
                                   ("/camera/side", fm.CompressedImage),
                                   ("/plate", fm.SceneUpdate)):
                schema = message.get_schema()
                self._channels[topic] = await server.add_channel({
                    "topic": topic, "encoding": "protobuf", "schemaName": schema.name,
                    "schema": base64.b64encode(schema.data).decode(), "schemaEncoding": "protobuf",
                })
            for topic, fields in (("/ball", dict(x="number", y="number", target_x="number",
                                                 target_y="number", predicted_x="number",
                                                 predicted_y="number", error="number",
                                                 tilt_x="number", tilt_y="number",
                                                 loop_hz="number", found="boolean")),
                                  ("/pid", dict(x="object", y="object")),
                                  ("/balance/state", dict(state="string", command="string")),
                                  ("/servos", dict(goal="object", measured="object", lag="object"))):
                self._channels[topic] = await server.add_channel({
                    "topic": topic, "encoding": "json",
                    "schemaName": CONTROL_SCHEMA if topic == "/balance/state" else topic.strip("/"),
                    "schema": _json_schema(fields), "schemaEncoding": "jsonschema",
                })
            self._ready.set()
            print(f"  live view on {self.url} -- connect Lichtblick there (docs/lichtblick.md)")
            loop = asyncio.get_running_loop()
            tasks = [asyncio.create_task(self._frames(server, loop))]
            if self.sim_cameras:
                tasks.append(asyncio.create_task(self._simulated(server, loop)))
            if self._side is not None:
                tasks.append(asyncio.create_task(self._side_frames(server, loop)))
            while not self._stop.is_set():
                await asyncio.sleep(0.1)
            for task in tasks:
                task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)

    async def _send(self, server: FoxgloveServer, topic: str, ns: int, payload: bytes) -> None:
        await server.send_message(self._channels[topic], ns, payload)

    async def _frames(self, server: FoxgloveServer, loop: asyncio.AbstractEventLoop) -> None:
        while True:
            frame = await loop.run_in_executor(None, self._next_frame)
            if frame is None:
                continue
            try:
                await self._send_frame(server, frame)
            except Exception:  # one bad frame must not end the view
                logger.exception("live view could not send a frame")

    async def _send_frame(self, server: FoxgloveServer, frame: Frame) -> None:
        ns = time.time_ns()
        ok, jpeg = cv2.imencode(".jpg", frame.image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            await self._send(server, "/camera/real", ns, fm.CompressedImage(
                timestamp=_stamp(ns), frame_id="camera", data=jpeg.tobytes(), format="jpeg").encode())
        if frame.ball is not None:
            self._trail.append(frame.ball)
        await self._send(server, "/camera/real/tracking", ns, self._annotations(frame, ns).encode())
        await self._send(server, "/plate", ns, self._scene(frame, ns).encode())
        error = math.hypot(frame.ball[0] - frame.target[0], frame.ball[1] - frame.target[1]) \
            if frame.ball else None
        await self._send(server, "/ball", ns, json.dumps({
            "found": frame.ball is not None,
            "x": frame.ball[0] if frame.ball else None, "y": frame.ball[1] if frame.ball else None,
            "target_x": frame.target[0], "target_y": frame.target[1],
            "predicted_x": frame.predicted[0] if frame.predicted else None,
            "predicted_y": frame.predicted[1] if frame.predicted else None,
            "error": error, "tilt_x": frame.tilt[0], "tilt_y": frame.tilt[1],
            "loop_hz": frame.loop_hz,
        }).encode())
        await self._send(server, "/balance/state", ns,
                         json.dumps({"state": frame.state, "command": ""}).encode())
        if frame.pid is not None:
            await self._send(server, "/pid", ns, json.dumps({
                axis: {"p": p, "i": i, "d": d, "sum": p + i + d}
                for axis, (p, i, d) in frame.pid.items()
            }).encode())
        goals, measured = frame.goals or {}, frame.measured or {}
        await self._send(server, "/servos", ns, json.dumps({
            "goal": {self.names[i]: v for i, v in goals.items()},
            "measured": {self.names[i]: v for i, v in measured.items()},
            "lag": {self.names[i]: goals[i] - measured[i] for i in goals if i in measured},
        }).encode())

    async def _side_frames(self, server: FoxgloveServer, loop: asyncio.AbstractEventLoop) -> None:
        while True:
            jpeg = await loop.run_in_executor(None, self._side.take)
            if jpeg is None:
                continue
            ns = time.time_ns()
            await self._send(server, "/camera/side", ns, fm.CompressedImage(
                timestamp=_stamp(ns), frame_id="side", data=jpeg, format="jpeg").encode())

    # -- drawing --------------------------------------------------------------
    def _px(self, mm: tuple[float, float], dx: float = 0.0, dy: float = 0.0) -> fm.Point2:
        return fm.Point2(x=self.cal.centre_x + mm[0] / self.cal.mm_per_px + dx,
                         y=self.cal.centre_y + mm[1] / self.cal.mm_per_px + dy)

    def _annotations(self, frame: Frame, ns: int) -> fm.ImageAnnotations:
        stamp = _stamp(ns)
        px_per_mm = 1.0 / self.cal.mm_per_px
        circles = [
            fm.CircleAnnotation(timestamp=stamp, position=self._px((0.0, 0.0)),
                                diameter=2 * self.cal.platform_radius_px * 0.97, thickness=1.5,
                                outline_color=_colour(GREY), fill_color=_colour((0, 0, 0, 0))),
            fm.CircleAnnotation(timestamp=stamp, position=self._px(frame.target), diameter=10 * px_per_mm,
                                thickness=2, outline_color=_colour(RED), fill_color=_colour((1, 0.15, 0.15, 0.15))),
        ]
        if frame.ball is not None:
            circles.append(fm.CircleAnnotation(
                timestamp=stamp, position=self._px(frame.ball), diameter=2 * (frame.ball_radius_px or 20),
                thickness=2.5, outline_color=_colour(GREEN), fill_color=_colour((0, 0, 0, 0))))
        if frame.predicted is not None:
            circles.append(fm.CircleAnnotation(
                timestamp=stamp, position=self._px(frame.predicted), diameter=8, thickness=2,
                outline_color=_colour(ORANGE), fill_color=_colour(ORANGE)))
        cross = 6 * px_per_mm
        centre = (0.0, 0.0)
        points = [fm.PointsAnnotation(
            timestamp=stamp, type=fm.PointsAnnotationType.LineList, thickness=1.5,
            outline_color=_colour(GREY),
            points=[self._px(centre, -cross), self._px(centre, cross),
                    self._px(centre, 0.0, -cross), self._px(centre, 0.0, cross)])]
        if len(self._trail) > 1:
            points.append(fm.PointsAnnotation(
                timestamp=stamp, type=fm.PointsAnnotationType.LineStrip, thickness=1.5,
                outline_color=_colour((0.1, 0.9, 0.2, 0.5)), points=[self._px(p) for p in self._trail]))
        text = "no ball" if frame.ball is None else (
            f"error {math.hypot(frame.ball[0] - frame.target[0], frame.ball[1] - frame.target[1]):4.1f} mm"
            f"   {frame.loop_hz:4.1f} Hz")
        text = ("READY - press Start   " if frame.state == "ready" else "BALANCING   ") + text
        texts = [fm.TextAnnotation(timestamp=stamp, position=fm.Point2(x=8, y=20), text=text, font_size=16,
                                   text_color=_colour(WHITE), background_color=_colour((0, 0, 0, 0.5)))]
        return fm.ImageAnnotations(circles=circles, points=points, texts=texts)

    def _scene(self, frame: Frame, ns: int) -> fm.SceneUpdate:
        # All sizes in mm, converted to scene units (SCENE_MM) here.
        def at(mm: tuple[float, float], z_mm: float) -> fm.Pose:
            return fm.Pose(position=fm.Vector3(x=mm[0] / SCENE_MM, y=-mm[1] / SCENE_MM, z=z_mm / SCENE_MM),
                           orientation=fm.Quaternion(x=0, y=0, z=0, w=1))

        def size(x_mm: float, z_mm: float) -> fm.Vector3:
            return fm.Vector3(x=x_mm / SCENE_MM, y=x_mm / SCENE_MM, z=z_mm / SCENE_MM)

        plate_d = 2 * self.plate_mm
        ball_d = self.cal.ball_mm
        cylinders = [
            fm.CylinderPrimitive(pose=at((0, 0), -2.0), size=size(plate_d, 4.0),
                                 bottom_scale=1, top_scale=1, color=_colour((0.15, 0.15, 0.17, 1.0))),
            fm.CylinderPrimitive(pose=at(frame.target, 0.5), size=size(10.0, 1.0),
                                 bottom_scale=1, top_scale=1, color=_colour((1, 0.15, 0.15, 0.8))),
        ]
        spheres = []
        if frame.ball is not None:
            spheres.append(fm.SpherePrimitive(pose=at(frame.ball, ball_d / 2), size=size(ball_d, ball_d),
                                              color=_colour((1.0, 0.75, 0.2, 1.0))))
        if frame.predicted is not None:
            spheres.append(fm.SpherePrimitive(pose=at(frame.predicted, 3.0), size=size(6.0, 6.0),
                                              color=_colour(ORANGE)))
        lines = []
        if len(self._trail) > 1:
            lines.append(fm.LinePrimitive(
                type=fm.LinePrimitiveLineType.LineStrip, pose=at((0, 0), 1.0), thickness=2,
                scale_invariant=True, color=_colour((0.1, 0.9, 0.2, 0.6)),
                points=[fm.Point3(x=p[0] / SCENE_MM, y=-p[1] / SCENE_MM, z=0.0) for p in self._trail]))
        entity = fm.SceneEntity(timestamp=_stamp(ns), frame_id="plate", id="plate", frame_locked=True,
                                cylinders=cylinders, spheres=spheres, lines=lines)
        return fm.SceneUpdate(entities=[entity])

    # -- simulation -------------------------------------------------------------
    async def _simulated(self, server: FoxgloveServer, loop: asyncio.AbstractEventLoop) -> None:
        from .simulation.remote import PACKAGE, IsaacError, IsaacSim

        sim = IsaacSim(context="ballbal_ui")
        armed = False
        try:
            while True:
                if not armed:
                    try:
                        await loop.run_in_executor(None, lambda: sim.run_file(PACKAGE / "isaac_cameras.py"))
                        armed = True
                    except IsaacError:
                        await asyncio.sleep(10.0)  # Isaac Sim not up (yet); the real camera carries on
                        continue
                started = time.perf_counter()
                try:
                    frames = await loop.run_in_executor(
                        None, lambda: sim.run("sim_cameras.grab()", timeout=5.0, echo=False))
                except IsaacError:
                    armed = False
                    continue
                ns = time.time_ns()
                for name, data in (frames or {}).items():
                    await self._send(server, f"/camera/sim/{name}", ns, fm.CompressedImage(
                        timestamp=_stamp(ns), frame_id=f"sim_{name}", data=base64.b64decode(data),
                        format="jpeg").encode())
                await asyncio.sleep(max(0.0, SIM_PERIOD - (time.perf_counter() - started)))
        finally:
            if armed:
                try:
                    sim.run("sim_cameras.destroy()", timeout=5.0, echo=False)
                except IsaacError:
                    pass
