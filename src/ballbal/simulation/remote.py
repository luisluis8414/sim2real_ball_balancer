"""Drive the running Isaac Sim from the repository, through its Python Server extension.

Isaac Sim runs its own Python with the ``omni`` modules; the repository environment cannot
import them. The Python Server extension (``isaacsim.code_editor.python_server``) executes code
sent over TCP inside Isaac Sim, so every simulation tool starts from the repository and sends
only the Isaac-side part there. Launch Isaac Sim with the server enabled (docs/simulation.md):

    isaacsim isaacsim.exp.full --enable isaacsim.code_editor.python_server

Plain standard library, so it works in any environment that has the repository.
"""

from __future__ import annotations

import json
import os
import socket
from pathlib import Path

PACKAGE = Path(__file__).resolve().parent
REPO = PACKAGE.parents[2]
SCENE = REPO / "models" / "usd" / "scene.usda"
DRIVER = PACKAGE / "isaac.py"

HOST = os.environ.get("BALLBAL_ISAAC_HOST", "127.0.0.1")
PORT = int(os.environ.get("BALLBAL_ISAAC_PORT", "8226"))
TOKEN = os.environ.get("BALLBAL_ISAAC_TOKEN") or None
CONTEXT = "ballbal"
LAUNCH = "isaacsim isaacsim.exp.full --enable isaacsim.code_editor.python_server"

_SERVOS = "__import__('builtins').__dict__.get('_sim_servos')"


class IsaacError(RuntimeError):
    """Isaac Sim is unreachable, or the code sent to it raised."""


class IsaacSim:
    """One running Isaac Sim, reached through its Python Server.

    Every request runs in the named context ``ballbal``, whose variables persist between
    requests: after ``arm()`` the driver's ``servos`` object is available to later calls.
    """

    def __init__(self, host: str = HOST, port: int = PORT, context: str = CONTEXT,
                 token: str | None = TOKEN) -> None:
        self.host, self.port, self.context, self.token = host, port, context, token

    # -- transport --------------------------------------------------------------
    def run(self, code: str, *, timeout: float = 30.0, echo: bool = True, **args):
        """Execute ``code`` inside Isaac Sim and return its value.

        A value comes back only when ``code`` is a single expression (dict keys arrive as
        strings, other objects as their repr). Keyword arguments are injected as variables.
        Top-level ``await`` works. What the code prints is echoed here unless ``echo`` is False.
        """
        envelope = {"code": code, "context": self.context, "timeout": timeout}
        if args:
            envelope["args"] = args
        if self.token:
            envelope["auth_token"] = self.token
        try:
            with socket.create_connection((self.host, self.port), timeout=timeout + 5) as s:
                s.sendall(json.dumps(envelope).encode())
                s.shutdown(socket.SHUT_WR)
                data = b"".join(iter(lambda: s.recv(1 << 16), b""))
        except OSError as exc:
            raise IsaacError(
                f"no Isaac Sim Python Server on {self.host}:{self.port} ({exc}).\n"
                f"Launch Isaac Sim with the server enabled:\n    {LAUNCH}"
            ) from exc
        reply = json.loads(data.decode())
        output = reply.get("output") or ""
        if echo and output:
            print(output, end="" if output.endswith("\n") else "\n")
        if reply.get("status") != "ok":
            detail = "".join(reply.get("traceback") or []) or f"{reply.get('ename')}: {reply.get('evalue')}"
            raise IsaacError(f"Isaac Sim raised:\n{detail}")
        return reply.get("result")

    def run_file(self, path: Path, *, timeout: float = 60.0, **args) -> None:
        """Execute an Isaac-side script from this repository in the ``ballbal`` context.

        Keyword arguments become variables the script can read.
        """
        path = Path(path).resolve()
        self.run(
            f"__file__ = {str(path)!r}\n"
            "exec(compile(open(__file__).read(), __file__, 'exec'))",
            timeout=timeout,
            **args,
        )

    def reachable(self) -> bool:
        try:
            self.run("1", timeout=5.0, echo=False)
        except IsaacError:
            return False
        return True

    # -- stage and timeline -------------------------------------------------------
    def scene(self) -> str | None:
        """Path of the stage open in Isaac Sim (None for an unsaved or no stage)."""
        return self.run(
            "(lambda s: (s.GetRootLayer().realPath or None) if s else None)"
            "(__import__('omni.usd').usd.get_context().get_stage())",
            echo=False,
        )

    def open_scene(self, path: Path = SCENE) -> bool:
        """Open the repository scene unless it is already open. Returns True when it opened it.

        Refuses to replace another scene file with unsaved changes: opening discards them
        silently. An unsaved (anonymous) stage is replaced -- Isaac Sim starts with one, and it
        counts as modified from the first frame.
        """
        path = Path(path).resolve()
        current = self.scene()
        if current and Path(current).resolve() == path:
            return False
        dirty = current and self.run(
            "__import__('omni.usd').usd.get_context().get_stage().GetRootLayer().dirty",
            echo=False,
        )
        if dirty:
            raise IsaacError(
                f"Isaac Sim has {current} open with unsaved changes; "
                f"save or close it there, then retry (would open {path})"
            )
        self.stop()
        self.run(
            "import omni.usd\n"
            f"_ok, _err = await omni.usd.get_context().open_stage_async({str(path)!r})\n"
            "if not _ok:\n"
            "    raise RuntimeError(f'cannot open stage: {_err}')",
            timeout=120.0,
        )
        return True

    def state(self) -> str:
        return self.run(
            "(lambda t: 'playing' if t.is_playing() else 'stopped' if t.is_stopped() else 'paused')"
            "(__import__('omni.timeline').timeline.get_timeline_interface())",
            echo=False,
        )

    def _timeline(self, action: str, frames: int) -> None:
        self.run(
            "import omni.kit.app, omni.timeline\n"
            f"omni.timeline.get_timeline_interface().{action}()\n"
            f"for _ in range({frames}):\n"
            "    await omni.kit.app.get_app().next_update_async()",
            timeout=30.0,
        )

    def play(self) -> None:
        self._timeline("play", 5)

    def pause(self) -> None:
        self._timeline("pause", 2)

    def stop(self) -> None:
        """Stop the timeline: the platform returns to the authored stage pose."""
        self._timeline("stop", 3)

    # -- servo driver -------------------------------------------------------------
    def armed(self) -> bool:
        """Whether the driver runs, and if so bind it as ``servos`` in this context."""
        return bool(self.run(f"(servos := {_SERVOS}) is not None", echo=False))

    def arm(self) -> None:
        """Open the scene if needed and (re-)arm the STS3215 servo models from the active profile."""
        self.open_scene()
        self.run_file(DRIVER)

    def ensure_armed(self) -> None:
        """Arm unless the driver already runs on the open scene."""
        if self.open_scene() or not self.armed():
            self.run_file(DRIVER)

    def start(self) -> None:
        """Scene open, servos armed, timeline playing: ready for commands."""
        self.ensure_armed()
        if self.state() != "playing":
            self.play()

    def goto(self, goals: dict[int, float], speed: int | None = None) -> float:
        """Like ``Rig.goto``: goal counts per axis id. Returns the simulation time of the command."""
        goals = {int(k): float(v) for k, v in goals.items()}
        return self.run(f"servos.goto({goals!r}, speed={speed!r})", echo=False)

    def home(self, speed: int | None = None) -> float:
        return self.run(f"servos.home(speed={speed!r})", echo=False)

    def positions(self) -> dict[int, float]:
        """Joint positions in counts; needs a playing timeline."""
        return {int(k): v for k, v in self.run("servos.positions()", echo=False).items()}

    def time(self) -> float:
        """Simulation time in seconds since Play."""
        return self.run("servos.time", echo=False)
