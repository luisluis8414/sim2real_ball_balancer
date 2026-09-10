"""Drive the simulated platform between its lowest and highest position while the timeline plays.

Run inside Isaac Sim (Script Editor, or the Python Server on port 8226) with
``models/usd/scene.usda`` open. It arms the STS3215 servo models and, on every Play, commands the
goals the real rig would get from ``ballbal goto``: all axes to min, then all to max, and back,
HOLD_S apart. How fast the platform gets there is the identified servo behaviour, not a
scripted trajectory. Re-running replaces the previous sweep; stop() removes it.
"""

import builtins
from pathlib import Path

import omni.kit.app
import omni.timeline
import omni.usd

def _driver_path():
    scene = Path(omni.usd.get_context().get_stage().GetRootLayer().realPath).resolve()
    for root in scene.parents:
        driver = root / "src" / "ballbal" / "simulation" / "isaac.py"
        if driver.is_file():
            return driver
    raise RuntimeError("open models/usd/scene.usda from the repository first")


exec(open(_driver_path()).read())

HOLD_S = 1.5  # time between goal writes


def stop():
    sub = getattr(builtins, "_platform_sweep_sub", None)
    if sub is not None:
        sub.unsubscribe()
        builtins._platform_sweep_sub = None


def start():
    stop()
    timeline = omni.timeline.get_timeline_interface()
    state = {"next": 0.0, "up": True}

    def on_update(_event):
        if not timeline.is_playing():
            state["next"], state["up"] = 0.0, False  # first goal after Play: min
            return
        servos = builtins._sim_servos  # profile limits, reloaded on Play
        if servos.time >= state["next"]:
            servos.goto({i: (a.max if state["up"] else a.min) for i, a in servos.axes.items()})
            state["up"] = not state["up"]
            state["next"] = servos.time + HOLD_S

    builtins._platform_sweep_sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
        on_update, name="platform_sweep"
    )
    print(f"platform sweep armed: min <-> max every {HOLD_S:.1f} s (press Play)")


start()
