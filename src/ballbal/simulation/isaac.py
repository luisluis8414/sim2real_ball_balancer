"""STS3215 servos for the Isaac Sim ball balancer, driven in encoder counts like the real rig.

Runs inside Isaac Sim. From the repository, ``python tools/simulation/sim.py start`` opens
``models/usd/scene.usda``, sends this file through the Python Server and presses Play (see
``remote.py``); the Script Editor works too. Afterwards, in the same interpreter:

    servos.goto({1: 1800, 2: 1300, 3: 1400})   # axis ids -> counts, exactly like Rig.goto

Limits, home, rest pose, goal speed and acceleration come from the active profile via
``profile.py``. It is re-read on every
Play from a stopped timeline, so a recalibrated rig applies to the next run without re-arming.

Each physics step advances an STS3215 behavioural model per axis (dead time, trapezoidal
profile, position-loop lag; identified on the real rig, see ``docs/servo-model.md``) and hands its
output to a stiff PhysX position drive. The drive torque is capped at the servo's torque, so a
load it cannot carry still slows it down.
"""

import builtins
import math
import sys
import types
from pathlib import Path

import omni.physx
import omni.timeline
import omni.usd



def _source_dir():
    """Find the simulation package from this file or from the open repository scene."""
    candidates = []
    if "__file__" in globals():
        candidates.append(Path(__file__).resolve().parent)
    scene = omni.usd.get_context().get_stage().GetRootLayer().realPath
    if scene:
        candidates.append(Path(scene).resolve().parent)
    for anchor in candidates:
        for folder in (anchor, *anchor.parents):
            if (folder / "servo_model.py").is_file() and (folder / "profile.py").is_file():
                return folder
            package = folder / "src" / "ballbal" / "simulation"
            if (package / "servo_model.py").is_file() and (package / "profile.py").is_file():
                return package
    raise RuntimeError(
        "simulation package not found; open models/usd/scene.usda from the repository "
        "and run src/ballbal/simulation/isaac.py"
    )


SOURCE_DIR = _source_dir()


def _fresh_module(name, path):
    """Compile a sim module from source: importlib.reload inside Kit hands back previously loaded code."""
    module = types.ModuleType(name)
    module.__file__ = str(path)
    sys.modules[name] = module
    exec(compile(Path(module.__file__).read_text(), module.__file__, "exec"), module.__dict__)
    return module


sts3215 = _fresh_module("ballbal_sim_servo", SOURCE_DIR / "servo_model.py")
rig_link = _fresh_module("ballbal_sim_profile", SOURCE_DIR / "profile.py")
STS3215, STS3215Params = sts3215.STS3215, sts3215.STS3215Params

ROBOT = "/World/ball_balancer"

# Tracking drive for the modelled servo position. USD angular drives use degrees.
DRIVE_STIFFNESS_NM_PER_RAD = 50.0
DRIVE_DAMPING_NMS_PER_RAD = 0.15
# STS3215 stall torque 19.5 kg*cm at 7.4 V, scaled to the 5.4 V supply (datasheet-derived, not measured)
MAX_TORQUE_NM = 19.5 * 0.0980665 * 5.4 / 7.4
MAX_JOINT_VELOCITY_DEG_S = 300.0  # above the servo's 218 deg/s, so the model sets the speed
HOME_POSE_TOLERANCE = 3  # counts between authored stage pose and calibrated home before warning
# Elbows are passive and unlimited (exported as "continuous" from Fusion): on the rig all three axes
# together reach ~136-141 deg, which folds the elbows past 180 deg. The servo limits bound the travel.


class SimServos:
    def __init__(self):
        self.stage = omni.usd.get_context().get_stage()
        self.time = 0.0
        self.schedule = []  # [(sim time, {id: counts})] for replaying recorded goals
        self._articulation = None
        self._load_rig()
        self._sub = omni.physx.get_physx_interface().subscribe_physics_step_events(self._on_step)
        self._tl_sub = omni.timeline.get_timeline_interface().get_timeline_event_stream().create_subscription_to_pop(
            self._on_timeline
        )

    # -- platform profile ---------------------------------------------------
    def _load_rig(self):
        self.rig = rig_link.load()
        self.axes = self.rig.axes
        self.params = rig_link.scaled_params(STS3215Params.load(), self.rig)
        self.joint = {i: self.stage.GetPrimAtPath(f"{ROBOT}/Physics/{a.sim_joint}") for i, a in self.axes.items()}
        # the models start where the authored stage pose puts each servo (set_home_pose.py authors home)
        self.start = {}
        for i, a in self.axes.items():
            deg = self.joint[i].GetAttribute("state:angular:physics:position").Get() or 0.0
            self.start[i] = a.zero + deg * 4096.0 / 360.0
        self.models = {i: STS3215(self.params, self.start[i]) for i in self.axes}
        self._configure_joints()
        off = {a.name: round(self.start[i] - a.home) for i, a in self.axes.items()
               if abs(self.start[i] - a.home) > HOME_POSE_TOLERANCE}
        if off:
            print(f"note: the stage pose is not the profile's home (counts off: {off}); "
                  "run tools/simulation/set_home_pose.py to make home the paused pose")

    def _configure_joints(self):
        k = DRIVE_STIFFNESS_NM_PER_RAD * math.pi / 180.0
        c = DRIVE_DAMPING_NMS_PER_RAD * math.pi / 180.0
        for i, a in self.axes.items():
            j = self.joint[i]
            # The lower link lands on the base at the CAD contact angle. The URDF base has no collider, so that
            # stop is the joint's lower limit: commanded below it, the axis stalls there like the rig does.
            j.GetAttribute("physics:lowerLimit").Set(max(a.deg(a.min), self.rig.contact_deg))
            j.GetAttribute("physics:upperLimit").Set(a.deg(a.max))
            j.GetAttribute("drive:angular:physics:stiffness").Set(k)
            j.GetAttribute("drive:angular:physics:damping").Set(c)
            j.GetAttribute("drive:angular:physics:maxForce").Set(MAX_TORQUE_NM)
            j.GetAttribute("drive:angular:physics:targetPosition").Set(a.deg(self.start[i]))
            j.GetAttribute("drive:angular:physics:targetVelocity").Set(0.0)
            j.GetAttribute("physxJoint:maxJointVelocity").Set(MAX_JOINT_VELOCITY_DEG_S)
        for n in (1, 2, 3):
            e = self.stage.GetPrimAtPath(f"{ROBOT}/Physics/elbow_{n}")
            e.GetAttribute("physics:lowerLimit").Set(float("-inf"))
            e.GetAttribute("physics:upperLimit").Set(float("inf"))

    def destroy(self):
        self._sub = None
        self._tl_sub = None

    # -- commands (same shape as ballbal's Rig) -------------------------------
    def clamp(self, axis_id, counts):
        a = self.axes[axis_id]
        return max(a.min, min(a.max, int(round(counts))))

    def goto(self, goals, speed=None):
        """Like Rig.goto: goal counts per axis id; ``speed`` in counts/s, None = rig.toml's speed."""
        for i, g in goals.items():
            self.models[i].set_goal(self.clamp(i, g), self.time, speed if speed is not None else self.rig.speed)
        return self.time

    def home(self, speed=None):
        return self.goto({i: a.home for i, a in self.axes.items()}, speed)

    def follow(self, positions):
        """Hold the axes at measured positions in counts, e.g. read from the real rig.

        A measurement already contains the servo's own dynamics, so the model is bypassed: it is
        reset to the reading and holds it until the next one. Keys may be ids or their strings.
        """
        for i, counts in positions.items():
            i = int(i)
            self.models[i].reset(self.clamp(i, counts))

    def positions(self):
        """Measured joint positions in counts (what PRESENT_POSITION would read); needs a running simulation.

        Read from the physics tensors: the joint-state attributes in USD only refresh with the
        render loop and lag the physics by up to a frame.
        """
        if self._articulation is None:
            from isaacsim.core.experimental.prims import Articulation

            self._articulation = Articulation(f"{ROBOT}/Geometry")
        q = self._articulation.get_dof_positions().numpy()[0]
        names = self._articulation.dof_names
        return {i: a.zero + math.degrees(float(q[names.index(a.sim_joint)])) * 4096.0 / 360.0 for i, a in self.axes.items()}

    def model_positions(self):
        return {i: m.position for i, m in self.models.items()}

    # -- stepping -------------------------------------------------------------
    def _on_step(self, dt):
        if self.time == 0.0 and rig_link.profile_mtime() != self.rig.mtime:
            # First step of a fresh run: pick up profile changes.
            self._load_rig()
            print(f"platform profile changed -> reloaded: {self._summary()}")
        while self.schedule and self.schedule[0][0] <= self.time:
            self.goto(self.schedule.pop(0)[1])
        self.time += dt
        for i, m in self.models.items():
            q_prev = m.position
            q = m.step(self.time, dt)
            j = self.joint[i]
            j.GetAttribute("drive:angular:physics:targetPosition").Set(self.axes[i].deg(q))
            # velocity feed-forward, so the drive's damping does not add lag behind the model
            j.GetAttribute("drive:angular:physics:targetVelocity").Set((q - q_prev) / dt * 360.0 / 4096.0)

    def _on_timeline(self, event):
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            self.time = 0.0
            self.schedule = []
            self._articulation = None
            for i, m in self.models.items():
                m.reset(self.start[i])
                self.joint[i].GetAttribute("drive:angular:physics:targetPosition").Set(self.axes[i].deg(self.start[i]))
                self.joint[i].GetAttribute("drive:angular:physics:targetVelocity").Set(0.0)

    def _summary(self):
        return "; ".join(f"{a.name} {a.min}..{a.max} home {a.home} rest {a.rest}" for a in self.axes.values()) + (
            f"; speed {self.rig.speed}, acceleration {self.rig.acceleration}")


old = getattr(builtins, "_sim_servos", None)
if old is not None:
    old.destroy()
sweep_sub = getattr(builtins, "_platform_sweep_sub", None)  # the old cosine sweep drove the same joints
if sweep_sub is not None:
    sweep_sub.unsubscribe()
    builtins._platform_sweep_sub = None
servos = builtins._sim_servos = SimServos()
print(f"STS3215 servos armed from {servos.rig.source}: {servos._summary()}; torque cap {MAX_TORQUE_NM:.2f} Nm")
