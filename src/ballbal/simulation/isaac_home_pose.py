"""Make the rig's home pose the authored (paused / not playing) pose of the Isaac Sim ball balancer.

Runs inside Isaac Sim after ``isaac.py``; ``tools/simulation/set_home_pose.py`` sends it through
the Python Server and awaits ``set_home_pose()``. The new pose is saved into the scene file.

The closed linkage has no closed-form pose to write, so the simulation solves it: play, drive
the STS3215 models to home, let it settle, record every link's transform and joint angle, stop,
and author those as the stage pose. Play then starts from home with nothing to jump.
"""

import builtins
import math
from pathlib import Path

import omni.kit.app
import omni.physx
import omni.timeline
import omni.usd
from pxr import Gf, UsdGeom

SERVOS_PY = Path(__file__).resolve().parent / "isaac.py"

SETTLE_S = 2.0
BASE = "/World/ball_balancer/Geometry/base_link"
LINKS = {  # moving links, parents before children (the USD hierarchy nests them)
    "lower_link_1_1": f"{BASE}/lower_link_1_1",
    "upper_link_1_1": f"{BASE}/lower_link_1_1/upper_link_1_1",
    "platform_link_1": f"{BASE}/lower_link_1_1/upper_link_1_1/platform_link_1",
    "lower_link_2_1": f"{BASE}/lower_link_2_1",
    "upper_link_2_1": f"{BASE}/lower_link_2_1/upper_link_2_1",
    "lower_link_3_1": f"{BASE}/lower_link_3_1",
    "upper_link_3_1": f"{BASE}/lower_link_3_1/upper_link_3_1",
}
JOINTS = ["servo_1", "servo_2", "servo_3", "elbow_1", "elbow_2", "elbow_3"]


def _world(px, path):
    t = px.get_rigidbody_transformation(path)
    r = t["rotation"]  # (x, y, z, w)
    m = Gf.Matrix4d()
    m.SetTransform(Gf.Rotation(Gf.Quatd(r[3], r[0], r[1], r[2])), Gf.Vec3d(*t["position"]))
    return m


async def set_home_pose():
    servos = builtins._sim_servos
    stage = omni.usd.get_context().get_stage()
    app, tl, px = omni.kit.app.get_app(), omni.timeline.get_timeline_interface(), omni.physx.get_physx_interface()

    tl.stop()
    await app.next_update_async()
    tl.play()
    await app.next_update_async()
    servos.home()
    while servos.time < SETTLE_S:
        await app.next_update_async()

    worlds = {name: _world(px, path) for name, path in LINKS.items()}
    worlds["base"] = _world(px, BASE)
    from isaacsim.core.experimental.prims import Articulation

    art = Articulation("/World/ball_balancer/Geometry")
    q = dict(zip(art.dof_names, (float(v) for v in art.get_dof_positions().numpy()[0])))
    tl.stop()
    for _ in range(3):
        await app.next_update_async()

    for name, path in LINKS.items():
        parent = path.rsplit("/", 1)[0]
        parent_world = worlds["base"] if parent == BASE else worlds[parent.rsplit("/", 1)[1]]
        local = worlds[name] * parent_world.GetInverse()  # Gf matrices compose row-vector style
        prim = stage.GetPrimAtPath(path)
        translate, orient = prim.GetAttribute("xformOp:translate"), prim.GetAttribute("xformOp:orient")
        translate.Set(type(translate.Get())(*local.ExtractTranslation()))
        quat = local.ExtractRotationQuat()
        orient.Set(type(orient.Get())(quat.GetReal(), *quat.GetImaginary()))
    for joint in JOINTS:
        prim = stage.GetPrimAtPath(f"/World/ball_balancer/Physics/{joint}")
        prim.GetAttribute("state:angular:physics:position").Set(math.degrees(q[joint]))
        prim.GetAttribute("state:angular:physics:velocity").Set(0.0)

    # re-arm the servos so their models start from the new authored pose (rebinds `servos` here too)
    exec(compile(SERVOS_PY.read_text(), str(SERVOS_PY), "exec"), globals())
    z = UsdGeom.Xformable(stage.GetPrimAtPath(LINKS["platform_link_1"])).ComputeLocalToWorldTransform(0).ExtractTranslation()[2]
    stage.GetRootLayer().Save()
    print(f"saved {stage.GetRootLayer().realPath}; home pose: " + ", ".join(f"{j} {math.degrees(q[j]):.2f} deg" for j in JOINTS)
          + f"; platform frame z {z * 1000:.1f} mm")
