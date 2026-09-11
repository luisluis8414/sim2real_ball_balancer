"""A marker for the real ball on the simulated plate, placed from the rig's camera tracking.

Runs inside Isaac Sim after ``isaac.py``; ``tools/simulation/ball_sync.py`` sends it through the
Python Server and then calls ``ball.place(x_mm, y_mm)`` for every camera frame.

Coordinates are in the plate frame, shared with the real rig: origin at the plate centre, +x
towards the axis_1 leg, +y a quarter turn counter-clockwise seen from above. The frame is taken
from the model itself -- centre = centroid of the three platform ball joints, direction = the
joint on axis_1's leg (``params.json`` sim_joints) -- so it holds for any rig whose legs map the
same way.

The marker is a child of the platform link, so it tilts and lifts with the simulated plate. It
is visual only (no collider, no rigid body), so it never pushes the mechanism, and it is authored
in the session layer, so it is never saved into ``scene.usda``.
"""

import builtins

import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics

ROBOT = "/World/ball_balancer"
PLATFORM = f"{ROBOT}/Geometry/base_link/lower_link_1_1/upper_link_1_1/platform_link_1"
MARKER = f"{PLATFORM}/real_ball"
BALL_JOINTS = f"{ROBOT}/Physics/spherical"
COLOUR = (1.0, 0.45, 0.05)


class BallMarker:
    def __init__(self, diameter_mm):
        self.stage = omni.usd.get_context().get_stage()
        self.session = self.stage.GetSessionLayer()
        self.radius = diameter_mm / 2000.0
        with Usd.EditContext(self.stage, self.session):
            self.stage.RemovePrim(MARKER)  # a re-run starts clean and keeps it out of the bounds below
        self._plate_frame()
        with Usd.EditContext(self.stage, self.session):
            sphere = UsdGeom.Sphere.Define(self.stage, MARKER)
            sphere.CreateRadiusAttr(self.radius)
            sphere.CreateExtentAttr([Gf.Vec3f(-self.radius), Gf.Vec3f(self.radius)])
            sphere.CreateDisplayColorAttr([Gf.Vec3f(*COLOUR)])
            self._translate = sphere.AddTranslateOp()
            self._translate.Set(self._point(0.0, 0.0))
        self.imageable = UsdGeom.Imageable(sphere)
        self.visible = True
        self.hide()

    def _plate_frame(self):
        """Plate centre, in-plane axes and top surface in the platform link's frame."""
        link = self.stage.GetPrimAtPath(PLATFORM)
        points = {}
        for n in (1, 2, 3):
            joint = UsdPhysics.SphericalJoint(self.stage.GetPrimAtPath(f"{BALL_JOINTS}/ball_{n}"))
            if joint.GetBody1Rel().GetTargets() != [link.GetPath()]:
                raise RuntimeError(f"ball_{n} does not end on {PLATFORM}")
            points[n] = Gf.Vec3d(joint.GetLocalPos1Attr().Get())
        centre = (points[1] + points[2] + points[3]) / 3.0
        servos = builtins._sim_servos
        leg = int(servos.axes[1].sim_joint.rsplit("_", 1)[1])  # axis_1's servo_n ends in ball_n
        e1 = points[leg] - centre
        e1 = Gf.Vec3d(e1[0], e1[1], 0.0).GetNormalized()
        self.e1, self.e2, self.normal = e1, Gf.Vec3d(-e1[1], e1[0], 0.0), Gf.Vec3d(0.0, 0.0, 1.0)
        self.centre = Gf.Vec3d(centre[0], centre[1], self._top(link))

    def _top(self, link):
        """Highest point of the platform's meshes in the link frame: the plate's top face.

        From the points themselves, not a bounding box: with the timeline playing, BBoxCache
        put the top 5.5 mm above the real face (15.7 against 10.3 mm), and the ball floated.
        """
        cache = UsdGeom.XformCache(0)
        top = None
        for prim in Usd.PrimRange(link, Usd.TraverseInstanceProxies()):
            if not prim.IsA(UsdGeom.Mesh) or prim.GetPath().HasPrefix(MARKER):
                continue
            to_link, _ = cache.ComputeRelativeTransform(prim, link)
            for point in UsdGeom.Mesh(prim).GetPointsAttr().Get() or []:
                z = to_link.Transform(Gf.Vec3d(point))[2]
                top = z if top is None or z > top else top
        if top is None:
            raise RuntimeError(f"no mesh under {PLATFORM} to find the plate's top face")
        return top

    def _point(self, x_mm, y_mm):
        return self.centre + self.e1 * (x_mm / 1000.0) + self.e2 * (y_mm / 1000.0) + self.normal * self.radius

    def place(self, x_mm, y_mm):
        """Put the ball at plate coordinates in mm and show it."""
        with Usd.EditContext(self.stage, self.session):
            self._translate.Set(self._point(x_mm, y_mm))
            if not self.visible:
                self.imageable.MakeVisible()
                self.visible = True

    def hide(self):
        """The camera lost the ball."""
        if self.visible:
            with Usd.EditContext(self.stage, self.session):
                self.imageable.MakeInvisible()
            self.visible = False

    def remove(self):
        with Usd.EditContext(self.stage, self.session):
            self.stage.RemovePrim(MARKER)


ball = builtins._sim_ball = BallMarker(globals().get("ball_mm", 40.0))
print(f"real-ball marker ready: {ball.radius * 2000:.0f} mm, +x towards {builtins._sim_servos.axes[1].name}")
