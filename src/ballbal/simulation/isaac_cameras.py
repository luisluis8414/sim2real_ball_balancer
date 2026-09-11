"""The simulated cameras as JPEG frames, for the Lichtblick live view (``ballbal balance --ui``).

Runs inside Isaac Sim; ``ballbal.ui`` sends it through the Python Server and then calls
``sim_cameras.grab()`` about ten times a second, which returns ``{name: base64 JPEG}``.

Each camera gets its own render product with an RGB annotator. The viewport is never pointed at
a scene camera: doing that lets Kit rewrite the camera's transform in the stage. Render products
render on every app update while they exist, so this costs GPU time for as long as the view runs;
``sim_cameras.destroy()`` releases them.
"""

import base64
import builtins

import cv2
import numpy as np
import omni.replicator.core as rep
import omni.usd

CAMERAS = {
    "overhead": ("/World/ball_balancer/Geometry/base_link/overhead_camera", (640, 360)),
    "view": ("/World/view_camera", (1280, 720)),
}


class SimCameras:
    def __init__(self):
        stage = omni.usd.get_context().get_stage()
        self.products, self.annotators = {}, {}
        for name, (path, size) in CAMERAS.items():
            if not stage.GetPrimAtPath(path).IsValid():
                print(f"sim camera {name}: no prim at {path}, skipped")
                continue
            product = rep.create.render_product(path, size, name=f"ballbal_ui_{name}")
            annotator = rep.AnnotatorRegistry.get_annotator("rgb")
            annotator.attach([product])
            self.products[name], self.annotators[name] = product, annotator

    def grab(self, quality=80):
        """Latest frame of every camera that has rendered one, as base64 JPEG."""
        frames = {}
        for name, annotator in self.annotators.items():
            data = np.asarray(annotator.get_data())
            if data.ndim != 3 or data.size == 0 or not data.any():
                continue
            bgr = cv2.cvtColor(data[..., :3], cv2.COLOR_RGB2BGR)
            ok, jpeg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
            if ok:
                frames[name] = base64.b64encode(jpeg.tobytes()).decode()
        return frames

    def destroy(self):
        for name, annotator in self.annotators.items():
            annotator.detach([self.products[name]])
            self.products[name].destroy()
        self.products, self.annotators = {}, {}


old = getattr(builtins, "_sim_cameras", None)
if old is not None:
    old.destroy()
sim_cameras = builtins._sim_cameras = SimCameras()
print(f"sim cameras for the live view: {', '.join(sim_cameras.annotators) or 'none'}")
