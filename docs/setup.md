# Setup

This guide takes a new build from an empty checkout to a calibrated platform.
`ballbal setup` assigns the servos, prepares and centres them, waits for the
mechanism to be assembled, records the lower pose and verifies the resulting
travel. Camera calibration is a separate final step.

> `profiles/platform-v2` belongs to the repository author's hardware. It is
> included only as a reference and for sim-to-real comparison. Never use its
> calibration to control another platform.

## Bill of materials

The electronics are intentionally vendor-independent except for the servos.
Equivalent cameras and serial adapters work when Linux exposes stable devices
under `/dev/v4l/by-id` and `/dev/serial/by-id`.

| Qty | Part | Notes |
| ---: | --- | --- |
| 3 | Feetech STS3215 servo | One half-duplex bus; each receives its own ID during setup |
| 1 | STS/SCS-compatible USB serial adapter | Feetech URT-1 or a compatible CH340/CH343 half-duplex servo board |
| 1 | Regulated 5 V DC supply | Must carry all three servos without voltage sag; power the servos through the bus board, not USB |
| 1 | UVC USB camera | Mounted vertically above the centre of the plate |
| 1 | Linux computer | Python 3.11 or newer and one USB port each for bus and camera |
| 1 | 40 mm ball | The diameter is entered during camera setup |
| 3 | Servo horn with retaining screw | Compatible with the STS3215 output spline |
| 1 set | Servo cables and power wiring | Three servos on the shared bus; daisy-chain or distribution board |
| 1 set | Mechanical pivots and fasteners | For three identical linkages, servo mounting and the camera mast; verify dimensions against the Fusion model before ordering |

Print the following files from [`models/stl/`](../models/stl):

| Qty | File |
| ---: | --- |
| 1 | [`base.stl`](../models/stl/base.stl) |
| 3 | [`lower_link.stl`](../models/stl/lower_link.stl) |
| 3 | [`upper_link.stl`](../models/stl/upper_link.stl) |
| 1 | [`platform.stl`](../models/stl/platform.stl) |
| 1 | [`guardrail.stl`](../models/stl/guardrail.stl) |
| 1 | [`camera_mast_and_foot.stl`](../models/stl/camera_mast_and_foot.stl) |
| 1 | [`camera_overhead_arm.stl`](../models/stl/camera_overhead_arm.stl) |
| 1 | [`ball_calibration.stl`](../models/stl/ball_calibration.stl), optional calibration aid |

The editable assembly is
[`models/fusion/ball_balancer.f3d`](../models/fusion/ball_balancer.f3d). Screw
lengths depend on the printed tolerances, chosen horns and camera, so they are
not fixed by the software repository.

## Quickstart

Run all commands from the repository root.

### 1. Install

```bash
uv venv .venv
uv pip install --python .venv/bin/python -e ".[vision]"
source .venv/bin/activate
```

After activation, use `ballbal` directly; no `.venv/bin/ballbal` prefix is
needed.

### 2. Create the platform profile

```bash
cp -r profiles/example profiles/my-platform
ballbal config set my-platform
ballbal config show
```

Edit `profiles/my-platform/rig.toml` only if the connection or shared geometry
differs. `home_offset` and `max_offset` are fixed distances above each measured
minimum. Generated calibration stays in the same profile.

### 3. Connect and calibrate

Connect the serial adapter and its 5 V servo supply, but do not assemble the
linkages yet. Then run:

```bash
ballbal ports
ballbal setup
```

Follow the prompts exactly. ID assignment requires one servo on the bus at a
time. Centring requires bare servo horns with no linkage attached. The guide
then pauses so the complete mechanism can be assembled.

For minimum calibration, torque is off. Move the complete platform by hand into
its common lowest pose and hold it there. One confirmation reads all three
positions together. Only the three minima are measured; home and maximum are
derived from the profile offsets. The final sweep moves the full mechanism, so
keep hands clear and stay near the power switch.

Resume an interrupted setup at a known step, for example:

```bash
ballbal setup --from calibrate
```

### 4. Select and calibrate the camera

Mount the camera above the plate, connect it, then run:

```bash
ballbal camera set
ballbal cam-setup
```

Both commands are interactive. Camera selection is stored by its stable device
path; later commands do not need `--device`. Camera setup records the platform
frame, image scale and ball colour in the active profile.

### 5. Verify and run

```bash
ballbal status
ballbal check
ballbal home
ballbal balance
```

`ballbal balance` is a dry run by default: camera and controller run, but servo
torque stays off. After checking the image, tracking and requested tilt, enable
real movement explicitly:

```bash
ballbal balance --live
```

Use `Ctrl-C` to stop; the command releases servo torque. Detailed commands and
safety behaviour are documented in [`docs/control.md`](control.md).
