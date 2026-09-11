# Live view in Lichtblick

`ballbal balance --ui` streams the balance loop to
[Lichtblick](https://github.com/lichtblick-suite/lichtblick) and takes new
targets from it:

- the top camera with tracking overlays, and a second camera from the side;
- setpoint, ball position and the controller's P, I and D terms;
- goal and measured position of every servo;
- both Isaac Sim cameras (overhead and view);
- a top-down plate where a click sets the point the ball is balanced to;
- Start and Stop buttons and the controller's state.

The layout in [`tools/lichtblick/ballbal_layout.json`](../tools/lichtblick/ballbal_layout.json)
has four tabs:

```text
Live    +---------------+----------------------+---------------------+
        | top camera    | side camera          | State | Start | Stop|
        | + tracking    |                      +---------------------+
        |               +----------------------+ plate, top down     |
        |               | Isaac Sim view       | click = new target  |
        |               |                      +---------------------+
        |               |                      | distance to target  |
        +---------------+----------------------+---------------------+

PID     +------------------------+------------------------+-----------+
        |                        |                        |State/btns |
        | x: setpoint, ball,     | y: setpoint, ball,     | top camera|
        |    predicted           |    predicted           | + tracking|
        +------------------------+------------------------+-----------+
        | x: P, I, D, sum        | y: P, I, D, sum        | distance  |
        +------------------------+------------------------+-----------+

Servos  +--------------------------------------+----------------------+
        |                                      | State | Start | Stop |
        | axis_1: goal, measured               | side camera          |
        | axis_2: goal, measured               |                      |
        | axis_3: goal, measured               |                      |
        | goal - measured, all axes            |                      |
        +--------------------------------------+----------------------+

Sim     top camera | Isaac Sim overhead | Isaac Sim view
```

P, I and D are in leg counts, the unit the gains are read in (see
[pid-optimisation.md](pid-optimisation.md)); their sum is the lean the
controller asks for on that axis, before the output limit. "predicted" is the
position the controller acts on, `predict_ms` ahead. `goal - measured` shows how
far each servo trails its command.

## Install

1. **Lichtblick** desktop app, release 1.29.1 or newer, from the
   [releases page](https://github.com/lichtblick-suite/lichtblick/releases):

   ```bash
   sudo apt install ./lichtblick-1.29.1-linux-amd64.deb
   ```

2. **The `ui` extra** in the repository environment:

   ```bash
   uv pip install -e ".[ui]"
   ```

   This adds `foxglove-sdk` for the message schemas and `foxglove-websocket` for
   the server.

## Run

```bash
ballbal balance --live --ui              # real rig
ballbal balance --live --ui --mirror     # and keep Isaac Sim in sync
ballbal circle --live --ui               # paths work too; a click ends the path
```

With `--ui` the program starts **ready**: cameras, tracking and plots run, the
servos are off and nothing is commanded. Balancing starts and stops from
Lichtblick:

- **Start balancing** energises the servos, levels the plate and balances.
- **Stop** levels the plate and turns the servos off again.
- The **State** panel shows `READY` (yellow) or `BALANCING` (green); the camera
  overlay says the same.

Start and Stop can be pressed any number of times. `--seconds` and a path's
clock count from Start. In the local preview window `s` does the same as the
buttons. Ctrl+C or `q` ends the program; the servos are released either way.

Without `--live` it is a dry run as usual: Start runs the controller, but the
servos stay off.

### Side camera

The side view is any second camera. With exactly two cameras attached it is
the one that is not tracked, without any setting. Otherwise choose it once, or
per run:

```bash
ballbal camera set --side 2              # stored; `ballbal camera show` marks it
ballbal balance --ui --side-camera 2     # this run only
ballbal balance --ui --side-camera none  # no side view
```

It is captured at 1280×720 MJPG on its own thread and forwarded as the camera
encodes it, so it costs the control loop nothing. Measured: the loop stays at
29.5 Hz with it running. It sends about 6 MB/s, which matters only when the view
is watched over a network.

In Lichtblick:

1. **Open connection → Foxglove WebSocket → `ws://localhost:8765`.**
2. **Layouts → Import from file →
   [`tools/lichtblick/ballbal_layout.json`](../tools/lichtblick/ballbal_layout.json).**
   Lichtblick keeps the imported copy; import again after the file changes.

   Alternatively copy it to `~/.lichtblick-suite/layouts/`, where Lichtblick
   looks for layouts when it starts; it then appears as `ballbal` under
   Layouts. It has to be a real file: Lichtblick skips symbolic links there.

   ```bash
   mkdir -p ~/.lichtblick-suite/layouts
   cp tools/lichtblick/ballbal_layout.json ~/.lichtblick-suite/layouts/ballbal.json
   ```

The simulated cameras appear when Isaac Sim is running with its Python Server
(see [simulation](simulation.md)) and its timeline is playing. `--mirror` and
`python tools/simulation/sim.py start` both play it. Isaac Sim renders these
extra cameras only while the timeline plays; while it is stopped the two panels
stay empty. The real camera does not depend on Isaac Sim.

## Setting a target with the mouse

In the plate panel (top right), open the publish tool in the panel's toolbar
(the cursor icon at the top right of the scene) and choose **Publish point**.
Then click on the plate. The click is sent on `/clicked_point` and becomes the
target immediately:

- the terminal prints `target set from Lichtblick: (+30, -20) mm`;
- the red marker moves in the plate panel and in the tracking overlay;
- a running `circle` or `square` path stops and the ball holds the new point.

Targets are kept within 75% of the plate radius, because the ball cannot
balance against the guardrail. The plate panel is oriented like the camera
image: right is right and up is up.

## What is sent

| Topic | Type | Content |
| --- | --- | --- |
| `/camera/real` | `foxglove.CompressedImage` | the square crop the tracker works on, 30 fps |
| `/camera/real/tracking` | `foxglove.ImageAnnotations` | plate circle, centre, ball, target (5 mm band), predicted position, trail, error and loop rate |
| `/camera/sim/overhead`, `/camera/sim/view` | `foxglove.CompressedImage` | Isaac Sim cameras, 10 fps; overhead 640×360, view 1280×720 (`CAMERAS` in `isaac_cameras.py`) |
| `/camera/side` | `foxglove.CompressedImage` | side camera, 1280×720, up to 30 fps |
| `/plate` | `foxglove.SceneUpdate` | plate, ball, target, prediction and trail in frame `plate`, in centimetres (Lichtblick's fixed-size click marker and axes stay small) |
| `/ball` | JSON | `x`, `y`, `target_x`, `target_y`, `predicted_x`, `predicted_y`, `error` (mm), `tilt_x`, `tilt_y`, `loop_hz`, `found` |
| `/pid` | JSON | `x.p`, `x.i`, `x.d`, `x.sum` and the same for `y`, in leg counts; only while the ball is seen |
| `/servos` | JSON | `goal.axis_N`, `measured.axis_N` and `lag.axis_N` (goal − measured) in counts |
| `/balance/state` | JSON | `state`: `ready` or `balancing` |
| `/clicked_point` | from Lichtblick | `geometry_msgs/PointStamped`, becomes the target |
| `/balance/command` | from Lichtblick | `{"command": "start"}` or `{"command": "stop"}`, sent by the two buttons |

"With" and "without tracking" are the same image: the annotation topic is
switched on in one image panel and off in the other (panel settings →
Annotations).

`/ball`, `/pid` and `/servos` can be plotted in any Plot panel, e.g.
`/ball.error`, `/pid.x.d` or `/servos.measured.axis_2`.

## How it fits together

The server runs inside the `balance` process on its own thread, because camera
and servo bus serve one process at a time. The control loop hands each frame
over and never waits: frames the server has not sent yet are replaced by newer
ones. Measured servo positions cost the loop one sync read per frame (about
1 ms), shared with `--mirror`. The simulated cameras are fetched from Isaac Sim
through its Python Server
([`isaac_cameras.py`](../src/ballbal/simulation/isaac_cameras.py)). Each
camera has its own render product, so the viewport's camera is never touched.

The server announces itself as a ROS 2 source (`ROS_DISTRO` in its metadata).
Lichtblick's 3D panel only publishes clicks to such sources. This is why it is
served with `foxglove-websocket`, which can send that metadata, rather than the
`foxglove-sdk` server; the messages themselves use the SDK's schemas.

`BALLBAL_UI_PORT` and `BALLBAL_UI_HOST` change the address (default
`127.0.0.1:8765`). Set the host to `0.0.0.0` to watch from another machine; the
server takes targets from anyone who can reach it.

## Troubleshooting

| Symptom | Cause and fix |
| --- | --- |
| `--ui needs the ui extra` | `uv pip install -e ".[ui]"` |
| `live view could not start ... address already in use` | Another `ballbal ... --ui` is running, or set `BALLBAL_UI_PORT`. |
| Sim camera panels empty | Isaac Sim not running with its Python Server, or timeline stopped. Use `--mirror` or `sim.py start`. |
| Side camera panel empty | No second camera found (the terminal prints `side camera ... on /camera/side` when there is one). Set it with `ballbal camera set --side` or `--side-camera`. |
| Old layout without tabs | Import [`ballbal_layout.json`](../tools/lichtblick/ballbal_layout.json) again; Lichtblick keeps the previously imported copy. |
| Start/Stop buttons greyed out or "Schema name not found" | Not connected to a running `ballbal ... --ui`; the button's schema `balance_control` comes from the server. |
| State panel shows `NOT CONNECTED` | No `/balance/state` messages: the program is not running or Lichtblick is connected to another address. |
| Clicks do nothing | Publish tool not set to **Publish point**, or the topic in the panel's publish settings is not `/clicked_point`. |
| Plate panel empty or "no coordinate frames" | Display frame must be `plate` (panel settings → Frame). The layout sets it. |
