# Platform simulation

The simulation is a digital twin of the platform in Isaac Sim. The three servo
axes are commanded in encoder counts, exactly like the real rig, and move the
way the identified STS3215 servos move. It is used to:

- try motions and control code without risking the hardware;
- check that the simulated platform matches the real one before relying on it
  (side-by-side comparison with the rig);
- mirror the real ball, tracked by the rig's camera, onto the simulated plate;
- provide a validated plant for later controller development.

Everything is controlled from this repository. Isaac Sim only runs and executes
what the repository scripts send to it.

> The included `platform-v2` profile and servo measurements come from the
> repository author's physical hardware. They are kept for reference and for the
> documented sim-to-real comparison, not as calibration for another platform.

## Install Isaac Sim 6.0.0.1

Install exactly the version this repository is developed and tested with:
**Isaac Sim 6.0.0.1**, pip installation, **Python 3.12**. Other versions may
change the scene format, the physics or the Python Server protocol.

Follow NVIDIA's official guide, choosing version 6.0.0 there:

- [Isaac Sim 6.0.0: Python environment installation](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/install_python.html)
- [Isaac Sim 6.0.0: system requirements](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/requirements.html)

Isaac Sim gets its own environment, separate from the repository's `.venv`. With
conda:

```bash
conda create -n isaacsim python=3.12
conda activate isaacsim
pip install "isaacsim[all,extscache]==6.0.0.1" --extra-index-url https://pypi.nvidia.com
```

Install the PyTorch build the official guide names before this step. On the
first launch Isaac Sim asks you to accept the NVIDIA EULA in the terminal.

The two environments never import each other. The repository's `.venv` holds
`ballbal`, the camera and the servo bus; the `isaacsim` environment holds Isaac
Sim. They talk over a local TCP socket (next section).

## Launch Isaac Sim

Start Isaac Sim with its Python Server extension enabled, in its own terminal:

```bash
conda activate isaacsim
isaacsim isaacsim.exp.full --enable isaacsim.code_editor.python_server
```

Wait until the window is up. The server now listens on `127.0.0.1:8226`. It
executes any Python it receives and has no authentication, so it stays bound to
localhost.

Then, in a second terminal with the repository environment active:

```bash
python tools/simulation/sim.py start
```

This opens [`models/usd/scene.usda`](../models/usd/scene.usda), arms the
simulated servos from the active profile and presses Play. The platform is now
ready for commands:

```bash
python tools/simulation/sim.py home
python tools/simulation/sim.py goto axis_1=1800 axis_2=1300 axis_3=1400
python tools/simulation/sim.py positions
```

The scene and servos use the profile selected with `ballbal config set <name>`,
the same one real control uses.

## How the repository controls Isaac Sim

```text
repository .venv                                   Isaac Sim (conda env isaacsim)
----------------                                   ------------------------------
tools/simulation/*.py                              isaacsim.code_editor.python_server
  └─ ballbal.simulation.remote.IsaacSim  ──TCP──►    executes the code in context "ballbal"
       run(code) / run_file(path)       127.0.0.1:8226     │
                                                            ├─ src/ballbal/simulation/isaac.py      servo driver
                                                            ├─ src/ballbal/simulation/isaac_ball.py real-ball marker
                                                            └─ src/ballbal/simulation/isaac_home_pose.py
```

- [`remote.py`](../src/ballbal/simulation/remote.py) is the client, plain
  standard library. It opens the scene, arms the driver, runs the timeline and
  sends commands. Isaac Sim's stdout is echoed in the repository terminal, and
  errors come back with Isaac's traceback.
- The Isaac-side files live in `src/ballbal/simulation/`. They are read from
  the checkout by Isaac Sim, so edits apply the next time they are sent
  (`sim.py arm` for the driver).
- Requests share the named context `ballbal`, so variables such as `servos`
  and `ball` persist between calls.
- `BALLBAL_ISAAC_HOST`, `BALLBAL_ISAAC_PORT` and `BALLBAL_ISAAC_TOKEN` override
  the address and add an auth token when the server is configured with
  `require_auth`.

Using it from your own code:

```python
from ballbal.simulation.remote import IsaacSim

sim = IsaacSim()
sim.start()                                  # scene open, servos armed, playing
sim.goto({1: 1800, 2: 1300, 3: 1400}, speed=1000)
print(sim.positions())                       # {1: 1799.8, 2: 1300.1, 3: 1399.9}
sim.run("servos.time")                       # any expression inside Isaac Sim
```

## Commands and tools

All scripts run from the repository root with the repository environment.

| Command | Purpose |
| --- | --- |
| `python tools/simulation/sim.py start` | Open the scene, arm the servos, press Play |
| `python tools/simulation/sim.py status` | Server, scene, timeline, profile and positions |
| `python tools/simulation/sim.py home [--speed N]` | Drive every axis to the profile's home |
| `python tools/simulation/sim.py goto 1=1800 axis_3=1400 [--speed N]` | Goal counts per axis, by id or name |
| `python tools/simulation/sim.py positions` | Simulated joint positions in counts |
| `python tools/simulation/sim.py play` / `pause` / `stop` | Timeline control |
| `python tools/simulation/sim.py arm` | Re-arm after a profile switch or driver edits |
| `python tools/simulation/sim.py exec "CODE"` | Run Python inside Isaac Sim and print its value |
| `python tools/simulation/sim.py run FILE` | Run an Isaac-side Python file inside Isaac Sim |
| [`python tools/simulation/platform_sweep.py`](../tools/simulation/platform_sweep.py) | All axes min ↔ max, then home |
| [`python tools/simulation/set_home_pose.py`](../tools/simulation/set_home_pose.py) | Save the calibrated home pose as the scene's stage pose |
| [`python tools/simulation/ball_sync.py`](../tools/simulation/ball_sync.py) | Mirror the real ball (and optionally the platform) into the simulation |
| [`python tools/simulation/compare_real.py`](../tools/simulation/compare_real.py) | Command real and simulated platforms side by side |
| [`identify.py`](../tools/simulation/identify.py), [`identify_tracking.py`](../tools/simulation/identify_tracking.py), [`analyze.py`](../tools/simulation/analyze.py) | Record and fit the servo model, see [servo-model.md](servo-model.md) |

Commands that move the platform run `start` first, so they work on a freshly
launched Isaac Sim.

## Mirror the real ball

```bash
python tools/simulation/ball_sync.py          # ball only
python tools/simulation/ball_sync.py --rig    # ball and real platform pose
```

Every camera frame the ball is found the way `ballbal balance` finds it. The
profile's `camera.json` supplies crop, plate centre, mm/px, ball colour and the
bearing of axis_1. The position is converted into the **plate frame** shared
with the simulation:

- origin at the plate centre;
- +x towards the axis_1 leg;
- +y a quarter turn counter-clockwise, seen from above.

Isaac Sim shows the ball as an orange sphere of the calibrated diameter lying on
the simulated plate. The marker tilts and lifts with the plate. It is visual
only, so it never pushes the mechanism, and it lives in the session layer, so it
is never saved into `scene.usda`. When the camera loses the ball, the marker is
hidden.

With `--rig` the real servo positions are read every frame. Torque is not
touched and nothing moves. The simulated axes are held at those readings, so the
simulated plate leans like the real one. The servo bus serves one process at a
time, so `--rig` does not run alongside `ballbal balance` or `ballbal jog`.

Isaac Sim usually accepts fewer updates per second than the camera delivers
(about 22 against 30 on the author's machine). The script forwards only the
newest sample and drops older ones, so the camera loop never waits for the
simulation. Options: `--no-window`, `--seconds S`, `--device`,
`--calibration`, `--every N`.

The camera's lens distortion is not corrected, the same as in `ballbal
balance`, so positions near the rim are less accurate than near the centre.

## What is simulated

```text
profiles/<name>/rig.toml ----------+
profiles/<name>/calibration.toml --+--> profile.py --> isaac.py --> PhysX joint drives
simulation/params.json ------------+                    |
                                                        +--> STS3215 model per axis (servo_model.py)
```

- **Profile:** limits, home, rest pose, goal speed and acceleration come from
  the active profile, the same files real control uses.
  [`profile.py`](../src/ballbal/simulation/profile.py) only adds values that
  exist in the simulation alone: the joint mapping, the CAD rest angle and the
  servo dynamics from [`params.json`](../src/ballbal/simulation/params.json).
- **Servo behaviour:** every physics step advances one behavioural model per
  axis: 15.5 ms dead time, trapezoidal profile, 34.7 ms position-loop lag. It
  was identified on the real rig.
  See [servo-model.md](servo-model.md) for measurements and validation.
- **Physics:** the model output drives a stiff PhysX position drive on each
  servo joint. Torque is capped at the servo's stall torque at 5.4 V, so a load
  the servo cannot carry also slows the simulated axis.

The scene has no simulated ball and no balancing controller yet. The ball marker
above only mirrors the real ball.

## Driver reference

Inside Isaac Sim, `servos` is the armed driver (`sim.py exec` and
`IsaacSim.run` see it):

| Call | Effect |
| --- | --- |
| `servos.goto({id: counts}, speed=None)` | Set goals like `Rig.goto`, clamped to the profile limits. `speed` in counts/s, default `rig.toml`'s `speed`. Returns the simulation time. |
| `servos.home(speed=None)` | Go to the profile's home pose. |
| `servos.follow({id: counts})` | Hold the axes at measured positions, bypassing the servo model. |
| `servos.positions()` | Measured joint positions in counts. Needs a playing timeline. |
| `servos.model_positions()` | Output of the servo models in counts, before PhysX. |
| `servos.time` | Simulation time in seconds since Play. Usually slower than wall time. |
| `servos.schedule` | List of `(sim time, {id: counts})` goals applied when their time is reached. |
| `servos.axes` | Axis definitions (`name`, `min`, `max`, `home`, `rest`, joint) from the profile. |

Timeline behaviour:

- **Play** starts stepping the servo models. On the first step of a run the
  profile is re-read if `rig.toml` or `calibration.toml` changed, so a
  recalibration applies to the next Play. After `ballbal config set <other>`
  run `sim.py arm`.
- **Pause** freezes models and physics.
- **Stop** resets the models and the platform to the authored stage pose and
  clears `servos.schedule`.

## Home pose after recalibration

When the profile's home no longer matches the pose stored in the scene, arming
prints:

```text
note: the stage pose is not the profile's home (counts off: {...}); run tools/simulation/set_home_pose.py ...
```

Run `python tools/simulation/set_home_pose.py`. It settles the simulated
platform at home, writes that pose into `models/usd/scene.usda` and saves the
file. Other unsaved changes to that stage in Isaac Sim are saved with it.

## Compare with the real rig

`compare_real.py` sends every leg (home → min → max → home) to the rig and the
simulation at the same moment, records both and prints where each axis ended up
and how long it took.

1. Isaac Sim is running with the Python Server. The script re-arms the servos
   and restarts the timeline itself.
2. The rig is powered, connected and set up for the active profile.
3. **Take the ball off the plate.**
4. Run:

   ```bash
   python tools/simulation/compare_real.py
   ```

   Options: `--speed` (counts/s, default 1000; the supply browns out when all
   three axes accelerate at full speed) and `--hold` (seconds per leg).

Do not stop the timeline or re-arm while the comparison runs. The real servos
are energised and the script waits for the simulation.

Raw recordings are written to `runtime/measurements/servo_model/`.

## Without the Python Server

The Isaac-side files also run in Isaac Sim's **Window → Script Editor**. Open
`scene.usda`, then run:

```python
exec(open("/path/to/sim2real_ball_balancer/src/ballbal/simulation/isaac.py").read())
```

Press Play and use `servos` in the editor. The repository scripts need the
Python Server.

## Troubleshooting

| Symptom | Cause and fix |
| --- | --- |
| `no Isaac Sim Python Server on 127.0.0.1:8226` | Isaac Sim is not running, or was launched without `--enable isaacsim.code_editor.python_server`. The extension can also be switched on under Window → Extensions → Python Server. |
| `... open with unsaved changes` | Another scene file with unsaved edits is open. Save or close it in Isaac Sim; the scripts never discard it. The unsaved start-up stage of a fresh Isaac Sim is replaced without asking. |
| `simulation package not found` | The driver was run outside the repository. Use `sim.py start`, or open `scene.usda` from the checkout. |
| `servos.positions()` fails | The timeline is not playing. `sim.py play`. |
| Wrong limits or profile | `ballbal config show`, then `sim.py arm`. |
| Stage-pose note when arming | Home changed since the scene was saved. Run `set_home_pose.py`. |
| Ball marker in the wrong direction | Check the bearing with `ballbal balance --check-tilt`; the marker uses the same bearing. |
| `ball_sync.py --rig`: port in use | Another `ballbal` process holds the servo bus. Stop it or run without `--rig`. |
