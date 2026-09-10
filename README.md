# Sim-to-real ball balancer

Tools to set up, calibrate, control and simulate the three-axis ball-balancer
platform.

## Quick start

Create the environment once from the repository root:

```bash
uv venv .venv
uv pip install --python .venv/bin/python -e ".[dev,vision]"
source .venv/bin/activate
```

After activation, the `ballbal` console command is on `PATH`; all examples
below assume the shell shows `(.venv)`.

Inspect the active profile without touching hardware:

```bash
ballbal config show
ballbal config validate
```

Select the overhead camera once in the same way:

```bash
ballbal camera set
# or directly: ballbal camera set 1
```

Camera commands then use the stored stable device path automatically.

For a new platform, copy `profiles/example/` to a named profile, make it the
local default, then run guided setup:

```bash
cp -r profiles/example profiles/my-platform
ballbal config set my-platform
ballbal setup
```

> **Do not use `profiles/platform-v2` to control another physical platform.**
> It contains calibration and motion values measured on the repository
> author's specific hardware. It remains in the repository only as a concrete
> reference and for reproducing sim-to-real comparisons. Every user must copy
> `profiles/example`, create their own named profile, and run setup/calibration.

Setup reads all lower references together after the complete platform has been
placed at its minimum pose by hand with torque off. The common geometry in
`rig.toml` derives `home = min + home_offset` and `max = min + max_offset`.

After setup, common control commands are:

```bash
ballbal status
ballbal home
ballbal jog
ballbal balance
```

The simulation needs Isaac Sim 6.0.0.1 in its own environment (see
[simulation](docs/simulation.md)). Launch it with its Python Server, then drive
it from the repository:

```bash
# terminal 1, Isaac Sim environment
isaacsim isaacsim.exp.full --enable isaacsim.code_editor.python_server
# terminal 2, repository environment
python tools/simulation/sim.py start
python tools/simulation/ball_sync.py
```

The simulation reads the same profile as real control.

## Repository map

| Path | Purpose |
| --- | --- |
| `src/ballbal/setup/` | first-time setup and minimum calibration |
| `src/ballbal/control/` | safe platform motion, tuning and balancing |
| `src/ballbal/hardware/` | serial bus and STS3215 implementation |
| `src/ballbal/vision/` | camera calibration and ball tracking |
| `src/ballbal/simulation/` | reusable simulation driver, profile adapter and servo model |
| `profiles/` | versioned platform configuration and durable calibration |
| `tools/simulation/` | identification and real/simulation comparison tools |
| `models/` | Fusion, STL, URDF, USD and fitted model artifacts |
| `runtime/` | ignored logs, raw measurements and caches |
| `docs/` | setup, control and architecture documentation |

## Configuration versus calibration

Each platform has one directory under `profiles/`:

```text
profiles/my-platform/
├── rig.toml           # safety, axis identity and fixed home/max offsets
├── calibration.toml   # generated min_position for each axis
└── camera.json        # generated camera calibration
```

The offsets and encoder values belong to that physical platform. Both real
control and simulation derive identical effective positions from its measured
minimum. Raw logs never become configuration; they go to `runtime/`.

See [setup](docs/setup.md), [control](docs/control.md),
[balancing](docs/balance.md), [simulation](docs/simulation.md), and
[architecture](docs/architecture.md).
