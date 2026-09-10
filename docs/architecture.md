# Architecture

The physical platform profile is the boundary between hardware control and the
simulation:

```text
profiles/<name>/rig.toml -----------+
                                     +--> ballbal control --> real servos
profiles/<name>/calibration.toml ---+
                                     +--> ballbal/simulation/profile.py --> Isaac Sim
ballbal/simulation/params.json ------+
```

The profile separates intent from measurement:

- `rig.toml`: bus settings, safety thresholds, axis IDs and shared home/max offsets;
- `calibration.toml`: one measured `min_position` per axis;
- `camera.json`: measured image geometry and colour calibration.

`profiles/platform-v2` is a hardware-specific snapshot from the repository
author's platform. It is retained only as reference data for the recorded
sim-to-real comparison; it is not a reusable default. A different physical rig
must use a new profile copied from `profiles/example`.

`runtime/state/active-profile` contains the machine-local default chosen with
`ballbal config set`. It is ignored by Git. Real control and simulation resolve
their profile through this same selection.

The simulation adds only properties that do not exist on the real rig, such as
the CAD contact angle, simulated joint mapping and identified servo dynamics.

Generated operational data is deliberately one-way: commands may write logs or
raw measurements to `runtime/`, but neither control nor simulation treats those
files as configuration.
