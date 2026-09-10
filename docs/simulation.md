# Platform simulation

Open [`models/usd/scene.usda`](../models/usd/scene.usda) in Isaac Sim 6.0, then run
[`src/ballbal/simulation/isaac.py`](../src/ballbal/simulation/isaac.py) in the Script Editor. The
driver loads the profile selected by `ballbal config set`, applies its calibrated
limits and home/rest reference, and advances the identified STS3215 model every
physics step.

Run `ballbal config set <name>` before launching Isaac Sim to use another profile.

The included `platform-v2` profile and servo measurements come from the
repository author's physical hardware. They are kept for reference and for the
documented sim-to-real comparison, not as calibration for another platform.

Useful scripts:

| Script | Purpose |
| --- | --- |
| [`src/ballbal/simulation/isaac.py`](../src/ballbal/simulation/isaac.py) | arm the simulated servos |
| [`tools/simulation/set_home_pose.py`](../tools/simulation/set_home_pose.py) | author the current calibrated home pose into the scene |
| [`tools/simulation/platform_sweep.py`](../tools/simulation/platform_sweep.py) | repeatedly command calibrated min/max goals |
| [`tools/simulation/compare_real.py`](../tools/simulation/compare_real.py) | command real and simulated platforms side by side |
| [`docs/servo-model.md`](servo-model.md) | identify and inspect real-servo dynamics |

Run the real/sim comparison from the repository environment while Isaac Sim's
Python Server extension listens on port 8226:

```bash
python tools/simulation/compare_real.py
```

Raw recordings are written to `runtime/measurements/servo_model/`. Runtime
parameters live in `src/ballbal/simulation/params.json`; the full reviewable fit
is generated as `models/servo/fit.json` by `tools/simulation/analyze.py`.
