# Runtime data

This directory contains generated, machine-local artifacts and is ignored by
Git except for this explanation.

- `logs/`: command, calibration, jog and balance logs
- `measurements/`: raw servo-identification and sim-vs-real recordings
- `cache/`: disposable generated state
- `state/active-profile`: local default selected by `ballbal config set`
- `state/active-camera`: stable camera path selected by `ballbal camera set`

Durable platform calibration is stored in `profiles/<name>/`; fitted simulation
parameters used at runtime are stored in `src/ballbal/simulation/params.json`.
The full reviewable fit is generated as `models/servo/fit.json`.
