# `platform-v2` — hardware-specific reference

This profile contains encoder calibration, camera calibration, motion settings
and identified behaviour from the repository author's physical ball-balancer.
It is committed so the recorded real-world runs and sim-to-real comparisons can
be reproduced and inspected.

It is **not** a default configuration for another platform. Encoder origins,
safe travel and camera geometry vary between builds. To operate different
hardware, copy `profiles/example` to a new named profile, select it with
`ballbal config set <name>`, and run that platform's own setup and calibration.
