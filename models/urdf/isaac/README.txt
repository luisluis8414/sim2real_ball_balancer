Ball balancer: corrected URDF import package

Import ball_balancer.urdf with mm_stl beside it. Units: metres, kg, radians.
Set FIX BASE. Do not merge fixed links or change link frames before restoring joints.
Eight links, seven joints. Meshes remain in millimetres with scale 0.001.
Measured masses: lower 9.08 g; upper including rod/head 7.38 g; platform 151.46 g.
Inertias integrated from closed STL surfaces using uniform effective density.
Actual infill and mixed-material distributions are not known.
Base inertia uses a placeholder mass of 1 kg; this is a fixed-base model only.

Servo limits are -20..100 degrees about the exported pose (Fusion -145 degrees). Full hardware
mapping follows the previously verified matching axes. Elbow limits: -24..165 degrees; source elbow zero is 0 degrees. Effort 0.5 Nm and velocity
0.5 rad/s are provisional test caps, not validated servo specifications.
Only servo_1/2/3 should receive drives. Elbows and spherical joints are passive.
Before meaningful dynamics: replace ball_1 with a spherical joint and add ball_2
and ball_3 using joint_restore.json. Configure closed-loop constraints according
to your Isaac Sim version. Do not simulate the open chain as the finished robot.
Base collision removed. Other collision meshes currently equal visual meshes. Review collision approximations
and disable adjacent-link collisions as appropriate; inspect platform contact.

Verified offline: mesh existence, closed surfaces, positive mass and inertia,
inertia triangle inequalities, tree connectivity, zero-pose visual transforms,
normalised axes. Not yet imported or dynamically tested in Isaac Sim.
Original raw export and Fusion design were not modified.
