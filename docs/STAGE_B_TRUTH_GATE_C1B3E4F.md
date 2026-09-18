# Stage B truth-gated validation: c1b3e4f

This note records the first validation group after correcting the false-positive
task gate. It is not a Stage B pass and does not replace the required independent
three-run group.

## Frozen runtime

- Git: `c1b3e4f69e1236877ac0e3b51a83c48534d5c908`
- Install: `/var/tmp/impact-stageb-truth-gate-c1b3e4f/install`
- Source tree SHA-256: `9454ac74a933873597d63962cb424ee8eb06c3da735bf560306969094f76a80e`
- Install manifest SHA-256: `87691a1055d6ff4fb62129fbd1716e495942ac48c572d11525dc5854b085da65`
- External assets: `/home/ld666/impact-deps/ardupilot` and `/home/ld666/impact-deps/ardupilot_gazebo`
- Renderer: CPU software `llvmpipe`; this is not GPU evidence.
- Configuration: `local_cpu`, normal simulation, full sensor configuration, seed 1000.

## No-arm smoke

Run directory:
`experiments/results/stage_b_truth_gate_c1b3e4f/normal-recovery-s1000-20260918T124607-daf4e6af`

The 30.003 s observation window passed. It recorded 206 odometry messages,
150 healthy ExternalNav status messages, 206 ExternalNav output messages, and
20 connected FCU states. The FCU remained `STABILIZE` and disarmed for the
whole window. ExternalNav status and output each had one adapter publisher;
MAVROS had the expected odometry subscription with endpoint identity evidence.
The mission node was absent, the bag was readable, and all owned process groups
exited without residuals, core dumps, or Gazebo segmentation faults.

## One controlled normal task

Run directory:
`experiments/results/stage_b_truth_gate_c1b3e4f/normal-baseline-s1000-20260918T124839-fb1f17f9`

- Task: `PASS`, `GOAL_REACHED`
- Independent truth endpoint distance: `0.187893 m`
- Frozen truth goal tolerance: `0.45 m`
- ATE RMS: `0.051585 m`
- Collision events: `0`
- DataFlash: `ARMED -> LAND_COMPLETE -> DISARMED`, termination confirmed
- Rosbag: readable, `rosbag_0.db3` 3,200,303,104 bytes
- Cleanup: all six owned groups recorded, no residuals

The run contains one MAVROS `Time jump detected` message after the task. It did
not coincide with an EKF/VisOdom health failure in this run and remains a
diagnostic item, not a justification to ignore time behavior.

## Corrected historical outcome

The prior `46762b4` recoverable run remains immutable. Its original runner said
`PASS`, but the independent truth endpoint was `71.510 m` from the frozen goal.
`acceptance-correction.json` reclassifies the task as `FAIL` while retaining
termination `PASS`. Its recovery audit is `NOT_DEMONSTRATED`: recovery was
triggered and some recovery trajectories were authorized, but no recovery step
completed, no post-step observation produced a mission reauthorization, and no
measured margin improvement was recorded.

## Current boundary

The truth gate and runtime binding are fixed and tested (`388` tests plus ROS
contract checks). This is one valid normal CPU sample only. Do not count it as a
three-run cold-start group, a formal Stage B result, GPU validation, or P5
exploration evidence. The recoverable scenario still needs a design-level review:
its entry-only longitudinal anchor is lost before the current short recovery
actions can restore longitudinal observability.
