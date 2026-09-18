# Stage B yaw command fix and controlled validation

Date: 2026-09-18

## Finding

The failed `unrecoverable/recovery/seed=1000` run at
`experiments/results/stage_b_cpu_75f4e96/unrecoverable-recovery-s1000/`
entered `BRAKE` while the current command was authorized and the measured
position was within the tracking threshold. The recorded command had a finite
position, velocity, acceleration and yaw, but `yaw_dot=NaN`. The arbiter's
finite-command gate correctly rejected that command; this was a trajectory
publisher defect, not evidence that the authorization gate should be relaxed.

## Minimal change

Commit `a0ee13e885a881c0485d49184eee9b08fbf4dd45`:

- guard zero, negative and non-finite trajectory callback intervals before
  calculating yaw rate;
- fall back to the last finite yaw and zero yaw rate if the filtered result is
  non-finite;
- expose the first fail-closed arbiter rejection reason in status telemetry;
- add a regression test for a non-finite command, retaining BRAKE behavior.

No flight, estimator, map, planner or authorization thresholds were changed.

## Verification

- Python/ROS regression: `373 passed`.
- Full bound build: `/var/tmp/impact-stageb-yawfix-committed.ibR2FK`.
- Runtime binding: `runtime-build-verification.json` reports Git
  `a0ee13e...`, source hash
  `76dcb18157ea8c8851b8315d4c0f397e7365bf52424b6b3bc26ad085dc234571`, and
  1667 installed files checked.
- Controlled CPU run:
  `experiments/results/stage_b_cpu_yawfix_a0ee13e/unrecoverable-recovery-s1000-20260918T101224-f488c47a`.
  It reached the goal, completed LAND and fresh FCU disarm confirmation, had
  zero collision samples and zero integrity-violation samples, and cleanup
  reported no residual process or log error.
- Rosbag inspection found 6722 `/impact/position_cmd` messages and zero
  non-finite position, velocity, acceleration, yaw or yaw-rate fields.
- The run-time graph audit found a single ExternalNav status publisher, a
  single adapter publisher for `odometry/out`, and the expected MAVROS
  subscription.

The renderer in this run was intentionally CPU software rendering (llvmpipe);
this is not GPU validation. Existing failed samples remain unchanged, and the
Stage B matrix and recovery-benefit claim remain open.
