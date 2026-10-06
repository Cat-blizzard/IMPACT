# GPU Stage B Matrix, 2026-10-06

## Completed Run

The frozen SIMULATED matrix completed all 120 independent tasks. Its monitor
finished at 2026-10-06 17:05:40 +08:00 and exited. The matrix batch, launcher,
SITL and matrix Gazebo processes were absent on the final process check.

- Source commit: `84a9b40280d0e4692d410c0311c580d5b8264a28`.
- Source SHA256: `b02ad949becc4c16575cf5629e8c472eaa21e16a1757c99553cd9913886e5134`.
- Protocol SHA256: `9478428765cb56b879e688180748a8e40853f40b0a5a3952990bb6a0badd6bce`.
- Profile: `server_gpu`; one serial flight slot.
- Matrix: 3 scenarios x 4 strategies x seeds 0-9.
- Current-source entry validation: GPU smoke, P4 ExternalNav and Stage A PASS.
- Development seeds 1000/1001/1002: completed PASS and causal audit PASS;
  seed 1002 required a retry after a preserved completed truth-goal failure.
- Legacy Frontier/P5 remains a separate failed diagnostic.

Evidence root:
`experiments/results/head_gpu_stage_b_formal_120_current`.
Task statistics use the first independently completed record for each
scenario/strategy/seed. A completed task failure is retained and never retried
to increase the success rate.

## Task Results

Each table cell is PASS / completed tasks. The total is 25 PASS and 95 FAIL.

| Scenario | Baseline | Conservative | Hard Gate | Recovery |
|---|---:|---:|---:|---:|
| normal | 10/10 | 0/10 | 8/10 | 0/10 |
| recoverable | 0/10 | 1/10 | 0/10 | 6/10 |
| unrecoverable | 0/10 | 0/10 | 0/10 | 0/10 |

The independent truth-goal gate explains several apparently successful
missions. In recoverable, baseline claimed completion in 9/10 tasks and
hard_gate in 10/10, but neither reached the true goal within the protocol's
tolerance. Recovery claimed completion in 8/10 and passed the true-goal gate
in 6/10. Across unrecoverable's 40 tasks, 14 claimed estimated completion and
none reached the true goal. Median unrecoverable ATE RMS was approximately
63.50 m; median final truth-goal distance was approximately 70.93 m.

Normal conservative had six task timeouts, two execution stalls and two
control-health failures. Normal recovery had eight task timeouts and two
control-health failures. The preserved telemetry shows much slower executed
motion than baseline/hard_gate despite frequent authorization. Investigation
must cover low-speed EGO replanning and actual controller execution as well as
repeated revocation and recovery selection.

## Recovery Evidence

The existing event-based causal audit demonstrated its full recorded chain in:

| Scenario | Recorded Chain | Chain And Task PASS |
|---|---:|---:|
| normal | 8/10 | 0/10 |
| recoverable | 9/10 | 6/10 |
| unrecoverable | 4/10 | 0/10 |

Recorded motion, a later observation and renewed mission authorization establish
execution of the mechanism. They do not establish useful or safe recovery.
Before/after certification uses each spline's own critical direction and
clearance. A larger alert limit or a different direction can improve the margin
while the unobservable longitudinal position still diverges. Unrecoverable
seed 3 provides a concrete example of recovery confirmations despite about
70 m of real displacement and task failure.

Recoverable recovery wins all six discordant paired seeds versus baseline and
hard_gate. The exploratory exact two-sided paired test gives p=0.03125;
adjusting the three within-scenario comparisons gives p=0.09375. This small
matrix therefore supports a result to investigate, not a broad significance
or safety claim. Normal recovery's severe regression must be resolved.

## Incomplete Attempts

There are 130 matching attempt records: 120 independently completed tasks and
10 incomplete attempts. The latter contain two interrupted attempts with stale
RUNNING metadata, three startup/runtime ERROR records, and five FAIL records
without accepted independent termination evidence. No corresponding process
remained active at completion.

During continuous supervision, three incomplete attempts triggered automatic
restarts: unrecoverable hard_gate seed 6, recovery seed 8 and hard_gate seed 9.
Their next attempts completed as task FAIL. The mission process reported
termination confirmed in all three original attempts, but the independent
DataFlash audit rejected the required termination sequence. These are audit
disagreements; they are not proof that all three failed to land physically.

An earlier hard_gate seed 2 attempt had both online and independent termination
unconfirmed. Its preserved full-flight recording confirms longitudinal drift
already during ASCEND/HOVER, substantial abnormal motion during DESCEND, and
grossly divergent FCU odometry. This is a substantive flight-health failure,
separate from same-timestamp event-order ambiguity.

Retries preserve all earlier evidence and cannot be presented as first-attempt
reliability. Conditioning statistics on completed records can also bias flight
reliability and duration estimates; report incomplete attempts alongside task
statistics.

## Evidence And Limits

Machine-readable results and diagnostics:

- `tasks.csv`: 120 independent task rows.
- `task-statistics.json`: task success and timeout rates.
- `summary.json`: all matching attempts, including retries and failures.
- `analysis/matrix-analysis.json`: metrics, failure modes, paired comparisons
  and evidence availability.
- `analysis/*-causal.json`: all 30 recovery task causal audits.
- `analysis/termination-unconfirmed-hard-gate-s2.json`: full-flight truth and
  FCU diagnostic from the preserved recording.
- `analysis/task-success-rates.png` and `.pdf`: Wilson-interval task plots.
- `supervision/matrix-completion.json`: protocol-bound completion record.
- `supervision/monitor-events.jsonl`: progress, retries and compression hashes.

Earlier supervision removed most raw rosbag and Gazebo recordings. JSON,
event/telemetry logs and DataFlash remain; a full raw replay of every completed
task is unavailable. Later recordings were retained using verified lossless
gzip compression. Restore each `.db3.gz` to its original `.db3` filename before
using the unchanged rosbag metadata.

Collision counts and most evaluation telemetry cover ACTIVE task segments.
Zero recorded active-segment collisions cannot establish whole-flight safety,
especially during ascent and emergency landing. Protection-level coverage is
projected onto the reported critical direction, not whole-mission coverage.
All results remain simulation evidence.

## Next Development Order

1. Fix navigation-health gating before flight and during termination. Replay
   hard_gate seed 2 and preserve FCU state, estimator status, odometry,
   ExternalNav status, commands and full-flight truth. Identify estimator
   inconsistency and unobservable-axis velocity growth using flight-available
   signals only. Acceptance: reject unsafe entry before ascent; no automated
   rearm after an in-flight health failure; every flown development task has
   fresh online termination evidence and a valid independent flight-log cycle.
   Test simultaneous LAND_COMPLETE/DISARMED records separately against the
   actual ArduPilot logging semantics without accepting missing landing evidence.

2. Diagnose normal's low-speed execution regression. Compare the same
   development worlds with baseline/conservative/hard_gate/recovery and record
   candidate/certified spline derivatives, replanning boundaries, arbiter
   decisions and emitted MAVROS commands. Separate stalls, slow controller
   progress and recovery excursions. Acceptance: conservative and recovery
   complete the healthy development cases within the frozen task deadline
   without weaker clearance, authorization or truth-goal gates.

3. Strengthen integrity and recovery validation. Check PL calibration on
   training-only data and audit innovations, weak-axis covariance and projected
   error. Compare information changes on a fixed reference trajectory/direction
   as well as each final spline's margin. Acceptance: a recovery confirmation
   requires fresh post-motion observations and demonstrated information or
   protection improvement; changing clearance alone cannot assert estimator
   recovery. Unrecoverable cases should stop or terminate with an explicit
   nonrecoverable diagnosis instead of estimated-goal success under divergence.

4. Validate the next revision on separate development seeds first. Rebuild and
   rerun GPU smoke, P4 and Stage A, then run at least three paired development
   seeds across the affected strategies/scenarios. Record first attempts and
   incomplete flights. Freeze a new source/protocol and use a new result root
   for the next formal matrix after the above gates pass. The completed matrix
   remains the preserved comparison, and test seeds 0-9 now serve as regression
   evidence rather than unseen evaluation data.

Keep GPU P4 as the runtime entry gate. Frontier/P5 and CPU diagnostics retain
their separate scopes. The immediate work is failure diagnosis and development
validation, followed by a new formal evaluation on an independently frozen
revision.
