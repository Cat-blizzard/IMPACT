# GPU development checkpoint, 2026-10-08

The latest source has passed the GPU smoke, P4, Stage A and three independent
paired development groups. The formal matrix is the next acceptance step;
its results are not established by these development runs. Earlier incomplete
states below are historical checkpoints. All paths are relative to the repository.

## Starting snapshot

HEAD: `356d3abfab8bcb4725874107d3295b8108424032`, with eight existing modified
files preserved. Runtime source SHA256 before this checkpoint's changes:
`6339f8e596aeedacab8435120c28747e78218760aa702875af6e2f5c640567b2`.
Only WSL `zyc111`, Linux user `ld666`, was used. GPU: NVIDIA RTX PRO 6000
Blackwell Workstation Edition, driver 596.49, WSL D3D12 hardware rendering.

That snapshot passed 415 tests, the live ROS authorization contract, full
14-package build, GPU smoke, P4, and Stage A. These are evidence for that
snapshot, not acceptance of subsequent source changes.

| Run | Recorded conclusion |
| --- | --- |
| `experiments/results/current_validation_20261008/smoke/normal-baseline-s1000-20261008T014720-6ef7e283` | Smoke PASS, no arm, readable bag and cleanup |
| `experiments/results/external_nav/current_validation_20261008_run01` | P4 PASS, GPS off, ExternalNav flight, LAND/disarm; ATE RMS 0.005909 m |
| `experiments/results/current_validation_20261008/stage-a/normal-baseline-s1000-20261008T020029-d11ed45a` | Stage A PASS, map content verified; flight authorization expiry audit INCOMPLETE, supported by separate installed-runtime contract |
| `experiments/results/current_validation_20261008/normal-low-speed/normal-recovery-s1000-20261008T130203-57d10a67` | One normal task PASS; ATE RMS 0.067380 m, truth goal error 0.348646 m |
| `experiments/results/current_validation_20261008/recoverable-dev/recoverable-recovery-s1000-20261008T072248-8b0fe592` | Task/truth PASS; recovery mechanism NOT_DEMONSTRATED |
| `experiments/results/current_validation_20261008/unrecoverable/unrecoverable-recovery-s1000-20261008T131448-8d8962bd` | Overall FAIL despite mission GOAL_REACHED/PASS; truth goal error 79.902556 m, ATE RMS 74.991595 m, 634 integrity violations; DataFlash termination and cleanup PASS |

Do not describe the Stage A flight as having demonstrated physical stopping
or lease expiry. Do not count the unrecoverable task as a success.

## Diagnostic evidence

`scripts/analyze_localization_health.py` reads completed ROS bags, including
before ACTIVE. It uses a fixed first-pose alignment, joins truth within 0.05 s
and geometry within 0.25 s, labels phases by recorded receipt order, and hashes
the bags and input records. Ground truth is used offline only. This analysis
is diagnostic, not acceptance evidence.

Report: `experiments/results/current_validation_20261008/phase-localization-health-diagnostic.json`.

The unrecoverable error already reaches 6.447 m in ASCEND, 8.812 m in HOVER,
and 9.042 m at the beginning of ACTIVE. This rules out an exclusively
mission-execution origin. Its preflight instantaneous minimum information
eigenvalue divided by information trace is about 0.008; normal is about 0.055.
FCU/LIO agreement is not independent localization evidence because FCU
ExternalNav consumes LIO.

The current recoverable end wall is within LiDAR range throughout the route.
Its entry-only metadata and comment do not describe the actual geometry.
`recovery-causal-audit.json` in the recoverable run explicitly records
`NOT_DEMONSTRATED`: no recovery forecast, step, post-step observation,
confirmation or correlated reauthorization.

Historical forced-trigger runs remain diagnostic failures. Position-forced
events and relaxed recovery criteria were reverted before this checkpoint.

## New development change

SITL mission health now consumes instantaneous `/localization/geometry`.
Before takeoff and through ASCEND/HOVER it requires at least 30 effective
points and a weakest-eigenvalue fraction of 0.02. The fraction is scale and
rotation invariant, so repeating the same weak constraints cannot pass it.
Missing, stale, frame-mismatched, asymmetric, nonfinite and invalid matrices
fail closed. ACTIVE rank loss remains the supervisor recovery responsibility;
invalid or missing observations still cause mission failure termination.

The 0.02 threshold is an explicit development policy, not a calibrated
accuracy guarantee. It needs current GPU and cross-seed validation. It does
not solve estimator bias, recovery causality, or active trajectory gating.
The existing independent DataFlash landing/disarm audit remains necessary.

Validation after this change:

- Full test command: `bash scripts/impact_test.sh /home/ld666/projects/IMPACT /home/ld666/impact-build`, with the dedicated build and ROS log environment.
- 430 tests passed in 9.07 s.
- Live ROS contract PASS: messages, TF, matching IDs, frame rejection, revocation, expiry and old-session isolation.
- Full GPU build: 14 packages finished, 10.6 s.

New runtime source SHA256:
`41ca28cc4b784481ad6d65493b428765cea9305b5a7e814e96da29ceb3055ed7`.
Build manifest SHA256:
`89d4b83417fe084151b3b0b72913bf24eeb7d3d897f55b7de2e34d95e6fa6074`.

New runtime evidence:

- Smoke `experiments/results/health_gate_validation_20261008/smoke/normal-baseline-s1000-20261008T132838-bd7713b2`: PASS, continuous 30.003 s, never armed, bag readable, cleanup PASS.
- P4 `experiments/results/external_nav/health_gate_validation_20261008_run01`: PASS, mission 52.801 s, ATE RMS 0.005451 m, maximum error 0.029328 m, final error 0.004728 m. Two preflight VISP gaps exceeded 300 ms; no recorded FCU fault. These observations remain in the report.
- Stage A `experiments/results/health_gate_validation_20261008/stage-a/normal-baseline-s1000-20261008T133243-3307cc44`: task and truth PASS, map verified, independent DataFlash landing confirmed. Original gate remains REVIEW_REQUIRED because the command omitted the contract file. The source-bound installed-runtime contract was then generated at `experiments/results/health_gate_validation_20261008/authorization-runtime-contract.json`.
- `stage-a-supplemental-review.json` records PASS using the existing gate function, all original checks and the matching contract. It hashes the original REVIEW_REQUIRED gate and does not overwrite it. The flight expiry audit remains INCOMPLETE; no claim of physical stopping or observed flight lease expiry is made.

## Remaining gates

### Report correction and preflight refusal revalidation

The first health-gated unrecoverable run
`experiments/results/health_gate_validation_20261008/unrecoverable-preflight/unrecoverable-recovery-s1000-20261008T133656-a848ff35`
refused takeoff and remained disarmed, with cleanup PASS. Its original record
remains FAIL/incomplete because the runner required ACTIVE samples, and its
final health summary was computed after changing phase to FAILED. Both
reporting defects were corrected without modifying that historical record.

The corrected source SHA256 is
`7ddd3dde40bf5e9bc764682e358480e29b0a861e54e271edb4fe1dd8301d3ce7`.
It passed 437 tests in 9.09 s, the live ROS contract, and a full 14-package
build in 10.7 s. Build command output is at
`/tmp/impact-health-report-build-20261008.log`.

Revalidation run:
`experiments/results/health_gate_validation_20261008/unrecoverable-preflight-report-fixed/unrecoverable-recovery-s1000-20261008T134207-e86ea996`.

- Overall task FAIL, `completed_record=true`, launcher exit 0, cleanup PASS.
- Explicit preflight refusal in VERIFY_NAV: `geometry_translation_unobservable`.
- Final health remains unhealthy and retains the geometry rejection.
- Fresh FCU STABILIZE/disarmed; no ARM or TAKEOFF command in mission events.
- DataFlash read 107,374 messages with no ARMED event: `never_armed_confirmed=true`.
- Flight landing audit remains FAIL/`confirmed=false`, correctly: no flight cycle occurred. Never-armed evidence is a separate field and is not landing evidence.

This is a completed algorithmic refusal, not task success or an infrastructure
failure. Classification requires an explicit refusal reason, fresh FCU
disarmed evidence and a readable independent never-armed DataFlash record.
The change does not allow missing evaluator evidence to pass an actual flight.

No experiments remain live after this checkpoint. The earlier smoke/P4/Stage A
evidence belongs to `41ca28...`; those gates still need fresh binding/runs for
the latest `7ddd3d...` source before freeze. Inflight exception termination,
recoverable convergence and paired groups remain incomplete.

1. Revalidate the changed source with GPU smoke, P4 and Stage A.
2. Demonstrate unrecoverable preflight refusal and independent never-armed termination; separately verify inflight failure termination.
3. Repair recoverable measured observability and estimator behavior; demonstrate full recovery causality and independent truth success without weakened criteria.
4. Complete seed 1000/1001/1002 paired baseline/recovery development groups with matching source, model, world and common configuration.
5. Finish normal execution diagnostics, freeze only after all required gates, then run and monitor the new 120-task matrix to audited completion.

Failures, interventions, infrastructure retries and algorithm results must
remain separate. No commit or push was made at this checkpoint.

## Active localization protection and bounded-geometry development

The supervisor now consumes instantaneous geometry independently of the
memory matrix. It rejects/revokes mission splines when translation is weak,
permits only a bounded 8 s recovery window, and fail-closes persistent loss.
Baseline/conservative/hard-gate arms terminate on the same measured health
loss. Reaching the estimated goal cannot complete an unapproved trajectory.
Recovery snapshots retain the measured weak direction. The unsupported
fixed preference for right-lateral recovery was removed; forecast cost ranks
the feasible actions.

Source `9be2b58939a2c17829615f2ac321207685da88ca6563964f16e96510881ed1ce`
passed 440 tests, ROS contract and 14-package build, then:

- Smoke `experiments/results/active_health_validation_20261008/smoke/normal-baseline-s1000-20261008T135718-4b6e46c1`: PASS, bag and cleanup PASS.
- P4 `experiments/results/external_nav/active_health_validation_20261008_run01`: PASS, mission 53.401 s, ATE RMS 0.006035 m, maximum 0.034032 m, final 0.004865 m. `first-failure.json` records cleanup `wait` exit 143; this is the normal launcher signal exit, retained as a script-reporting observation.
- Stage A `experiments/results/active_health_validation_20261008/stage-a/normal-baseline-s1000-20261008T140155-892128d3`: gate PASS with matching runtime contract explicitly supplied, truth goal distance 0.184114 m, ATE RMS 0.066875 m, zero integrity violations. Flight lease expiry/physical stopping remain unproven.

### Inflight data-loss probe

`scripts/probe_inflight_localization_loss.py` pauses only the selected
development run's FAST-LIO process, verifies its process group and identity,
and resumes it after 2 s. Diagnostic interventions are never paired or formal
acceptance samples.

The first probe run
`experiments/results/active_health_validation_20261008/inflight-loss/normal-recovery-s1000-20261008T140449-0b884dbc`
was injected after ACTIVE ended, during DESCEND. Its task PASS does not prove
ACTIVE fault handling. The original record and probe artifact remain intact.
The probe was corrected to require fresh telemetry and the current ACTIVE
mission phase immediately before injection.

Corrected probe source:
`3a2a56511851061439f7cca93a3570c96e52fafe502253c5c5987d93d5d55137`.
Run:
`experiments/results/active_health_validation_20261008/inflight-loss-active/normal-recovery-s1000-20261008T140854-c6f45ecc`.

- Pause 2.002608 s during ACTIVE; mission detects `extnav:source_stale` about 0.50 s after pause.
- Supervisor revokes `DISABLED_OR_STALE`; mission requests LAND and retains task failure.
- Overall FAIL, completed record true, launcher exit 0, cleanup PASS.
- 61 ACTIVE evaluation samples, no collisions, ATE RMS 0.060734 m, zero integrity violations.
- Independent DataFlash PASS: ARMED -> NOT_LANDED -> LAND_COMPLETE -> DISARMED; one confirmed flight cycle.

This proves the inflight stale-localization branch under a controlled process
pause. It does not prove geometric recovery or all failure branches, and uses
the normal world so a healthy takeoff is possible. Unrecoverable geometry
continues to require preflight refusal.

### Recoverable scene under development

The recoverable scene now proposes an entry cap at x=-18 m, a forward cap at
x=26 m, and an actual model LiDAR range of 22 m. This creates a bounded interval
of low longitudinal support rather than leaving the forward cap visible for
the whole route. Ideal first-return rays include static box occlusion through
`scripts/scenario_visibility.py`; measured FAST-LIO geometry remains
authoritative. No recovery acceptance is claimed for this scene yet.

Recoverable EKF variance protection is no longer disabled; its cross-odom
limit is restored from 50 m to the common 5 m. Mission tolerance is the common
0.45 m, unchanged independent truth tolerance. No forced position trigger or
relaxed information-gain criterion is used.

443 tests and the ROS contract pass after these changes. The new build and
recoverable diagnostic are the next checks; this later source still requires
fresh entry gates before freeze. Paired development and formal120 remain
prohibited until the complete recovery and truth requirements pass.

## Bounded-scene results and estimator diagnosis

The x=26 m forward-cap run at
`experiments/results/bounded_geometry_validation_20261008/recoverable/recoverable-recovery-s1000-20261008T141524-2d91d8a6`
passed independent task truth (goal error 0.414556 m, ATE RMS 0.052673 m),
but did not trigger recovery. Its mechanism remains NOT_DEMONSTRATED.

The x=27 m run at
`experiments/results/bounded_geometry_validation_20261008/recoverable-gap27/recoverable-recovery-s1000-20261008T142026-01c1fa7c`
used source `fede2f3b09fc3f3298c06c8d12421aa251268a9d8d28fa0f772028cd3078b12e`.
It completed as task FAIL with cleanup and independent DataFlash flight
termination PASS. Its recorded mechanism audit passes, including rejection
of the first observation's worsened information and confirmation of the
second observation's fixed-direction information gain. However, ATE RMS is
0.849471 m and final independent truth-goal error is 0.663283 m, above the
unchanged 0.45 m requirement. Recovery acceptance therefore fails.

The phase diagnostic in that result root shows healthy takeoff: maximum
ASCEND/HOVER errors 0.047168/0.047873 m. Error first exceeds 0.45 m at
sim 72.4 s with raw weak fraction 0.003890, yet LIO longitudinal variance
is only 3.62e-7 m2. Error exceeds 1 m at sim 74.0 s. Subsequent geometric
support and temporal information improve, but a roughly 1 m longitudinal
offset remains. New information does not establish correction of existing
map/pose bias. Single-frame geometry restoration also resets the continuous
loss watchdog, which needs sustained restoration evidence.

### Centered plane-fit candidate

The original FAST-LIO plane fit fixes the unnormalized plane intercept and
solves five neighbours with QR. The development replacement centers the
neighbours and uses the covariance's smallest eigenvector, rejecting line
support and isotropic noise. A compiled test calls the production header and
checks planes through the origin, translated planes, lines and invalid data.

Source `cfa692991fa95a13275339043e036f1aa83f94f986488926553fbe17ded96f70`
passed 444 tests, ROS contract and the full 14-package build. Diagnostic:
`experiments/results/centered_plane_validation_20261008/recoverable/recoverable-recovery-s1000-20261008T143042-39cb030d`.
It still fails: truth-goal error 0.577021 m, roughly 0.95 m localization bias,
recorded recovery mechanism PASS, independent flight termination and cleanup
PASS. Centering alone does not solve the flight failure.

A further candidate removes translation Jacobian sensitivity in directions
whose measured instantaneous information fraction is below 0.02. The original
geometry remains published; no position/truth trigger is introduced. It is
enabled only by the IMPACT launch parameter, with the default disabled for
other launch paths. This candidate still requires runtime validation and
cannot be described as accepted. Final GPU entry gates, normal cross-seed
checks and all-three-scenario paired development groups remain outstanding.

### Observability protection and recovery-selection candidates

Source `65bf576a57287790127f096189cdd80d3e751bdcab88f2932375dfc273053fcb`
passed 444 tests, ROS contract and full build. Run
`experiments/results/observable_translation_validation_20261008/recoverable/recoverable-recovery-s1000-20261008T143740-75bf564e`
reduces ACTIVE ATE RMS to 0.127204 m, but terminates after 8.059 s of continuous
measured rank loss. This is completed task FAIL, independent DataFlash landing
PASS and cleanup PASS. No task or recovery acceptance is claimed.

Reading the recorded information map at degradation shows known longitudinal
surfels near the entry cap. At the real 22 m range, a forward vertical step
reduces predicted longitudinal information from about 7.63 to 0.37; returning
toward the entry cap retains support. The existing 2.8 m radius is suitable
for nearby collision forecasting, but omits these known distant anchors.

Recovery selection now separately forecasts the executed short step's fixed
weak-direction information at the actual sensor range, tapering the final
0.5 m of range. It filters non-improving predictions before ranking costs.
Forecasts remain optimistic predictions, not proof of visibility or motion
authorization; final certification and measured observations still govern.
The backtrack target comes from a measured healthy odometry pose, with a
stronger 0.05 weak-fraction policy for anchors than the 0.02 health gate.

Two subsequent candidates terminate with independent landing and cleanup
PASS, with no recovery acceptance:

- `experiments/results/fixed_direction_recovery_validation_20261008/recoverable/recoverable-recovery-s1000-20261008T144837-d1e20cdd`, source `75c0a49bfe6bd667a989c2d96bab579ba3dbec79f1cd6651273cb38401119a61`: ATE RMS 0.222660 m; the weaker anchor is too close for a reliable replan/return.
- `experiments/results/strong_anchor_validation_20261008/recoverable/recoverable-recovery-s1000-20261008T145545-7cee5385`, source `35b3163e9784d7c0518f15b50484f74e14be9be9128410812f63ea486819e1f0`: ATE RMS 0.044745 m; completed backtracks improve raw information but fail the unchanged covariance and memory-gain requirements. Continuous/sustained loss protection terminates at 8.036 s.

Health restoration now needs 1 s of advancing geometry observation stamps;
a single healthy frame cannot reset the loss deadline. Recovery confirmation
requires newer post-step raw information and temporal information on the same
fixed direction, in addition to covariance reduction and margin improvement.
An arrival refinement reduces recovery target tolerance from 0.2 to 0.1 m
and waits a full configured 3 s memory window after arrival. Source
`09e3f079e1f8bedcdfba0cacc5197b367f0ddabaed1d9e755a22526b7823e954`
passes 457 tests, ROS contract and 14-package build. Its runtime diagnosis is
pending; the stricter criteria do not retroactively validate prior runs.

### Development gate coverage and normal diagnosis

The paired gate now requires 18 completed tasks: seeds 1000/1001/1002, each
with all three scenarios and baseline/recovery. Normal requires truth-task
PASS on both arms; recoverable requires recovery truth-task and causal PASS,
with a collision-free, independently terminated baseline; unrecoverable
requires explicit geometry refusal and independent never-armed evidence on
both arms. Directed fault probes are excluded. Duplicate completed samples
for one source/seed/arm are rejected rather than selecting a passing retry.

Offline reading of the previously passing normal seed 1000 bag shows 25
certified splines, approximately 2.2 s between replans, and ACTIVE command
speed median/p95/maximum 0.232272/0.273632/0.305669 m/s. The arbiter emits
matching certified commands; one mission request and no recovery cycle occur.
This supports the current planner time/velocity scaling for that snapshot.
Other development seeds and the final source still require fresh flights.

### Arrival, aperture and short-trajectory diagnostics

The full-memory-window candidate
`experiments/results/anchor_arrival_validation_20261008/recoverable/recoverable-recovery-s1000-20261008T145948-ad623a48`
completes as task FAIL with independent landing and cleanup PASS. ATE RMS is
0.062421 m. Backtracking restores instantaneous geometry for 1 s, but memory
information remains below the pre-loss reference, so recovery confirmation
is correctly rejected. Subsequent steps fail the 8 s geometry-loss deadline.

Ideal first-return ray analysis led to a new physical recoverable hypothesis:
a static elevated face at x=5.5 m, bottom z=3.6 m, width 1.2 m, height 0.5 m.
It lies above the certified flight/braking envelope. At nominal sensor pose
x=4.4 m, raising z from 2.32 to 2.57 m raises ideal weak information fraction
from below 0.02 to above 0.02. Scene schema is now 8. This is a new development
scene, not retroactive acceptance of the cap-only scene.

Forecasts now account for the actual sensor's vertical aperture and height
offset. Vertical/lateral steps keep the current horizontal/longitudinal base
instead of adding half a metre of implicit forward travel. Feasibility
forecasting evaluates this executed short path. Forecast visibility remains
approximate: attitude and dynamic occlusion are not modeled.

Elevated-anchor run
`experiments/results/elevated_anchor_validation_20261008/recoverable/recoverable-recovery-s1000-20261008T150738-b52543f8`
(source `afc69c8e4ba838ffc5ed724a3102853ca2ebcb1514425f30548f4f10081f56b6`)
still terminates as FAIL. Raw lidar contains 526 anchor-face points near
sim 69.5 s; the recorded map contains eight anchor surfels. Thus the structure
is genuinely rendered/sensed, but recovery movement does not reach its target.

Trajectory diagnosis identifies two execution defects. The trajectory server
continues outputting its endpoint hold after spline duration, but certification
used to reject an empty remaining domain. A bounded recovery-only endpoint
hold now rechecks current collision clearance, tracking error and covariance;
mission expiry is unchanged. Source
`34f75cb0d6e55da52f44431fd3090f32de352301f95e367492c75d0e8a8f713a`
passes 461 tests, ROS contract and full build. Its run
`experiments/results/terminal_hold_validation_20261008/recoverable/recoverable-recovery-s1000-20261008T151458-1ff0bae5`
also fails, with independently confirmed landing and cleanup.

Reading the actual certified spline shows requested z=2.253696 m versus
optimized endpoint z=2.099570 m. Initial polynomial sampling omitted its true
endpoint; the rebound optimizer also treated the endpoint as a soft objective.
Uniform initial sampling now includes the endpoint. The intermediate run
`experiments/results/planner_endpoint_validation_20261008/recoverable/recoverable-recovery-s1000-20261008T151932-607a7734`
still fails. The next candidate fixes the final control points for short
(<1 m) goals, retaining optimization of interior points and final certification.
It has passed 461 tests, ROS contract and full build. The fixed-endpoint run
`experiments/results/fixed_endpoint_validation_20261008/recoverable/recoverable-recovery-s1000-20261008T152431-baf8de19`
completes the step and observation window, but measured information worsens.
It remains FAIL with independent landing and cleanup PASS (ATE RMS 0.1035 m).

### Supported-anchor outcome and matching diagnosis

The subsequent schema-8 candidate enlarges the overhead anchor to centre
`[5.5, 0, 4.1]`, size `[0.2, 2.4, 1.0]`, preserving bottom z=3.6 m.
Source `94cc275641041e3a1fc1bc692cbf27194b609397c2c85d10dc5d3df5544b7c32`
passes 461 tests, ROS contract and a full 14-package build. Its run
`experiments/results/supported_anchor_validation_20261008/recoverable/recoverable-recovery-s1000-20261008T153244-ba7b6afb`
also remains FAIL, with independent landing and cleanup PASS. It is
collision-free; ATE RMS is 0.111017 m and the closest truth goal distance
is 7.567866 m, so this is not task success.

Degradation occurs at sim 70.871 s, the vertical recovery step finishes at
74.051 s, and a new observation is assessed at 77.064 s. On the fixed weak
direction, raw information falls from 18.7324 to 3.58491, memory information
from 1204.42 to 139.092, and variance increases from 1.70698e-5 to
1.44718e-4 m2. Recovery is correctly rejected; continuous geometry loss
triggers fail-closed termination after 8.002 s. Enlarging the physical face
did not resolve the discrepancy between raw visible returns and accepted
longitudinal constraints.

The optional per-point FAST-LIO matching diagnostics record the transformed
candidate, rejection stage, fitted normal/residual, actual neighbour covariance
eigenvalues/distance and accepted information weight. They describe the final
measurement evaluation before the final filter update, not a truth-aligned
oracle or acceptance signal. The explicit `IMPACT_MATCHING_DIAGNOSTICS=1`
switch enables the computationally expensive per-point eigensolve; it is off
in normal runs.

The instrumented run
`experiments/results/matching_diagnostic_20261008/recoverable-recovery-s1000-20261008T155448-e90d84d9`
uses source `64df1d12fd10325f94a3f1a654c96310ee05e99db60ca5de4f8731db94eab95f`.
It is FAIL with independent LAND_COMPLETE/DISARM and cleanup PASS; ATE RMS is
0.116971 m and final truth goal distance is 7.516778 m. The actual matching
diagnostic and offline alignment audit are retained in `matching-diagnostic.json`.
At sim 68.9 s, 70 sampled candidates aligned with the overhead face are
accepted with strong longitudinal normals. By 74.2 s, after the vertical
recovery action, only 14 candidates align with the physical face; seven fail
the plane fit and the accepted normals have longitudinal components 0.005 to
0.52. At 77.2 s no sampled candidate aligns with the face. Raw returns remain
abundant while registered clouds cease to align with the independent physical
face. This supports degraded map/pose correspondence during recovery, but does
not identify one estimator defect conclusively. The predicted recovery gain is
not validated by execution; confirmation correctly rejects it. The next step
is to inspect forecast inputs and recovery motion against these measured
correspondence changes before changing estimator acceptance. No thresholds or
acceptance criteria are relaxed.

### Healthy-scan map gate and bounded recovery candidate

FAST-LIO's incremental map and P10 temporal surfel map now skip scans unless
the measured translational information has weak fraction at least 0.02; P10
also requires geometry and registered-cloud stamps within 0.25 s. The P10
threshold defaults off for other launch users and is enabled by IMPACT only.
Compiled tests cover healthy, weak, stale and insufficient-support cases.

The map-gate run
`experiments/results/map_freeze_validation_20261008/recoverable-recovery-s1000-20261008T161629-dae10100`
is FAIL, with independent LAND_COMPLETE/DISARM and cleanup PASS; ATE RMS is
0.075064 m. A refined offline source-pose audit shows 39 to 42 downsampled
matching candidates from the physical anchor face during and after the 0.25 m
recovery rise, with weighted longitudinal information around 10.9 to 12.3.
At the observation check raw information is 14.2002 versus 19.3239 before
recovery, memory information 305.591 versus 1277.105, and variance
6.60529e-5 versus 1.60958e-5 m2. The scan map gate improves alignment evidence
but does not meet recovery acceptance.

A development run then tested a 0.50 m vertical offset, still inside the
existing certified altitude envelope. Geometry became healthy for over 1 s,
but no recovery step passed the fixed-direction raw + memory + covariance
confirmation gate; repeated planning ended with collision-free independent
landing and cleanup PASS. At 310.6 s no margin-feasible candidate remained.
ATE and final evaluation are in
`experiments/results/half_meter_recovery_20261008/recoverable-recovery-s1000-20261008T163124-d065e1cc`.
The source used a 420 s scenario budget, revealing that recovery could keep
replanning without a cycle-wide deadline after geometry health briefly
returned.

The next source candidate requires forecast information to predict at least
the same 1e-6 m2 covariance reduction required by measured confirmation, and
adds a 45 s total recovery budget ending in fail-closed termination. These
changes pass 464 tests and ROS contract; full build and runtime are pending.
The rebuilt budget-validation run
`experiments/results/recovery_budget_validation_20261008/recoverable-recovery-s1000-20261008T165253-6cb83730`
uses source `e2685a46a03506209de67dbb7c0a36c2f0680929f35138cf269d5e1e79fff285`.
Its forecast now records predicted covariance reductions for candidates, and
the run terminates independently at sim 87.815 s when measured geometry has
been continuously unobservable for 8.042 s. The actual information check
rejects the recovery (memory and covariance do not improve); ATE RMS is
0.083459 m, the final truth goal distance is 7.944912 m, and the record has
LAND_COMPLETE/DISARM and cleanup PASS. The 45 s cycle budget was not the
trigger in this run because the shorter continuous-loss safety deadline fired
first.

### Per-action baseline and information-gain ranking

Recovery now snapshots fixed-direction raw information, temporal information,
and covariance immediately before each recovery action. It keeps the original
degradation snapshot separately for audit. Candidates must predict at least
the measured 1e-6 m2 covariance gain; among feasible actions, ranking prefers
the largest predicted fixed-direction covariance reduction before using cost
as a tie-breaker. This follows evidence that a low-cost lateral step reduced
measured raw x information from 11.70 to 2.61 while a vertical option had a
stronger predicted gain.

Run
`experiments/results/step_baseline_validation_20261008/recoverable-recovery-s1000-20261008T175053-8d53dfdf`
completed with independent landing/cleanup PASS and task FAIL (ATE RMS
0.070408 m, final goal distance 7.610509 m). The per-action baseline avoids
comparing against a stale degradation snapshot, but the executed action still
worsened raw, memory, and covariance measures.

Run
`experiments/results/gain_rank_validation_20261008/recoverable-recovery-s1000-20261008T175957-36daf646`
produced two genuine action-correlated information improvements. For request
10, fixed-direction variance fell from 4.57539e-5 to 4.08798e-5 m2, raw
information rose from 15.6725 to 25.1132, and memory information rose from
441.597 to 494.645. Its current trajectory certificate passed the absolute
margin check, but an extra historical-delta rule rejected it because margin
was lower than at initial loss. That rule has been removed; current collision,
speed, covariance, and absolute reserve certification remain mandatory, as do
all three measured information improvements. A focused test verifies a
negative historical margin delta cannot negate valid information recovery,
while insufficient current margin remains rejected.

The latest built source is
`a73770663654e53cf6c886e7f077c4073be1b2109a6a7e024c8c187b6ed61a0a`.
Run
`experiments/results/information_release_validation_20261008/recoverable-recovery-s1000-20261008T181013-3017f006`
is FAIL: degradation at sim 70.4 s, backtrack and up-offset steps, then
fail-closed at 82.9 s after 8 s of continuous unobservability, before an action
meets the measured information gate. Independent LAND_COMPLETE/DISARM and
cleanup pass; ATE RMS is 0.045905 m and final truth goal distance 7.826242 m.
The new release rule has not yet been exercised in a successful flight, so
recoverable acceptance remains open.

### Normal low-speed seeds

Seeds 1001 and 1002 both passed on the current source hash
`a73770663654e53cf6c886e7f077c4073be1b2109a6a7e024c8c187b6ed61a0a`:

- `experiments/results/normal_seed_validation_20261008/normal-recovery-s1001-20261008T181534-7bbc03b8`: truth goal distance 0.354035 m, ATE RMS 0.060261 m, 544 authorized mission samples over 54.4 s, median/p95 speed 0.2218/0.2945 m/s, 25 certifications with no rejection or stall.
- `experiments/results/normal_seed_validation_20261008/normal-recovery-s1002-20261008T181829-a5522222`: truth goal distance 0.348395 m, ATE RMS 0.063327 m, 527 authorized mission samples over 52.7 s, median/p95 speed 0.2268/0.2870 m/s, 24 certifications with no rejection or stall.

Both have completed records and cleanup PASS. These results, together with
the retained seed 1000 pass, support normal low-speed execution over these
three seeds. Smoke, P4 and Stage A pass evidence above is bound to the earlier
source hash `64df1d12...`; it must be rerun after recoverable behavior is
stabilized and the final source is frozen.

### Rebuilt source gates

The prior full build manifest is source
`64df1d12fd10325f94a3f1a654c96310ee05e99db60ca5de4f8731db94eab95f` at HEAD
`356d3abfab8bcb4725874107d3295b8108424032`. With this exact install:

- GPU smoke PASS: `experiments/results/gpu_smoke_recovery_budget_20261008/normal-recovery-s1000-20261008T170408-9f44206e`.
- GPU P4 ExternalNav PASS: `experiments/results/external_nav/recovery-budget-20261008-171536-p4`, including GPS-off, EKF ExternalNav sources, square-and-return, landing and disarm.
- Stage A PASS: `experiments/results/stage_a_recovery_budget_20261008/normal-recovery-s1000-20261008T172135-2b43375b`, including actual goal reach, collision-free evaluation, authorization contract, map audit and termination evidence.

These gates validate the rebuilt source and normal target-navigation path. They
do not waive the recoverable causal-and-truth gate, which is still failing, so
the 18-task paired development gate and formal 120-task matrix remain blocked.

No paired development groups, formal tasks, commits or pushes have been
produced. These development
iterations do not satisfy recovery-plus-truth acceptance yet.

## Forward-anchor follow-up

The latest recoverable runtime source before this follow-up was
`1af662a7fa34b04367b35c61c98112ecebb837786eb7cbb0e246d92e9662fdf1`.
The generated scene incorrectly used an uncertified fixed release waypoint at
x=6.25 m; its trajectory had only about 0.036 m certified margin. That
waypoint was removed. For the next scene hypothesis, the recoverable end wall
was moved from x=27 m to x=26.5 m, keeping the same 22 m LiDAR range and every
health, information, margin and truth threshold unchanged. Ideal first-return
rays suggest a low-altitude weak interval near x=4.8-5.5 m and healthier
geometry farther forward, but this is only a design diagnostic.

The matching diagnostic run
`experiments/results/recovery_matching_forward_diagnostic_20261008/recoverable-recovery-s1000-20261008T193526-075aba42`
used source `51f7a03d0812dcca61f8e840f116425f2ffc29adfa7cb54f0c89c0c3bac58291`.
It failed closed at 115.78 s simulated time after one recovery step and a new
observation did not improve information. The matching PointCloud2 records
showed no accepted longitudinal matches to the x=26.5 m end wall while the
vehicle remained near the recovery area; accepted matches there were mainly
from the overhead anchor, generally a few dozen per scan. This supports a
range/support limitation for the far cap, not an estimator bug conclusion.
The run has independent LAND_COMPLETE/DISARM and cleanup PASS, zero collision
events, and truth-goal error about 7.58 m. It is a failure, not recovery
acceptance.

The recoverable follow-up source is currently
`51f7a03d0812dcca61f8e840f116425f2ffc29adfa7cb54f0c89c0c3bac58291` and its
full 14-package GPU build and 467-test suite passed. The installed runtime
build manifest is bound to this source. The corresponding end-wall run
`experiments/results/recovery_forward_anchor_validation_20261008/recoverable-recovery-s1000-20261008T193125-f036abb6`
also failed: one authorized recovery action completed, but its fixed-direction
information and margin worsened, so no mission trajectory was reauthorized.
Independent landing/disarm and cleanup passed. The causal audit records
`NOT_DEMONSTRATED`; there are still no accepted paired groups or formal matrix
tasks. Smoke, P4 and Stage A need fresh runs for this current source before
freeze.

For the same source, normal GPU smoke passed at
`experiments/results/recovery_forward_anchor_validation_20261008/smoke/normal-recovery-s1000-20261008T194328-c6444ab1`.
P4 passed at
`experiments/results/external_nav/recovery-forward-anchor-20261008`: GPS was
disabled, EKF sources were ExternalNav, square-and-return and landing/disarm
passed, ATE RMS was 0.004749 m, and no ExternalNav receive gap reached 300 ms.
Stage A flight and map audit passed at
`experiments/results/recovery_forward_anchor_validation_20261008/stage-a/normal-recovery-s1000-20261008T194841-60e9bad7`;
independent truth-goal error was 0.343049 m and collision count was zero.
Its original gate remains `REVIEW_REQUIRED` because the first supplemental
contract command inherited the source-tree `PYTHONPATH`. A separately generated
clean installed-runtime contract passed, and
`stage-a-supplemental-review.json` records PASS using the existing gate
function, matching source hash and original audit hashes. The flight lease
expiry audit remains incomplete; this review makes no physical stopping or
flight lease-expiry claim.

The current development state still has no fully successful recoverable
run, no three-seed paired groups and no formal 120-task matrix. Recoverable
still blocks freeze and delivery.

## Continuation and goal-arrival reserve

The wider elevated anchor produced the first online recovery confirmation in
`experiments/results/recoverable_wide_anchor_validation_20261008/recoverable-recovery-s1000-20261008T201056-81cb616d`.
Its mission reported GOAL_REACHED, but independent truth error was 0.489573 m,
above the unchanged 0.45 m gate. It remains an overall failure. The measured
fixed-direction information and covariance improved after an action; a
negative historical margin delta was not evidence that this improvement was
absent, because that margin compared different paths and critical directions.

The all-segment 0.24 m/s experiment
`experiments/results/recoverable_wide_anchor_slow_validation_20261008/recoverable-recovery-s1000-20261008T202514-cbf69de1`
also failed: truth error 0.471472 m, with no actual degradation/recovery cycle.
It had independent LAND_COMPLETE/DISARM and cleanup PASS. Its source was
`7b268ba33c409bb8c4b88b3b6fe9c2870cdc04a8a7db1b671e169372fd302547`.
Reducing the entire mission speed changed the degradation behavior without
solving early goal completion.

Mission arrival tolerance is now 0.30 m for every scenario and policy arm,
leaving execution/estimation headroom inside the independent 0.45 m truth
gate. Recovery/conservative speed is restored to 0.30 m/s. No independent
truth, health, information-gain, or absolute safety reserve was relaxed.

The first arrival-reserve sample, source
`f7f66208f66ed248b4977ec775769e4ce50f222b0e1bf09458d776be6d06c27a`,
`experiments/results/recoverable_goal_reserve_validation_20261008/recoverable-recovery-s1000-20261008T203459-ed574a5c`,
repeatedly recovered at the entry anchor and then lost geometry on forward
execution. It made eight confirmations but remained near x=4 m, finally
terminating on `extnav:source_stale`; task FAIL, truth error 7.843424 m,
zero collisions, independent landing/disarm and cleanup PASS. This exposes a
recovery retry loop, not task acceptance.

The supervisor now ranks feasible recovery actions by predicted fixed-direction
support that survives a 0.5 m forward continuation. Predictions only rank
actions; live trajectory certification and all measured information gates
remain necessary. The 45 s recovery budget is cumulative across confirmations
within the mission, preventing repeated local recovery from resetting the
budget indefinitely. The causal auditor now requires the same correlated
raw/memory information and covariance gain plus a current absolute margin
reserve; historical margin gain is retained as a diagnostic statistic.
Regression checks reject missing/insufficient current certificates even when
information gain exists, and exercise the shared recovery budget.

Current source/build:
`979e773fe2efe556838d98ea522ec56dc8f47434dabbe1539037abdb087df9ae`.
469 tests, live ROS authorization contract, and full 14-package GPU build PASS.

First complete recovery-plus-truth PASS:
`experiments/results/recoverable_continuation_validation_20261008/recoverable-recovery-s1000-20261008T204529-b4c19259`.

- Real geometry loss at sim 72.343 s, authorization revoked.
- Authorized bounded actions, fresh post-step observations; earlier attempts
  without adequate information gain remain recorded and rejected.
- Final successful step completed at 113.091 s, observation at 116.191 s,
  mission recertified and RECOVERY_CONFIRMED at 116.291 s.
- Goal reached, independent truth error 0.337269 m, collision count zero.
- DataFlash landing/disarm, completed record, cleanup, and causal audit PASS.

This is one development seed only. Seeds 1001/1002, current-source GPU entry
gates and the three paired development groups remain required before freeze.
No commit, push or formal matrix has been made. The supplemental audits of
earlier runs are new files, preserving their original audits and failures;
mechanism-only PASS does not convert a failed truth/task result into acceptance.

The subsequent estimator/map and recovery-action diagnostics found that the
vehicle was often executing only the midpoint of a forward probe. The
recoverable action selector therefore removed most of the useful forward
component while ranking the candidate. The final development change executes
the bounded candidate at its 75% sample (except the explicit backtrack/hover
actions), retaining its measured forward and vertical displacement. This does
not bypass certification; every resulting spline still undergoes the same
clearance, speed, covariance, information and authorization checks.

The intermediate variants remain recorded: thicker anchor, fine map voxel and
height-continuation-only runs either failed preflight, exhausted the cumulative
recovery budget, or were rejected by the independent clearance gate. The
current candidate change passes 471 tests, the live ROS contract and a full
14-package GPU build. A final-source seed 1002 run
`experiments/results/recoverable_forward_candidate_validation_20261008/recoverable-recovery-s1002-20261008T215844-c3bc6be3`
passes task and independent truth (0.223284 m), zero collisions, DataFlash
landing/disarm, cleanup, and causal audit. It records real degradation,
multiple recovery actions, same-direction information gain, reauthorization,
and a bounded elevated continuation to the forward support. This is not yet a
three-seed gate: final-source seeds 1000 and 1001 must be rerun.

Cross-seed validation has not passed. On the same source:

- Seed 1001 (`recoverable-recovery-s1001-20261008T204927-443804fa` in the
  continuation results directory) failed closed at sim 86.924 s after 12.046 s
  of unobservable geometry. Following the up-offset, raw fixed-direction
  information fell from 18.8815 to 1.5204 and variance rose from 2.38176e-5 to
  1.88278e-4 m2. Task FAIL, truth goal error 6.967391 m, zero collisions,
  independent landing/disarm and cleanup PASS.
- Seed 1002 (`recoverable-recovery-s1002-20261008T205354-6e61fd28`) also
  terminated on persistent unobservability after one earlier confirmation.
  The complete run remains a failure even though a local information chain
  existed. No cross-seed recovery acceptance or paired-development gate is
  claimed.

Offline bag inspection of seed 1001 shows information loss during the climb:
at sim 74.8 s estimated/truth x were 4.283/4.241 m and weak fraction 0.01952;
at 78.9 s x were 4.223/4.384 m and weak fraction 0.00226; at 81.9 s x were
4.285/4.787 m and weak fraction 0.00205. This points to loss of accepted
longitudinal support during the recovery action. A matching-instrumented
seed 1001 rerun is diagnostic only and cannot replace the recorded failure.

## Final source and paired-development acceptance

Frozen runtime source SHA256:
`d20e3ad2259442a698432432c69d0f63be711ed819d9df3cc83b981ab7ec782e`.
The dedicated `/home/ld666/impact-build/build-manifest.json` is a full
14-package build of this source. Its SHA256 is
`df9748b4d0783347e6e224c1bac968081e16b333c162a401254790d8f86c62d0`.
471 tests and the installed ROS authorization contract passed.

The final recovery action retains 75% of the finite candidate's forward and
vertical displacement, prioritizes a feasible up-offset over a backtrack that
would remain in the weak region, preserves the restored altitude, and uses a
bounded forward continuation before returning to the final goal. The elevated
anchor begins at x=5.7 m and z=4.1 m. The 0.30 m command-arrival tolerance and
independent 0.45 m truth gate apply consistently; no truth threshold was relaxed.
Fixed-direction temporal/raw information gains, covariance reduction, fresh
action-correlated observations and the current absolute certified reserve are
required for reauthorization. Cumulative recovery budgets prevent retry loops.

`experiments/results/final_paired_development_20261008` contains exactly one
eligible completed current-source record for each of the 18 paired tasks.
The earlier seed 1000 failure is retained separately and has an older source
hash, so it is excluded by the existing validator rather than overwritten.

| Seed | Normal baseline/recovery | Recoverable baseline/recovery | Unrecoverable baseline/recovery |
| --- | --- | --- | --- |
| 1000 | PASS / PASS | FAIL / PASS | FAIL / FAIL |
| 1001 | PASS / PASS | FAIL / PASS | FAIL / FAIL |
| 1002 | PASS / PASS | FAIL / PASS | FAIL / FAIL |

Recoverable baseline failures are completed geometry-loss flight outcomes
with independent landing/disarm evidence. Recovery records for all three
seeds have real degradation, authorization withdrawal, authorized recovery
actions, fresh information-improving observations, recertification, independent
truth success, causal-audit PASS, DataFlash landing/disarm and cleanup PASS.
Unrecoverable failures are explicit preflight
`geometry_translation_unobservable` refusals, unhealthy final health, fresh
disarmed FCU states and independent `never_armed_confirmed=true`. They are
completed algorithm outcomes, not successful navigation or infrastructure errors.
Matched world/model/configuration/calibration hashes and distinct session IDs
were checked for all pairs by `paired_development_evidence()`.

The complete current-source validation was then rerun:

- Marker: `experiments/results/pre_matrix_validation_20261008/server-validation.json`, PASS.
- Smoke: `pre_matrix_validation_20261008/smoke/normal-baseline-s1000-20261008T233205-50ef4c18`, PASS.
- P4: `external_nav/server-validation-pre_matrix_validation_20261008-3a502a7e`, PASS.
- Stage A: `pre_matrix_validation_20261008/stage-a/normal-baseline-s1000-20261008T233544-70157309`, PASS.
- Stage A truth-goal distance 0.166521 m, ATE RMS 0.055077 m, zero collision and integrity-violation samples, map content verified.
- Three paired development groups: PASS, 18 eligible completed tasks.

Two initial wrapper invocations failed before launching flights because shell
environment paths were absent. Explicit `IMPACT_BUILD_ROOT`, `ARDUPILOT_ROOT`
and `ARDUPILOT_GAZEBO_ROOT` restored the already existing installation. This
was not a GPU failure or an algorithm sample. The final complete validation
marker is bound to the real external binary hashes. Historical Frontier/P5
remains an independent FAIL diagnostic, outside the P4/Stage A gate.

The Stage A flight still does not prove physical stopping or flight lease
expiry. The matching installed-runtime authorization contract passes; the
flight expiry audit remains INCOMPLETE. Development acceptance does not imply
a statistical recovery benefit. The formal 3-scenario x 4-strategy x 10-seed
matrix must use a new results root and the frozen source/external fingerprints.

## Lossless storage preparation

Old, stopped, non-current-source experiments are being converted from raw
ROS SQLite bags to Zstandard archives to provide space for the matrix. Reports,
metadata, original failures and all bag content are retained. Each converted
bag has a `.db3.zst.archive.json` containing its original path, size, SHA256,
archive SHA256 and full round-trip verification. A raw bag is removed only
after the complete compressed stream has been read back and its original
SHA256 and byte count reproduced. Current-source development/validation bags
are excluded from this conversion.

To read or replay an archived historical bag, decompress its `.db3.zst` file
to the original `.db3` path and verify the original SHA256 against its sidecar.
Original audit hashes remain valid for the restored bytes. Metadata retains
the original bag filenames; direct replay therefore requires restoration first.
