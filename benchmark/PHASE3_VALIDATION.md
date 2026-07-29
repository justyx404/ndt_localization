# Phase 3 validation

Date: 2026-07-29
Platform: Intel i7-1360P, ROS 2 Humble, x86-64

Phase 3 is complete on the development set. The localizer now treats
`/initialpose` as a covariance-bounded search prior, evaluates deterministic
position/yaw hypotheses in a separate worker, refines and ranks distinct
solutions, and requires three consistent scans before publishing a new
correction. Tracking loss starts the same bounded search in `RELOCALIZING`
while FAST-LIO continues using the last validated correction.

The complete held-out perturbation and mismatched-map suite remains Phase 5
work. Recorded `/odometry_map` is regression/pseudo-ground-truth, not
independent survey-grade ground truth.

## Implementation

- The search envelope is `2.5 * covariance standard deviation`, clamped to
  1--10 m in translation and 15--180 degrees in yaw.
- Position and yaw come from the prior. Roll and pitch are replaced with the
  gravity-aligned FAST-LIO tilt at the prior timestamp.
- Thirteen deterministic XY seeds and five yaw seeds produce at most 65
  combined hypotheses.
- Coarse PCL NDT uses a 0.5 m map, 0.4 m scan, at most 2,000 scan points,
  2.0 m resolution, and eight iterations.
- The best three spatially distinct coarse results are refined with the
  Phase 2 local NDT settings and at most 4,000 scan points.
- Refined candidates are rejected above fitness 0.5. Distinct candidates
  within a 0.01 score margin are rejected as ambiguous.
- Initialization has a global 2 s watchdog. Its work runs independently of
  the 80 ms scan-decision worker, and a result completing after its generation
  or deadline cannot change localization state.
- Recovery uses a 5 m/90 degree envelope and retries after 500 ms following a
  failed attempt. The last validated correction remains publishable until a
  replacement has passed three confirmations.
- `/localization/trigger_relocalization` provides a controlled test hook for
  the same recovery path used automatically after repeated tracking failures.

KISS-Matcher and `small_gicp` were not available in the workspace or system
installation. The dependency-free CPU implementation therefore uses bounded
coarse-to-fine PCL NDT as the equivalent first Phase 3 global-registration
evaluation. A wider matcher comparison remains Phase 4.

## Deterministic tests

The package passes 18 C++ core tests and five Python benchmark-math tests.
Phase 3 adds coverage for:

- covariance-to-search-envelope bounds;
- gravity-constrained roll/pitch;
- deterministic combined position/yaw hypotheses;
- score-margin ambiguity rejection;
- retaining the last correction during relocalization;
- swapping to a new correction only after confirmation;
- retaining the fallback correction after failed recovery.

## Nominal development-bag replay

All four unperturbed development bags entered `TRACKING`. No scan decision
exceeded the 80 ms application deadline.

| Bag | Init search (ms) | Decision p95 / max (ms) | Translation p95 / max (m) | Rotation p95 / max (deg) |
|---|---:|---:|---:|---:|
| `mine_nav1_r1` | 376.427 | 49.132 / 77.708 | 0.018 / 0.062 | 0.411 / 1.292 |
| `mine_nav2_r1` | 332.390 | 50.984 / 78.868 | 0.041 / 0.117 | 1.033 / 3.363 |
| `mine_nav3_r1` | 357.357 | 48.456 / 77.700 | 0.033 / 0.077 | 0.644 / 2.076 |
| `mine_nav4_r1` | 324.806 | 47.813 / 76.400 | 0.033 / 0.287 | 0.905 / 5.031 |

The p95 translation range remains 1.8--4.1 cm, matching the Phase 2
regression range. The maximum values also match the prior bounded-workload
results to rounding. Initialization used 65 hypotheses and three refinement
candidates in each run.

## Convergence-basin comparison

`mine_nav1_r1` was initialized from a prior displaced by 10 m and 180 degrees
with covariance that permits the full 10 m/180 degree configured envelope.

| Initializer | Result | Best score | Search time (ms) | Published map odometry |
|---|---|---:|---:|---:|
| One prior seed | Explicit failure | 0.618342 | 67.931 | 0 |
| 65 combined hypotheses | Confirmed `TRACKING` | 0.062718 | 251.815 | 455 |

The single-seed candidate was rejected by the 0.5 fitness limit. The
multi-hypothesis search evaluated and converged all 65 coarse candidates,
refined three distinct candidates, and selected a solution with a 0.082315
margin over the next distinct result. After confirmation, translation error
against the recorded reference was 0.018 m p95 and 0.062 m maximum; rotation
error was 0.410 degrees p95 and 1.292 degrees maximum. The scan-decision
maximum was 78.117 ms with zero deadline overruns.

This is a measured expansion of the tested initialization basin, not a claim
that every continuous pose inside the envelope will converge. Phase 5 retains
the full perturbation sweep.

## False-acceptance check

The same 10 m/180 degree incorrect prior was replayed with tight covariance,
which limits the allowed search to 1 m/15 degrees. The best refined result had
fitness 0.736104 and was explicitly rejected in 212.234 ms:

- state returned to `UNINITIALIZED`;
- accepted scan decisions: 0;
- localizer `map -> odom_lidar` publications: 0;
- localizer `/odometry_map` publications: 0.

The focused Phase 3 development scenarios therefore produced zero known false
acceptances. Ambiguous-runner-up rejection is deterministic core-test
coverage; the full mismatched-map suite is intentionally reserved for Phase 5.

## Recovery

A controlled trigger forced a tracking node through
`TRACKING -> RELOCALIZING -> TRACKING`.

- The recovery search evaluated 65 hypotheses, refined three, and selected a
  result in 277.925 ms.
- Best fitness was 0.064894 with a 0.121738 distinct-candidate margin.
- Three confirmations completed 0.542 s after the recorded recovery-start
  timestamp.
- `correction_valid` remained true for every recovery state event.
- Four scored reference samples were published while the state was
  `RELOCALIZING`; the fallback correction was not suspended.
- The run published 454 localized odometry/TF samples with translation error
  of 0.018 m p95 and 0.062 m maximum.
- Scan-decision p95/maximum was 48.948/77.989 ms with zero overruns.

## Exit criteria

- Initialization basin materially exceeds the single-seed comparison:
  **pass**.
- Focused development scenarios have zero known false acceptances:
  **pass**.
- Initialization success or failure is explicit and below the 2 s watchdog:
  **pass**.
- Nominal trajectory and 80 ms tracking-decision behavior do not regress:
  **pass**.
- Recovery retains FAST-LIO output and confirms before replacing correction:
  **pass**.

## Reproduction

```bash
ros2 run ndt_localization run_development_baseline.py \
  --bags-root /home/spotbot/Workspace/spot_nav_ws \
  --output-dir /tmp/ndt_phase3_nominal_final \
  --rate 1.0 \
  --initial-pose-delay 10.0 \
  --reference-repeatability-runs 1 \
  --force

ros2 launch ndt_localization benchmark_replay.launch.py \
  bag_path:=/home/spotbot/Workspace/spot_nav_ws/mine_nav1_r1 \
  output_directory:=/tmp/ndt_phase3_10m_180deg_final \
  run_name:=mine_nav1_r1_10m_180deg \
  translation_x:=10.0 yaw_degrees:=180.0 \
  position_sigma:=10.0 yaw_sigma_degrees:=180.0

ros2 launch ndt_localization benchmark_replay.launch.py \
  bag_path:=/home/spotbot/Workspace/spot_nav_ws/mine_nav1_r1 \
  output_directory:=/tmp/ndt_phase3_outside_covariance_final \
  run_name:=mine_nav1_r1_outside_covariance \
  translation_x:=10.0 yaw_degrees:=180.0 \
  position_sigma:=0.25 yaw_sigma_degrees:=5.0

# The single-seed comparison used an otherwise-default parameter file with:
# /**:
#   ros__parameters:
#     localization:
#       initialization_max_hypotheses: 1

# Start the recovery replay:
ROS_DOMAIN_ID=139 ros2 launch ndt_localization \
  benchmark_replay.launch.py \
  bag_path:=/home/spotbot/Workspace/spot_nav_ws/mine_nav1_r1 \
  output_directory:=/tmp/ndt_phase3_recovery_final \
  run_name:=mine_nav1_r1_recovery

# After it enters TRACKING, call from another shell:
ROS_DOMAIN_ID=139 ros2 service call \
  /localization/trigger_relocalization std_srvs/srv/Trigger '{}'
```

The validated package-local configuration SHA-256 is
`ffc483b23410719d0699df2c851ae8b7ce5388ced39a34c59c5a34230e275449`.
Generated CSV/JSON artifacts remain outside the source tree in the `/tmp`
directories shown above.
