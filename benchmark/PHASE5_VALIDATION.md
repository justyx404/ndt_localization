# Phase 5 held-out validation

Date: 2026-07-29  
Platform: Intel i7-1360P, ROS 2 Humble, x86-64

Phase 5 validates the production localizer on held-out bags, artificial
initial-pose perturbations, input faults, recovery, tracking headroom, and
false-acceptance cases. No production parameter was retuned during this phase.

## Artificial initial poses

Recorded `/initialpose` was not used because it is absent or unsuitable in
several bags. Every run instead constructed an artificial prior from the
isolated `/reference/odometry_map` stream:

- the pose came from the nearest reference sample;
- the header stamp came from an actual `/odometry_lio` message accepted by
  the localizer;
- the reference-to-odometry timestamp gap was limited to 250 ms;
- requested translation/yaw perturbations and matching covariance were then
  applied;
- the bag's recorded `/initialpose`, when present, remained isolated under
  `/reference/initialpose`.

Stamping the pose from the reference stream itself was rejected as a harness
design because the recorded streams do not always have identical header
timestamps. Using an actual localizer odometry stamp removed that artifact
without exposing the node to reference topics.

## Safety hardening found by the held-out suite

The first held-out pass found an early interval in `mine_nav1_r3` where a
geometrically plausible but wrong corridor solution could be confirmed.
Phase 5 therefore adds three internal checks without adding ROS parameters:

- coarse and refined initialization candidates must remain inside a
  covariance-supported acceptance envelope around the prior;
- each confirmation is checked against the originally selected candidate,
  rather than allowing pairwise confirmation deltas to chain away from it;
- each confirmation pose must also remain inside the original prior's
  acceptance envelope at that scan timestamp.

The acceptance envelope is `2.5 * sigma`, with the validated 0.5 m/10 degree
minimum and 10 percent numerical slack, capped by the existing configured
search limits before slack.

An intermediate Phase 5 build retried a weak robust-search result on newer
scans during the same two-second attempt. Adversarial replay showed that this
could eventually find and accept an alias. The final implementation retains
the conservative Phase 3 rule: the first weak/invalid robust-search result
fails that attempt. This is intentionally an availability-for-safety choice.

The final checks add no diagnostic publisher, decision taxonomy, Python
runtime, YAML setting, or public parameter.

## Dataset split

The four `*_r1` bags remained development data. The primary held-out set is:

- `mine_nav1_r2` through `mine_nav1_r5`;
- `mine_nav2_r2` through `mine_nav2_r5`;
- `mine_nav3_r2` through `mine_nav3_r5`;
- `mine_nav4_r2` through `mine_nav4_r5`.

`mine_nav1_r3` used a prior at 20 seconds because its 10-second interval is
the explicit ambiguous-location negative case. All other current primary
bags used a prior after 10 seconds.

## Held-out 1x replay

All 16 bags localized from an artificial prior without changing production
parameters.

| Bag | Prior (s) | Outputs | Translation p95 / max (m) | Rotation p95 / max (deg) |
|---|---:|---:|---:|---:|
| `mine_nav1_r2` | 10 | 464 | 0.0325 / 0.2399 | 0.668 / 2.524 |
| `mine_nav1_r3` | 20 | 232 | 0.0236 / 0.0513 | 0.682 / 2.692 |
| `mine_nav1_r4` | 10 | 218 | 0.0228 / 0.0721 | 0.569 / 1.015 |
| `mine_nav1_r5` | 10 | 244 | 0.0155 / 0.0464 | 0.438 / 1.691 |
| `mine_nav2_r2` | 10 | 511 | 0.0401 / 0.1404 | 0.970 / 4.047 |
| `mine_nav2_r3` | 10 | 584 | 0.0579 / 0.2073 | 1.012 / 4.602 |
| `mine_nav2_r4` | 10 | 526 | 0.0315 / 0.1476 | 0.774 / 2.534 |
| `mine_nav2_r5` | 10 | 489 | 0.0356 / 0.1084 | 0.987 / 2.957 |
| `mine_nav3_r2` | 10 | 1,178 | 0.0306 / 0.1527 | 0.650 / 4.529 |
| `mine_nav3_r3` | 10 | 1,190 | 0.0298 / 0.1981 | 0.608 / 3.389 |
| `mine_nav3_r4` | 10 | 1,356 | 0.0262 / 0.1047 | 0.529 / 2.040 |
| `mine_nav3_r5` | 10 | 1,164 | 0.0277 / 0.1341 | 0.721 / 4.302 |
| `mine_nav4_r2` | 10 | 958 | 0.0189 / 0.1415 | 0.401 / 1.718 |
| `mine_nav4_r3` | 10 | 1,145 | 0.0452 / 0.1148 | 0.641 / 5.390 |
| `mine_nav4_r4` | 10 | 1,768 | 0.0398 / 0.0981 | 0.619 / 3.154 |
| `mine_nav4_r5` | 10 | 1,500 | 0.0414 / 0.1047 | 1.170 / 7.288 |

The current 16-bag set contains 13,527 localized outputs and 13,320
reference-scored samples. Worst translation p95/max is 0.0579/0.2399 m.
Worst rotation p95/max is 1.170/7.288 degrees. The two rotation maxima above
5 degrees are isolated tracking samples in `mine_nav4_r3` and
the current `mine_nav4_r5`; their translation errors remain about 0.1 m. Every first
validated pose is within 0.2399 m and 4.529 degrees, satisfying the
provisional successful-initialization criterion.

## Perturbation sweep

The retained 1x perturbation sweep used `mine_nav1_r2`. Translation and yaw
covariance were chosen so `2.5 * sigma` covered the requested perturbation.
Every successful result stayed within the provisional 0.5 m/5 degree
initialization threshold.

| Artificial prior | Outputs | Translation p95 / max (m) | Rotation p95 / max (deg) |
|---|---:|---:|---:|
| 0.5 m | 455 | 0.0316 / 0.3837 | 0.618 / 1.852 |
| 1 m | 448 | 0.0325 / 0.1418 | 0.668 / 1.977 |
| 2 m | 448 | 0.0325 / 0.1418 | 0.668 / 1.977 |
| 5 deg | 464 | 0.0366 / 0.2187 | 0.794 / 4.489 |
| 180 deg | 464 | 0.0329 / 0.0851 | 0.668 / 1.852 |
| 2 m / 30 deg | 451 | 0.0343 / 0.1109 | 0.668 / 1.852 |
| 5 m / 90 deg | 464 | 0.0312 / 0.1368 | 0.609 / 1.852 |
| 10 m / 180 deg | 452 | 0.0325 / 0.1124 | 0.684 / 1.852 |

All reported perturbation runs use 1x replay. The final binary was rerun for
nominal, 0.5 m, 5 m/90 degree, and 10 m/180 degree cases; the reported
final-edge results above come from those reruns.

## Input faults and false acceptance

All fault cases used the final binary. A temporary relay outside the source
and install trees deterministically dropped every fifth message or applied
alternating timestamp offsets.

| Scenario | Localized outputs | Translation p95 / max (m) | Rotation p95 / max (deg) |
|---|---:|---:|---:|
| 20% scan drop | 451 | 0.0392 / 0.0991 | 0.716 / 1.474 |
| 20% odometry drop | 371 | 0.0440 / 0.2386 | 1.175 / 2.581 |
| +/-10 ms scan and odometry jitter | 464 | 0.0332 / 0.2244 | 0.690 / 2.318 |
| 10 m/180 deg outside tight covariance | 0 | explicit failure | explicit failure |
| pose from a segment 35 s earlier | 0 | explicit failure | explicit failure |

The mismatched-segment case copied a reference pose from 35 seconds earlier,
stamped it with the current localizer odometry time, and assigned tight
covariance. The first robust search failed and the final fail-closed policy
published no transform. Together with the outside-covariance case, the final
suite has zero known false-positive acceptances.

## Recovery and tracking headroom

`/localization/trigger_relocalization` returned success after nominal
tracking. The run retained 448 localized outputs with translation error
0.0325/0.1418 m p95/max and rotation error 0.668/1.977 degrees p95/max.

After initialization completed at 1x, replay was increased to 2x to isolate
tracking headroom. The run retained 465 outputs with translation error
0.0558/0.1831 m p95/max and rotation error 1.269/4.874 degrees p95/max,
remaining inside the successful-pose accuracy threshold.

## Build and production audit

- The package builds cleanly with `colcon`.
- All 18 localization-core and three point-cloud utility cases pass.
- All production translation units retain explicit `-O3` optimization.
- Cppcheck reports no warning, performance, or portability finding. Its three
  `useStlAlgorithm` style suggestions are intentionally retained because the
  explicit point-selection and candidate-distinction loops are clearer.
- `git diff --check` passes.
- The installed runtime contains the C++ localization executable and
  `config/localization.yaml`; internal static libraries and headers are not
  installed.
- The installed runtime contains no replay Python, benchmark launch, replay
  QoS, or synthetic-pose file.

## Exit criteria

- All 16 current held-out primary bags localize from a valid artificial prior
  time:
  **pass**.
- Successful first poses remain within 0.5 m and 5 degrees: **pass**.
- Final outside-covariance and mismatched-segment cases have zero false
  acceptances: **pass**.
- Drop, jitter, recovery, and 2x tracking after 1x initialization retain
  bounded output and trajectory accuracy: **pass**.
- The distinct `test_data/mine_nav3_r5` artifact is absent: **documented
  dataset exception**.
- Production parameters were not retuned on the available `mine_nav3_r5`
  stress proxy: **pass**.

## Measurement limits

Recorded `/odometry_map` is regression pseudo-ground-truth reconstructed from
the isolated reference transform; it is not independent survey-grade ground
truth. Production diagnostic publishers were intentionally removed before
this phase, so the final replays measure output continuity and trajectory
error but do not recreate the earlier per-scan latency distributions. The
80 ms latest-only decision path remains covered by the Phase 2/3 evidence,
the unchanged deadline worker, and deterministic tests.

The plan names a distinct `test_data/mine_nav3_r5` stress artifact. No
`test_data` directory exists anywhere under the workspace. The available
root-level `mine_nav3_r5` is included in the primary held-out set; it was not
used for parameter tuning. The missing distinct artifact is the only planned
dataset that could not be executed.

Generated replay artifacts remained outside the repository and are not
production dependencies.
