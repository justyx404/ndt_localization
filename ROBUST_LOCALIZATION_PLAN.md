# Robust and Bounded-Latency Localization Plan

Status: Proposed  
Date: 2026-07-24  
Package: `ndt_localization`

## 1. Objective

Improve the map-based localization system so that:

1. initialization is robust to inaccurate position and yaw estimates;
2. incorrect registrations are rejected instead of changing `map -> odom_lidar`;
3. normal tracking has a bounded application-level compute budget;
4. FAST-LIO odometry remains available when map matching is rejected or times out;
5. loss of localization is detected explicitly and recovery is automatic;
6. behavior and performance are reproducibly verified using the recorded mine ROS 2 bags.

Correctness takes priority over always returning a map correction. When the system cannot establish a trustworthy match before its deadline, it must report rejection and continue with FAST-LIO prediction.

## 2. Scope and constraints

- ROS 2 Humble on x86-64.
- Development machine: Intel i7-1360P, 16 logical CPUs.
- No NVIDIA GPU is available on the current machine.
- LiDAR input is approximately 10 Hz.
- The map is the recorded/published mine point-cloud map.
- There is no office map, so functional validation will be entirely bag-driven.
- GPU-dependent global localization, such as the fastest 3D-BBS configuration, is not part of the initial implementation.
- A hard real-time guarantee cannot be certified on a general-purpose Linux system from algorithm settings alone. The implementation will enforce an application-level decision deadline and report measured worst-case behavior.

## 3. Recorded data

The workspace contains:

- 21 primary `mine_nav*_r*` bags;
- one additional `test_data/mine_nav3_r5` variant;
- approximately 153 GiB of bag data;
- approximately 39.6 minutes of recorded operation.

Every bag contains:

- `/cloud_registered_body`;
- `/odometry_lio`;
- `/global_map`;
- `/odometry_map`;
- `/tf`.

Six bags contain `/initialpose`:

- `mine_nav1_r1`;
- `mine_nav1_r2`;
- `mine_nav1_r3`;
- `mine_nav1_r5`;
- `mine_nav2_r1`;
- `mine_nav3_r1`.

The absence of `/initialpose` from the other bags does not prevent initialization testing because the reference map pose can be reconstructed from recorded transforms.

## 4. Reference-pose reconstruction

The recorded frame chain has been verified as:

```text
map ── recorded /tf ──> odom_lidar ── /odometry_lio ──> base_link
```

The corresponding reference pose at timestamp \(t\) is:

\[
T_{\text{map,base}}(t)
=
T_{\text{map,odom}}(t)
\cdot
T_{\text{odom,base}}(t)
\]

where:

- \(T_{\text{map,odom}}(t)\) comes from the recorded `map -> odom_lidar` transform;
- \(T_{\text{odom,base}}(t)\) comes from `/odometry_lio`;
- both transforms are interpolated or selected at a common timestamp.

The reconstructed pose will be cross-checked against:

1. recorded `/odometry_map`, whose frame relationship is `map -> base_link`;
2. the six bags that contain an actual `/initialpose`.

Recorded `map -> odom_lidar`, `/odometry_map`, and `/initialpose` are reference-only data. They must never be provided to the localization algorithm during a test run. They will be remapped under a reference namespace or consumed by an offline evaluator.

The reference trajectory was produced by the existing localization system, so it is regression/pseudo-ground-truth rather than independent survey-grade ground truth. It is adequate for comparing convergence basins, trajectory consistency, correction jumps, recovery behavior, and runtime. Any claimed absolute accuracy must retain this limitation.

## 5. Current implementation risks

The current implementation in [`src/localization.cpp`](src/localization.cpp) has several behaviors that must be addressed:

- `/initialpose` is accepted directly without registration verification.
- Initial pose covariance is ignored.
- Initial pose and scan processing use the latest odometry rather than odometry synchronized to their timestamps.
- An initial pose can arrive before valid odometry.
- Any `hasConverged()` result can update `map_to_odom_`.
- Numerical convergence is not sufficient evidence that the correct local minimum was found.
- The whole published map is used as the NDT target.
- Input size is not capped deterministically.
- The active [`lio_localization.yaml`](../spot_navigation/config/lio_localization.yaml) does not configure the newer NDT scan/map downsampling parameters, so they default to disabled.
- The scan subscription depth is 10, which permits stale work to accumulate.
- PCL NDT `align()` is not deadline-aware; `ndt_max_iter` bounds iteration count but not wall-clock duration.
- There is no explicit `LOST` or `RELOCALIZING` state.

## 6. Target architecture

```text
                    validated coarse pose
UNINITIALIZED/LOST ─────────────────────────> TRACKING
        │                  initializer             │
        │                                          │
        └<── timeout/rejection ────────────────────┤
                                                   │
                         repeated tracking failure │
                         or invalid correction ────┘
```

### 6.1 State model

- `UNINITIALIZED`: map and odometry may be available, but no validated global pose exists.
- `INITIALIZING`: a bounded coarse/global search is in progress.
- `TRACKING`: local scan-to-map matching may update `map -> odom_lidar`.
- `LOST`: map corrections are suspended; FAST-LIO prediction continues.
- `RELOCALIZING`: background global recovery is running without blocking odometry.

### 6.2 Tracking behavior

For each eligible scan:

1. obtain odometry interpolated at the scan timestamp;
2. select a bounded local map target;
3. deterministically filter and cap scan points;
4. run local registration within a fixed budget;
5. validate the result;
6. update `map -> odom_lidar` only when validation passes;
7. otherwise retain the last valid correction and publish a rejection reason.

### 6.3 Initialization behavior

`/initialpose` becomes a search prior, not an accepted transform:

1. validate frame and timestamp;
2. interpret covariance as the initial search region;
3. use IMU/FAST-LIO gravity to constrain roll and pitch;
4. evaluate multiple position/yaw hypotheses;
5. refine promising hypotheses locally;
6. reject ambiguous or weak solutions;
7. require agreement across consecutive scans before entering `TRACKING`.

## 7. Benchmark design

### 7.1 Dataset split

Development/tuning set:

- `mine_nav1_r1`;
- `mine_nav2_r1`;
- `mine_nav3_r1`;
- `mine_nav4_r1`.

Held-out validation:

- all remaining primary `mine_nav*_r*` bags.

Final stress case:

- `test_data/mine_nav3_r5`.

Parameters must not be tuned against held-out results.

### 7.2 Replay isolation

Topics provided to the localizer:

- `/global_map`;
- `/odometry_lio`;
- `/cloud_registered_body`;
- synthetic `/initialpose` when required.

Reference-only topics:

- recorded `/tf`;
- recorded `/odometry_map`;
- recorded `/initialpose`.

The test node's outputs will use a separate namespace. ROS simulated time will be used for repeatability.

### 7.3 Replay rates

- `1x`: nominal sensor rate and primary functional benchmark.
- `2x`: backlog and throughput test.
- `4x` or maximum sustainable rate: overload behavior.

Overload must cause deliberate scan dropping or timeout rejection, not increasing message age or the later application of stale transforms.

### 7.4 Initialization perturbations

At selected timestamps, the benchmark will reset the localizer and create synthetic priors from the reconstructed reference pose.

Provisional perturbation bins:

- translation: `0.5`, `1`, `2`, `5`, and `10` m;
- yaw: `5`, `15`, `30`, `90`, and `180` degrees;
- roll and pitch: constrained around the gravity estimate;
- combined translation/yaw perturbations;
- covariance values consistent with each perturbation range.

Tests must include:

- correct prior with realistic covariance;
- inaccurate prior;
- highly ambiguous prior;
- prior outside the local NDT convergence basin;
- forced loss followed by relocalization;
- partial and dropped scan sequences;
- timestamp delay and jitter;
- mismatched scan/map segments as false-acceptance tests.

## 8. Metrics

### 8.1 Initialization

- initialization success rate by translation/yaw bin;
- false-acceptance count;
- explicit rejection/timeout count;
- time to first validated pose;
- time to enter `TRACKING`;
- translation and rotation error against reconstructed reference;
- agreement across confirmation scans;
- score margin between the best and next-best hypothesis.

### 8.2 Tracking accuracy and safety

- translation and yaw error against recorded reference;
- relative trajectory error;
- maximum correction jump;
- number and duration of `LOST` intervals;
- successful recovery rate and recovery time;
- rejected registrations by reason;
- stale result count;
- invalid transform count;
- continuity of FAST-LIO fallback output.

### 8.3 Performance

Record per-stage and end-to-end measurements:

- point conversion/filtering time;
- local-map selection time;
- matcher time;
- validation time;
- total scan decision time;
- input-message age at decision time;
- scan points before/after filtering;
- target points/voxels;
- registration iterations;
- CPU utilization;
- memory usage;
- deadline miss/timeout count.

Report:

- p50;
- p90;
- p95;
- p99;
- p99.9;
- maximum observed value.

Average runtime alone is not an acceptance metric.

## 9. Provisional acceptance criteria

These values are initial engineering targets and will be finalized after measuring the untouched baseline.

### 9.1 Tracking

- Every scan selected for processing produces an accept/reject/timeout decision within an 80 ms application budget.
- No registration result older than the latest accepted processing generation is applied.
- No unvalidated result updates `map -> odom_lidar`.
- The scan queue does not grow under nominal 1x replay.
- At overload rates, old scans are dropped and output age remains bounded.
- FAST-LIO prediction remains available during rejection, timeout, and `LOST`.

### 9.2 Initialization

- Initialization returns a validated pose or explicit failure within a provisional 2 s budget.
- A failed/ambiguous initialization must not be accepted merely to meet the deadline.
- Zero known false-positive acceptances in the complete benchmark suite.
- Provisional successful-pose threshold: within 0.5 m and 5 degrees of the reconstructed reference after confirmation.
- Initialization success and failure must be observable through diagnostics.

### 9.3 Regression

- No meaningful degradation on the unperturbed mine replay set relative to the current trajectory.
- No new discontinuities in `/odometry_map`.
- All behavior changes are covered by deterministic tests or reproducible bag-replay scenarios.

## 10. Implementation phases

### Phase 0: Benchmark and instrumentation

Implement measurement before changing localization behavior.

Deliverables:

- replay launch/configuration;
- topic remapping and reference isolation;
- synchronized reference-pose reconstruction;
- synthetic initial-pose injector;
- per-scan diagnostic message/topic;
- CSV/JSON metrics output;
- baseline report for the four development bags.

Exit criterion:

- repeated runs produce consistent reference poses and metrics;
- reconstructed poses agree with `/odometry_map`;
- inference agrees with recorded `/initialpose` after timestamp alignment.

### Phase 1: Timestamp and state correctness

Implement:

- bounded odometry buffer;
- timestamp interpolation;
- frame/timestamp/covariance validation;
- explicit localization states;
- rejection reason codes;
- prevention of unvalidated transform updates;
- consecutive-scan confirmation after initialization.

Exit criterion:

- malformed, stale, premature, and ambiguous inputs cannot produce a map correction;
- existing unperturbed development bags still track.

### Phase 2: Bounded tracking workload

Implement:

- `KEEP_LAST(1)` or equivalent latest-only scan handling;
- dedicated registration worker;
- generation IDs to invalidate stale work;
- deterministic scan filtering and maximum point count;
- bounded local map tiles or submaps;
- fixed matcher iteration/work limits;
- deadline-aware cancellation or timeout behavior;
- FAST-LIO fallback.

Exit criterion:

- nominal replay meets the decision budget;
- accelerated replay does not create an increasing backlog;
- late results are discarded.

### Phase 3: Robust initialization and recovery

First implementation:

- covariance-aware multi-hypothesis search;
- coarse-to-fine NDT or comparable registration;
- gravity-constrained roll/pitch;
- hypothesis ranking and ambiguity rejection;
- local refinement;
- automatic transition to `LOST` and background relocalization.

CPU global-registration evaluation:

- KISS-Matcher or an equivalent robust global matcher;
- map tiling/candidate retrieval if whole-map matching is too expensive;
- local NDT/GICP refinement and multi-scan confirmation.

Exit criterion:

- initialization basin materially exceeds the current single-seed NDT basin;
- false acceptance remains zero on the test suite;
- initialization failure is explicit and bounded.

### Phase 4: Local matcher comparison

Define a common local matcher interface and compare:

- hardened PCL NDT;
- optimized/parallel NDT if dependency cost is acceptable;
- `small_gicp` GICP/VGICP.

Selection criteria:

- mine replay accuracy;
- convergence reliability;
- inlier and Hessian/localizability information;
- deadline misses;
- CPU utilization;
- maximum observed latency;
- dependency and maintenance cost.

The algorithm with the lowest average runtime will not automatically be selected.

### Phase 5: Held-out validation

Run:

- all held-out primary bags;
- initialization perturbation sweeps;
- forced loss/recovery;
- dropped scan and odometry scenarios;
- timestamp jitter;
- `1x`, `2x`, and overload replay;
- false-acceptance/mismatched segment tests;
- final `test_data/mine_nav3_r5` stress run.

Exit criterion:

- acceptance criteria are met or every exception is documented with evidence;
- final parameters are derived without retuning on the stress case.

## 11. Expected code and artifact changes

Likely package changes:

- refactor registration/state logic out of the ROS callback wrapper;
- add matcher and initializer interfaces;
- add localization state and diagnostic types;
- update [`include/localization.h`](include/localization.h);
- update [`src/localization.cpp`](src/localization.cpp);
- update [`config/localization.yaml`](config/localization.yaml);
- update [`launch/localization.launch.py`](launch/localization.launch.py);
- update the active [`lio_localization.yaml`](../spot_navigation/config/lio_localization.yaml);
- add unit/integration tests;
- add replay/benchmark tooling without modifying the source bags.

Generated benchmark artifacts should include:

- per-run CSV;
- aggregate Markdown report;
- latency distributions;
- initialization success heatmaps;
- failure/rejection summaries;
- exact command and configuration used for every run.

## 12. Decision gates

1. Do not change the matcher until the baseline harness is trustworthy.
2. Do not add a global-registration dependency until multi-hypothesis NDT is measured.
3. Do not claim hard real-time behavior from percentile measurements.
4. Do not treat recorded `/odometry_map` as independent absolute ground truth.
5. Do not accept a pose solely because an optimizer reports convergence.
6. Prefer explicit localization failure over a confident but incorrect map transform.

## 13. Relevant references

- [3D-BBS: Global Localization for 3D Point Cloud Scan Matching Using Branch-and-Bound Algorithm](https://arxiv.org/abs/2310.10023)
- [KISS-Matcher: Fast and Robust Point Cloud Registration Revisited](https://arxiv.org/abs/2409.15615)
- [Quatro++: Robust Global Registration Exploiting Ground Segmentation](https://arxiv.org/abs/2311.00928)
- [small_gicp: Efficient and Parallel Algorithms for Point Cloud Registration](https://joss.theoj.org/papers/10.21105/joss.06948)
- [X-ICP: Localizability-Aware LiDAR Registration](https://www.research-collection.ethz.ch/items/e286a52c-2afc-492f-8b00-a9ab295665be)
- [Autoware NDT Scan Matcher](https://autowarefoundation.github.io/autoware_core/pr-602/localization/autoware_ndt_scan_matcher/)
- [ROS 2 Real-Time Programming Background](https://design.ros2.org/articles/realtime_background.html)
