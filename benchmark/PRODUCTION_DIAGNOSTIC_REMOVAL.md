# Production diagnostic and decision-code removal

Date: 2026-07-29
Platform: Intel i7-1360P, ROS 2 Humble, x86-64

This cleanup removes the production diagnostic-message interface and the
flattened `DecisionCode` taxonomy. It does not change registration settings,
state transitions, validation thresholds, worker scheduling, or deadline
enforcement.

## Changes

- removed `DecisionCode` and its 55 values;
- removed the scan, state, initialization, and late-result diagnostic
  publishers and their metric-only data structures;
- removed the `diagnostic_msgs` build and runtime dependency;
- reduced odometry lookup outcomes to the five statuses that control retry and
  interpolation behavior: available, unavailable, too old, too new, and
  interpolation gap;
- represented simple validation outcomes as booleans;
- kept one node-local, three-value registration-input status because the
  registration worker must distinguish ready, invalid scan, and rejected
  preparation;
- simplified robust initialization to return success and the selected pose;
- removed stale state-machine accessors that were used only by diagnostics.

The six main production headers and implementations decreased from 3,832 to
2,804 lines. `src/localization.cpp` decreased from 2,027 to 1,411 lines.

No diagnostic strings are published. The relocalization service responses and
a small number of map/initial-pose log messages remain because they are
operator-facing service and logging interfaces, not diagnostic topics.

## Verification

- the package builds successfully with ROS 2 Humble;
- all 21 C++ test cases pass (23 entries in the generated test report);
- `cppcheck` reports no correctness, warning, performance, or portability
  findings after suppressing its known Eigen macro-parser errors;
- `ros2 launch spot_navigation lio_localization.launch.py --show-args`
  resolves the production launch;
- `ros2 pkg executables ndt_localization` exposes only
  `localization_node`;
- the installed package contains no Python executables, replay launch file, or
  replay YAML.

## Recorded-bag regression

The removed replay harness was recovered temporarily from commit `2588308`,
used against `mine_nav1_r1`, and removed from the install tree after each run.
The evaluator still compared `/odometry_map` and `map -> odom_lidar` directly;
its diagnostic-derived arrays were empty by design.

### Nominal prior

| Metric | Pre-removal refactor | Diagnostics removed |
|---|---:|---:|
| Localized odometry outputs | 454 | 454 |
| Evaluation samples | 397 | 397 |
| Translation p95 / max (m) | 0.017571 / 0.061592 | 0.017571 / 0.061592 |
| Rotation p95 / max (deg) | 0.410647 / 1.291926 | 0.410647 / 1.291926 |

Artifacts: `/tmp/ndt_diag_removal_nominal.1frUoD`

### 10 m / 180 degree prior

| Metric | Pre-removal refactor | Diagnostics removed |
|---|---:|---:|
| Localized odometry outputs | 455 | 455 |
| Evaluation samples | 398 | 398 |
| Translation p95 / max (m) | 0.017568 / 0.061592 | 0.017568 / 0.061592 |
| Rotation p95 / max (deg) | 0.409668 / 1.291926 | 0.409668 / 1.291926 |

Artifacts: `/tmp/ndt_diag_removal_10m_180.rbZR0r`

Both runs used the same initial-pose timestamp,
`1771008662.6837466`. The matching output counts and trajectory errors show
that removing diagnostic plumbing did not change localization behavior,
including convergence from the difficult prior.

Production no longer publishes per-scan latency, so the current replay cannot
independently reconstruct those timing distributions. The deadline worker,
generation invalidation, matcher deadlines, and their deterministic tests
remain in place. The last instrumented pre-removal runs observed maxima of
77.611 ms for the nominal prior and 78.013 ms for the difficult prior, both
inside the 80 ms application deadline.
