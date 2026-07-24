# Initial development-bag observations

Date: 2026-07-24
Platform: Intel i7-1360P, ROS 2 Humble, x86-64

These are Phase 0 observations of the untouched matching behavior with added
instrumentation. Recorded localization is regression/pseudo-ground-truth, not
independent survey-grade truth.

## Completed nominal replay

`mine_nav1_r1` completed at 1x with an unperturbed synthetic initial pose
derived from recorded map odometry.

| Metric | Result |
|---|---:|
| Scan diagnostics | 525 |
| Baseline accepted | 493 (93.9%) |
| Decisions over 80 ms | 296 |
| Decision latency p50 / p95 / max | 89.328 / 182.909 / 361.223 ms |
| Input age p95 / max | 1050.450 / 1133.293 ms |
| Reference reconstruction translation error p95 / max | 0.00000108 / 0.00000229 m |
| Localizer translation error p95 / max | 10.924 / 11.676 m |

Reference reconstruction agrees numerically with recorded `/odometry_map`.
The large localizer error and deadline-miss count are baseline failures, not
acceptance results. They reinforce the plan's decision gates: optimizer
convergence alone cannot validate a correction, and callback work must not
permit message age to grow without bound.

The bag's recorded `/initialpose` differs from the reconstructed reference by
10.196 m and 135.152 degrees at its timestamp. This confirms that recorded
initial poses must be treated as priors/test inputs, not ground truth.

## Incomplete nominal replays

`mine_nav2_r1` did not terminate within the manual observation window and was
interrupted. The initial harness used reliable replay without an external wall
timeout, so slow or pathological baseline registration could stall the suite.
`mine_nav3_r1` and `mine_nav4_r1` were not started after that behavior was
observed.

The runner now derives a configurable wall timeout from each bag's recorded
duration, requests graceful shutdown, and records an explicit failed run if the
timeout is exceeded. A complete four-bag baseline remains required before
Phase 0 can be declared complete.

## Reproduction

```bash
ros2 run ndt_localization run_development_baseline.py \
  --bags-root /home/spotbot/Workspace/spot_nav_ws \
  --output-dir /tmp/ndt_development_baseline \
  --rate 1.0
```
