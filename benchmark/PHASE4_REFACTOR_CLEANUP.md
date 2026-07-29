# Post-Phase 4 localization refactor cleanup

Date: 2026-07-29
Platform: Intel i7-1360P, ROS 2 Humble, x86-64

This cleanup removes confirmed stale code and separates tracking registration
stages without changing the three-worker scheduling model, state transitions,
matcher settings, or deadline policy.

## Changes

- removed the tracking `fitness_score` diagnostic, which had no producer and
  was always `NaN`;
- removed redundant assignments, ignored timestamp-age outputs, an unreachable
  immediate-rejection branch, and the unused `OdometryBuffer::clear()` API;
- removed unused ROS/PCL build dependencies and implementation-only includes
  from the node header;
- introduced shared point-cloud utilities for voxel filtering, deterministic
  point capping, and radius-submap extraction;
- reused those utilities in tracking and robust initialization, eliminating
  the duplicated implementations and avoiding copies when filtering or
  capping is a no-op;
- split tracking into explicit input preparation, matcher execution,
  validation, and state-commit stages;
- reduced `processScanTask()` from 299 lines to 154 lines while preserving the
  registration, initialization, and deadline workers.

Three deterministic tests cover no-op cloud reuse, capped sample selection,
and radius-submap extraction. The existing 18 C++ core tests and 5 Python
benchmark tests remain unchanged.

Static analysis reports no correctness, warning, performance, or portability
findings. Its two remaining style suggestions prefer `std::transform` over
the direct indexed point-copy loops; the loops are retained because they make
the deterministic sampling and PCL insertion behavior explicit.

## Replay comparison

Both comparison runs use the same `mine_nav1_r1` initial-pose timestamp as the
Phase 4 interface baseline. Wall-clock runtime naturally varies between runs.

### Nominal prior

| Metric | Phase 4 interface | Refactored |
|---|---:|---:|
| Initialization decision | selected | selected |
| Initialization search (ms) | 366.701 | 369.306 |
| Best fitness | 0.062444 | 0.062444 |
| Translation p95 / max (m) | 0.017571 / 0.061592 | 0.017571 / 0.061592 |
| Rotation p95 / max (deg) | 0.410647 / 1.291926 | 0.410647 / 1.291926 |
| Scan decision p95 / max (ms) | 48.911 / 77.637 | 48.263 / 77.611 |
| Deadline overruns | 0 | 0 |
| Localized odometry outputs | 454 | 454 |

### 10 m / 180 degree prior

| Metric | Phase 4 interface | Refactored |
|---|---:|---:|
| Initialization decision | selected | selected |
| Initialization search (ms) | 262.512 | 259.843 |
| Best fitness / margin | 0.062718 / 0.082315 | 0.062718 / 0.082315 |
| Translation p95 / max (m) | 0.017568 / 0.061592 | 0.017568 / 0.061592 |
| Rotation p95 / max (deg) | 0.409668 / 1.291926 | 0.409668 / 1.291926 |
| Scan decision p95 / max (ms) | 49.058 / 77.597 | 48.425 / 78.013 |
| Deadline overruns | 0 | 0 |
| Localized odometry outputs | 455 | 455 |

Trajectory errors, initializer scores, ambiguity margin, and output counts are
identical. Both refactored runs remained within the 80 ms decision bound with
no watchdog rejection. Superseded late matcher results remained diagnostic
only and could not update the correction.

## Reproduction

```bash
ros2 launch ndt_localization benchmark_replay.launch.py \
  bag_path:=/home/spotbot/Workspace/spot_nav_ws/mine_nav1_r1 \
  output_directory:=/tmp/ndt_phase4_cleanup_nominal_v2 \
  run_name:=mine_nav1_r1_cleanup_nominal_v2

ros2 launch ndt_localization benchmark_replay.launch.py \
  bag_path:=/home/spotbot/Workspace/spot_nav_ws/mine_nav1_r1 \
  output_directory:=/tmp/ndt_phase4_cleanup_10m_180deg_v2 \
  run_name:=mine_nav1_r1_cleanup_10m_180deg_v2 \
  translation_x:=10.0 yaw_degrees:=180.0 \
  position_sigma:=10.0 yaw_sigma_degrees:=180.0
```

Generated artifacts remain outside the source tree in the `/tmp` directories
shown above.
