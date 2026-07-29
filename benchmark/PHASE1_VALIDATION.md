# Phase 1 validation

Date: 2026-07-29
Platform: Intel i7-1360P, ROS 2 Humble, x86-64

Phase 1 is complete. The localizer now synchronizes scans and initial-pose
priors against bounded odometry history, validates external inputs and
registration results, and withholds `map -> odom_lidar` until three
consecutive registration corrections agree.

## Safety behavior

- The explicit states are `UNINITIALIZED`, `INITIALIZING`, `TRACKING`,
  `LOST`, and `RELOCALIZING`.
- A valid initial pose is a prior only. It does not make a correction valid.
- Initial-pose frame, timestamp, pose, quaternion, covariance symmetry,
  covariance positive-semidefiniteness, and configured ambiguity bounds are
  checked before initialization.
- Scan and odometry frames, timestamps, and poses are checked before use.
- Odometry is linearly interpolated in translation and SLERPed in rotation at
  the input timestamp. Extrapolation and excessive interpolation gaps are
  rejected.
- Non-finite, non-rigid, non-converged, and excessive-jump registration
  results cannot update the correction.
- Invalid initialization observations reset the consecutive-confirmation
  count. Scans at or before the initialization timestamp cannot confirm it.
- Repeated tracking registration failures enter `LOST`; the last valid
  correction remains available for FAST-LIO prediction.
- Every rejection and transition has a stable reason code in the diagnostic
  output.

The deterministic test suite contains 10 C++ core tests and 5 Python
benchmark-math tests. It covers buffer bounds/interpolation, timestamp
ordering, malformed and ambiguous covariance, transform validation,
confirmation gating, and correction retention in `LOST`.

## Development-bag replay

All four unperturbed development bags completed at 1x without a timeout. Each
run used a synthetic unperturbed prior after 10 seconds and entered `TRACKING`
only after three confirmations.

| Bag | Scans | Accepted | Translation p95 / max (m) | Rotation p95 / max (deg) | Decision p95 (ms) | Max input age (ms) |
|---|---:|---:|---:|---:|---:|---:|
| `mine_nav1_r1` | 563 | 81.2% | 0.004 / 0.068 | 0.131 / 0.830 | 181.258 | 437.764 |
| `mine_nav2_r1` | 924 | 88.2% | 0.026 / 0.166 | 0.718 / 3.133 | 173.000 | 517.953 |
| `mine_nav3_r1` | 1,616 | 92.2% | 0.016 / 0.089 | 0.326 / 3.979 | 171.395 | 438.085 |
| `mine_nav4_r1` | 1,416 | 92.7% | 0.020 / 0.232 | 0.447 / 4.934 | 79.515 | 223.267 |

The Phase 0 maximum translation errors for bags 2, 3, and 4 were 5.429 m,
8.713 m, and 25.114 m. Timestamp synchronization and result validation reduced
those maxima to 0.166 m, 0.089 m, and 0.232 m respectively. Localizer TF and
map-odometry sample counts matched within every run.

The remaining decision-budget misses are expected Phase 2 work. Phase 1 does
not yet bound PCL NDT wall time or replace the scan callback queue with a
deadline-aware worker.

## Negative replay

`mine_nav1_r1` was replayed with an otherwise correct prior whose position
standard deviation was 20 m. The prior was rejected as
`initial_pose_covariance_ambiguous`. Across all 563 scans:

- state remained `UNINITIALIZED`;
- accepted scan decisions: 0;
- localizer `map -> odom_lidar` TF publications: 0;
- localizer `/odometry_map` publications: 0.

This directly verifies that an ambiguous prior cannot publish a correction.
Malformed, stale, future, premature, and invalid-transform paths are covered
by the deterministic core tests and use the same production validation
functions.

## Reproduction

```bash
ros2 run ndt_localization run_development_baseline.py \
  --bags-root /home/spotbot/Workspace/spot_nav_ws \
  --output-dir /tmp/ndt_phase1_final_v2 \
  --rate 1.0 \
  --initial-pose-delay 10.0 \
  --reference-repeatability-runs 1 \
  --force

ros2 launch ndt_localization benchmark_replay.launch.py \
  bag_path:=/home/spotbot/Workspace/spot_nav_ws/mine_nav1_r1 \
  output_directory:=/tmp/ndt_phase1_ambiguous_final \
  run_name:=mine_nav1_ambiguous \
  rate:=1.0 \
  initial_pose_delay:=10.0 \
  position_sigma:=20.0
```

The development replay used localizer configuration SHA-256
`6aa33351012e63e351f4c15f4fab748244662a6fe5d5a2f71369a09855e82222`.
Generated CSV/JSON artifacts remain outside the source tree under the two
`/tmp` directories shown above.
