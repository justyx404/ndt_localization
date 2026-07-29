# Phase 4 localization interface minimization

Date: 2026-07-29
Platform: Intel i7-1360P, ROS 2 Humble, x86-64

The planned local-matcher comparison was skipped. This replacement Phase 4
reduces the localization node's public configuration surface while preserving
the validated Phase 3 behavior.

## Configuration ownership

The two production configuration files now have separate responsibilities:

- `spot_navigation/config/lio_localization.yaml` contains deployment-wide
  integration values: the three shared frame IDs, FAST-LIO and sensor setup,
  global-map processing, and terrain processing. It contains no localization
  algorithm tuning.
- `ndt_localization/config/localization.yaml` is the single source for the 17
  localization choices that operators may reasonably tune for another map,
  compute platform, or acceptance policy. It does not duplicate frame IDs.

`spot_navigation/launch/lio_localization.launch.py` loads the dedicated
localization file first and the shared Spot file second. The localizer
therefore receives tuning from its own package and frame IDs from the shared
deployment file. Standalone launches use the node's matching frame defaults.

The workspace uses symlink installation, so the paths discussed during
deployment resolve to these source files:

```text
build/spot_navigation/config/lio_localization.yaml
  -> src/spot_navigation/config/lio_localization.yaml

install/ndt_localization/share/ndt_localization/config/localization.yaml
  -> src/ndt_localization/config/localization.yaml
```

## Public localization parameters

The ROS localization interface was reduced from 58 parameters to 17:

| Parameter | Effect |
|---|---|
| `ndt_resolution` | Tracking and refinement NDT cell size |
| `ndt_map_leaf_size` | Global-map downsampling |
| `local_map_radius_m` | Radius of the map supplied to registration |
| `max_local_map_points` | CPU/memory cap for each local target |
| `max_scan_points` | CPU cap for each scan and refinement |
| `registration_deadline_ms` | Application decision deadline |
| `max_result_translation_delta_m` | Rejects implausible translation jumps |
| `max_result_rotation_delta_deg` | Rejects implausible rotation jumps |
| `max_consecutive_rejections` | Tracking failures allowed before `LOST` |
| `initialization_timeout_ms` | Global initialization/recovery watchdog |
| `initialization_max_translation_span_m` | Maximum initial search envelope |
| `initialization_max_yaw_span_deg` | Maximum initial yaw search envelope |
| `initialization_confirmation_scans` | Consistent scans required to publish |
| `initialization_max_fitness_score` | Absolute hypothesis acceptance limit |
| `initialization_min_score_margin` | Ambiguous-hypothesis rejection limit |
| `recovery_translation_span_m` | Recovery envelope around last correction |
| `recovery_yaw_span_deg` | Recovery yaw envelope |

These parameters remain public because they express workload, environment
scale, or externally meaningful safety/acceptance policy.

## Internal implementation policy

The other 41 former ROS parameters are now named compiled defaults. They are
implementation mechanics with no validated per-deployment use case:

- NDT line-search step, convergence epsilon, iteration count, scan leaf size,
  minimum target size, runtime logging, and full fitness-pass policy;
- odometry buffer capacity, interpolation gap, message-age limits, future
  tolerance, quaternion tolerance, and covariance numerical tolerances;
- confirmation agreement thresholds, watchdog reserves, covariance multiplier,
  minimum search spans, and relocalization retry cadence;
- coarse-search sampling, resolution, step, convergence, iteration, hypothesis,
  refinement-candidate, scoring-range, and distinct-candidate mechanics;
- diagnostic publication policy, which is always enabled.

The values are not anonymous literals. Node-wide mechanics use `constexpr`
names in `localization.cpp`; coarse-to-fine search mechanics use the typed
`RobustInitializer::Config` defaults. Changing one of them is therefore an
algorithm change that must be reviewed and benchmarked, rather than an
untracked deployment override.

## Live interface verification

The built node was launched with both production parameter files and inspected
through the ROS parameter service:

- localization parameters declared: 17;
- expected localization parameters: 17;
- missing or extra localization parameters: 0;
- frame IDs: `map`, `odom_lidar`, and `base_link`;
- removed internal localization parameters still declared: 0.

The build passed, as did all 18 C++ localization-core tests and all 5 Python
benchmark-math tests.

## Replay comparison

Both replays used `mine_nav1_r1` at 1x with the new 17-parameter file. Phase 3
used the explicit 58-parameter profile.

### Nominal prior

| Metric | Phase 3 | Reduced interface |
|---|---:|---:|
| Initialization decision | selected | selected |
| Initialization search (ms) | 376.427 | 366.701 |
| Best fitness | 0.062444 | 0.062444 |
| Translation p95 / max (m) | 0.017571 / 0.061592 | 0.017571 / 0.061592 |
| Rotation p95 / max (deg) | 0.410647 / 1.291926 | 0.410647 / 1.291926 |
| Scan decision p95 / max (ms) | 49.132 / 77.708 | 48.911 / 77.637 |
| Deadline overruns | 0 | 0 |
| Localized odometry outputs | 454 | 454 |

### 10 m / 180 degree prior

| Metric | Phase 3 | Reduced interface |
|---|---:|---:|
| Initialization decision | selected | selected |
| Initialization search (ms) | 251.815 | 262.512 |
| Best fitness / margin | 0.062718 / 0.082315 | 0.062718 / 0.082315 |
| Translation p95 / max (m) | 0.017568 / 0.061592 | 0.017568 / 0.061592 |
| Rotation p95 / max (deg) | 0.409668 / 1.291926 | 0.409668 / 1.291926 |
| Scan decision p95 / max (ms) | 50.310 / 78.117 | 49.058 / 77.597 |
| Deadline overruns | 0 | 0 |
| Localized odometry outputs | 455 | 455 |

Fitness, ambiguity margin, trajectory error, and output counts are identical.
Initialization and tracking runtime differences are normal wall-clock
scheduling variation. One 10 m / 180 degree matcher completion was reported
after its scan had been superseded; it was discarded by design, did not cross
the 80 ms decision deadline, and did not change the output count or trajectory.

## Exit criteria

- Public localization interface reduced from 58 to 17: **pass**.
- Combined Spot configuration contains no localization tuning: **pass**.
- Removed parameters retain their validated Phase 3 behavior internally:
  **pass**.
- Nominal and large-perturbation accuracy do not regress: **pass**.
- Tracking decisions remain within 80 ms: **pass**.

## Reproduction

```bash
ros2 launch ndt_localization benchmark_replay.launch.py \
  bag_path:=/home/spotbot/Workspace/spot_nav_ws/mine_nav1_r1 \
  output_directory:=/tmp/ndt_phase4_interface_nominal_v1 \
  run_name:=mine_nav1_r1_minimal_interface

ros2 launch ndt_localization benchmark_replay.launch.py \
  bag_path:=/home/spotbot/Workspace/spot_nav_ws/mine_nav1_r1 \
  output_directory:=/tmp/ndt_phase4_interface_10m_180deg_v1 \
  run_name:=mine_nav1_r1_minimal_interface_10m_180deg \
  translation_x:=10.0 yaw_degrees:=180.0 \
  position_sigma:=10.0 yaw_sigma_degrees:=180.0
```

Generated benchmark artifacts remain outside the source tree in the `/tmp`
directories shown above.
