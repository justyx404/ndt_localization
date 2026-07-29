# Phase 4 configuration minimization

Date: 2026-07-29
Platform: Intel i7-1360P, ROS 2 Humble, x86-64

The planned local-matcher comparison was skipped. The replacement Phase 4
minimizes the active deployment configuration without changing the Phase 3
localization profile.

## Configuration policy

The deployment rule is:

> Only override a localization parameter when the deployment value differs
> from the validated node default.

The 40 localization values in
`spot_navigation/config/lio_localization.yaml` were all identical to their
compiled defaults. The complete `localization:` block was therefore removed.
The explicit 58-parameter profile remains in
`ndt_localization/config/localization.yaml` for auditing, benchmarking, and
intentional tuning.

The workspace uses symlink installation:

```text
build/spot_navigation/config/lio_localization.yaml
  -> src/spot_navigation/config/lio_localization.yaml

install/spot_navigation/share/spot_navigation/config/lio_localization.yaml
  -> build/spot_navigation/config/lio_localization.yaml
```

Editing the source file therefore updates all three paths. No generated build
artifact is maintained separately.

## Effective-parameter equivalence

The localization node was launched with the minimal deployment file and its
live parameter set was dumped through the ROS parameter service. It was
compared recursively with the explicit package-local profile:

- explicit localization parameters: 58;
- effective localization parameters: 58;
- missing parameters: 0;
- extra localization parameters: 0;
- differing values: 0;
- differing frame IDs: 0.

The simplified deployment file therefore changes configuration representation
only. It does not change an effective matcher, workload, validation,
initialization, recovery, or deadline value.

## Replay comparison

Both replays used `mine_nav1_r1` at 1x. Phase 3 used the explicit package
profile; Phase 4 passed the minimal deployment file directly to the localizer.

### Nominal prior

| Metric | Phase 3 explicit | Phase 4 minimal |
|---|---:|---:|
| Initialization decision | selected | selected |
| Initialization search (ms) | 376.427 | 374.650 |
| Best fitness | 0.062444 | 0.062444 |
| Translation p95 / max (m) | 0.017571 / 0.061592 | 0.017571 / 0.061592 |
| Rotation p95 / max (deg) | 0.410647 / 1.291926 | 0.410647 / 1.291926 |
| Scan decision p95 / max (ms) | 49.132 / 77.708 | 50.749 / 79.129 |
| Deadline overruns | 0 | 0 |
| Localized odometry outputs | 454 | 454 |

### 10 m/180 degree prior

| Metric | Phase 3 explicit | Phase 4 minimal |
|---|---:|---:|
| Initialization decision | selected | selected |
| Initialization search (ms) | 251.815 | 259.184 |
| Best fitness / margin | 0.062718 / 0.082315 | 0.062718 / 0.082315 |
| Translation p95 / max (m) | 0.017568 / 0.061592 | 0.017568 / 0.061592 |
| Rotation p95 / max (deg) | 0.409668 / 1.291926 | 0.409668 / 1.291926 |
| Scan decision p95 / max (ms) | 50.310 / 78.117 | 49.045 / 79.145 |
| Deadline overruns | 0 | 0 |
| Localized odometry outputs | 455 | 455 |

Fitness, ambiguity margin, trajectory error, and output counts are identical.
The small runtime variation is normal wall-clock scheduling noise and remains
inside the 80 ms application decision bound.

## Exit criteria

- Active deployment localization overrides reduced from 40 to 0: **pass**.
- Effective values exactly match the Phase 3 profile: **pass**.
- Nominal accuracy and initialization do not regress: **pass**.
- Large-perturbation convergence does not regress: **pass**.
- Tracking decisions remain within 80 ms: **pass**.

## Reproduction

```bash
ros2 launch ndt_localization benchmark_replay.launch.py \
  bag_path:=/home/spotbot/Workspace/spot_nav_ws/mine_nav1_r1 \
  output_directory:=/tmp/ndt_phase4_minimal_nominal \
  run_name:=mine_nav1_r1_minimal_config \
  config_file:=/home/spotbot/Workspace/spot_nav_ws/src/spot_navigation/config/lio_localization.yaml

ros2 launch ndt_localization benchmark_replay.launch.py \
  bag_path:=/home/spotbot/Workspace/spot_nav_ws/mine_nav1_r1 \
  output_directory:=/tmp/ndt_phase4_minimal_10m_180deg \
  run_name:=mine_nav1_r1_minimal_10m_180deg \
  translation_x:=10.0 yaw_degrees:=180.0 \
  position_sigma:=10.0 yaw_sigma_degrees:=180.0 \
  config_file:=/home/spotbot/Workspace/spot_nav_ws/src/spot_navigation/config/lio_localization.yaml
```

The minimal deployment configuration SHA-256 is
`ff6d2810cf3c24132919ed5245572b6de216078ba4413f6f74c83458c8f28ac2`.
Generated benchmark artifacts remain outside the source tree in the `/tmp`
directories shown above.
