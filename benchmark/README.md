# Localization benchmark harness

This harness implements Phase 0 of `ROBUST_LOCALIZATION_PLAN.md`. It keeps
recorded localization outputs isolated from the node under test:

- recorded `/tf` becomes `/reference/tf`;
- recorded `/odometry_map` becomes `/reference/odometry_map`;
- recorded `/initialpose` becomes `/reference/initialpose`;
- the localizer receives only `/global_map`, `/odometry_lio`,
  `/cloud_registered_body`, and a generated `/benchmark/initialpose`;
- localizer output is written under `/benchmark`.

The evaluator reconstructs the reference pose as:

```text
T_map_base(t) = T_map_odom(t) * T_odom_base(t)
```

Translation is linearly interpolated and orientation uses quaternion SLERP.
Samples are rejected when either side of the interpolation interval exceeds the
configured maximum gap.

## One-bag replay

Build and source the workspace, then run:

```bash
ros2 launch ndt_localization benchmark_replay.launch.py \
  bag_path:=/absolute/path/to/mine_nav1_r1 \
  output_directory:=/tmp/ndt_baseline/mine_nav1_r1 \
  run_name:=mine_nav1_r1 \
  rate:=1.0
```

Controlled perturbations are launch arguments:

```bash
ros2 launch ndt_localization benchmark_replay.launch.py \
  bag_path:=/absolute/path/to/mine_nav1_r1 \
  output_directory:=/tmp/ndt_perturbed/mine_nav1_r1 \
  translation_x:=2.0 yaw_degrees:=30.0 \
  position_sigma:=2.0 yaw_sigma_degrees:=30.0
```

Each run writes:

- `reference_pose_metrics.csv`;
- `initial_pose_checks.csv`;
- `scan_metrics.csv`;
- `summary.json`;
- `report.md`.

## Four-bag development baseline

```bash
ros2 run ndt_localization run_development_baseline.py \
  --bags-root /home/spotbot/Workspace/spot_nav_ws \
  --output-dir /tmp/ndt_development_baseline \
  --rate 1.0
```

The aggregate `development_baseline.md` and
`development_baseline.json` preserve the exact reproduction command. Generated
data belongs in an external results directory; source bags are never modified.
Each replay also has a wall timeout derived from its recorded duration, so a
pathological registration or reliable-transport stall becomes an explicit
failed run instead of blocking the suite indefinitely. The timeout factor and
startup allowance are configurable CLI arguments.

Recorded `/odometry_map` is regression/pseudo-ground-truth produced by the
existing localization system. Reports must not describe it as independent
survey-grade truth.
