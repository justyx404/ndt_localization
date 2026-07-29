# Localization benchmark harness

This harness implements the measurement foundation from Phase 0, the
state/validation observability required by Phase 1, the bounded-workload
measurements required by Phase 2, and the initialization/recovery measurements
required by Phase 3 of `ROBUST_LOCALIZATION_PLAN.md`. It keeps recorded
localization outputs isolated from the node under test:

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
Samples are unavailable when either side of the interpolation interval exceeds
the configured maximum gap.

## Production parameter ownership

`ndt_localization/config/localization.yaml` contains the 17 public localization
tuning choices. Shared frames and the FAST-LIO/sensor/map pipeline remain in
`spot_navigation/config/lio_localization.yaml`; the production Spot launch
loads both files for the localizer. Optimizer mechanics, numerical tolerances,
queue policy, and diagnostics are named compiled defaults and are intentionally
absent from the ROS parameter interface. The complete classification and
regression evidence are in `PHASE4_CONFIG_MINIMIZATION.md`; the subsequent
dead-code and tracking-pipeline cleanup is recorded in
`PHASE4_REFACTOR_CLEANUP.md`.

## Offline reference validation

The deterministic offline analyzer reads only the four reference topics
directly from a bag, without DDS or the localizer:

```bash
ros2 run ndt_localization analyze_reference_bag.py \
  --bag-path /absolute/path/to/mine_nav1_r1 \
  --output-directory /tmp/ndt_reference/mine_nav1_r1 \
  --repeatability-runs 2
```

It cross-checks reconstructed poses against recorded `/odometry_map`. For a
recorded `/initialpose`, it evaluates both the message header time and bag
storage time. The storage time represents when rosbag recorded and later
replays the message; this resolves the delayed-header semantics present in the
mine bags. Both interpretations and the selected one are retained in CSV and
JSON.

## One-bag live replay

Build and source the workspace, then run:

```bash
ros2 launch ndt_localization benchmark_replay.launch.py \
  bag_path:=/absolute/path/to/mine_nav1_r1 \
  output_directory:=/tmp/ndt_baseline/mine_nav1_r1 \
  run_name:=mine_nav1_r1 \
  rate:=1.0 \
  initial_pose_delay:=10.0
```

The default synthetic prior is copied from recorded map odometry 10 seconds
after its first sample. Localizer trajectory error is scored strictly after
that prior timestamp because the odometry sample at the same timestamp may
have been published before the reset callback. Pre-initialization scan
decisions and latency remain in the report.

Controlled perturbations are launch arguments:

```bash
ros2 launch ndt_localization benchmark_replay.launch.py \
  bag_path:=/absolute/path/to/mine_nav1_r1 \
  output_directory:=/tmp/ndt_perturbed/mine_nav1_r1 \
  initial_pose_delay:=10.0 \
  translation_x:=2.0 yaw_degrees:=30.0 \
  position_sigma:=2.0 yaw_sigma_degrees:=30.0
```

Each live run writes:

- `reference_pose_metrics.csv`;
- `initial_pose_checks.csv`;
- `scan_metrics.csv`;
- `state_events.csv`;
- `late_results.csv`;
- `initialization_searches.csv`;
- `summary.json`;
- `report.md`.

`scan_metrics.csv` includes the localization state, correction-valid flag,
confirmation/rejection counters, synchronized-odometry gaps, and candidate
transform deltas for every scan decision. Phase 2 fields include generation,
queue wait, decision deadline, deadline status, deterministic scan cap, local
map size, and stage timings. `late_results.csv` records matcher work that
finished after its scan had already been superseded or timed out; such results
are diagnostic-only and cannot update the correction. `state_events.csv`
records initialization, validation rejection, and loss transitions.
`initialization_searches.csv` records the covariance-derived search envelope,
hypothesis/convergence/refinement counts, stage runtime, best and second-best
scores, ambiguity margin, final decision, and whether the search was a
recovery attempt. The summary keeps reference and localizer TF counts separate as
`map_to_odom_transforms` and `localization_map_to_odom_transforms`.

The localizer exposes `/localization/trigger_relocalization`
(`std_srvs/srv/Trigger`) for controlled recovery tests. Automatic recovery
uses the same bounded path after tracking enters `LOST`.

## Four-bag development baseline

```bash
ros2 run ndt_localization run_development_baseline.py \
  --bags-root /home/spotbot/Workspace/spot_nav_ws \
  --output-dir /tmp/ndt_development_baseline \
  --rate 1.0 \
  --initial-pose-delay 10.0
```

Each bag directory contains separate `reference/` and `replay/` artifacts plus
`run_status.json`. The aggregate `development_baseline.md` and
`development_baseline.json` preserve the reproduction command.

The runner performs two offline reference passes by default, then a live
replay. Each live replay has a wall timeout derived from its recorded duration.
A timeout is retained as a measured baseline outcome, including any partial
summary produced during graceful shutdown; a true harness or reference failure
returns a nonzero exit status. Existing completed runs can be resumed unless
`--force` is supplied.

Generated data belongs in an external results directory; source bags are never
modified. Recorded `/odometry_map` is regression/pseudo-ground-truth produced
by the existing localization system, not independent survey-grade truth.
