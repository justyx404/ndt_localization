# Phase 2 validation

Date: 2026-07-29
Platform: Intel i7-1360P, ROS 2 Humble, x86-64

Phase 2 is complete. Scan registration now has bounded inputs, latest-only
work scheduling, an application-level decision deadline, and explicit
discarding of matcher results that finish after supersession or timeout.

## Workload and fallback behavior

- The scan subscription uses reliable `KEEP_LAST(1)` QoS.
- A dedicated registration thread owns PCL NDT. At most one scan is active and
  one latest scan is pending; a newer scan supersedes all older undecided work.
- Every task has a monotonically increasing generation. Initial-pose resets
  and newer scans invalidate older generations before they can change the
  correction.
- Scan input is deterministically capped at 4,000 uniformly selected points.
- The map is voxel-filtered at 0.15 m. Each registration uses a 35 m radius
  local map capped at 120,000 points and rejects targets below 1,000 points.
- NDT is limited to 15 iterations. Its separate full-cloud fitness pass
  remains disabled.
- The public scan-decision budget is 80 ms. A watchdog selects a timeout 1 ms
  early to allow for scheduler wake-up latency.
- PCL NDT cannot be interrupted mid-call. A timed-out or superseded task is
  marked decided immediately; if `align()` later returns, the result is
  published only as `ndt_localization/late_result` and cannot reach the state
  update path.
- FAST-LIO odometry publication is independent of the registration worker.
  After a correction has been validated, rejected, superseded, and timed-out
  scans continue to use the last valid `map -> odom_lidar` correction.

The deterministic suite now contains 12 C++ core tests and 5 Python
benchmark-math tests. Phase 2 adds exact deadline-boundary and deterministic
point-selection coverage. Bag replay exercises the worker, generation,
timeout, late-result, and fallback paths.

## Nominal development-bag replay

All four unperturbed development bags completed at 1x and entered `TRACKING`.
Every recorded scan diagnostic made its accept/reject/timeout decision within
the 80 ms application budget.

| Bag | Decisions | Timeouts / superseded / late | Decision p95 / max (ms) | Queue p95 / max (ms) | Translation p95 / max (m) |
|---|---:|---:|---:|---:|---:|
| `mine_nav1_r1` | 562 | 0 / 1 / 1 | 52.260 / 77.518 | 0.141 / 10.423 | 0.018 / 0.062 |
| `mine_nav2_r1` | 923 | 3 / 0 / 3 | 49.720 / 79.218 | 0.143 / 0.197 | 0.041 / 0.117 |
| `mine_nav3_r1` | 1,612 | 0 / 1 / 1 | 48.135 / 78.597 | 0.142 / 8.705 | 0.033 / 0.077 |
| `mine_nav4_r1` | 1,416 | 0 / 0 / 0 | 47.021 / 77.133 | 0.125 / 0.197 | 0.033 / 0.287 |

Deadline overruns were zero in every bag. The three bag-2 registrations that
could not finish safely were deliberately timed out before 80 ms; their later
completions were discarded. The maximum observed capped scan and local-map
sizes were 4,000 and 106,434 points respectively.

Phase 2 targets latency and stale-work safety, not a lower trajectory error.
The nominal p95 translation error remains between 1.8 cm and 4.1 cm. Bag 4 has
one 0.287 m maximum-error outlier, compared with 0.232 m in Phase 1, while its
p95 remains 0.033 m. Recorded localization is regression/pseudo-ground-truth,
not independent survey-grade ground truth.

## Accelerated replay

`mine_nav1_r1` completed at 2x with a validated correction:

- 560 decisions, 90 superseded scans, and no timeouts or deadline overruns;
- decision p95 / max of 50.104 / 65.605 ms;
- queue-wait p95 / max of 15.697 / 50.325 ms;
- 85 matcher calls finished after supersession and were discarded;
- translation p95 / max of 0.028 / 0.071 m;
- 452 map-odometry and TF outputs, continuously driven by valid FAST-LIO
  odometry after initialization.

For the overload test, replay began at 1x so the synthetic prior and three
confirmations could establish `TRACKING`, then the live bag player was changed
to 4x. This avoids measuring DDS loss in the synthetic-prior test harness as
localizer initialization behavior.

At 4x tracking load:

- 561 decisions, 383 superseded scans, and 2 intentional timeouts;
- zero decision overruns, with p95 / max of 37.756 / 79.197 ms;
- queue-wait p95 / max of 27.905 / 40.256 ms;
- 211 late matcher completions (209 superseded, 2 timed out), all discarded;
- late NDT calls completed as late as 101.028 ms without applying a stale
  correction;
- the state remained `TRACKING`, and 458 map-odometry and TF outputs continued
  from FAST-LIO using the last valid correction.

Once 4x load stabilized, successive 5-second scan windows had mean queue waits
of 15.9 to 19.4 ms and mean input ages of 171.9 to 186.1 ms. Neither metric
grew with elapsed replay time. The bounded queue therefore shed old work
instead of accumulating a backlog.

## Exit criteria

- Nominal replay meets the 80 ms decision budget: **pass**.
- Accelerated replay does not create an increasing backlog: **pass**.
- Matcher results completing after timeout or supersession are discarded:
  **pass**.
- FAST-LIO remains available with the last validated map correction:
  **pass**.

## Reproduction

```bash
ros2 run ndt_localization run_development_baseline.py \
  --bags-root /home/spotbot/Workspace/spot_nav_ws \
  --output-dir /tmp/ndt_phase2_nominal_final \
  --rate 1.0 \
  --initial-pose-delay 10.0 \
  --reference-repeatability-runs 1 \
  --force

ros2 launch ndt_localization benchmark_replay.launch.py \
  bag_path:=/home/spotbot/Workspace/spot_nav_ws/mine_nav1_r1 \
  output_directory:=/tmp/ndt_phase2_mine1_2x_final \
  run_name:=mine_nav1_r1_2x \
  rate:=2.0 \
  initial_pose_delay:=10.0

# Start the overload run at 1x and wait for TRACKING.
ROS_DOMAIN_ID=125 ros2 launch ndt_localization \
  benchmark_replay.launch.py \
  bag_path:=/home/spotbot/Workspace/spot_nav_ws/mine_nav1_r1 \
  output_directory:=/tmp/ndt_phase2_mine1_1x_to_4x_final \
  run_name:=mine_nav1_r1_1x_to_4x \
  rate:=1.0 \
  initial_pose_delay:=10.0

# In a second shell, switch the active player to 4x.
ROS_DOMAIN_ID=125 ros2 service call /rosbag2_player/set_rate \
  rosbag2_interfaces/srv/SetRate "{rate: 4.0}"
```

The validated localizer configuration SHA-256 is
`82d4298e14085ff5984ac19baeb448a2c9041c12298aeacefc50e78f45abec74`.
Generated CSV/JSON artifacts remain outside the source tree in the `/tmp`
directories shown above.
