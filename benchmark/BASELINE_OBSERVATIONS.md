# Phase 0 development baseline

Date: 2026-07-29
Platform: Intel i7-1360P, ROS 2 Humble, x86-64

Phase 0 is complete. The benchmark harness completed all four nominal
development-bag replays, reference reconstruction repeated deterministically,
and all six recorded initial poses agreed with the reconstructed trajectory
after timestamp alignment.

Recorded localization is regression/pseudo-ground-truth produced by the
existing system, not independent survey-grade truth. The matching results
below characterize the current implementation; they are not acceptance
results for the planned robust localizer.

## Development-bag results

Each reference bag was analyzed twice offline. Each live run used 1x replay,
an unperturbed synthetic prior at 10 seconds, the default 80 ms decision
deadline, and fitness-score computation disabled so instrumentation does not
add a full nearest-neighbor scoring pass. Trajectory error is evaluated
strictly after the synthetic prior timestamp.

| Bag | Status | Scans | Accepted | Reference translation p95 / max (m) | Localizer translation p95 / max (m) | Localizer rotation p95 / max (deg) |
|---|---|---:|---:|---:|---:|---:|
| `mine_nav1_r1` | completed | 563 | 82.1% | 0.000001077 / 0.000002292 | 0.003 / 0.068 | 0.113 / 0.830 |
| `mine_nav2_r1` | completed | 924 | 88.6% | 0.000001763 / 0.000003269 | 4.279 / 5.429 | 52.092 / 72.592 |
| `mine_nav3_r1` | completed | 1,616 | 93.5% | 0.000003791 / 0.000006963 | 0.015 / 8.713 | 0.310 / 143.006 |
| `mine_nav4_r1` | completed | 1,416 | 92.9% | 0.000005183 / 0.000010091 | 0.025 / 25.114 | 0.651 / 114.277 |

| Bag | Decision p50 / p95 / max (ms) | Input age p95 / max (ms) | Decisions over 80 ms |
|---|---:|---:|---:|
| `mine_nav1_r1` | 46.050 / 190.809 / 207.181 | 369.556 / 539.052 | 120 |
| `mine_nav2_r1` | 73.511 / 187.063 / 351.647 | 827.696 / 998.811 | 377 |
| `mine_nav3_r1` | 45.052 / 180.773 / 194.241 | 199.517 / 400.145 | 249 |
| `mine_nav4_r1` | 47.095 / 83.753 / 205.277 | 112.255 / 221.132 | 170 |

There were no replay timeouts and no harness/reference failures. The reference
rotation-error p95 values were at most 0.000011198 degrees.

## Repeatability

The two complete offline passes for every development bag produced identical
SHA-256 digests over all reconstructed-pose and initial-pose rows.

Two independent live `mine_nav1_r1` runs also produced the same 563 scan
decisions and the same decision counts: 462 accepted, 70 waiting for initial
pose, and 31 waiting for odometry. Total-latency p95 differed by 0.30% and the
maximum by 3.08%; input-age p95 differed by 5.82%. Post-initialization
translation-error p95 differed by 0.69%. These variations are expected
wall-clock scheduling effects; reference poses and logical decisions remained
stable.

## Recorded initial-pose timestamp cross-check

The mine bags record an initial-pose message's publication later than the
timestamp in its header. Comparing only at the header timestamp can therefore
produce a false disagreement. The analyzer evaluates both times and retains
both results. Bag storage time was the closer interpretation in all six bags:

| Bag | Storage minus header (ms) | Translation error (m) | Rotation error (deg) | Within 0.5 m / 5 deg |
|---|---:|---:|---:|:---:|
| `mine_nav1_r1` | +606.858 | 0.049 | 1.461 | yes |
| `mine_nav1_r2` | +46.610 | 0.024 | 1.293 | yes |
| `mine_nav1_r3` | -49.450 | 0.055 | 0.695 | yes |
| `mine_nav1_r5` | -25.368 | 0.008 | 0.800 | yes |
| `mine_nav2_r1` | -62.663 | 0.065 | 1.146 | yes |
| `mine_nav3_r1` | -43.678 | 0.029 | 0.139 | yes |

Each cross-check was also repeated twice with an identical digest.

## Baseline findings

The Phase 0 harness meets its exit criteria, but the current localization
behavior does not meet the plan's provisional tracking criteria:

- every bag has decisions over the 80 ms budget;
- input age reaches 0.999 seconds, showing that callback work can accumulate;
- `mine_nav2_r1` remains in a wrong registration basin for much of the run;
- `mine_nav3_r1` and `mine_nav4_r1` contain isolated large-error matches;
- optimizer convergence alone is therefore not a valid acceptance test.

These findings establish the work for Phase 1: timestamp/state correctness,
explicit rejection reasons, and prevention of unvalidated transform updates.

## Reproduction

```bash
ros2 run ndt_localization run_development_baseline.py \
  --bags-root /home/spotbot/Workspace/spot_nav_ws \
  --output-dir /tmp/ndt_phase0_baseline \
  --rate 1.0 \
  --initial-pose-delay 10.0 \
  --timeout-factor 1.5 \
  --timeout-overhead-seconds 30.0 \
  --reference-repeatability-runs 2 \
  --force
```

The completed run used localizer configuration SHA-256
`83510e33cd369b42386b353126f8695f0b6d00df66b267fdd34e76b436d455d5`.
Raw CSV/JSON artifacts were generated outside the source tree under
`/tmp/ndt_phase0_final_delay10.SXnpLm`.
