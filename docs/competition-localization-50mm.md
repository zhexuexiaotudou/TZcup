# Dynamic localization diagnostic — 2026-09-13

**FAIL; 50 mm has not been demonstrated.** Candidate `9542db7`, based on cleaning result `a9691b4`. Only candidate pack A was applied. B/C, mapping, perception, full CI, PR and deployment were outside this bounded run. Prior cleaning geometry and its runtime overlay were preserved.

## Changes and verification

The probe defaults AMCL `tf_broadcast=false`, leaving global EKF as the intended map→odom owner. `PROBE_AMCL_TF_BROADCAST=true` restores the old diagnostic behavior; it does not relax the authority validator. The collector now seals normally through a stop file. The route driver uses Nav2 and safety commands, never ground truth. Ground truth is recorded for offline scoring only.

The scorer composes map→odom with odom→base_footprint TF and compares that navigation map pose against timestamp-paired ground truth under the public start-coordinate transform. There is no fitted alignment, timestamp shift, odom-as-map fallback, or ground-truth correction of control. Fused odometry is reported separately. Planar reference equivalence must be rechecked if the vehicle's base frames change.

Focused verification: 29 localization tests passed; Python compilation and runner Bash syntax passed. Only `sanitation_localization_acceptance` was rebuilt remotely; build returned 0. The prior cleaning overlay remained sourced. Full CI was deliberately not run under the task scope.

## First and only dynamic run

`run-01` returned validation code 2. Nav2 aborted (result 6) at simulation time 12.953 s, after starting at 3.345 s. Driver wall duration was 65.015 s. The intended later return leg and 45 s anomaly observation were **not reached**.

| Measurement | Result |
|---|---:|
| Maximum GT displacement from first recorded position | 1.668454 m; required >2 m |
| GT path length | 1.789321 m |
| Dynamic reference epochs | 359 |
| Strictly paired epochs | 4 |
| Pair coverage | 1.1142% |
| Navigation map-pose RMSE, paired diagnostic samples | 12.152933 m |
| P95, paired diagnostic samples | 12.514197 m |
| Maximum, paired diagnostic samples | 12.538851 m |
| Fused-vs-navigation TF planar discrepancy, maximum | 0.002478 m |

These four-sample error statistics are diagnostic only, not a valid completed dynamic acceptance dataset. Neither the displacement nor sampling conditions passed, and the measured errors greatly exceed 0.05 m. No interpolation tolerance was relaxed after observing the result.

## Evidence and next root cause

The local `/odom` finishes near (1.668308, 0.000031) m while public-aligned ground truth finishes near (1.668305, 0.000008) m. GNSS odometry and AMCL positions remain plausible. Global fused odometry instead reaches approximately (0.833791, -12.511396) m; its initial stationary lateral velocity is already about 0.475 m/s. Navigation consequently reports sensor origins outside the costmap and aborts. No large planar IMU acceleration or nonfinite raw-input event was detected by the diagnostic threshold during the captured interval. This does not exonerate the inputs at the unobserved 45 s epoch.

The next priority is **global fusion measurement-frame and effective-parameter diagnosis**. `/odometry/gps` has frame `odom` and zero reported planar covariance, while the global filter works in `map`. Check whether its measurement transform feeds back through the filter's own map→odom transform and how zero covariance is treated. This is a hypothesis, not a proven cause. The installed YAML specifies 30 Hz global fusion, but recorded global output is approximately 10 Hz; parameter dumps contain empty parameter dictionaries. Neither observation proves that parameters failed to load. Resolve the effective runtime configuration before changing fusion weights or launching another run. No speculative second run was performed.

AMCL's actual parameter snapshot confirms `tf_broadcast=false`. The collector observed one map→odom received GID across 97 messages, but that GID could not be joined to a discovered node endpoint. Thus unique `/global_ekf` ownership is **unverified**, not proven to have multiple publishers. The runtime uses Cyclone DDS; an upstream report describes the same publication-handle/GUID mismatch: <https://github.com/ros2/rmw_cyclonedds/issues/377>. This instrumentation issue does not explain the meter-scale fusion drift. The validator retains the failure instead of guessing an owner.

## Retained evidence and rollback

Cloud evidence: `/root/autodl-tmp/tzcup-competition-sim-only-20260912/evidence/localization-50mm-20260913-01`. Local worktree copy: `.work/localization-50mm-20260913-01/evidence`. The archive includes sealed MCAP, raw-input diagnosis, paired statistics, TF registry, parameters, navigation logs, source binding and resource-release census. No matching task-partition/domain processes remained and the formal simulation lock was available after the run.

Rollback point is `a9691b4`; revert candidate `9542db7` if abandoning A. Keep the prior cleaning overlay and evidence. No branch, worktree or evidence cleanup has been performed. Project documentation is synchronized; no global memory write was authorized or performed.

## Actual Gazebo 3D recording entry

No video was recorded, and no extra demonstration run was launched. The rental host has no `DISPLAY`, `WAYLAND_DISPLAY` or X11 socket directory; `ffmpeg` was not found on either the host or guest PATH. A working display/session and capture/encoder path remain prerequisites.

The existing Windows/WSLg entry for a real Gazebo window is:

```powershell
& 'F:\Project\TZcup\.workspace\worktrees\TZcup-simulation-only-completion\scripts\run_gazebo_cleaning_demo.ps1' -MapSize small -SimulationSpeed normal -DynamicObstacleTrials 1
```

This is a source-inspected launch entry, not an executed or verified video-production command. It requires the configured `TZcup-Ubuntu-24.04` WSL runtime and WSLg. It launches Gazebo-only native mission controls and explicitly disables dashboard video. Capture the actual Gazebo window with a configured screen recorder; demonstrate the vehicle, grounded rotating brushes, dirt disappearing, obstacle avoidance and physical E-stop in the resulting footage. The GUI Stop button alone is not evidence of the physical E-stop path. The small demo entry is not a replay of this localization run and does not establish any acceptance metric. Do not substitute RViz, a dashboard, a map pointer or a plot for the requested footage.
