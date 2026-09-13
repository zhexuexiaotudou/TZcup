# Localization replay gate — 2026-09-13

**NO-GO for the second dynamic run.** No Gazebo was started during this phase. The original run remains FAIL: 4/359 strict pairs, max 12.5388507595 m, displacement 1.668454 m. The last dynamic opportunity has not been consumed.

## Scoped candidate and provenance

Applied only the handoff's two fusion files and three changes: navsat filtered-odometry input `/odom` → `/localization/fused_odom`; global EKF target frequency 30 → 50 Hz; global transform timeout 0.10 → 0 seconds. No datum, yaw, magnetic declination, fusion weights, GT, scoring thresholds, cleaning, perception or mapping changes.

The actual installed baseline YAML and launch file were copied and hashed before application. They exactly match the handoff baseline hashes `520ece15…` and `a259ec90…`. The new installed overlay files match candidate hashes `75e7d64d…` and `da0229db…`. Full paths/hashes are in `installed-baseline-binding.json` and `installed-candidate-binding.json`.

Only `sanitation_localization` was rebuilt into a separate overlay; build succeeded. Six handoff tests and the existing 29 focused localization tests passed. Actual apply → reverse-check → reverse → apply-check → apply succeeded; `git diff --check` passed. Previously accepted cleaning changes remain intact.

## Actual replay parameters and CLI

Separate domains 94 and 95 replayed the same sealed inputs through baseline and candidate global EKF/navsat processes. Neither local EKF nor Gazebo/Nav2 was started. Replayed topics were `/clock`, `/odom`, `/imu/data`, `/gnss/fix`, `/amcl_pose`, `/tf_static`; the odom→base_footprint transform was reconstructed from the recorded `/odom`. Old global TF, old fused output and GT were excluded from fusion inputs. Original message timestamps were preserved. Wall playback rate was 0.25×; this is a diagnostic replay, not an equivalent live runtime load.

`effective-parameters.json` contains per-key service responses; optional failures do not erase other values. Baseline global frequency/timeout are 30 Hz/0.1 s; candidate values are 50 Hz/0 s. Candidate CLI confirms navsat remaps `odometry/filtered` to `/localization/fused_odom` and uses the new installed YAML. Navsat service values include frequency 20 Hz, delay 0, magnetic declination 0, yaw offset 0, zero altitude true, odometry yaw false, wait-for-datum false and transform timeout 0. The optional `datum` request returned no value; the original run's precise startup datum cannot be recovered from this bag.

The prior run's empty bulk dumps were accompanied by uninitialized optional `imu1` / `imu0` warnings; they are not evidence of missing configuration. This phase captures new replay processes, not retroactive service snapshots of the terminated original processes. No local EKF service snapshot is claimed because local EKF was intentionally not running.

Installed package metadata: robot_localization 3.8.3, rclcpp 28.1.21, rclpy 7.1.11, rmw_cyclonedds_cpp 2.2.3, rmw_fastrtps_cpp 8.4.4. Package XML provenance is retained.

## Replay observations

| Observation | Baseline replay | Candidate replay |
|---|---:|---:|
| Fused samples | 95 | 475 |
| GNSS odometry frame | odom | map |
| Maximum absolute fused y | 1.706496 m | 0.034300 m |
| Final fused x/y | −1.988361 / −1.706496 m | 1.659022 / 0.028707 m |
| Median output interval | 0.101 s | 0.020 s |
| Maximum output interval | 0.231 s | 0.020 s |

This reproduces meter-scale growth in the baseline and its disappearance in the candidate over the available interval, supporting the measurement-frame/timing candidate. The combined three-change experiment does not isolate the causal contribution of each edit. The source bag starts after startup and ends near 13 s; neither 45 s behavior nor full 60 s dynamics was tested. Absolute y is a drift indicator, **not planar GT error or a replacement for the map-pose accuracy metric**. Candidate map TF accuracy and full strict-pair coverage are not certified.

## Binding reason for NO-GO: TF identity

The existing native C++ collector was calibrated with a known single map→odom publisher named `/known_single_tf_publisher`:

* Cyclone publisher and collector: 296 received messages; message GID `4b3481990072d3120000000000000000`, discovered TF endpoint `0110f8ea7961deb7184ccd9c00001603`. Exact lookup fails even with a known sole publisher.
* Fast DDS publisher and collector: 283 messages; message and endpoint GID both `010f66e8b928169a0000000000001303`; exact node binding succeeds.
* Cyclone publisher with Fast DDS collector under the current localhost discovery configuration: no messages or endpoints discovered. This does not establish general cross-vendor incompatibility; it does mean this attempted instrumentation path cannot identify the current runtime's TF owner.

Therefore the candidate's actual global TF owner remains unverified. A single received GID is not proof of node ownership, and this result does not demonstrate duplicate TF publishers. No approximate GID matching or guessed endpoint substitution was used. Moving the whole runtime to another RMW or implementing Cyclone publication-handle resolution would exceed the authorized two-file fusion candidate; neither was done. The dynamic run is withheld as required.

## Retained instrumentation failures

Initial probe setup attempted parameter queries before the clock was available; it was stopped and corrected to seed only the first recorded clock before inspection. The next setup used MCAP dynamic objects directly with rclpy and failed before replay; corrected code deserializes original CDR bytes into native ROS messages. Python GID attempts exposed that this rclpy API provides dictionary metadata without `publisher_gid`; final identity conclusions come from the existing C++ collector, not those failed Python probes. All attempts and logs remain in the evidence archive. No failed attempt was counted as a dynamic run or an acceptance pass.

The valid baseline/candidate replay processes completed with return codes `[0, 0]`. Task-created ROS CLI daemons were stopped after recording; no other project process was targeted. Final domain 94/95/96 census is empty and the formal Gazebo lock is available: **SIM_RESOURCE_RELEASED=true**, at 2026-09-13 08:12:10 UTC.

## Evidence, retained state and next step

Cloud evidence root: `/root/autodl-tmp/tzcup-competition-sim-only-20260912/evidence/localization-50mm-replay-20260913-01`. Local copy: `.work/localization-round2-handoff/replay-evidence`. Archive contains actual baseline/candidate files, full hashes, effective parameters, node CLI, navsat/bulk dumps, traces, all failed probe logs, native calibration outputs, software versions and resource release. Original MCAP remains in the unchanged first-run archive and is hash-bound by each replay record.

The candidate is retained as an **unaccepted** change in the isolated task worktree. Runtime rollback is to omit the new localization overlay and retain the prior runtime plus cleaning overlay. Source rollback is the handoff patch's reverse application after reverse-check, without undoing prior cleaning commits. No worktree/branch/evidence cleanup, PR, full CI, deployment, video or global memory write occurred. Project closeout documentation is synchronized.

Next work is a bounded solution for exact TF publisher identity under the existing Cyclone runtime, followed by rechecking the GO conditions. Until that succeeds, do not launch the remaining dynamic trial. Even if ownership is resolved, this replay alone does not prove max ≤50 mm.
