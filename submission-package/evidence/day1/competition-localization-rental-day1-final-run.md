# Rental-host localization final diagnostic

**Date:** 2026-09-14

**Evidence commit:** `47e3cb3`

**Revision:** `a2ffa243040a7b6ed7bf720c53df2d73c7f978f6`

**Status:** `FAIL`

**Failure:** `map_max_error_over_50mm_or_missing`

## Host and run

The diagnostic ran on the existing AutoDL rental host:

* container: `autodl-container-ca410red1k-e4e33c0c`
* GPU: NVIDIA GeForce RTX 3080 Ti, 12288 MiB
* canonical root: `/root/autodl-tmp/tzcup-competition-sim-only-20260912`
* runtime: `/root/autodl-tmp/tzcup-competition-sim-only-20260912/runtime/runtime-ws-1a211400-lifecycle-health-v4-r1-27cac7f7773f`
* run root: `/root/autodl-tmp/tzcup-competition-sim-only-20260912/evidence/localization-day1-final-20260914-02`
* output: `.../run-01`
* ROS domain: `95`
* Gazebo partition: `tzcup_localization_day1_final_20260914_02`
* XDG runtime: `/tmp/tzcup_localization_day1_final_20260914_02_xdg`

The existing runtime and persisted dependencies were reused. Only the corrected
capture script from `a2ffa24` was copied into the fresh source tree; no package
rebuild was required.

## Preconditions

All 20 required runtime parameters matched on the first attempt:

* `/amcl global_frame_id`: `map`
* `/amcl odom_frame_id`: `odom`
* `/amcl tf_broadcast`: `false`
* both EKF ownership/frame contracts: passed
* all NavSat ownership flags: passed

The generated local and global costmap inflation radii were `0.56` and `0.56`.
The route completed both Nav2 goals with results `[4, 4]`. Ground truth was not
used for control.

## Scored result

| Metric | Value |
|---|---:|
| Dynamic reference samples | 2711 |
| Strict paired samples | 2711 |
| Pair coverage | 1.0 |
| Displacement | 5.91568862847169 m |
| Path length | 11.762108455660483 m |
| RMSE | 0.039271209503686726 m |
| P95 | 0.08377985790742562 m |
| Maximum error | 0.12632780257795784 m |
| Fused-only maximum error | 0.12605500679362278 m |
| Fused-vs-TF-chain maximum | 0.010183036666251586 m |
| TF future maximum | 0.0010000000000047748 s |
| TF stale maximum | 0.0 s |

The sole predeclared-criterion failure is the maximum map-pose error exceeding
`0.05 m`. The official brief asks for RMSE, P95, and maximum error but does not
state which statistic defines the `<=50 mm` gate. Under an alternative RMSE
interpretation the run is `PASS_PARTIAL`; this does not establish an official
PASS.

## MCAP and resources

The sealed MCAP contains 194,064 messages over 549.157930504 seconds and is
129.9 MiB. It is not copied into this package. GPU/CPU monitoring recorded:

* CPU average/minimum/maximum: `34.356% / 21.618% / 42.124%`
* GPU average/minimum/maximum: `3.475% / 0% / 87%`
* Maximum GPU memory: `3004 MiB`

The remote formal Gazebo lock was available after the run, no domain-95 runtime
process remained, and the final GPU state was idle. Orphan PID `829511` remains
untouched on domain 99.

The full evidence remains on the rental host. A compact result archive is
committed under `artifacts/day1_localization_20260914/`.
