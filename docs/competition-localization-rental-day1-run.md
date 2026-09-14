# Rental-host localization diagnostic

**Date:** 2026-09-14

**Dispatched revision:** `7410988b29868c04ccab4d51fc9d8a8825623b6c`

**Status:** `NOT_MEASURED`

**Decision:** no further dynamic retry; preserve the remote pre-dynamic blocker.

## Host and runtime

The run executed on the existing AutoDL rental host:

* container: `autodl-container-ca410red1k-e4e33c0c`
* GPU: NVIDIA GeForce RTX 3080 Ti, 12288 MiB
* canonical root: `/root/autodl-tmp/tzcup-competition-sim-only-20260912`
* runtime: `/root/autodl-tmp/tzcup-competition-sim-only-20260912/runtime/runtime-ws-1a211400-lifecycle-health-v4-r1-27cac7f7773f`
* fresh run root: `/root/autodl-tmp/tzcup-competition-sim-only-20260912/evidence/localization-day1-20260914-01`
* fresh XDG runtime: `/tmp/tzcup_localization_day1_20260914_rental_01_xdg`
* ROS domain: `97`
* Gazebo partition: `tzcup_localization_day1_20260914_rental_01`

The canonical runtime was reused. A required overlay was built only for the
localization/campus/vehicle/cleaning source delta. The orphaned bridge PID
`829511` was inspected read-only and left untouched; it belongs to
`ROS_DOMAIN_ID=99` and `GZ_PARTITION=tzcup_competition_dirt_99`.

## 0.56 radius verification

The generated run `nav2.yaml` contained:

* local costmap `inflation_radius`: `0.56`
* global costmap `inflation_radius`: `0.56`

The remote materialization contract tests passed, including the override of a
stale 0.55 underlay with the verified 0.56 profile value. The validator was not
relaxed.

## AMCL capture blocker

The remote capture completed with `all_expected=false`. Every one of the 20
required parameters failed all three attempts because the retry command passed
`5.0` to `ros2 param get --timeout`, which accepts only an integer:

```text
ros2 param get: error: argument --timeout: invalid int value: '5.0'
```

This includes `/amcl global_frame_id`, `/amcl tf_broadcast`, both EKF node
parameter sets, and `/navsat_transform`. The runner exited with return code `2`
before creating a bag, TF authority report, route result, or scorer result.

The type bug was corrected after the run in commit
`a2ffa243040a7b6ed7bf720c53df2d73c7f978f6`; no second dynamic run was made.

## Results

| Result | Value |
|---|---:|
| MCAP files | `0` |
| Dynamic reference samples | `0` |
| Strict paired samples | `0` |
| TF authority reports | `0` |
| Route samples | `0` |
| RMSE | `null` |
| P95 | `null` |
| Maximum error | `null` |
| Displacement | `null` |
| Pair coverage | `null` |

GPU samples during the host run: 123 samples, average utilization `0.984%`,
maximum `4%`, maximum memory use `334 MiB`. Final GPU state was `0%`, `1 MiB`.

## Resource release

The remote formal Gazebo lock was available after the run and no domain-97
runtime process remained. The local WSL lock was also free. The domain-99
orphan bridge remains untouched for separate ownership review.

Remote evidence is retained at the run root above and in the downloaded local
archive:

`.work/day1-localization-run/evidence/remote-day1-20260914-01.tar.gz`
