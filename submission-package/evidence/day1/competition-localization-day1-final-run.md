# Final bounded localization diagnostic

**Date:** 2026-09-14

**Code revision:** `06b99b6cb64e91ec470ac5623ded4c6db22df321`

**Status:** `NOT_MEASURED`

**Decision:** stopped permanently before dynamic data; no retry authorized.

## Config fix

The stale runtime underlay contained local and global costmap inflation radii of
0.55/0.55. The formal profile now declares
`nav2_inflation_radius_m: 0.56`, and materialization writes that verified value
into both costmaps while retaining the strict inset-plus-padding validator.

The final generated formal-campus config contained:

* local costmap `inflation_radius`: `0.56`
* global costmap `inflation_radius`: `0.56`

The five focused source suites passed `61` tests. The profile validator passed
with `nav2_footprint_padding_m=0.01` and `nav2_inflation_radius_m=0.56`.

## Run 02

Gazebo started under the formal lock. The launch passed the prior costmap
blocker. The run then stopped before bagging, TF authority capture, route
execution, or offline scoring because parameter capture returned
`all_expected=false`.

The only unmatched key was:

```text
/amcl global_frame_id: TimeoutExpired after 10.0 seconds
```

Every other required key matched, including `/amcl tf_broadcast=false`, both
EKF ownership/frame fields, `/odometry/gps`, and the NavSat ownership fields.
The runner exits on the sealed parameter mismatch, so no dynamic dataset was
created and the 50 mm criterion was not evaluated.

| Result | Value |
|---|---:|
| MCAP files | 0 |
| Dynamic reference samples | 0 |
| Strict paired samples | 0 |
| Route samples | 0 |
| TF authority reports | 0 |
| RMSE | null |
| P95 | null |
| Maximum error | null |
| Displacement | null |
| Pair coverage | null |

## Retained evidence

Raw run evidence:
`.work/day1-localization-run/evidence/run-02`

The raw directory preserves the generated formal-campus Nav2 YAML, effective
parameter JSON, AMCL and EKF parameter responses, launch/navigation logs, dirt
bridge log, and copied map masks. No MCAP or TF authority report exists.

Final resource check:

```text
SIM_RESOURCE_RELEASED=true
```

The formal Gazebo lock is free and no Gazebo or ROS runtime process remains.
No further run was attempted.
