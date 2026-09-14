# Competition localization root-cause diagnosis

**Date:** 2026-09-14  
**Baseline:** `0685688`  
**Branch:** `codex/day1-localization-diagnosis`  
**Scope:** static and offline diagnosis only. No Gazebo, ROS simulation, S100P,
dynamic run, or second acceptance run was started.

## Decision

**GO for one bounded dynamic diagnostic run of the patched cleaning
configuration.** The run must use `PROBE_FOCUS_VALIDATION=1`,
`PROBE_AMCL_TF_BROADCAST=false`, the isolated runtime wrapper, a fresh output
directory, per-key runtime parameter responses, and the offline focus scorer.

**NO-GO for any `<= 50 mm` or formal acceptance claim from existing evidence.**
The four strictly paired diagnostic samples are not an acceptance dataset. The
first run remains FAIL and the previously recorded maximum error remains
`12.5388507595 m`.

The exact Cyclone DDS message-GID-to-endpoint join remains unavailable. The
scorer therefore permits one narrow fallback: exactly one runtime `map->odom`
GID is attributable to `/global_ekf` only when a hash-bound static contract
proves that `/global_ekf` is the sole configured cleaning authority and the live
graph proves `/global_ekf` publishes `/localization/fused_odom` and subscribes
to `/odometry/gps`. Any other combination remains fail-closed.

## Proven facts

### 1. Configured cleaning TF ownership is unique

* `formal_fusion.yaml` sets `local_ekf.publish_tf=true` in `odom`, and
  `global_ekf.publish_tf=true` in `map`
  (`starter_ws/src/sanitation_localization/config/formal_fusion.yaml:7-21`,
  `:61-80`).
* `formal_vehicle_sim.launch.py` starts the one local EKF and one
  `navsat_transform`, while leaving global fusion disabled
  (`starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py:896-913`).
* `formal_campus.launch.py` adds the one cleaning-mode global EKF and explicitly
  disables the duplicate NavSat instance
  (`starter_ws/src/sanitation_formal_campus_integration/launch/formal_campus.launch.py:386-395`).
* The competition probe defaults AMCL `tf_broadcast=false`
  (`scripts/run_competition_motion_cleaning_probe.sh:13-14`, `:26-29`), and the
  formal map lifecycle also forces it false
  (`starter_ws/src/sanitation_formal_campus_integration/launch/formal_campus_map_lifecycle.launch.py:127-129`).
* NavSat UTM and Cartesian TF broadcasts are disabled
  (`starter_ws/src/sanitation_localization/config/formal_fusion.yaml:47-59`).

Therefore `/global_ekf` is the only configured cleaning publisher of
`map->odom`. This is a configuration fact. The original runtime owner identity
was not recovered because the Cyclone collector observed one message GID that
could not be joined to the discovered endpoint GID.

### 2. `/odometry/gps` is frame-dependent on its odometry input

Robot Localization 3.8.3 `navsat_transform.cpp` derives
`world_frame_id_` from `odometry/filtered.header.frame_id` at line 820 and
`base_link_frame_id_` from `child_frame_id` at line 821. It publishes
`/odometry/gps` only when both GPS and odometry updates are present at line
895. The output frame and stamp are copied at lines 554-555.

The old unconditional remap fed `/odom` to NavSat, so `/odometry/gps` was in
`odom`. The global EKF works in `map` and consumes that topic, creating a
self-referential measurement transform through the EKF's own `map->odom`
output. The replay candidate changed the input to
`/localization/fused_odom`, which makes `/odometry/gps` map-referenced.

The retained candidate introduced a separate mapping regression: mapping
disables global fusion, so `/localization/fused_odom` does not exist. NavSat
then receives no odometry, the source gate at line 895 remains false, and the
mapping consistency consumer still expects `/odometry/gps`.

The fix makes the NavSat input mode-dependent:

* `slam` mapping selects `/odom`;
* `amcl` cleaning selects `/localization/fused_odom`.

See `starter_ws/src/sanitation_formal_campus_integration/sanitation_formal_campus_integration/contract.py:29-41`,
`starter_ws/src/sanitation_formal_campus_integration/launch/formal_campus.launch.py:224-229`,
`starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py:53-60`,
and `starter_ws/src/sanitation_localization/launch/formal_localization_fusion.launch.py:14-31`.

### 3. The existing sample count is not acceptance evidence

The prior run had 359 dynamic reference epochs but only 4 strictly paired
diagnostic samples. Its displacement was 1.668454 m, below the required 2 m.
The scorer still enforces at least 100 pairs, at least 95% coverage, strict
original timestamps, and a 50 mm maximum error.

## Supported hypotheses, not proven causes

### Frame feedback caused the baseline drift

The replay reproduces meter-scale lateral growth with the `odom`-frame NavSat
input and removes it with the map-frame input. The source mechanism above
explains why. The replay changed frame selection, global frequency, and
transform timeout together, so the individual causal contribution is not
isolated.

### Zero GNSS covariance can make the GPS update overly strong

The formal topic adapter preserves message contents and frames unchanged
(`starter_ws/src/sanitation_formal_campus_integration/sanitation_formal_campus_integration/topic_adapter.py:16-20`).
Robot Localization copies GNSS covariance into `/odometry/gps`
(`navsat_transform.cpp:747`, `:934-935`). The library contains explicit
zero/negative-diagonal covariance diagnostics at
`ros_filter.cpp:2615-2635`, but this evidence does not establish that zero
covariance caused the observed drift. It remains a runtime check, not a tuning
target.

The in-repository `sanitation_gnss_sim` assigns nonzero planar covariance
(`starter_ws/src/sanitation_gnss_sim/sanitation_gnss_sim/node.py:97-100`), but
the formal campus path uses the Gazebo native NavSat bridge. No repository code
is proven to assign the bridge message's covariance.

### Clock pairing may hide or misclassify TF freshness

`competition_localization_focus.py` uses original message timestamps and does
not shift or interpolate across unbounded gaps. It records `tf_before_clock` and
bounds future/stale TF at 50 ms/100 ms. The four-sample run is still invalid
because the acceptance denominator and motion precondition were not met. No
clock-pairing defect is proven.

## Implemented changes

* Added a ROS-independent backend-to-NavSat selector.
* Made the fusion launch accept the NavSat odometry source instead of
  hard-coding the cleaning candidate.
* Wired the selector through `formal_vehicle_sim` and `formal_campus`.
* Added a static authority contract to the offline scorer. It can attribute a
  single unjoinable Cyclone GID only when the configured owner and live global
  EKF graph both independently agree.
* Added a per-key parameter collector for `/local_ekf`, `/global_ekf`, `/amcl`,
  and `/navsat_transform`; the scorer now requires its sealed, all-matched JSON.
* Added focused tests for mapping/cleaning selection, TF ownership, and the
  static-contract fallback, plus parameter parsing and fail-closed tests.

## Verification

Commands and results are recorded in the task handoff. The focused suites
passed (`44 passed`), Python compilation passed, and `git diff --check` passed.
A broader localization/campus run passed `290` tests with `2` skipped after excluding
`test_frontier_global_costmap_window.py`; that excluded file has an unrelated
pre-existing test-fixture `NameError: Bool is not defined`.

## Next-run preconditions

1. Build only from the committed branch revision in this worktree.
2. Run one cleaning-mode diagnostic in the isolated domain/partition.
3. Keep `PROBE_AMCL_TF_BROADCAST=false`.
4. Capture per-key effective parameters for `/global_ekf`, `/local_ekf`,
   `/amcl`, and `/navsat_transform`; bulk parameter dumps alone are insufficient.
5. Require `/localization/fused_odom` to be `map -> base_footprint`,
   `/odometry/gps` to be `map` with an empty child frame, and
   `/odom` to be `odom -> base_footprint`.
6. Require the static authority contract, one runtime `map->odom` GID, live
   `/global_ekf`, and the exact offline pair/coverage gates before scoring.
7. Preserve `primary.rc`, collector output, MCAP hashes, parameter responses,
   source binding, and the rollback revision even on failure.
