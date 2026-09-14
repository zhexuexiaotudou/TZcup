# Day-one dynamic-avoidance recovery root-cause

**Status:** `ROOT_CAUSE_CLASSIFIED_READY_FOR_ONE_BOUNDED_RUN`

**Evidence scope:** bounded CPU, log, archive, and source review only. No
Gazebo or ROS runtime was started for this diagnosis.

## Conclusion

Run-19b, run-21, and run-22 all reached the scheduled interaction and produced
collision-monitor intervention with zero physical collision. The task did not
complete because the global `map` pose used by Nav2 diverged during recovery,
while AMCL and local odometry remained plausible.

The deterministic failure chain is:

1. A pedestrian interaction interrupts the planned path.
2. Nav2 enters recovery, including spin behavior while the vehicle is nearly
   stationary.
3. The cleaning-mode global EKF consumes `/odometry/gps`, whose map-frame
   transform is produced by `navsat_transform` from
   `/localization/fused_odom`, the global EKF's own output.
4. The feedback measurement rotates with the changing heading and pushes the
   global `map->odom` transform tens of metres away from the AMCL pose.
5. Nav2's global costmap sees the sensor/base origin outside the saved map and
   planner requests start outside bounds.
6. The controller and recovery chain abort; the goal is never completed.

The frame-feedback mechanism is proven for the existing localization
diagnostic and is the only mechanism consistent with the recovered logs. The
remaining uncertainty is the exact contribution of GNSS and IMU covariance
during spin; that uncertainty is why the repair is an opt-in bounded diagnostic
override rather than a global production tuning change.

## Evidence

| Run | Interaction evidence | Recovery evidence | Goal result |
|---|---|---|---|
| run-19b | selected candidates `1`, monitor interventions `1`, nearest scan `0.7500 m` | global fusion already inconsistent | not succeeded |
| run-20 | none | safety permit blocked before movement | action result `6` |
| run-21 | candidates `2`, monitor interventions `4` | three recovery cycles; AMCL ended at `(3.05498, 0.06727) m`; planner start became `(-0.29, 49.56) m` | timeout, not succeeded |
| run-22 | candidates `3`, monitor interventions `8` | seven recoveries; AMCL ended at `(2.27675, 0.02893) m`; planner start became `(0.78, -33.72)` then `(-71.67, -153.66) m` | aborted, status `6` |

Run-21 and run-22 both recorded:

* candidate timestamps in ROS simulation nanoseconds;
* safety enabled throughout the useful movement window;
* zero physical collisions;
* collision-monitor interventions and gated commands;
* AMCL and local odometry close to each other;
* global costmap origins tens to hundreds of metres outside the map.

Representative run-22 lines:

```text
planner_server: GridBased plugin failed to plan from (0.78, -33.72) to (6.00, 0.00)
planner_server: GridBased plugin failed to plan from (-71.67, -153.66) to (6.00, 0.00): Start Coordinates ... outside bounds
bt_navigator: [navigate_to_pose] [ActionServer] Aborting handle.
bt_navigator: Goal failed
```

The collision monitor was therefore not the primary blocker. It detected the
interaction, gated motion, and prevented physical contact. The failure occurred
because recovery moved a corrupted global frame, then planning could not
recover from that corrupted state.

## Minimal repair

The global EKF now accepts an explicit `global_gnss_odometry_topic` launch
override. Production defaults to `/odometry/gps`.

The day-one recovery harness sets:

```text
FORMAL_GLOBAL_GNSS_ODOMETRY_TOPIC=/odometry/gps_disabled
```

That topic has no publisher. `navsat_transform`, GNSS sensors, AMCL, local EKF,
saved-map localization, and `map->odom` ownership by `global_ekf` remain
enabled. Only the self-referential GNSS pose measurement is removed for the
bounded diagnostic run. This does not alter any collision threshold, safety
permission, route, evaluator criterion, or official `>=95%` definition.

## Single-run command

After preparing a fresh source tree and frozen runtime exactly as in run-22:

```bash
export FORMAL_DYNAMIC_SINGLE_RUN_ROOT=/root/autodl-tmp/tzcup-competition-sim-only-20260912/evidence/day1-dynamic-avoidance-recovery-20260914-23/run-23
export FORMAL_DYNAMIC_SINGLE_RUN_RUNTIME_WS=/root/autodl-tmp/tzcup-competition-sim-only-20260912/runtime/runtime-ws-1a211400-lifecycle-health-v4-r1-27cac7f7773f
export FORMAL_DYNAMIC_SINGLE_RUN_CLOSURE_MANIFEST=/root/autodl-tmp/tzcup-competition-sim-only-20260912/evidence/day1-dynamic-avoidance-recovery-20260914-23/final_runtime_closure_manifest_smoke_23.json
export FORMAL_DYNAMIC_SINGLE_RUN_EPISODE_ROOT=/root/autodl-tmp/tzcup-competition-sim-only-20260912/evidence/day1-mapping-run-20260914-06/episode
export FORMAL_DYNAMIC_MAP_SOURCE_MODE=OFFLINE_RAYCAST_MAPPING
export FORMAL_DYNAMIC_OFFLINE_MAP_SOURCE_ROOT=/workspace/tzcup-competition-sim-only-20260912/source/TZcup-day1-avoidance-run23-dynamic/artifacts/day1_dynamic_avoidance_offline_map_20260914/offline_map_source
export FORMAL_DYNAMIC_SINGLE_RUN_PROTOCOL=/workspace/tzcup-competition-sim-only-20260912/source/TZcup-day1-avoidance-run23-dynamic/config/dynamic_avoidance_functional_smoke_protocol.json
export FORMAL_VEHICLE_RUNTIME_WS=/workspace/tzcup-competition-sim-only-20260912/runtime/runtime-ws-1a211400-lifecycle-health-v4-r1-27cac7f7773f
export FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST=/root/autodl-tmp/tzcup-competition-sim-only-20260912/evidence/day1-dynamic-avoidance-recovery-20260914-23/final_runtime_closure_manifest_smoke_23.json
export FORMAL_DYNAMIC_EPISODE_ROOT=/workspace/tzcup-competition-sim-only-20260912/evidence/day1-mapping-run-20260914-06/episode
export FORMAL_ACCEPTANCE_SESSION_STATUS=/root/autodl-tmp/tzcup-competition-sim-only-20260912/evidence/day1-dynamic-avoidance-recovery-20260914-23/formal_acceptance_session_smoke_23.json
export FORMAL_VEHICLE_SNAPSHOT_MANIFEST=/workspace/tzcup-competition-sim-only-20260912/source/TZcup-day1-avoidance-run23-dynamic/reports/engineering/formal_vehicle_snapshot_manifest.json
export ROS_DOMAIN_ID=221
export GZ_PARTITION=tzcup_dynamic_avoidance_recovery_20260914_23
bash scripts/run_day1_avoidance_recovery_single_trial.sh
```

The harness refuses a non-empty run root, fixes the diagnostic GNSS override,
executes exactly one existing single-run wrapper, and then writes a fail-closed
receipt. A successful run is labelled
`FUNCTIONAL_PASS_1_OF_1_NOT_OFFICIAL_95`; the receipt keeps the official metric
at `NOT_MEASURED`.

## Resource contract

The run must have exactly one Gazebo owner and use the existing formal lock and
partition. The inner wrapper is responsible for process-group cleanup. Before
declaring success, require:

* `/tmp/tzcup_formal_gazebo.lock` is available;
* no process remains with the recovery partition or run root;
* the run root contains the timeline, runtime telemetry, environment telemetry,
  dynamic launch log, and `single_run_evaluation.json`;
* the receipt passes every functional gate and records no costmap drift or
  planner outside-bounds failure.

No second Gazebo trial is allowed for the same run root. Any retry needs a new
fresh run id and a new predeclared route manifest.
