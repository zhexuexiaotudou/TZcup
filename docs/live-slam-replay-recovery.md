# Day-1 live-SLAM replay recovery

**Status:** `FAIL_CLOSED / NOT_REPLAYABLE`

**Run-06 source revision:** `6021e7b52ebf6a56aa7113b7b6d7742cbdfe0692`

**Remote run root:**
`/root/autodl-tmp/tzcup-competition-sim-only-20260912/evidence/day1-mapping-run-20260914-06`

**Fail-closed receipt:**
`reports/mapping/day1_live_slam_replay_preflight_20260914.json`

## Decision

The run-06 timeout retained two closed MCAP files, but neither contains a
two-dimensional LiDAR scan, a point cloud, or any TF stream. The configured
`slam_toolbox` mapping mode subscribes to `/scan` and requires a resolvable
`odom -> base` transform. An odometry-only MCAP cannot be replayed as SLAM,
and the existing offline raycast reconstruction cannot be upgraded to live
SLAM merely because it computes a similar area.

The 20,000 m2 area gate therefore remains `NOT_MEASURED` for live SLAM. This
record does not change the status of the offline raycast result or the
20,000 m2 lifecycle field sample.

## Retained input hashes

| Role | MCAP bytes | MCAP SHA-256 | Metadata SHA-256 |
|---|---:|---|---|
| `early_recording_audit` | `395927215` | `754c867d638c9cbcd7b7528e420a0392a79b9d5fb1c28b8d4b22b57f5f5fc672` | `487af1251703a9278067a58d047c43f92e602a7d119fc5f8043f58ecf5662bf7` |
| `mapping_localization_diagnostic` | `57608905` | `fb95d12e5d901389631b2dd5719d88c198fb54985ff68412d8a4067879fcc503` | `b2db23f8769834ca630ff7500b78084ae14347e10c8d181ce537e0560263f559` |

Both files have a valid MCAP header/footer and closed rosbag2 metadata:

- early bag: 565,089 messages, 3,570.477621837 s
- localization bag: 58,538 messages, 3,571.421521860 s

## Topic coverage

The combined recorded topic set is:

- `/clock`
- `/base_controller/cmd_vel`
- `/formal_mapping/lifecycle_status`
- `/formal_vehicle/auxiliary/critical_safety_relay_diagnostic_json`
- `/gnss/fix`
- `/odom`
- `/odom/unfiltered`
- `/odometry/gps`
- `/safety/relay_cycle_diagnostic_json`
- `/safety/status_json`
- `/ground_truth/odom` in the localization-only bag

Missing for the configured SLAM replay:

| Required topic | Expected type | Result |
|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | absent |
| `/tf` | `tf2_msgs/msg/TFMessage` | absent |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | absent |

The run did not retain a `.pgm`, `.yaml` occupancy map, pose graph, or other
SLAM serialization checkpoint. The only PGMs are the pre-run
`geofence_keepout.pgm` and `neutral_speed.pgm` masks; they are not SLAM maps.

## Audit command

```powershell
py -3 scripts/audit_live_slam_replay_inputs.py `
  --bag-dir "$RUN_ROOT/saved-map/early_recording_audit" `
  --bag-dir "$RUN_ROOT/saved-map/mapping_localization_diagnostic" `
  --run-root "$RUN_ROOT" `
  --source-revision 6021e7b52ebf6a56aa7113b7b6d7742cbdfe0692 `
  --output reports/mapping/day1_live_slam_replay_preflight_20260914.json `
  --require-pass
```

The command exits `2` when the input is not replayable. That is the expected
run-06 result. It never starts Gazebo, ROS, or `slam_toolbox`.

The checked-in receipt was generated on the rental host inside the validated
Noble PRoot rootfs. Its SHA-256 is
`00e551436efbe961bbd18d5e29891f0a612599c99e8c35b5d500697e68e16407`;
the recorded audit exit code is `2`. The audit script hash bound by the receipt
is `61e6435e35fccbf195b2b9b542fc5bf4d5e68ec3b6e46f77a983627f9149fc10`.

## Shortest valid recovery path

A future mapping run must record a dedicated closed MCAP containing at least
`/scan`, `/tf`, `/tf_static`, `/clock`, and `/odom` while online SLAM is
active. The recording must close normally before the Gazebo session is
destroyed, and its metadata, data SHA-256, source revision, runtime closure,
and Gazebo partition must remain bound to one fresh run root.

That recording can then be replayed without Gazebo, with `use_sim_time=true`,
using the existing `slam.yaml`, followed by an occupancy-map save. Only the
resulting `occupancy.pgm`/`occupancy.yaml` pair may be passed to
`scripts/verify_map_area.py` for the 20,000 m2 area gate. A new live run must
wait for exclusive Gazebo ownership and must not be started by this recovery
worker.
