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

## Run-10 admission recheck

**Status:** `FAIL_CLOSED / NOT_REPLAYABLE`

Run-10 closed normally with a 7.280473990-second MCAP, but it still cannot
enter offline SLAM replay. The copied input retained:

| Topic | Type | Messages |
|---|---|---:|
| `/scan` | `sensor_msgs/msg/LaserScan` | 28 |
| `/clock` | `rosgraph_msgs/msg/Clock` | 664 |
| `/odom` | `nav_msgs/msg/Odometry` | 33 |

Both required transform streams were absent:

| Required topic | Expected type | Result |
|---|---|---|
| `/tf` | `tf2_msgs/msg/TFMessage` | absent |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | absent |

The source bag was copied to an independent offline directory before audit.
The copy preserved both source hashes:

- MCAP: `4e871c6377c294f1b0ba991529b8200bede60730359b6af30c879da15e67b5dd`
- metadata: `f38728c13009722250267accc188618af8ba84e6e14f4edb436f889a37230685`

The fail-closed receipt is
`reports/mapping/day1_live_slam_run10_preflight_20260914.json` with SHA-256
`2cb5cc58ef16d1d79fe05f6460c34761dc26edf546d5fe9de24c5ac45f440cc4`.
The audit exits `2`, reports zero input errors, and identifies exactly
`/tf` and `/tf_static` as missing. No ROS node, Gazebo process, or
`slam_toolbox` process was started.

The audit now permits zero-message topic declarations in valid rosbag2
metadata while still requiring at least one message on every admitted replay
topic. Run-10 remains immutable evidence only; replay admission has moved to
run-11, which must record both transform topics in a normally closed bag.

## Run-11 offline replay

**Status:** `REPLAY_EXECUTED / MAP QUALITY GATE FAILED`

Run-11 closed with all five admission topics present:

| Topic | Type | Messages |
|---|---|---:|
| `/scan` | `sensor_msgs/msg/LaserScan` | 33 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 100 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 1 |
| `/clock` | `rosgraph_msgs/msg/Clock` | 820 |
| `/odom` | `nav_msgs/msg/Odometry` | 41 |

The admitted MCAP was copied to an independent offline directory. Its MCAP
SHA-256 is
`43144b144db0fd06eaaea637a72609541b726f20fe338de3fd5a85cc5b6c880e`
and its metadata SHA-256 is
`8de12b0abb715ed8abb469f0620a0bfa124bd61436c0feaa7b7b477b0c14bbf8`.

The existing `slam_toolbox` online-async mapping mode was replayed without
Gazebo on ROS domain `122` with localhost-only discovery. A `0.5x` replay
produced a PGM/YAML map with no queue-full message drop:

- `occupancy.pgm`: `c9012cd1b557e540528d081d584deb021bcca6037b05684f92b4659b0ff6db51`
- `occupancy.yaml`: `fcfc50b930c1f1caa4ffa2343f77987782c1a4dac21ca1ef39a696d7d9a13462`
- grid: `176 x 392` cells at `0.05 m`
- known area: `2.92 m2`
- occupied area: `0.055 m2`
- free area: `2.865 m2`
- unknown fraction: `0.9830707913966839`

The map is real, but the 20,000 m2 area gate is false. The project quality gate
also fails because the vertical span is `19.6 m` and the known area is below
`150 m2`. A `1.0x` comparison produced `2.79 m2`, `98.3865%` unknown, and one
queue-full drop, so the `0.5x` map is retained as the better replay result.

The complete machine-readable result is
`reports/mapping/day1_live_slam_run11_replay_20260914.json`. The replay used
`scripts/run_day1_offline_slam_replay.sh`; the PRoot `setsid` wrapper did not
return after map save even though all ROS children exited. The task-owned
wrapper process group was terminated after a read-only census. Run-12 remained
the sole Gazebo owner and was not affected.
