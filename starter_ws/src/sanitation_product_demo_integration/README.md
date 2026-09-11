# Formal product demo integration

This package is the sole top-level composition point for the full PC demo. It
starts the formal campus vehicle and Nav2, then the real DOSOD + EdgeSAM product
adapter, the frozen active-cleaning policy, and the contact-gated physical
UR5e/Robotiq grasp-and-bin executor. The ordinary coverage server is disabled
in this mode so there is only one planning owner.

The vehicle remains fail-closed with main power off and E-stop asserted after
launch. After the gate reports `control_owners_ready=true`, an operator must
publish one fresh low-to-high `/product_demo/operator_start` command. The gate,
not the operator, then continuously issues the main-power, E-stop and reset
commands. Model and policy paths are mandatory; placeholders are rejected.
The exact-map successful FullCoverage distance is mandatory as a hard task
distance budget. Return-to-start distance is reported separately and is not
charged to the cleaning-task metric.

This is a subsequent-task entry, not a first-map shortcut. The required
`saved_map_artifact_dir` must come from
`formal_campus_map_lifecycle.launch.py mission_mode:=mapping`. Product demo
always composes the same lifecycle with `mission_mode:=cleaning`; before AMCL
or Nav2 starts it validates the exact map ID, `observed_fraction >= 0.95`,
truth-boundary flags and SHA-256 hashes for every saved map/support artifact.
There is no fallback to the legacy world-derived occupancy map.

```bash
ros2 launch sanitation_product_demo_integration product_demo.launch.py \
  world:=/tmp/episode/public/world.sdf \
  episode_manifest:=/tmp/episode/public/episode_manifest.json \
  saved_map_artifact_dir:=/tmp/formal-first-map \
  perception_artifact_root:=/tmp/perception-artifacts \
  policy_checkpoint:=/tmp/policy.json maximum_task_distance_m:=18000 \
  episode_seed:=823873385
```

After all readiness diagnostics are green, the operator arms the simulation
with one latched command:

```bash
ros2 topic pub --once /product_demo/operator_start std_msgs/msg/Bool '{data: true}'
```

The gate requires fresh, independently identified planner and executor
heartbeats. Its transient-local status includes `control_owners_ready`, a gate
`instance_id`, monotonically increasing `sequence`, and
`published_at_unix_ns`; a runner must accept only a newly received, fresh
status from one instance. The pre-arm readiness means the three-second start
window is not consumed while the runtime is still initialising. After the reset sequence, only a fresh true
`/safety/actuators_enabled` receipt confirms `/product_demo/operator_armed`.
Loss of either owner or of a previously confirmed permit immediately opens main
power, asserts E-stop and requires a new low-to-high operator request. Gate
death is also covered by the simulation input watchdog. Mission completion is
published only after return to the fixed start, and automatically returns the
simulated vehicle to the safe state. These simulated ROS receipts are not
physical interlock or braking acceptance.

## One-episode end-to-end acceptance

`scripts/run_formal_single_episode_cleaning_mission.sh` is the only supported
way to claim the integrated mission gate. It starts one `product_demo` launch
(and therefore one Gazebo cleaning process), runs one observation-only
collector, and then aggregates and validates that same live run. The episode
ID, episode seed, frozen-session identity, Gazebo launch PID, ROS domain and
Gazebo partition are copied into every metric-source row. A source row with a
different identity, zero live samples, a changed input hash, or a historical
artifact class fails closed.

Product sources are planner diagnostics, mission completion, paths, grasp
results and odometry. Ground-dirt contact/cleaned area, water balance, physical
dry-bin contents and pedestrian/collision evidence use separate one-way
evaluator bridges under `/evaluation/single_episode/*`; these names are not
consumed by any product node. The runner never teleports the vehicle and sends
only the public `/product_demo/operator_start` command. The output directory
retains `raw_collection.json`, `aggregate.json`, `validation.json` and logs.

The live gate remains pending until a generated episode publicly records all
four cube material classes and a successful same-map FullCoverage report binds
the exact map ID and task-distance ceiling. Return distance is read from the
planner's frozen task-completion boundary and excluded from efficiency.
