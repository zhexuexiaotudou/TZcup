# Formal campus runtime startup diagnosis and acceptance

Date: 2026-08-26

## Result

The formal 200 m x 100 m campus runtime passed a real single-host WSL startup
probe. Readiness was reached in 204.714 seconds with the initial emergency stop
still asserted. The launch then exited through its dedicated process group and
left no process carrying the test Gazebo partition or output path.

Runtime evidence is retained at:

- `.work/formal_campus_runtime_fixed_191/formal_campus_runtime_readiness.json`
- `.work/formal_campus_runtime_fixed_191/formal_campus.launch.log`

## Root cause

The failure was a startup/discovery race amplified by WSL DrvFS load. The old
launch materialized the large campus inputs, then started Gazebo, bridges,
controllers, Nav2, filters and coverage as roughly 30 DDS participants in one
scheduler tick. A separately started `ros2 ... --no-daemon` process could see
an empty graph while lifecycle managers waited for services. The same launch
later discovered `robot_description`, initialized `controller_manager`, and
activated lifecycle nodes, which rules out a persistent package, domain-ID,
remap or `robot_description` defect.

## Fix

`formal_campus.launch.py` now stages the participant groups: spawn initializer
at 8 seconds, controller spawners at 12 seconds, pedestrians at 15 seconds,
Nav2 at 20 seconds, and coverage at 45 seconds. Controller-manager discovery
and service/switch waits are explicitly bounded.

`scripts/run_formal_campus_runtime.sh` fixes the exact overlay source order,
validates inputs and domain range, selects LOCALHOST DDS discovery and a unique
Gazebo partition, starts the launch in a dedicated process group, and starts
the readiness participant before the delayed autonomy groups. It retains the
initial E-stop and propagates a nonzero validator or early-launch exit.

`scripts/validate_formal_campus_runtime.py` requires live discovery and samples;
it does not command motion or consume evaluator-private truth. Shutdown targets
only the launch process group: SIGINT, a bounded ten-second wait, then SIGKILL
for exact survivors. No broad `pkill` is used.

## Reproduction

From WSL in the repository root:

```bash
export ROS_DOMAIN_ID=191
export GZ_PARTITION=tzcup_formal_campus_fixed_191
export FORMAL_CAMPUS_OUTPUT_ROOT="$PWD/.work/formal_campus_runtime_fixed_191"
export FORMAL_CAMPUS_READINESS_TIMEOUT_S=240
bash scripts/run_formal_campus_runtime.sh
```

The runner internally sources, in order:

1. `/opt/ros/jazzy/setup.bash`
2. `.work/stage1_20260826_023716/install/setup.bash`
3. `/home/zhexu/tzcup_integrated_build_20260826_v3/install/setup.bash`
4. `/home/zhexu/tzcup_integrated_build_20260826_v3/install_formal_campus_agent/setup.bash`

## Recorded gates

- Required nodes: 7/7 discovered, including `controller_manager`, `map_server`,
  `amcl`, `coverage_server`, the adapter and safety manager.
- Controllers: joint-state, base, arm, gripper, cleaning and storage active;
  brush and recovery intentionally inactive.
- Lifecycle: map, AMCL, keepout/speed maps and filter-info servers, and coverage
  all active.
- Samples: map 1, odometry 66, 2D scan 76, MID360 point clouds 33.
- Initial E-stop: one observed sample, asserted; the runner never clears it.
- Discovery scope: `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, domain 191.

The acceptance claim is limited to deterministic single-host WSL startup and
readiness. It does not claim mission motion, cleaning execution, or evaluator
score acceptance.

## Residual teardown observation

After readiness passed, the process-group SIGINT exposed shutdown-only noise in
upstream nodes: `bt_navigator` reported an invalid ROS context and exited -11,
while `simulation_safety_inputs` and the topic adapter reported context-already-
shutdown exceptions. Most nodes reported clean exit, the runner returned zero,
and no process using the test partition or output path remained. This does not
invalidate the recorded steady-state readiness gates, but the evidence does not
claim that every upstream ROS node has exception-free simultaneous teardown.
