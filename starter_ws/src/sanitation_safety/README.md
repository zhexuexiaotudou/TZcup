# Formal whole-vehicle actuator interlock contract

`whole_vehicle_safety_manager` is the sole command gateway for the formal
vehicle's base, brush and recovery-pump controllers.  Its power-up state is
inhibited.  A permit requires fresh front/rear bumper, safety-relay, BMS-fault,
traction-permit and control-heartbeat inputs with the manual emergency stop
released. The BMS fault and traction permit have independent watchdogs; neither
may be inferred from the other.

The cleaning-motor observer's
`/model/tzcup_formal_sanitation_vehicle/cleaning_motors/fault_active` is also a
required fresh input. Missing, stale, overtemperature, or latched-stall state
inhibits every actuator through the same atomic decision. The observer never
writes a joint command, so `gz_ros2_control` remains the only physical joint
authority; the electrical/thermal contract is documented in
`docs/cleaning-actuator-motor-realism.md`.

The formal launch integration must satisfy all of these conditions:

1. Complete the formal controller spawner first, then start
   `whole_vehicle_safety_manager` in the same ROS domain as
   `/controller_manager`.  This ordering guarantees that the position
   controllers are active before the manager takes ownership of their hold
   commands.
2. The manager is the only publisher to `/base_controller/cmd_vel`,
   `/brush_controller/commands`, `/recovery_controller/commands`,
   `/cleaning_controller/joint_trajectory`,
   `/arm_controller/joint_trajectory`, and
   `/gripper_controller/joint_trajectory`. While inhibited it also reasserts a
   fixed zero target on `/service_controller/joint_trajectory`; other position
   controllers hold their captured joint positions. Producers
   publish base commands to `/cmd_vel_gate`, three-element brush velocity arrays
   to `/safety/command/brush`, and one-element pump velocity arrays to
   `/safety/command/pump`.  Existing formal acceptance scripts that publish
   brush/pump commands must use or remap to these two safety input topics.
   Cleaning-lift, arm and gripper producers must use their standard action
   clients rather than publish directly to controller trajectory topics.
3. Keep the standard trajectory action names unchanged:
   `/cleaning_controller/follow_joint_trajectory`,
   `/arm_controller/follow_joint_trajectory`, and
   `/gripper_controller/follow_joint_trajectory`, plus the storage trajectory
   action. On inhibition the manager
   repeatedly cancels all goals and publishes short trajectories at the current
   `/joint_states` positions.  Keeping these position controllers active avoids
   an unpowered Gazebo gravity drop while the manager provides a software brake.
   Normal `FollowJointTrajectory` clients need no remap and retain the
   controller's native result and tolerance behavior when permitted.
4. The manager owns the active/inactive lifecycle only for
   `brush_controller` and `recovery_controller`.  It repeatedly deactivates
   those velocity controllers while inhibited and reactivates them when
   permitted.  Do not externally reactivate them after manager startup.
   `cleaning_controller`, `arm_controller`, `gripper_controller`,
   `storage_controller`, and `service_controller` remain
   active so zero velocity and position-hold commands have an executing
   controller. There is no ros2_control `base_controller`; the A300 command
   adapter consumes the manager's `/base_controller/cmd_vel` safety boundary.
5. If any UR5e joint leaves the transport anchor, or the manipulation executor
   asserts `/manipulation/base_motion_inhibited`, the manager outputs a zero
   base command and reports `manipulator_base_inhibit` while keeping actuator
   power and the arm controller enabled. This lets the arm finish or retreat
   safely without allowing simultaneous vehicle motion.

## Wastewater service-drain safety

`service_drain_safety_manager` starts only after the safe controller loading
sequence. It can open `wastewater_drain_valve_joint` only when the request,
stationary-wheel state, stopped cleaning joints, stopped recovery pump, open
physical service cap, connected hose, valid tank level, global actuator permit
and high-power branch are all fresh and true. The cap state is derived from
`wastewater_drain_service_cap_joint` in `/joint_states`; hose presence is
derived from non-empty `ros_gz_interfaces/Contacts` on the dedicated
`/formal_vehicle/service/raw/drain_hose_contact` coupling sensor. Missing,
stale, false or clock-rollback input commands the joint to zero and publishes
`service_drain_open=false` to the water-recovery plant. The plant has its own
0.25 s command watchdog so loss of the ROS manager cannot leave drainage
latched open.

The physical drain command is a product ROS-to-Gazebo bridge. Ground-water
truth, tank reset and filter fault injection remain evaluator-only interfaces.

The machine acceptance entry point is:

```bash
python scripts/validate_whole_vehicle_actuator_interlock.py \
  --output /tmp/whole_vehicle_actuator_interlock.json
```

Run it against a live formal vehicle simulation after the launch wiring above
is present.  It proves locked zero brush/pump output, velocity-controller
deactivation, normal forwarding after enable, cancellation of a live arm goal,
and bounded arm/gripper/cleaning drift throughout a second fail-closed window.

## Simulation safety inputs and auxiliary product state

The `simulation_safety_inputs` executable is the formal simulation's healthy
power/relay input source. It subscribes to the measured, latched
`/emergency_stop` emitted only by the physical Gazebo auxiliary bridge, and
continuously publishes `/safety/relay_enabled` and
`/safety/control_heartbeat`. Stopping
the node therefore removes the relay and heartbeat refresh and the independent
whole-vehicle manager closes on its normal input timeouts.  The initial estop
fallback is active, while the physical plugin itself also starts latched unless
launch explicitly overrides `simulation_initial_estop_active`.
The relay output means actuator-enable contact, not just relay-coil supply: it
closes only with main power requested, E-stop released, no service charging and
sufficient simulated SOC.  Main power off therefore cannot permit motion.

Simulation-only commands are deliberately namespaced away from product state:

- `/formal_vehicle/simulation/command/emergency_stop`
- `/formal_vehicle/simulation/command/main_power`

The emergency-stop command is bridged into the physical plugin, which owns the
latch, 6 mm plunger state and reset rules. The measured `/emergency_stop` state
and the main-power command must both refresh within 0.5 s at this adapter;
staleness opens the relay and main-power request. Clearing requires a false
external request followed by the separate
`/formal_vehicle/simulation/command/emergency_stop_reset` request while safety
power is available and the physical plunger is released.

Gazebo bumper collision events enter on the simulation-only raw topics
`/formal_vehicle/simulation/raw/{front,rear}_bumper/contact`.  Gazebo publishes
events while colliding but does not provide a periodic clear state, so this
node acts like the real normally-closed bumper interface: while each raw bridge
publisher is present it emits a 20 Hz clear/contact state on the product
`/safety/{front,rear}_bumper/contact` topics and briefly latches live contact
events.  If either bridge disappears, that product heartbeat stops and the
independent safety manager inhibits all actuators on its bumper timeout.
- `/formal_vehicle/simulation/command/charge_connected`
- `/formal_vehicle/simulation/command/work_lights`
- `/formal_vehicle/simulation/command/tail_lights`
- `/formal_vehicle/simulation/command/warning_lights`

Product-facing outputs are:

- `/formal_vehicle/power/charge_requested`,
  and `/formal_vehicle/power/main_power_requested` (simulation request boundary);
- `/formal_vehicle/power/load_request_w` (aggregate auxiliary demand);
- `/formal_vehicle/power/branches/{safety,low_voltage,high_power}/enabled`;
- `/formal_vehicle/lighting/{work_lights_on,tail_lights_on,warning_lights_on}`;
- `/formal_vehicle/auxiliary/status_json`.

`simulation_safety_inputs` does not own the final BMS or charge-interface state.
It consumes `/formal_vehicle/power/battery_state` from the sole
`a300_bms_simulator` writer and consumes
`/formal_vehicle/power/charge_connected` from the sole
`charge_interface_manager` writer. The legacy-named simulation command
`/formal_vehicle/simulation/command/charge_connected` represents an operator
charge request; its product-side publication is `charge_requested`, not
`charge_connected`.

The simulation adapter no longer fabricates or publishes a
`charge_plug_present` Boolean. The charge manager consumes real Gazebo contact
samples from `/formal_vehicle/service/raw/charge_plug_contact`. The drain
manager independently consumes the coupling sensor on
`/formal_vehicle/service/raw/drain_hose_contact`; both are one-way GZ-to-ROS
raw `ros_gz_interfaces/msg/Contacts` bridges.

The state JSON binds these functions to the committed lighting, charge-port,
power-distribution, isolated-converter, relay and estop links/data.  It declares
`SIMULATION_ENGINEERING_ONLY`, reports voltage/SOC from a fresh BMS
`BatteryState`, and emits no evaluation topic. The active power boundary is the A300 aggregate 1024 Wh pack
and 650 W charger limit. Simplified branch loads remain simulation engineering
values rather than identified A300 or S100 measurements. The node publishes
requested, power-qualified lamp states. The independent Gazebo auxiliary plugin
consumes those states, changes real visual emissive materials and photometric
lights, and returns separate `/formal_vehicle/lighting/*_applied`
acknowledgements.

The adapter expires a stale manager-owned `charge_connected` value after
`charge_connected_timeout_sec` (0.25 s by default), forces it false, and emits
`charge_connected_fresh` in `/formal_vehicle/auxiliary/status_json`. This is
independent of the charge manager's own all-input 0.25 s freshness gate.

`RUNTIME_REVALIDATION_PENDING`: the implementation and static contract checks
are closed, but a fresh live ROS graph run must still stop the charge manager
and observe `charge_connected=false` plus `charge_connected_fresh=false` before
the downstream fail-closed path is accepted.
