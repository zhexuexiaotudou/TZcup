# sanitation_gazebo_auxiliary

Independent Gazebo Harmonic support for the formal vehicle's lighting visuals
and physical emergency-stop latch. The package is deliberately separate from
`sanitation_vehicle_description` and `sanitation_safety`, so its core behavior
can be built and tested without changing a frozen vehicle or safety release.

## Implemented boundary

- deterministic work, tail and 1 Hz warning-beacon state from simulation time;
- fail-closed emergency-stop latch with separate reset request and safety-power
  reset permit;
- measured and commanded `emergency_stop_plunger_joint`: the ROS engineering
  press command moves the real 0--6 mm prismatic joint, while a measured
  position of at least 5 mm asserts the switch. Missing/invalid joint feedback
  is treated as pressed;
- measured main-power chain: a 90-degree
  `main_power_isolator_handle_joint` is driven from operator intent, while the
  4 mm normally-open `main_power_contactor_armature_joint` can close only when
  the isolator is physically ON, safety power is available, the E-stop latch
  is released and the safety node requests its coil. Missing joint feedback is
  published as open;
- a real gz-sim8 System plugin which changes the existing named visual
  `sdf::Material::Emissive` values and publishes applied/latch acknowledgements;
- a two-axis passive squeegee model whose prismatic float and pitch joints are
  driven by bounded spring-damper effort, including a 10.8 N nominal blade
  preload, live measured position/velocity/effort telemetry, and a dedicated
  blade-ground contact sensor for three-phase free/contact/recovery acceptance;
- two model-parented front `sdf::LightType::SPOT` sources, two rear red tail
  `POINT` sources and four model-parented warning `POINT` sources. Their poses and directions follow the
  vehicle, and their flash phase freezes when simulation time is paused;
- Gazebo Transport implementation with an explicit one-way ROS-Gazebo bridge
  in `formal_vehicle_sim.launch.py`. The bridge is the sole formal ROS writer
  of `/emergency_stop`.

The latch starts asserted by default. Releasing the 6 mm plunger does not clear
it: reset succeeds only after the external E-stop request is false, the measured
plunger is below 5 mm, and the safety-power branch is available. The latched
state is republished every physics update so late bridge/manager startup cannot
miss the initial fail-closed state. Front illumination uses two spot lights;
the two tail lamps and four warning corners combine emissive surfaces with red
or amber point-light output.

The formal vehicle embeds exactly one plugin block. The installed
`config/formal_auxiliary_visual_system.sdf` is its reference copy. The expected visual names
are `front_work_light_left_visual`, `front_work_light_right_visual`,
`rear_tail_light_left_visual`, `rear_tail_light_right_visual`, and
`corner_beacons_visual`.

ROS commands and observed outputs are deliberately separate:

- command/request: `/formal_vehicle/lighting/*_on`,
  `/formal_vehicle/simulation/command/emergency_stop`,
  `/formal_vehicle/simulation/command/emergency_stop_plunger_pressed`,
  `/formal_vehicle/simulation/command/emergency_stop_reset`,
  `/formal_vehicle/simulation/command/main_power`,
  `/formal_vehicle/power/main_contactor_command`, and
  `/formal_vehicle/power/branches/safety/enabled`;
- physical/applied state: `/emergency_stop` and
  `/formal_vehicle/lighting/*_applied`,
  `/formal_vehicle/power/main_isolator_closed`, and
  `/formal_vehicle/power/main_contactor_closed`.

`simulation_safety_inputs` subscribes to `/emergency_stop`; it never publishes
that topic. This keeps the physical latch in the safety path instead of
allowing the engineering command source to spoof the product state.

The squeegee axes export state only through `joint_state_broadcaster`; neither
has a ros2_control command interface. `run_formal_function_positions_runtime.sh`
raises the mechanism to observe the free spring reference, lowers it until the
named blade collision produces ground contact and downward preload, then raises
it again and requires contact release plus return to the initial free position.
The report binds the live contact collision pairs, joint-state samples and the
exact force/torque commands applied by `SqueegeeComplianceSystem`.
The same separation applies to main power: the safety node accepts the two
physical applied-state topics only while their heartbeats are fresh, commands
the contactor coil separately, and derives the high-power branch from measured
armature closure. A Boolean main-power request alone cannot assert the branch.
