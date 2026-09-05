# A300 40 Ah power-system model

This package implements the public Husky A300 40 Ah boundaries: LiFePO4,
25.6 V nominal, 40 Ah, 1024 Wh, 60 A continuous pack output, a 100 A breaker,
and the supported 650 W / 23.5 A charger ceiling. The values are sourced from the current
[Husky A300 user manual](https://docs.clearpathrobotics.com/docs_robots/outdoor_robots/husky/a300/user_manual_husky/).

The public manual does not provide the pack open-circuit-voltage curve,
internal resistance, thermal capacitance, cell voltages, or unit-specific
health calibration. The corresponding configuration values are therefore
named `engineering_*`, the ROS status reports the simulation-only evidence
class, and cell measurements remain `NaN`. They require real-vehicle
identification before hardware-correlated acceptance.

`a300_bms_simulator` publishes `sensor_msgs/msg/BatteryState` on
`/formal_vehicle/power/battery_state`. Charging is accepted only while the
E-stop is active and main power is off; over-100 A demand latches the breaker,
which can be reset only in the same service state.

## Topic ownership and freshness

`a300_bms_simulator is the sole writer` for `/formal_vehicle/power/battery_state`,
`battery_soc`, `bms_fault`, and `traction_permitted`. The battery state, rather
than the simulation auxiliary adapter, is the authoritative SOC source.

`charge_interface_manager is the sole writer` for
`/formal_vehicle/power/charge_enable`, `charge_connected`, `charge_request_w`,
and `charge_status_json`. Plug presence is derived only from non-empty
`ros_gz_interfaces/msg/Contacts` on
`/formal_vehicle/service/raw/charge_plug_contact`; the former synthetic Boolean
product input has been removed. It accepts charging only when the request, plug,
traction, E-stop, main-power, BMS, charge-door/lock joint state, and odometry
inputs have all refreshed within `input_timeout_sec` (0.25 s by default).
`simulation_safety_inputs` owns only the simulation request boundary:
`charge_requested`, `main_power_requested`, and `load_request_w`.

The Gazebo contact sensor is bound to the named
`charge_receptacle_contact_collision` and publishes GZ Contacts on
`/formal_vehicle/gazebo/charge_receptacle/contact`. The default vehicle launch
bridges it one way (GZ to ROS) to the raw product topic above. Empty contact
arrays mean no plug; a missing/stale stream fails the manager's shared 0.25 s
freshness gate.

The committed component register statically checks these publishers by exact
topic, message class, source file, and single-writer ownership. That is a source
contract, not a substitute for a live ROS graph writer-count check.

The BMS independently expires `charge_request_w` after 0.25 s and publishes
`charge_request_fresh` in `bms_status_json`. The auxiliary adapter independently
expires the manager-owned `charge_connected` state after 0.25 s and publishes
`charge_connected_fresh` in its status JSON. The charge core rejects non-finite
or non-positive rated power, and the manager rejects non-finite or non-positive
timing, rating, door, lock, and stationary-threshold parameters at startup.

`RUNTIME_REVALIDATION_PENDING`: these source and pure-core gates are closed,
but a fresh live ROS graph run must still demonstrate both watchdog transitions
after their upstream publishers are stopped. Do not treat static ownership or
unit-test evidence as that live acceptance result.
