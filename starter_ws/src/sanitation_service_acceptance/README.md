# Physical service-interface acceptance

This package is evaluator-only. It places a static service fixture at the
formal vehicle origin so real Gazebo collisions occur between a cylindrical
charge plug and `charge_receptacle_contact_collision`, and between a hose tip
and `wastewater_drain_coupling_contact_collision`. Moving the fixture 4 m in X
creates the no-contact rejection episodes without publishing a fake Boolean.

The vehicle's normal model does not contain service-actuation plugins.
`run_formal_service_interface_acceptance.sh` expands a temporary vehicle model
with `service_acceptance_interfaces:=true`. Only that generated evidence model
accepts evaluator commands for the passive door, latch and service cap under
`/formal_vehicle/evaluation/service/*`. Product nodes continue to consume only:

- `/formal_vehicle/service/raw/charge_plug_contact` (`Contacts`);
- `/formal_vehicle/service/raw/drain_hose_contact` (`Contacts`);
- `/joint_states` for door, lock, cap and valve positions;
- normal BMS, charge, safety and drain status topics.

Every episode expands the vehicle with the final `8.30 kg` wastewater load.
The collector requires the product-facing sensed tank-level signal to reach at
least `0.99`; the aggregate validator separately checks that both simulation
capacity declarations and the storage payload clamp remain exactly `8.30 kg`.

The eight fresh simulation episodes cover charge acceptance, missing plug,
closed door, open lock, drain acceptance, missing hose, closed service cap and
simultaneous charge/drain requests. The last case requires charge to remain
available while drainage stays closed. The collector subscribes to no world
pose, entity-state, ground-truth or evaluator-truth topic.

After building the workspace, run:

```bash
bash scripts/run_formal_service_interface_acceptance.sh
```

Per-episode evidence is written under
`artifacts/formal_service_interface_episodes/`; the fail-closed aggregate is
`artifacts/formal_service_interface_acceptance.json`. Static tests do not count
as live contact acceptance. Until all eight artifacts pass in a fresh Gazebo
run, the runtime gate remains open.
