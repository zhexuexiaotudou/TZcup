# Formal mechanical interface datum crosswalk

`config/high_fidelity_vehicle/formal_mechanical_interface_datums.yaml` is the
single machine-readable crosswalk for the nominal mechanical interfaces below.
It is derived only from the current layout, component register and expanded
URDF.  The validator first checks SHA-256 for those three inputs, then checks
the zero-joint URDF FK, component-register bindings, functional interfaces and
declared URDF joints.

The reference is the existing `base_footprint` root.  Poses are existing URDF
`xyz_m` and `rpy_rad` values; the crosswalk deliberately does not introduce a
new drawing-axis convention. `chassis_top_plate` means the model frame
`payload_deck_link`, not a claimed machined surface or an inspected physical
plane.

| Interface | Unique datum chain | Zero-joint endpoint pose in `base_footprint` (m) | Register binding |
| --- | --- | --- | --- |
| Chassis top plate to arm base | `payload_deck_link` → `arm_mount_link` → `ur5e_base_link` → `ur5e_base_link_inertia` | arm base `[0.100, -0.200, 0.4791]` | `manipulator` / `manipulation` |
| Chassis top plate to sensor tower | `payload_deck_link` → `sensor_mast_link` | mast base `[0.420, 0.000, 0.3891]` | `sensor_tower` |
| Chassis to cleaning head | `base_footprint` → `cleaning_mechanism_mount_link` → `cleaning_lift_carriage_link` → `central_roller_link` | roller axis `[0.155, 0.000, 0.2000]` | `cleaning_head_deployment` / `cleaning_head_lift` |
| Chassis top plate to dry bin | `payload_deck_link` → `storage_system_mount_link` → `dry_bin_link` | bin origin `[-0.205, 0.160, 0.5691]` | `dry_storage` |
| Chassis top plate to wastewater tank | `payload_deck_link` → `storage_system_mount_link` → `wastewater_tank_link` | tank origin `[-0.205, -0.245, 0.5041]` | `wastewater_storage` / `wet_storage` |
| Chassis to charge receptacle | `base_footprint` → `charge_port_housing_link` → `charge_receptacle_link` | receptacle `[0.250, 0.402, 0.4831]` | `exterior_service_and_contact_safety` / `charge_interface` |
| Wastewater tank to drain hose | `wastewater_tank_link` → `wastewater_drain_valve_body_link` → `wastewater_drain_coupling_link` | hose coupling `[-0.555, -0.305, 0.4801]` | `exterior_service_and_contact_safety` / `wastewater_drain` |

## Snapshot and validation

The checked source snapshot is SHA-256-bound to:

| Input | SHA-256 |
| --- | --- |
| `config/high_fidelity_vehicle/formal_vehicle_layout.yaml` | `39ed88e52e29252df67a9a60b5efbce719557fccc0a0606b37d8635646fbe355` |
| `config/high_fidelity_vehicle/formal_vehicle_component_register.yaml` | `d2995a21dea8ac398d615af677ed9a3108ec869326c552af3a6e8065a48e3333` |
| `reports/engineering/formal_competition_vehicle.urdf` | `700be6ce11a0d0c86f506c1bbd89b2a1503855bb83372fe656858d383d04b19b` |

Run only the Windows static checks:

```powershell
py -3 scripts/validate_formal_mechanical_interface_datums.py
py -3 -m pytest scripts/test_formal_mechanical_interface_datums.py -q
```

Any source change intentionally fails the snapshot hash gate. Regenerate this
crosswalk only after re-auditing every affected layout/register/URDF number;
do not simply replace a digest.

## Boundary

`STATIC_DERIVED_SNAPSHOT_BOUND_NOT_MANUFACTURING_RELEASE` is not a fabrication,
procurement or vehicle-release result. It contains no manufacturing tolerances,
bolt torque/preload, material strength/fatigue, machining/hole pattern,
real-hardware fit/service clearance, collision/CoG, dynamic-contact, or
physical acceptance claim. The static tolerances mirror the existing component
register's URDF comparison tolerances only; they are not manufacturing
tolerances.
