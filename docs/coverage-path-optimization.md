# Coverage path optimization and semantic visualization

## Purpose

The small Gazebo competition demo now uses `SKID_STEER_OPTIMIZED` by default.
It keeps the proven OpenNav Coverage / Fields2Cover swath generator, but treats
its generated curves only as a compatibility input. The runtime orders the
straight swaths itself and replaces Dubins loops with explicit skid-steer
components. The previous 0.35 m continuous-Dubins profile remains available as
`coverage_demo_overlap.yaml` plus `competition_demo_area.yaml`.

## Planning and execution contract

`CoveragePlan` is the single versioned contract shared by the planner,
executor and Gazebo panel. Every component has a stable ID, geometry, brush
state, speed profile and one of these types:

- `TRANSIT`, `SWATH`, `ROTATE`, `SHIFT`, `BACKUP`, `OBSTACLE_BYPASS`,
  `REPAIR_SWATH`, `RETURN_HOME`.
- Only `SWATH` and `REPAIR_SWATH` may enable the brush.
- Primary swaths are sorted along their common normal and alternate direction,
  so the route is an adjacent lawnmower pattern.
- Connectors use rotate-translate-rotate. A large heading mismatch may select a
  bounded backup. A connector leaving the footprint-safe polygon fails closed
  and is handed to the collision-checked Nav2 bypass path.
- Missed cells are split into connected residual regions. They are never joined
  by a brush-on line across clean ground. At most one repair pass and 10% of the
  primary swath length are allowed by the optimized demo profile.

The optimized small-field configuration uses a 0.42 m swath spacing for the
physical 0.65 m brush. Candidate spacings are 0.42, 0.46, 0.48, 0.50 and
0.52 m; the first live 0.48 m trial reached only 97.17% and was therefore
rejected. The 0.35 m profile is retained only as a fail-closed legacy fallback. The pure
optimizer evaluates angles from 0 through 175 degrees in five-degree steps and
chooses the route with the lowest path/connector/turn cost.

## ROS interfaces

The plan and its semantic layers are published as compact JSON strings:

- `/coverage/full_plan` (`TRANSIENT_LOCAL`)
- `/coverage/planned_swaths`, `/coverage/planned_connectors`,
  `/coverage/planned_repairs` (`TRANSIENT_LOCAL`)
- `/coverage/current_component_path`
- `/coverage/component_state`
- `/coverage/actual_cleaning_trajectory`,
  `/coverage/actual_transit_trajectory`, `/coverage/actual_repair_trajectory`

`/coverage/current_path` remains a compatibility alias. Gazebo telemetry is now
`tzcup.gazebo_cleaning_telemetry.v2`, while retaining the v1 `planned_path` and
`trajectory` fields for old consumers.

## Gazebo panel

The map uses independent layers instead of one merged polyline:

- yellow solid: planned cleaning swaths;
- gray dashed: planned brush-off connectors;
- white: current component;
- cyan: actual brush-on primary cleaning;
- orange dashed: actual brush-off transit and connectors;
- purple: planned and actual residual repair;
- green cells: empirically cleaned area.

The 规划, 实际 and 补扫 checkboxes independently toggle those layers. Motion
segments are split whenever the semantic state changes, preventing the UI from
drawing a false diagonal between two non-contiguous cleaning passes.

## Run and compatibility

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_gazebo_cleaning_demo.ps1 -SimulationSpeed turbo
```

The optimized profile is selected automatically for `-MapSize small`. The
normal manual Start, Pause, Resume, Stop and Close Gazebo controls are unchanged.
For a legacy comparison, invoke `run_visual_demo.sh` after pointing
`mission_template` and `coverage_params` to the two legacy files listed above.

## Sim-to-real boundary

The semantic split is designed to map to real controllers, but simulation does
not prove physical performance. Before vehicle deployment, calibrate brush
width and forward offset under load, identify yaw overshoot and lateral slip on
each surface, enforce motor-current and thermal limits for in-place rotation,
and replace Gazebo ground-truth coverage with localization- and brush-contact-
derived coverage. Dynamic-obstacle blocking must be validated with real lidar
latency and braking distance. The real controller must preserve the fail-closed
brush invariant even during process restart, network loss and emergency stop.

Required promotion gates are SIL multi-seed replay, HIL actuator/safety testing,
closed-site low-speed trials, calibrated coverage measurement, and an operator-
approved rollback to the retained legacy profile.
