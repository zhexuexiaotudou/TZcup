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

The optimized small-field configuration selects a 0.52 m planning spacing for
the physical 0.65 m brush. Candidate spacings are 0.42, 0.46, 0.48, 0.50 and
0.52 m; the first live 0.48 m trial reached only 97.17% and was therefore
rejected. A map-normal affine execution calibration (`1.06`, `0.0264 m`) adds
tracking-error margin without changing the selected planning candidate. It was
fitted offline against seeds 118, 119, 120 and the retained failing seed 123;
ground truth remains evaluation-only and is never read by the online controller.
The 0.35 m profile is retained only as a fail-closed legacy fallback. The pure
optimizer evaluates angles from 0 through 175 degrees in five-degree steps and
chooses the route with the lowest path/connector/turn cost.

The hybrid localizer smooths RTK global anchors in the odometry frame and
rejects scan corrections that disagree with a fresh RTK fix by more than
0.10 m. Wheel/IMU propagation remains unsmoothed, so this improves absolute
cross-seed stability without adding motion lag or reducing the 0.05 m gate.
Because the independent small demo does not share geometry with the frozen
Stage4V scan map, it explicitly selects the RTK + wheel/IMU lane. Medium and
large mapped scenes retain hybrid scan fallback. The final Coverage success
gate includes per-seed localization RMSE, so a path-only pass cannot mask a
localization regression.
The same report separates executed brush-on, brush-off and state-transition
distance. It computes two separate brush-center diagnostics: absolute
cross-track error to the nearest planned primary swath for map alignment, and
steady-state straightness after removing each swath run's median offset. The
straightness metric uses the central 80 percent of every executed swath and
fails the mission when P95 exceeds 0.08 m. Localization retains its independent
0.05 m RMSE gate, so a fixed frame bias cannot masquerade as path weave. These
reports retain `primary_swath_lateral_error` as an alias of the absolute
cross-track diagnostic for existing evidence readers.
metrics turn the visual claim of a tidy lawnmower route into machine-checkable
execution evidence.
The simulated `rtk_fixed` capability contract retains 0.02 m white noise and
0.005 m fixed-bias sigma, with long-term random walk calibrated to
0.001 m/sqrt(s). Float, multipath and denied profiles retain their stronger
degradation. A real receiver must replace this assumption with field logs.

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
Cold-start readiness is observed by one persistent ROS graph node, including
the required action services and exact Nav2 lifecycle state IDs. This avoids
restarting DDS discovery on every poll and records `runtime_readiness.json` on
both success and timeout.
The pre-mission false emergency-stop availability pulse uses the same
persistent-node approach and requires both matching safety/HMI subscribers and
dashboard observation before coverage execution starts.
The launcher supervises the Coverage process with `setsid --wait`, so a
context-dependent intermediate fork cannot be mistaken for mission completion.
For a legacy comparison, pass `--coverage-profile legacy` to
`run_visual_demo.sh` or `-CoverageProfile legacy` to either Windows launcher.
The default and explicit optimized spelling are `optimized`.
Long headless multi-seed acceptance can use
`scripts/run_frozen_coverage_trial.ps1`; it fixes the ROS domain, assigns a
per-trial Gazebo partition and always runs the same independent small-field
mission while retaining a dedicated launcher log and evidence directory.
The complete A/B entry is `scripts/run_coverage_optimizer_matrix.sh`. Its
output must be outside the Git worktree because repository hygiene rejects raw
MCAP/DB3 payloads even when ignored. `scripts/coverage_optimizer_report.py`
aggregates the retained directories and writes the comparison plus a streaming
SHA-256 manifest without replacing any raw run.

Repair evidence is a separate ten-seed matrix launched by
`scripts/run_coverage_repair_matrix.sh`. Its evaluation-only injection disables
the brush over one bounded portion of a primary swath, selected from the fused
localization path fraction. It does not modify a Nav2 goal or publish vehicle
commands, and ground truth remains evaluation-only. A valid run must observe
the injected miss, execute exactly one connected-component repair pass, recover
at least 99.5% coverage, keep repair length within 10% of the primary swaths,
and explicitly report that ground truth was not used for control. The length
gate uses the brush-on path actually sent to `RepairPath`, not merely the
residual planner's nominal geometry; transit reaches the residual endpoint with
the brush disabled, so hidden lead-in and overrun cannot inflate repeat area.
Residual swaths are brush-centre paths over the missed-cell span: their circular
swept footprint supplies the endpoint radius, and the planner does not extend
the centreline by that radius a second time. Before execution, each multi-cell
repair trims one quarter brush width from each endpoint and applies the same
offline map-normal affine calibration as the primary swaths; the report keeps
both nominal residual and commanded brush-centre segments. A one-cell residual
is represented by one raster-cell width of physical centreline and is never
trimmed, so Nav2 always receives a non-degenerate heading.

The entry preflight normally receives keepout and speed masks through
transient-local topics. If cold DDS discovery misses either latched sample, the
coverage task calls that mask MapServer's authoritative `GetMap` service within
the same bounded readiness window. Entry selection remains fail-closed unless
the global costmap and both configured masks cover the staging point.

The retained optimized seed 132 bag is checked with
`scripts/verify_coverage_mcap_replay.sh`. That gate first executes a real
`ros2 bag play` into remapped replay topics, then uses the MCAP sequential
reader to reconstruct the state sequence, stable component IDs, semantic plan,
brush transitions, localization, command, and planned/actual trajectory topic
inventory. A readable storage file without a completed play process or a
`COMPLETED` terminal state fails the replay gate.

Controller failures that classify a cleaning swath as blocked now enter an
explicit state machine. The report retains normalized blocked intervals, first
observation time, obstacle state, retry count and terminal reason. A retry is
brush-off and delayed by at least 10 seconds; two failed retries transition to
`DEFERRED`, so the controller cannot oscillate forever at one obstacle. This
mechanism is implemented, but the production recovery claim remains gated on
20 valid live interactions with at least 95 percent mission resumption.
The reproducible small-field entry is
`run_gazebo_cleaning_demo.ps1 -DynamicObstacleTrials 20`; a non-static physical
pedestrian starts parked outside the arena and the probe moves it across the
current swath. Only lidar-observed, collision-free interactions followed by
measured vehicle progress count as valid. The probe prefers the ROS-Gazebo
`SetEntityPose` service and falls back to the selected world's native Gazebo
Transport `/world/<name>/set_pose` service when that ROS bridge is absent;
every report records the backend. The native call receives a separate
10-second cold-discovery budget and must return Boolean `true`. The pose is set
at model-frame ground height (`z=0`); the pedestrian collision cylinder already
contains its own 0.85 m vertical offset, so it is not lifted above the planar
LiDAR a second time. A service
return alone is insufficient: the interaction
also requires a lidar range below 2.0 m and at least a 0.15 m drop from the
pre-injection scan. The formal trajectory uses five physical pose samples and
a 0.5-second requested hold budget; this keeps a crossing observable while the
obstacle clears the centreline before the vehicle reaches it, instead of using
a slow teleport sequence that manufactures a near collision.
WSLg and operator demos retain Ogre2. A Docker-only headless matrix may pass
`--simulation-render-engine ogre` when its runtime has CUDA but no EGL graphics
context; the selected engine is materialized in the retained runtime SDF and
is never inferred silently. Ogre runs with an explicit X display instead of
Gazebo's Ogre2-only EGL headless path.

## Sim-to-real boundary

The selected execution profile does not publish chassis commands from the
coverage task. Straight cleaning uses the dedicated Nav2 `CleanPath`
controller at 0.65 m/s with a fixed 0.75 m lookahead; repairs use
`RepairPath`; connector rotation and translation use Nav2 `Spin`,
`DriveOnHeading`, or `BackUp` behaviors. The velocity smoother, collision
monitor, safety gate, and behavior collision projection therefore remain in
the command path. Gazebo 2x/3x real-time factor is independent of these
physical component limits.

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

The small-field launcher deliberately uses a `SMALL_FIELD_LIDAR_ONLY` collision
monitor profile: demo debris is traversable and delayed renderer point clouds
must not masquerade as physical obstacles. RGB-D remains published, but is not
an actuator-safety source in this profile. Medium/large and every real vehicle
must restore the multi-source lidar + height-aware RGB-D contract. `turbo` is a
developer experiment only; `fast` is the highest supported demo mode.
