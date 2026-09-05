# Formal campus integration

This package is the narrow compatibility layer between the formal high-fidelity
vehicle and the older campus autonomy graph. The vehicle remains a four-wheel
skid-steer platform. Nothing in this package claims physical Ackermann steering.

The launch layer performs five bounded jobs:

1. starts the unchanged formal vehicle launch in a generated campus world;
2. places the vehicle once, before task execution, from
   `public/episode_manifest.json#vehicle_start_pose_map`;
3. republishes formal sensors under compatibility names and sends raw physical
   wheel odometry to the local localization fusion path;
4. routes Nav2 collision monitor output `/cmd_vel_gate` directly through
   `whole_vehicle_safety_manager`, which is the sole publisher of
   `TwistStamped` to `/base_controller/cmd_vel` (the legacy velocity gate is
   disabled in this launch, so it cannot become a second command writer);
5. materializes scenario-matched occupancy, keepout, speed and coverage
   geometry files from public inputs before Nav2 starts.

Nav2 parameters are materialized at launch from the canonical repository file
`config/high_fidelity_vehicle/formal_motion_cleaning_profile.yaml`. Both Nav2
costmaps use `motion_footprints.transport_stowed`; the old 0.80 m x 0.72 m
footprint is rejected. Coverage width comes from the same profile's declared
effective cleaning width.

The map materializer reads only `public/episode_manifest.json`,
`public/world.sdf` and the formal motion profile. It rejects evaluator-only
manifest fields and never reads dirt truth. An embedded proxy vehicle is
rejected; the only vehicle is the formal URDF started by
`formal_vehicle_sim.launch.py`. Static collision geometry from the public
world becomes the occupancy map; the formal transport footprint plus a
safety margin becomes the keepout inflation; a wider configurable band becomes
the speed-reduction mask. All maps share one map-frame origin, resolution and
extent. The public geofence forms the lethal boundary, and the resolved vehicle
start must be free in both occupancy and keepout maps.

Generated `walker_*` bodies are intentionally absent from all static maps even
though their SDF bodies are declared static for environment-only SetPose
driving. Dirt visuals and dynamic 3 cm litter cubes are absent too. Pedestrian
motion defaults to enabled and resolves the sibling
`environment/pedestrian_schedule.json`; launch fails closed if that file is
missing. This environment schedule is never exposed as robot-control truth.

`mission_geometry.yaml` uses the public geofence, the same inflated static
obstacles, `motion_footprints.cleaning_deployed`, and the declared effective
cleaning width. Both OpenNav `robot_width` and `operation_width` use that same
value, and the headland is derived from the deployed footprint radius. It
contains no dirt patches or evaluator targets. Navigation
and the Coverage server therefore default to enabled; actual mission execution
remains a separate operator/task-runner action.

Simulation safety inputs default to enabled while the initial emergency stop
defaults to active. The launch never clears it. Motion remains inhibited until
an operator gate continuously commands `main_power=true` and
`emergency_stop=false`,
and all required safety inputs are fresh.

Localization has one writer per transform.  The physical A300 plant publishes
raw wheel odometry on `/odom/unfiltered`; the local `robot_localization` EKF
fuses wheel odometry and IMU, publishes canonical `/odom`, and alone owns
`odom -> base_footprint`.  During mapping, slam_toolbox alone owns
`map -> odom`.  During saved-map cleaning, AMCL supplies the lidar/map pose
measurement with TF broadcasting disabled, GNSS remains a second absolute
measurement, and the global EKF alone owns `map -> odom`.  MID360, wrist RGB-D
and the two side-rear fisheye streams remain on their formal native topics for
perception and manipulation consumers.

Example after generating an episode:

```bash
export TZCUP_REPOSITORY_ROOT=/path/to/TZcup
ros2 launch sanitation_formal_campus_integration formal_campus.launch.py \
  world:=/tmp/episode/public/world.sdf \
  episode_manifest:=/tmp/episode/public/episode_manifest.json \
  world_name:=campus_formal \
  runtime_artifact_dir:=/tmp/formal-campus-runtime
```

The runtime directory receives `occupancy.yaml`, `keepout_mask.yaml`,
`speed_mask.yaml`, `mission_geometry.yaml` and
`materialization_contract.yaml`, together with their PGM images. Supplying a
fixed directory makes the mission geometry available to the task runner;
otherwise the launch creates a process-specific temporary directory and logs
its absolute path.

## Deterministic WSL runtime acceptance

Use `scripts/run_formal_campus_runtime.sh` for the formal single-host WSL
acceptance run. It sources the overlays in the required order: ROS Jazzy,
Stage 1 (which supplies OpenNav Coverage), the current V3 runtime, then the
formal-campus overlay. The runner rejects an invalid domain or missing input,
sets `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, assigns an isolated Gazebo
partition, starts the launch in its own process group, and never clears the
initial E-stop.

The LOCALHOST setting is intentional. On WSL, launching Gazebo, bridges,
controllers, Nav2 filters and coverage as roughly 30 Fast DDS participants in
one scheduler tick produced delayed discovery and a temporarily empty graph
from a newly started external CLI. It was not a missing package, domain-ID
mismatch, or missing `robot_description`: the same run eventually converged.
The launch now starts the vehicle first, controllers at 12 seconds, navigation
at 20 seconds and coverage at 45 seconds. Controller service waits are bounded
at 180 seconds. A readiness probe joins before the delayed groups and makes a
fail-closed decision from live node, lifecycle, controller, topic-sample and
E-stop evidence.

From the repository root, the complete default command is:

```bash
ROS_DOMAIN_ID=191 \
FORMAL_CAMPUS_OUTPUT_ROOT="$PWD/.work/formal_campus_runtime_acceptance" \
bash scripts/run_formal_campus_runtime.sh
```

The runner returns zero only if controller states are safe, map/AMCL/filter/
coverage lifecycle nodes are active, and `/map`, `/scan`, MID360 points,
`/odom` and an asserted `/emergency_stop` have all produced samples. On exit it
sends SIGINT to the exact launch process group, waits up to ten seconds, then
uses SIGKILL only for survivors. It does not use broad process-name cleanup.
The JSON decision is written to
`formal_campus_runtime_readiness.json`; launch output remains beside it for
diagnosis.

## First-map then saved-map lifecycle

`formal_campus_map_lifecycle.launch.py` is the product path for the frozen
200 m x 100 m, open-boundary campus. It is intentionally separate from the
legacy `formal_campus.launch.py` materializer described above. The legacy
world-derived occupancy map is not connected to any product node in this
lifecycle.

In `mission_mode:=mapping`, the formal UTM self-filter converts raw `/scan`
into canonical `/scan/navigation`; slam_toolbox, AMCL, both Nav2 obstacle
layers and the single collision monitor all consume that same filtered scan.
The filter removes only the two angle-and-range sectors proven by expanded-
URDF mesh rays, writes masked hits as `NaN` (never invented free space), and
leaves external returns in those angles untouched beyond the self-hit range.
slam_toolbox also consumes `/odom`, while the frontier explorer submits only
known-free map frontiers to Nav2. It never
publishes velocity commands. Nav2 and the scan-based collision monitor retain
obstacle avoidance, and the whole-vehicle safety manager remains the sole
writer to the physical A300 skid-steer controller. Dirt, garbage, pedestrian
schedules, Gazebo model names and evaluator topics are not mapping inputs.
The command chain is explicit: Nav2 `/cmd_vel_nav` -> smoother
`/cmd_vel_smoothed` -> collision monitor `/cmd_vel_gate` -> whole-vehicle
safety manager -> `/base_controller/cmd_vel`. The optional legacy velocity
gate in `slam.launch.py` is disabled by the lifecycle launch.
On ROS 2 Jazzy, `nav2_bringup` starts the single `collision_monitor` instance
and `lifecycle_manager_navigation` configures/activates it. The lifecycle
launch must not start a second same-name monitor; runtime acceptance requires
exactly one node and one publisher on `/cmd_vel_gate`.

The static Nav2 parameter file starts conservatively with the transport
envelope. `formal_dynamic_footprint_manager` then publishes the current
`PolygonStamped` footprint to both costmaps: the wider brush sweep while the
cleaning lift is at its work position, and the full arm sweep whenever the arm
leaves its transport anchor or manipulation explicitly inhibits base motion.
Until all arm joints are observed it fails closed with the arm envelope.

The initial public fixed start is converted to local SLAM pose `(0, 0, 0)`.
For the sparse open-campus acceptance profile, lidar remains the occupancy and
collision source, while the local EKF propagates wheel+IMU odometry
with scan matching, scan barycenter correction and loop closing enabled. The
wheel+IMU EKF supplies the continuous odometry prior, while transformed GNSS
odometry is required as an independent drift-consistency gate before save. This is a bounded
four-source mapping chain rather than wheel-only dead reckoning. Saved-map
cleaning starts the global AMCL+GNSS fusion path with unique TF ownership as
described above.
The manager latches the first odometry sample only when it is within the fixed
start tolerance. A small locally complete map cannot pass: cells outside the
current OccupancyGrid extent remain part of the 20,000 m2 denominator. At
least three stable samples at `observed_fraction >= 0.95` are required before
calling `/slam_toolbox/save_map`. The YAML, image, geofence masks and product
contracts are SHA-256 sealed in `map_lifecycle_manifest.json`.

In `mission_mode:=cleaning`, launch validates that manifest, map ID, 95% gate
and every hash before starting AMCL/Nav2. The saved `occupancy.yaml` is the
only localization map. The geofence keepout is derived solely from the public
field boundary; static obstacles come from SLAM, not `world.sdf`.  The default
`cleaning_planner:=full_coverage` starts the deterministic Coverage fallback.
`cleaning_planner:=rl_dirt_priority` disables the fallback server so that the
cycle-free top-level `sanitation_product_demo_integration` package can own the
PC DOSOD+EdgeSAM adapter and the truth-free RL observation, policy, trajectory
and coordinator nodes.  That top-level launch fails before simulation if its
verified model directory, frozen policy checkpoint, or positive same-map
FullCoverage distance ceiling is missing.

```bash
# First task: explore and save. A new empty directory is mandatory.
ros2 launch sanitation_formal_campus_integration \
  formal_campus_map_lifecycle.launch.py \
  mission_mode:=mapping world:=/tmp/episode/public/world.sdf \
  episode_manifest:=/tmp/episode/public/episode_manifest.json \
  map_artifact_dir:=/tmp/formal-first-map

# Later tasks: launch fails before Nav2 if any saved artifact is invalid.
ros2 launch sanitation_formal_campus_integration \
  formal_campus_map_lifecycle.launch.py \
  mission_mode:=cleaning world:=/tmp/episode/public/world.sdf \
  episode_manifest:=/tmp/episode/public/episode_manifest.json \
  map_artifact_dir:=/tmp/formal-first-map

# Dirt-priority product mode. The distance is the successful FullCoverage
# baseline on this same saved map and is a hard RL acceptance ceiling.
ros2 launch sanitation_product_demo_integration product_demo.launch.py \
  world:=/tmp/episode/public/world.sdf \
  episode_manifest:=/tmp/episode/public/episode_manifest.json \
  saved_map_artifact_dir:=/tmp/formal-first-map \
  perception_artifact_root:=/tmp/formal-perception-assets \
  policy_checkpoint:=/tmp/formal-rl/formal_planning/q_policy.json \
  maximum_task_distance_m:=3450.0
```

The launch preserves the existing fail-closed power-on behavior. An operator
must still provide fresh safety inputs and explicitly clear E-stop; mapping
does not bypass that gate. This package also does not claim that a complete
200 m x 100 m simulation has run merely because the nodes build or launch.

### Saved-map dynamic-obstacle acceptance

`scripts/run_formal_dynamic_obstacle_avoidance.sh` is the fail-closed runtime
gate for the later-task pedestrian case. It refuses to start Gazebo until the
same episode has a SHA-256-valid `map_lifecycle_manifest.json` with at least
95% observed area. Once admitted, it builds the affected packages from the
current checkout, starts the formal transport-stowed vehicle with AMCL/Nav2,
and continuously commands the normal simulation operator/safety inputs.

Each unpinned run records a new environment seed. Exactly eight existing
walker models are used; three receive obstacle-free randomized routes crossing
the fixed 30 m public mission corridor, while the other five retain their
episode-generated random routes. The schedule is environment/evaluator-only:
the product sends a goal derived from the public fixed start and never reads a
pedestrian waypoint, speed or entity pose. Dynamic interaction identity is
verified only after the run by matching collision-monitor candidate times to
the environment schedule.

The coordinate boundary is explicit: the source-world fixed start
`(-98, 0)` becomes `(0, 0)` in the saved SLAM map, so the formal Nav2 goal is
map-local `(30, 0)` while the corresponding Gazebo corridor ends at source
world `(-68, 0)`. Walker evaluator poses are transformed into the saved-map
frame only after command execution. The runtime also emits a build manifest
whose source/install SHA-256 pairs bind the lifecycle and navigation launches,
Nav2 parameters, safety manager, simulation safety inputs, vehicle launch and
environment driver to the current checkout; all six Gazebo control plugin
libraries must be present. The same manifest records SHA-256 identities for
the map collector and source-run dynamic orchestration scripts, and binds the
formal UTM-30LX self-filter config and implementation to the installed bytes.

The generated campus world does not load Gazebo's Contact system. Before the
cleaning run, the acceptance runner therefore creates an instrumented copy
that adds exactly that world plugin. A second manifest seals both world
hashes, proves that every other XML element is unchanged, and records the
eight retained walker IDs. This makes the bumper-contact heartbeat and zero-
collision evidence physical while keeping campus geometry and product inputs
identical to the admitted cleaning world.

The gate requires a succeeded `NavigateToPose`, physical odometry travel,
AMCL map-frame detour, at least one evaluator-confirmed pedestrian interaction,
zero bumper contacts, zero public-geofence violations, exactly one
`collision_monitor` publisher on `/cmd_vel_gate`, and exactly one
`whole_vehicle_safety_manager` publisher on
`/base_controller/cmd_vel`. Static tests, graph startup, an invalid saved map,
or a collector timeout leave
`artifacts/formal_dynamic_obstacle_avoidance_acceptance.json` explicitly
`FORMAL_DYNAMIC_OBSTACLE_AVOIDANCE_ACCEPTANCE_BLOCKED`.
