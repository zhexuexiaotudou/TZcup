# sanitation_campus_scenario

This ROS 2 package generates deterministic Gazebo Harmonic campus worlds and
randomized cleaning episodes without requiring a vehicle URDF.

Two fixed-area profiles are provided; map index 0 keeps the baseline dimensions
and every other map derives a bounded rectangular aspect ratio from its layout:

- `research`: baseline 106 m x 53 m, always 5,618 m2;
- `formal`: baseline 200 m x 100 m, always 20,000 m2.

Both fields are open at the perimeter. The boundary is a `map`-frame geofence,
not a physical wall. Ground plane, geofence, reserved vehicle start and all
sampled entities use the derived per-map dimensions. Static buildings, poles,
bins, trees and benches use real-world dimensions. The split freezes 32/8/12
train/validation/hidden maps.
Each train map has 200 missions and each validation/hidden map has 100 missions,
for 8,400 missions total. Missions on the same map share the layout seed while
dirt, cube, pedestrian and sensor seeds are independent per mission. Cubes are
single-layer 30 mm objects, at most 20 per mission, and are placed outside the
configured whole-vehicle side-pick parking clearance around fixed assets. The
public cube contract separately publishes `grasp_reach_radius_m`; placement
clearance must never be reused as an arm reach/success threshold. Every dirt rectangle has a
fixed area of 1.0 m2; its aspect ratio and yaw vary by mission. Conservative
rotated bounding circles keep dirt rectangles mutually exclusive, so the
recorded union area is the actual sum rather than an overlap estimate.

Generate one episode:

```bash
ros2 run sanitation_campus_scenario sanitation-campus-scenario generate \
  --config $(ros2 pkg prefix sanitation_campus_scenario)/share/sanitation_campus_scenario/config/default_scenario.yaml \
  --profile research --split train --map-index 0 --mission-index 0 \
  --output /tmp/campus_train_map_000_mission_000
```

Generate the frozen 32/8/12 map split and its 8,400 mission index entries (without generating 8,400 worlds):

```bash
ros2 run sanitation_campus_scenario sanitation-campus-scenario split-index \
  --config $(ros2 pkg prefix sanitation_campus_scenario)/share/sanitation_campus_scenario/config/default_scenario.yaml \
  --output /tmp/campus_split_index.json
```

The command refuses to overwrite an existing path. It writes into a temporary
directory, validates the SDF and JSON files, and atomically publishes the whole
directory only after all checks pass.

## Public, evaluator and environment boundary

Generation creates three deliberately separate roots:

- `public/` contains only `world.sdf` and `episode_manifest.json`. This is the
  only directory made available to the controller. Its manifest contains no
  role seed, truth/evaluator path or pedestrian schedule path, including for
  hidden missions.
- `evaluator/` contains the evaluator manifest and exact ground truth. Exact
  state is declared under `/evaluation/scenario_ground_truth` with
  `control_use_prohibited=true`.
- `environment/` contains the pedestrian schedule. It is mounted only into the
  environment driver, never the robot control graph.

The primitive pedestrian bodies are initially static in SDF. Gazebo's
`set_pose` transport service must first be bridged to ROS, then the environment
driver is started with the environment-only schedule:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /world/campus_research/set_pose@ros_gz_interfaces/srv/SetEntityPose

ros2 run sanitation_campus_scenario sanitation-campus-pedestrian-driver \
  --ros-args -p schedule_path:=/tmp/campus_train_map_000_mission_000/environment/pedestrian_schedule.json
```

The driver calls Gazebo's world `SetEntityPose` service and publishes only a
health/status message. It does not publish future trajectories or exact poses
to the robot control graph. This package does not silently claim motion merely
because trajectories were requested. If the service is unavailable or rejects
an update, the driver enters an explicit error state and cancels further motion.

`--include-proxy` adds a static 0.60 m x 0.40 m visualization/planning proxy.
It is explicitly named `proxy_chassis_not_urdf`, carries no sensors or control
plugin and must not be presented as the real robot model.

`split-index` is an evaluator-side reproducibility artifact because it contains
the frozen seed roles. Do not mount it into the controller or include it in a
hidden mission's public bundle.
