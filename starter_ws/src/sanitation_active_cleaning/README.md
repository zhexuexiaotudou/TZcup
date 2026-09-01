# sanitation_active_cleaning

Pure-Python, URDF-independent research environment for active sanitation
planning. Policies emit global `Pose2D` reference trajectories; this package
does not emit wheel, steering, or velocity commands.

The public observation contains only the accumulated belief map, currently
observed pedestrian positions, known obstacles, and detected dirt/targets.
Hidden dirt and target truth require an identity-checked evaluation token and
are used only for the Oracle upper bound and final metrics.

Scenario adapters may pass a `TaskLayout` containing explicit dirt regions,
cube IDs/coordinates, and pedestrian start poses to `ActiveCleaningEnv`. The
layout remains private environment truth: policies receive only detections and
the accumulated belief snapshot. Omitting it retains seeded random generation.

An optional dependency-free `GraspVerifier` callback can connect the planner to
a manipulation adapter. A cube is cleared only when the callback returns
`GraspVerificationResult(verified_in_bin=True)`. Without a callback, the
configured probability model is retained and reports
`grasp_verification_mode=simulated_probability`.

Run a paired-seed comparison after installing the package:

```bash
ros2 run sanitation_active_cleaning active_cleaning_demo \
  --config "$(ros2 pkg prefix sanitation_active_cleaning)/share/sanitation_active_cleaning/config/demo_task.json" \
  --seeds 101,102,103 \
  --output /tmp/active_cleaning_report.json
```

The report scores observation coverage, swept dirty area, successfully grasped
discrete targets, safety, and executed chassis task distance. Return travel,
wall-clock time, waiting, arm motion, and energy are reported as out of scope.

Train the included belief-only tabular Q-learning planner with disjoint fixed
seed splits:

```bash
ros2 run sanitation_active_cleaning active_cleaning_train \
  --config "$(ros2 pkg prefix sanitation_active_cleaning)/share/sanitation_active_cleaning/config/demo_task.json" \
  --train-seeds 100:140 --validation-seeds 200:210 --test-seeds 300:310 \
  --policy-seed 7 --checkpoint /tmp/q_policy.json --report /tmp/q_report.json
```

Its discrete choices are `target`, `dirt`, `frontier`, and `wait`; every
movement choice is converted to a curvature-limited reference trajectory. The
checkpoint records `truth_access_used=false`, and training creates environments
without an evaluation token.

## Formal dual-mode boundary

The research environment above is not the formal vehicle planner. The formal
contract is `config/formal_dual_mode_planning.yaml`: traditional full coverage
provides both fallback behavior and the paired per-episode distance ceiling;
the RL mode must consume the materialized public campus map plus
`/perception/garbage/targets` and `/perception/ground_dirt/masks`, emit a global
trajectory, and execute it through Nav2 `FollowPath`. Evaluator truth is
permitted only in the final scorer, never in either planner's observation.

`formal_planning_preflight` fails closed until all of the following are real:
the formal map runtime, a truth-free product-observation bridge, a Nav2
trajectory executor, a passing DOSOD+EdgeSAM preflight, a frozen truth-free
checkpoint, fixed-area/variable-aspect disjoint map splits, a paired full
coverage baseline, and held-out product-input validation meeting the 95%
thresholds without exceeding the baseline chassis distance. The existing
idealized-sensor tabular Q-learning demo is intentionally not accepted as that
evidence.

The formal product runtime requires both a frozen policy checkpoint and the
successful FullCoverage distance for the exact runtime map:

```bash
ros2 launch sanitation_active_cleaning formal_active_cleaning.launch.py \
  runtime_root:=/tmp/formal-campus-runtime \
  policy_checkpoint:=/tmp/formal-evidence/formal_planning/q_policy.json \
  maximum_task_distance_m:=3450.0 episode_seed:=823873385
```

The observation bridge accepts only the product topics
`/perception/ground_dirt/masks` and `/perception/garbage/targets`. The mask must
already have completed RGB-D ground projection: it is a contiguous `mono8` or
`8UC1` map-frame raster with exactly the public occupancy-grid dimensions and
row order. Value 0 is unobserved, 1 is observed clean, and 2..255 encodes dirty
confidence. Camera-frame masks are rejected rather than treated as map data.
The bridge publishes the accumulated trinary belief as
`/active_cleaning/ground_dirt_belief`, filtered product targets as
`/active_cleaning/garbage_targets`, and a freshness permit as
`/active_cleaning/observation_ready`.

The executor accepts `nav_msgs/Path` on `/active_cleaning/trajectory`. Every
pose and segment is checked against the public mission geofence and keepouts,
finite-value, frame, quaternion, spacing, count, and length limits before the
path is submitted to Nav2 `/follow_path`. It never publishes drive commands;
Nav2 output continues through collision monitoring and the whole-vehicle safety
manager. A stale/false `/safety/actuators_enabled`, explicit cancel, rejected
goal, failed result, or node shutdown cancels or blocks execution fail-closed.
Starting the formal launch now requires an explicit frozen checkpoint and runs
the product-observation bridge, truth-free policy planner, and FollowPath
executor. It never trains online and does not itself prove the policy passed
held-out product-input acceptance.

Discrete targets are first converted to a base-relative arm parking window at
`x=0.300 m, y=-0.950 m`; proximity to the vehicle centre alone can never issue
a grasp. The planner forwards the target's complete map-frame 3-D pose,
quaternion, measured dimensions and confidence in the schema-v2 grasp request;
missing or nonphysical geometry is rejected. Random material is deliberately
reported as `unknown` because the task randomizes mass independently of colour;
the dry-bin scale verifies an allowed material mass only after the physical
drop, so evaluator truth never enters grasp control. Brush, roller, pump and water recovery stay at zero until the cleaning
lift has physically reached its work pose with fresh joint feedback. After the
95% observation/cleaning gate, task distance is frozen for scoring and the
vehicle must navigate back to its fixed start before mission completion.

The formal multi-map trainer consumes the frozen campus generator split. It
materializes public maps, initializes each training simulator from evaluator
truth without exposing that truth to the policy, and freezes a shared
belief-only Q table across disjoint train/validation/hidden map IDs:

```bash
ros2 run sanitation_active_cleaning formal_active_cleaning_train \
  --scenario-config /path/to/default_scenario.yaml \
  --motion-profile /path/to/formal_motion_cleaning_profile.yaml \
  --work-root /tmp/formal-planning-work \
  --evidence-root /tmp/formal-evidence \
  --train 0:0,1:0 --validation 0:0 --test 1:0
```

The planning grid is a documented downsample of the 0.1 m product cost map;
rectangular dirty patches retain their generated area/aspect geometry. Hybrid
A* produces constant-curvature forward or reverse segments around static
assets, and Nav2 RPP is configured to allow reversing without inserting
in-place rotate-to-heading commands. The resulting validation report remains
`research_only_not_product_acceptance` until the held-out run consumes actual
DOSOD+EdgeSAM ROS outputs and physical-in-Gazebo bin verification.
