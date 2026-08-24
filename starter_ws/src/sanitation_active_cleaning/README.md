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
