# Competition motion and cleaning diagnostic, 2026-09-13

Scope: COMPETITION_HARD_MINIMUM; candidate base `1a211400ca2dc4d6007e4fd838e1f2835cfd1dc8`.
Use the existing isolated worktree and built runtime. No formal acceptance, full rebuild,
GitHub delivery, deployment or HBM claim is made by this diagnostic.

2026-09-13 follow-up: the bounded width / steady raster productivity / full moving E-stop results and retained contact-instrumentation failures are documented in [competition-width-efficiency-estop.md](competition-width-efficiency-estop.md).

Localization follow-up: [dynamic 50 mm diagnostic](competition-localization-50mm.md) records the failed first route, global-fusion drift, unresolved TF identity mapping, retained raw evidence and the real Gazebo GUI entry. Localization is not accepted.

Perception follow-up: [independent 30-frame scoring](competition-perception-score.md) records the single controlled RGB-D run, separate raw/policy P/R, corrected transition-frame truth and released resources. Recognition quality failed; no map-accuracy or cleaning-loop acceptance is claimed.

## Proven prior command chain

`NavigateToPose accepted -> controller /cmd_vel_nav (Twist, max 0.45 m/s)
-> smoother /cmd_vel_smoothed (Twist, max 0.45 m/s)
-> collision_monitor /cmd_vel_gate (Twist, max 0)
-> whole_vehicle_safety_manager /base_controller/cmd_vel (TwistStamped, max 0)
-> A300 drivetrain adapter/plugin`.

Mission-05 MCAP has 4,224 Nav2, 5,633 smoothed, 101 gate and 5,899 base commands.
The collision monitor reports `FootprintApproach`, action type 3. The supplemental
launcher copied generic Nav2 settings and consumed raw `/scan`, omitting the
formal lifecycle launch's mesh-derived self filter. Raw near returns around
2.094–2.356 radians, 0.10–0.23 m, lie inside the committed self-occlusion mask.
The diagnostic restores `/scan/navigation` for AMCL, both costmaps and collision
monitor; collision checking remains enabled. Live before/after motion is required
before calling this a verified fix. Twist type mismatch is ruled out by MCAP.

## Proven cleaning command disconnect

Ground-dirt-03 launched `enable_safety_manager:=false`, while its Probe published
brush arrays to `/safety/command/brush` and drive commands to `/cmd_vel_gate`.
Those are safety-manager inputs, not actuator-controller inputs. `wait_ready`
requires subscribers at both, so it never reached `run` or `set_work_pose`.
The launch log proves all eight requested controllers were configured and active;
`roller_ready=false` is a physical work-state predicate, not a spawner failure.

Mission-05 only published `/brush_enabled=true`. The safety manager uses this as
a speed-qualification state input; `_on_dry_brush_active` does not command motors.
The diagnostic supplies real brush arrays through the safety manager and a
cleaning-lift trajectory, with physical power/reset inputs and heartbeat.

## Run the bounded diagnostic

Run `scripts/run_competition_motion_cleaning_probe.sh` inside the existing native
guest with SOURCE, RUNTIME, OUTPUT (fresh), EPISODE, MAP_SOURCE and DRIVER set to
the authorized source, built runtime, evidence, public scenario, existing saved
map and `competition_motion_cleaning_driver.py` paths. The 120-second driver uses
a real Nav2 goal and records Gazebo model odometry solely for evaluation.
The shell records MCAP and seals it with SIGTERM before scoped runtime cleanup.
SIGINT is inherited as ignored by background commands in this shell context;
the first run's recorder was normally sealed with SIGTERM and its entire MCAP
was CRC-decoded locally. No reindexing or reconstructed footer is used.
The initial fixture uses the previous mission's map/start and a 3 m goal.

Preparation is separate from the 120-second motion window. The lift command uses
the 4.8 mm/s rated speed (20.9 s commanded trajectory for 100 mm), is sent once
after physical safety permit, and waits for measured `roller_ready`. A temporary
zero-only `/cmd_vel_gate` publisher maintains stationary command freshness during
preparation and is destroyed before the Nav2 goal. A 420 s wall cap accommodates
the observed low real-time factor and mechanical settling; it does not lower the
95 mm work-position/contact predicates. Optional `PROBE_SECONDS`, `PROBE_GOAL_X`
and `PROBE_ESTOP_DISTANCE` select the one continuous demonstration; the E-stop
triggers only after native GT shows movement and the specified travel distance.

Acceptance: nonzero base command, >2 m native model-odometry displacement,
`roller_ready=true`, and nonzero measured roller joint speed. A successful
diagnostic alone does not establish route completion, dirt removal, perception,
obstacle avoidance, moving E-stop, 20,000 m² mapping or measured productivity.

Local check: existing self-filter and lift-recovery tests 17 passed; driver Python compilation and
guest Bash syntax check passed. No ROS package source changed or rebuild needed.

Retained diagnostic failures: run 01 moved 2.982573 m and spun the roller at
12 rad/s, but its repeated early lift trajectories left only 44.6 mm at the
120 s endpoint. Run 02 sent the rated trajectory once but its 240 s preparation
cap expired at 85.95 mm. Neither is a passing joint probe. Run 03 retains the
same contact threshold with a longer bounded preparation phase. All evidence is
under the authorized cloud root `evidence/motion-cleaning-20260913-0{1,2,3}`;
local copies are in this worktree's `.work/motion-cleaning-20260913-01`.

## Camera-only development perception follow-up

The campus launch does not start a perception node. The user-authorized follow-up
reuses `garbage_perception_node` and the committed Stage5B learned ONNX model,
without starting the legacy Stage5B launch's ground-truth publisher. The new
`competition_development_perception.launch.py` maps the three legacy camera
inputs to the formal front D435 RGB, registered depth and CameraInfo topics.
The model card explicitly limits these weights to a procedural development
domain; emitted classes/positions do not establish competition accuracy.

Shortest startup after a campus camera runtime and map/TF are ready:

```bash
PYTHONPATH="$PERCEPTION_PYTHONPATH:$PYTHONPATH" ros2 launch \
  "$PERCEPTION_LAUNCH" model_path:="$PERCEPTION_MODEL"
```

Here `PERCEPTION_LAUNCH` is the absolute new launch file, `PERCEPTION_MODEL` is
`$SOURCE/artifacts/stage5b_20260719_review/stage5b_learned_perception.onnx`, and
`PERCEPTION_PYTHONPATH` is the task-local ONNX Runtime 1.22.1 wheel extraction.
The built ROS package is reused; the launch file executes directly by path.
`PROBE_CAMERAS=true` enables real sensors while `enable_training_gt=false` stays
fixed. The read-only `competition_perception_smoke.py` saves real RGB/depth bytes,
metadata, class/map-position samples and the product's actual subscription graph.

Smoke 01 received 235 RGB, 243 depth and 258 CameraInfo messages, and 117 target
messages in 90 wall seconds. Product inference counted 112 frames; live input
audit had no truth/evaluator topic. This is a camera-to-class-and-position
transport PASS only. `competition_perception_pass` remains false as reported by
the existing model/node. The integrated follow-up records perception alongside
the same physical route, cleaning and E-stop sequence; no target truth or
preconfigured object location is substituted for inference.
