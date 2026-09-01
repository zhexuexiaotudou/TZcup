# A300 drivetrain realism integration

Status: formally wired into the 180-link final snapshot; deterministic URDF,
component-register and source-contract validation passed. Session-bound live
mobility, mapping, safety and dynamic-obstacle revalidation is still required.
This status must not be represented as runtime acceptance.

## Locked evidence and truth boundary

The base description is pinned to Clearpath `clearpath_common` commit
`b0f6d920422ad302372a1c65e31d61648da884ed` (BSD-3-Clause). The exact inputs
are the A300 top-level, motor, suspension-beam, outdoor-wheel and wheel-chain
Xacros plus `clearpath_control/config/a300/control/diff_4wd.yaml`. The public
A300 manual is the source for the 78.5 kg curb mass, 101.5 kg flat-ground
payload boundary, 25.6 V / 40 Ah battery, 17 A continuous current per motor,
60 A battery continuous current, 1080 W aggregate motor output, 2 m/s maximum
speed and resistive E-stop brake.

Clearpath explicitly describes A300 as having no suspension. The upstream
part called `suspension_beam` is therefore a bolted structural beam and must
remain connected by fixed joints. Adding springs, dampers or wheel travel to
this chain would reduce fidelity.

The locked upstream description does not publish separate motor, beam or
spacer masses, wheel-side torque constant, peak current, torque-speed curve,
control gain, torque slew, brake delay or brake torque. Values for these items
in the preparation contract are engineering allocations or calibration
parameters, not official A300 specifications. They must be replaced or
correlated using measured or identified CAD/vehicle data before a
hardware-correlated claim is allowed.

## Detailed rigid-body chain

For each side, `base_link` connects to an explicit spacer at
`[0, +/-0.192, 0.03763] m`. The spacer connects by a fixed joint to the
structural beam at local `y = +/-0.0159 m`. Front and rear motor bodies connect
to the beam at local `x = +/-0.256 m`, `y = +/-0.0095 m`, `z = -0.0085 m`.
Each motor has a fixed mount offset of `y = +/-0.0655 m`; the outdoor wheel is
the only continuous joint and rotates about local `[0, 1, 0]`. The composition
places each tyre centre at `|y| = 0.2829 m`, consistent with the locked 0.562 m
track to rounding.

The already-vendored Clearpath meshes are used for the motor, structural beam,
spacer and handed outdoor tyres. Their SHA-256 values are frozen in
`a300_drivetrain_realism_contract.yaml`. No primitive visual replacement is
planned. Primitive collision geometry may remain where it is more stable than
a triangle collision mesh, but it must be dimensionally tied to the source.

The provisional mass split assigns the published 78.5 kg curb mass across the
chassis/internal structure, four motor assemblies, two beams, two spacers,
four 2.5 kg outdoor wheels and 4 kg standard top plate. This is a redistribution
of the current aggregate chassis inertia, never additional mass. The split
must still be refined from measurements or CAD mass properties without
changing the published total.

## Effort-domain plant

`A300DrivetrainPlantCore` takes four commanded and measured wheel speeds and
returns four wheel torques. The drive request is proportional to speed error,
then limited by the minimum of the engineering low-speed torque cap, the
continuous per-motor current cap and the torque-speed power envelope. A second
aggregate limiter enforces 1080 W. Current estimation enforces both 17 A per
motor and 60 A at the battery. Drive torque also observes an engineering slew
rate.

The product fail-safe order is explicit: invalid numeric input, E-stop,
actuator-disable, any motor fault, and command age greater than 0.5 s all
inhibit the complete four-wheel drivetrain. Propulsion torque is removed
immediately. The separate resistive-brake model activates after an engineering
response delay and ramps to an engineering brake-torque cap opposite measured
wheel motion. Invalid wheel-speed feedback produces zero torque instead of a
fabricated braking estimate.

The Gazebo system consumes only Twist references, safety state,
fault state, bus voltage and measured joint velocity. It does not use actor
truth or estimate moving-obstacle velocity. A typed ROS adapter converts only
the unique whole-vehicle safety manager's final `TwistStamped` into the Gazebo
`Twist` transport and fails to zero/disabled on stale or invalid input. The
plugin derives odometry from measured wheel joint velocities and never reads
world pose truth.

The plugin and adapter are built by `sanitation_gazebo_control` and the formal
vehicle loads them exactly once. The former diff-drive controller and its four
wheel velocity command interfaces are removed, so the effort plant is the
only wheel command writer.

## Final command, odometry and TF authority plan

The formal migration removes `diff_drive_controller` completely rather than
leaving it loaded or inactive. The four wheel joints retain position and
velocity state interfaces only; `A300DrivetrainPlantSystem` is the sole writer
of `JointForceCmd`. The whole-vehicle safety manager remains the sole final ROS
command publisher on `/base_controller/cmd_vel`; the typed adapter is its only
consumer and emits the fail-closed Twist/enable pair bridged into Gazebo.

The plant integrates measured wheel-joint velocities and publishes raw
odometry only on Gazebo
`/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/odom`. A one-way bridge
maps that message directly to ROS `/odom/unfiltered`; neither the plant nor the
bridge publishes TF. `local_ekf` consumes `/odom/unfiltered` and IMU, is the
only `/odom` publisher, and is the only authority for
`odom -> base_footprint`. Its `base_link_frame` therefore changes from
`base_link` to `base_footprint`. The fixed robot-state-publisher edge
`base_footprint -> base_link` remains unchanged.

In mapping mode, `slam_toolbox` alone owns `map -> odom`. In saved-map cleaning
mode, `global_ekf` alone owns that edge and AMCL remains a pose measurement with
`tf_broadcast=false`. The sensor compatibility adapter has no odometry path.
The plant publishes `/odom/unfiltered` directly to the local EKF, which publishes
`/odom`. This leaves one owner for every selected
odometry and TF edge while preserving the `/odom` interface consumed by Nav2
and the formal dynamic-obstacle collector.

## Integration and acceptance sequence

The source integration follows this controlled migration; the remaining steps
are runtime acceptance and hardware correlation:

1. Keep the fixed spacer/beam/motor/mount links and the redistributed existing
   78.5 kg inertia; do not add an invented suspension joint.
2. Keep `base_controller` absent from controller manager/spawners and keep all
   four wheel velocity command interfaces before enabling the effort plant, so
   only one system writes wheel commands.
3. Use the typed adapter whose enable comes from the unique product
   safety authority. Gazebo helper topics must not become a second final
   publisher.
4. Resolve the current frozen model's use of 0.1651 m for control against the
   upstream controller's 0.1625 m effective odometry radius. Retain 0.1651 m
   as the physical collision radius.
5. Identify torque/current/brake parameters from acceleration, coast-down,
   loaded-grade and stopping tests. Validate stopping distance in the actual
   target environment as required by the A300 manual.
6. Start mapping localization with local EKF and SLAM TF ownership; start
   cleaning localization with local/global EKF and AMCL TF disabled. Prove one
   `/odom` owner and one publisher per `odom -> base_footprint` and
   `map -> odom` edge.
7. Re-run URDF mass/inertia, joint-interface, map lifecycle, mobility,
   dynamic-pedestrian, collision, E-stop and whole-vehicle safety acceptance.

Offline tests cover source/mesh hashes, no-suspension topology, exact transform
composition, mass conservation, speed clamp, torque-speed/current/power limits,
torque slew, timeout, delayed braking, E-stop, global motor-fault inhibition,
invalid-input safety, exact-once plant loading, the sole wheel writer, typed
safety bridge, raw odometry without TF, and local-EKF odometry/TF authority.
