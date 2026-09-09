# Formal simulation incident ledger

This file records runtime failures that are expensive to rediscover.  A failed
attempt is not retried from the same runtime tree after source changes: rebuild
the affected packages, use fresh ROS/Gazebo identities and a fresh evidence
root, then pass the smallest applicable gate before starting a long run.

## 2026-09-10: final preview initial Spin produced no observed yaw

- Attempt: `final-product-visual-finalclose-20260910T014035`.
- Terminal result: explorer state `initial_scan_sweep_blocked`, reason
  `spin_failed`, Nav2 status `6`, error code `701` (`TIMEOUT`).
- Direct evidence: `behavior_server` accepted the Spin action, requested
  `1.57 rad`, and the HMI observed a non-zero commanded angular velocity on
  `/base_controller/cmd_vel`.  The live `/odom` and `map -> base_footprint`
  yaw did not change materially before the action timed out.
- Do not repeat these diagnoses: the formal vehicle plugin, command adapter,
  native bridge and plant all use the same `FL, FR, RL, RR` order; all four
  wheel joints use the declared `+Y` axis; the ROS and Gazebo bridges copy
  `angular.z` without remapping.  Do not relax the Nav2 footprint or blindly
  flip URDF wheel axes to address this incident.
- Remaining boundary: the prior run did not retain
  `/safety/status_json`, the A300 plant input/enable/commanded-wheel/measured-
  wheel status, or raw `/odom/unfiltered` during the failure.  Therefore it
  cannot distinguish an effective-permit dropout from a DART effort/contact
  response failure.
- Prevention added: the plant status now exposes input Twist, effective
  actuator enable, commanded and measured wheel speeds, applied torques and
  reconstructed odometry Twist.  A pure kinematics regression freezes the
  expected `v=0, wz=0.35` left/right signs.  The formal mobility probe has a
  short real-DART Spin gate; this gate must pass before the final preview is
  restarted.
- Long-run rule: never start the map/clean preview merely because Spin was
  accepted or a command topic was non-zero.  Require effective permit, opposite
  left/right measured wheel means, non-zero same-sign raw odometry yaw rate,
  accumulated yaw, and a bounded stop after the zero command.

### Proven wheel-contact root cause and bounded correction

- The first corrected plant runs still could not rotate because the wheel
  contact contract had been dropped by URDF-to-SDF conversion.  A named wheel
  collision combined with a collision-level Gazebo extension produced runtime
  wheel collisions without `<surface>`; DART therefore used its default
  friction instead of the intended longitudinal/lateral values.
- The accepted source form is an unnamed wheel `<collision>` plus a link-level
  `<gazebo reference="..._wheel_link">` extension.  Before every long run,
  expand the complete vehicle with `xacro`, convert it with `gz sdf -p`, and
  require all four runtime wheel collisions to contain `mu=0.9`, `mu2=0.72`,
  `fdir1=1 0 0`, `kp=1000000`, `kd=100`, `min_depth=0.001` and
  `max_vel=0.1`.  Source-text assertions alone are not sufficient.
- Fresh DART probes after the surface fix established real counter-rotation:
  P110 reached about `0.0537 rad` yaw in 2 simulation seconds; P120 reached
  about `0.0826 rad`, with opposite-sign left/right wheel displacement and a
  peak yaw rate near `0.0537 rad/s`.  The old `0.1 rad / 2 sim s` diagnostic
  threshold was not met, but the vehicle was no longer locked.
- `120 N m/(rad/s)` is the upper allowed counter-rotation speed-error gain.
  At the `0.25 rad/s` Spin target it requires about `51.88 N m` per wheel and
  `59.29 A` total, immediately below the declared `60 A` continuous battery
  boundary.  Never increase the gain above 120 or weaken the current limit to
  chase a short diagnostic threshold.  Forward/general gain remains 12.
- With the measured low real-time factor, the Nav2 initial Spin uses a
  `60 s` simulation allowance and `600 s` wall-clock result deadline.  This is
  a calibrated execution bound, not permission to remove the no-progress
  watchdog.
- The first full preview after the contact fix exposed a second-order mismatch:
  Nav2 behavior_server emitted its configured minimum `0.20 rad/s`, not the
  `0.25 rad/s` used by the successful breakaway probe.  Plant telemetry showed
  about `-42/+41 N m` commanded torque but all measured wheels still creeping
  in the same direction, followed by a genuine Spin time-allowance failure.
  The shared Nav2 minimum rotation is therefore `0.25 rad/s`; keep the dynamic
  plant telemetry gate so a future parameter overlay cannot silently restore
  the ineffective `0.20 rad/s` command.

## 2026-09-10: preview could stay alive without proving product progress

- Symptom found during predictive review: many mapping/cleaning failures used
  direct `exit`, so the trap could tear down the HMI before publishing an
  immutable terminal receipt.  Cleaning could also remain in reload/localize
  for the full 24-hour completion timeout without ever proving live coverage.
- Prevention added: all expected runtime failures and normal completion now
  pass through one `PRODUCT_TERMINAL` publisher which waits for a matching HMI
  receipt and saves `hmi_terminal_telemetry.json`.  Mapping guards retain the
  actual source state/reason, and cleaning must enter a live
  `PLANNING|TRANSIT|CLEANING` state within the shared 600-second progress bound.
- `--preflight` is now a real admission pass rather than a parameter-only
  check.  It executes the Windows memory gate, obtains the Gazebo lease,
  sources the selected runtime, verifies all required package prefixes resolve
  below that install, and performs the complete expanded-wheel-surface check.
  Use a fresh preflight root because its evidence is intentionally retained.
- `gz sdf -p` currently warns that `gz_frame_id` is not part of core SDF and
  copies it through as an extension.  The fields remain serialized; record the
  warning, but do not confuse it with a dropped wheel surface or retry the run
  solely because of this known conversion warning.
- Expose the detailed cleaning-motor status stream in the HMI and terminate a
  mapping preview after a bounded 90-second persistent motor fault.  A safety
  reason of `cleaning_motor_fault_active` alone is not enough attribution;
  retain the per-motor `command_timeout`, `stall`, `overtemperature`, or
  `invalid_input` reason before changing controller or safety semantics.
- The first typed-telemetry run identified `cleaning_lift:stall` as the sole
  persistent motor fault. After the latch, safety correctly disabled the
  actuators and the mirror held `command == measured_position ~= 0.001303 m`
  with zero current, so that final frame was not a new zero-current stall.
  The detailed assembly consistently seats about 1.3 mm above authored zero;
  the old 0.5 mm observer deadband therefore synthesized rated current and
  latched a false lower-stop stall. The lift-only deadband is now 2 mm and a
  long-step regression proves the seated offset stays idle. Do not clear a
  live latch from its post-stop zero-current frame, and do not weaken the
  100 mm deployment stall test or the safety latch/reset semantics.
- After the corrected Spin completed, the frontier selector rejected the
  nearest known-free seed as `internal_seed_not_footprint_safe`. A live full
  `/map` probe showed the current vehicle footprint contained 486 free cells,
  160 unknown cells and zero occupied cells: lidar cannot ray-clear the cells
  hidden under its own body. The selector now permits only a clearance-bounded
  bootstrap through known-free center cells to the first fully safe anchor;
  goal/reachable traversal still rejects unknown, and the narrow-corridor test
  remains fail-closed. Do not solve this incident by enabling Navfn unknown
  traversal or by unconditionally shrinking the vehicle clearance.
- The first retry still emitted the old `internal_seed_not_footprint_safe`
  result although the source no longer contained that return.  The selected
  install copy was about three hours older than the source.  Every preview now
  writes `runtime_overlay_freshness.json` and refuses to launch unless the
  current interpreter's complete Python packages, installed launch/config
  payloads, and successful C++ build receipt all resolve below and match the
  selected runtime workspace.  A package-prefix check alone cannot prove that
  the code inside that prefix is current.
- Once the fresh bootstrap code ran, its next diagnostic was
  `no_footprint_safe_bootstrap_anchor`.  The nominal 0.95 m circular envelope
  had been implemented as a 41-by-41 square at 0.05 m resolution, requiring
  corner cells as far as 1.414 m from the candidate to be known-free.  The
  selector now checks a conservative cell-intersection circle with radius
  `0.95 + resolution/sqrt(2)`; it still rejects every unknown/occupied cell
  inside that mask and retains the bounded known-free bootstrap.  Do not
  restore a square approximation or extend the bootstrap through unknown.
- A subsequent live map proved that the remaining unknown cells were not an
  excuse to enlarge the bootstrap.  The UTM-30LX is nominally 270 degrees, but
  the two mesh-derived rear-edge self-return masks leave only about 242.5
  degrees of contiguous usable scan.  The original 90-degree Spin therefore
  covered only about 332.5 degrees and deterministically left a 27.5-degree
  unknown wedge beside the vehicle.  The initial Nav2 Spin is now 135 degrees,
  covering the 117.5-degree effective blind sector with 17.5 degrees of margin
  for action tolerance, beam discretization and mount uncertainty.
  If the masks change, recompute the sweep; do not bypass unknown cells or
  shrink the physical footprint.
- Position-controller spawning and brush/recovery loading previously raced
  each other while the safety manager already attempted strict activation.
  This produced deterministic `controller does not exist` and strict-switch
  rejection noise and could hide a missing cleaning controller.  The launch
  is now sequenced as position group completion, then inactive brush/recovery
  loading, then the sole safety manager.  Keep strict switching; do not replace
  this ordering with `BEST_EFFORT`.
- A forced stale-run stop also proved that dashboard/support children inherited
  the Gazebo lease descriptor. They now close fd 9 at exec, so an orphaned HMI
  cannot retain `/tmp/tzcup_formal_gazebo.lock` after the simulator is gone.
- Process cleanup must not assume every background helper is its own process
  group.  The preview now resolves each helper's actual PGID, signals the group
  only when `PGID == PID`, otherwise signals the exact PID, and bounds TERM and
  KILL waits.  Both heavy launch children also close the Gazebo lease FD before
  exec so a crashed parent cannot leave a survivor holding a ghost lock.
- The saved-map coverage executor originally published only on phase changes;
  a long Nav2 action therefore made its HMI source stale even while healthy,
  while the preview stopped checking that source after the first CLEANING
  receipt.  The executor now republishes its current state with a monotonic
  sequence once per second, and the preview continuously rejects unavailable
  or FAILED executor state instead of waiting for the 24-hour outer timeout.
- A live Nav2 arc commanded `v=-0.12 m/s, w=0.35 rad/s`, corresponding to left
  and right wheel targets of about `-1.344` and `-0.133 rad/s`, while all four
  measured wheels collapsed to about `-0.725 rad/s` and measured yaw remained
  zero.  The plant selected its high tyre-scrub breakaway gain only when side
  commands had opposite signs, so same-direction skid-steer arcs incorrectly
  used the weak straight-line gain.  The selector now treats every unequal
  left/right side command as differential steering; equal-side straight motion
  still uses the general gain, and the existing torque, current, power and slew
  limits remain authoritative.  Retain the captured same-direction arc as a
  regression case; do not wait for a Nav2 progress timeout to diagnose this
  signature again.
- The 135-degree sweep then reached the selector but still returned
  `no_footprint_safe_bootstrap_anchor`: the lidar self-filter necessarily left
  unknown cells under the vehicle, so a fully known 0.95 m envelope could not
  be placed at the initial pose.  Public scenario manifests now carry only the
  generator-derived 1.5 m collision-free start certificate.  The explorer may
  use it once, after the initial sweep, to cross grid-internal unknown cells
  wholly inside that circle and reach a fully known-safe anchor.  Occupied,
  geofence-outside and raster-outside cells remain forbidden, and subsequent
  goals receive no certificate.  Do not read evaluator truth or generalize
  this exception to normal frontier traversal.  The eligibility gate allows
  at most 0.75 m of map-frame start estimate drift because the physical Spin
  already demonstrated 0.53 m; this gate does not enlarge the certified circle
  because every excused unknown cell is still checked against its boundary.
- A dynamically expanding SLAM raster has implicit unknown space immediately
  outside its current array.  The selector previously discarded every
  known-free raster-edge cell before testing the geofence, which could report
  zero usable frontiers even though the physical field continued.  A raster
  edge is now a frontier only when its adjacent world coordinate remains
  inside the public geofence; the search window also includes the larger of
  frontier standoff and clearance.  Keep `raw_frontier_evaluated` separate
  from its count so an early bootstrap failure is not misdiagnosed as a scan
  with zero frontiers.
- Low-real-time-factor execution exposed a controller-switch re-entry race:
  the timer saw a completed future before its callback recorded the
  authoritative state and issued another STRICT request.  The safety manager
  now retains exclusive ownership until the callback records state and clears
  the future.  Simulator-only sensor heartbeat thresholds are 1-2 seconds to
  tolerate CPU scheduling at low RTF; the command timeout and hardware
  defaults remain unchanged.  Repeated activate/deactivate cycles or STRICT
  rejection are therefore runtime regressions, not harmless startup noise.

## Runtime interpretation traps

- `/base_controller/cmd_vel` is the safety manager's **commanded** velocity,
  not measured vehicle velocity.  Execution claims must use plant wheel
  feedback and `/odom/unfiltered`; the HMI labels command and measurement
  separately.
- A shell opened after launch may see no ROS graph when it does not inherit the
  run's ROS domain, localhost isolation and CycloneDDS configuration.  Treat
  that result as an environment mismatch, not proof that a runtime topic is
  absent.
- Low real-time factor is not a stalled clock.  The failed attempt ran near
  `0.12-0.15 RTF`; simulation-duration gates therefore also need a wall-clock
  hard limit and a separate no-progress watchdog.
- Source ROS before enabling shell nounset.  Sourcing Jazzy under `set -u` can
  fail on setup variables such as `AMENT_TRACE_SETUP_FILES`; the runner
  temporarily disables nounset only around the two setup files and immediately
  restores it.
- With the sequenced campus launch, `controller does not exist` or strict
  switch rejection during loading is no longer an accepted transient.  Treat
  either as a startup regression and require both managed controllers plus the
  effective safety permit before motion.
- An explorer terminal state must be published to the HMI before teardown.
  The runner records `PRODUCT_TERMINAL`, waits for its HMI receipt, saves a
  terminal telemetry snapshot, and preserves the original explorer state and
  reason in `terminal.json`.

## Resource and retry discipline

- Run Windows/WSL memory preflight before launch and keep the runtime memory
  watchdog active.  One heavy Gazebo product run per WSL instance is the safe
  default; use agents for source, test and evidence work rather than starting
  competing simulators.
- A failed or source-obsolete run is cleaned by its exact ROS domain and
  Gazebo partition.  Verify that no process from that identity survives before
  reusing memory.
- Preserve failed logs and telemetry under the attempt's evidence root.  Do
  not overwrite them with a retry and do not interpret an older PASS artifact
  as evidence for changed source.
