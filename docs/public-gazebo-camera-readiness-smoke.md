# Public Gazebo camera readiness smoke

`scripts/run_public_gazebo_camera_readiness_smoke.sh` is a bounded, `NON_FORMAL`
one-pair readiness smoke. It does not read or alter the canonical 25-scene plan,
does not materialize a dataset tensor, and does not publish GT, navigation,
coverage, or control commands.

The caller supplies a newly created empty regular output directory, a public
world and matching public episode manifest, frozen regular setup files, a
legal isolated `ROS_DOMAIN_ID`, the shared Gazebo lock, and an explicit total
deadline. All supplied paths must be regular, non-link paths inside this
worktree; the sole system exception is the exact regular
`/opt/ros/jazzy/setup.bash`. The runner bounds preflight and setup
materialization by the same deadline, emits only a whitelisted ROS/Gazebo
environment snapshot (never inherited credentials), launches the existing
camera-only campus path in its own verified `setsid` PGID with navigation and
coverage disabled, and binds the existing 9 GiB group-RSS watchdog to it.

The production probe is fixed to
`scripts/public_gazebo_camera_pair_readiness.py`; it cannot be overridden by a
caller. It accepts one pair only when the frame ID and timestamp are exact,
publisher identities/types are present, CameraInfo dimensions/intrinsics are
valid, and image encoding is `rgb8`, `bgr8`, `rgba8`, or `bgra8` with enough
row stride and bytes. Each pending topic cache is capped at four entries. A
`NON_FORMAL_CAMERA_READY` receipt is written only after the same canonical
probe revalidates its strict report and after exact probe/launch PGID plus
partition cleanup reports no survivors. The receipt hashes the probe, report,
all bound inputs, and watchdog evidence.

`PUBLIC_GAZEBO_CAMERA_SMOKE_ROS_SETUP` is required and must name the exact
system setup above or a regular worktree-local setup file. A real Jazzy invocation passes
`/opt/ros/jazzy/setup.bash`; the fixture passes only its own empty regular
stub, so source-only CI never assumes that ROS is installed.

The receipt records the true exit code, partition, watchdog limit, deadline,
and exact-partition zero-survivor cleanup. A result of
`NON_FORMAL_CAMERA_READY` is only transport readiness; it is not a pilot,
full calibration, formal acceptance, or a stored camera frame.
