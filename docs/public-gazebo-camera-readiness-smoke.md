# Public Gazebo camera readiness smoke

`scripts/run_public_gazebo_camera_readiness_smoke.sh` is a bounded, `NON_FORMAL`
one-pair readiness smoke. It does not read or alter the canonical 25-scene plan,
does not materialize a dataset tensor, and does not publish GT, navigation,
coverage, or control commands.

The caller supplies a newly created empty regular output directory, a public
world and matching public episode manifest, frozen regular setup files, a
legal isolated `ROS_DOMAIN_ID`, the shared Gazebo lock, and an explicit total
deadline. The runner writes first to the admitted root, performs the existing
memory preflight, launches the existing camera-only campus path in its own
`setsid` PGID with navigation and coverage disabled, and binds the existing
9 GiB group-RSS watchdog to that PGID. It accepts exactly one RGB/CameraInfo
pair only when frame ID and source timestamp match and CameraInfo dimensions
and intrinsics are valid.

The receipt records the true exit code, partition, watchdog limit, deadline,
and exact-partition zero-survivor cleanup. A result of
`NON_FORMAL_CAMERA_READY` is only transport readiness; it is not a pilot,
full calibration, formal acceptance, or a stored camera frame.
