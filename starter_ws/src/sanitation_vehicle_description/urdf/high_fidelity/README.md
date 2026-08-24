# High-fidelity vehicle module notes

## A300 platform module

`a300_platform.xacro` exports `hf_a300_platform()`. It creates the REP-105
`base_footprint`/`base_link` pair, `payload_deck_link`, `sensor_mast_link`,
`arm_mount_link`, and four independently articulated wheels. The project-owned
primitive geometry stays inside the published 0.990 x 0.698 x 0.372 m A300
envelope and the physical platform-link masses total 78.5 kg. The wheel joints
are:

- `front_left_wheel_joint`
- `front_right_wheel_joint`
- `rear_left_wheel_joint`
- `rear_right_wheel_joint`

The 0.580 m wheelbase and 0.584 m tyre-centre track are current layout values,
not fabrication measurements. They must pass the dimensioned-drawing gate.
The tyre collision radius is 0.186 m and the tyre width is 0.114 m. The model
uses skid-steer wheel axes because the selected A300 is a skid-steer platform.

The platform macro also installs the 12.0 kg, 0.460 x 0.254 x 0.449 m UR
e-Series standard control-box cabinet (the public W x D x H dimensions, mounted
upright) and an S100/Journey 6 reference compute enclosure. The S100 assembly is
explicitly SKU-pending: its project reference envelope is 0.200 x 0.150 x
0.080 m, while `s100_board_reference_link` carries only numerical epsilon mass.
This preserves the frozen 2 kg enclosure/I/O allowance without inventing the
user's eventual board dimensions or board mass.

## Sensor suite module

`sensor_suite.xacro` exports
`hf_sensor_suite(mast_parent, base_parent, wrist_parent)`. For the formal
assembly these arguments are `sensor_mast_link`, `base_link`, and `tool0`.
The macro provides separate physical links, joints, inertias and Gazebo sensor
blocks for:

- Hokuyo UTM-30LX: 270 degrees, 0.1-30 m, 40 Hz;
- Livox MID-360 approximation: 360 x 59 degrees, 0.1-40 m, 10 Hz;
- front and wrist Intel D435 depth cameras: 87 x 58 degrees at 30 Hz;
- two independent Arducam B0202 fisheye cameras: 150 x 129 degree physical
  envelope, independent frames and topics at 30 Hz;
- ZED-F9P/ANN-MB-00 GNSS at 10 Hz; and
- VN-100 IMU at 200 Hz.

Gazebo's camera sensor uses a pinhole projection for each fisheye image. The
150-degree visibility envelope is simulated, while the real equidistant lens
calibration and distortion are applied downstream in ROS. MID-360 is represented
by a dense raster GPU lidar; its real non-repeating Livox scan pattern remains a
sensor-plugin calibration task. Exact sensor transforms remain subject to the
self-occlusion and 3 cm target-visibility gate.

## Manipulator and gripper module

`manipulator_stack.xacro` exports `hf_manipulator_stack(parent)` and is called
with `parent="arm_mount_link"`. It provides a complete six-axis UR5e chain with
the nominal 0.425 m upper arm, 0.3922 m forearm, 0.1333/0.0997/0.0996 m wrist
offsets, a `tool0` frame, and a project-authored primitive collision model. The
UR5e physical-link masses sum to 20.6 kg. The command joints are:

- `shoulder_pan_joint`
- `shoulder_lift_joint`
- `elbow_joint`
- `wrist_1_joint`
- `wrist_2_joint`
- `wrist_3_joint`

The 0.9 kg Robotiq 2F-85 model contains a palm, left/right outer and inner
knuckles, finger bodies and contact pads. `robotiq_85_left_knuckle_joint` is the
single commanded closure joint. The right outer knuckle and both inner knuckles
use URDF `mimic` relations. This keeps the gripper one-DOF while retaining
separate contact geometry. Final mesh-level self-collision, vendor calibration
offsets, effort tuning and grasp-contact tuning remain formal simulation gates.
