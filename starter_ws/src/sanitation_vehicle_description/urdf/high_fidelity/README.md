# High-fidelity vehicle module notes

## A300 platform module

`a300_platform.xacro` exports `hf_a300_platform()`. It creates the REP-105
`base_footprint`/`base_link` pair, `payload_deck_link`, `sensor_mast_link`,
`arm_mount_link`, and four independently articulated wheels. The visible base,
livery, lights, top plate and tyres use the pinned Clearpath BSD meshes; reduced
boxes/cylinders are collision geometry only. Base, wheels and standard top plate
retain the published 78.5 kg A300 curb-mass allocation. The wheel joints
are:

- `front_left_wheel_joint`
- `front_right_wheel_joint`
- `rear_left_wheel_joint`
- `rear_right_wheel_joint`

The current Clearpath model supplies the 0.512 m wheelbase and 0.566 m
tyre-centre track. The mesh/collision tyre radius is 0.1651 m and width is
0.1143 m; the controller uses the separate 0.1625 m effective rolling radius.
The model uses skid-steer wheel axes because the selected A300 is skid-steer.

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
The macro provides separate physical mount links, device links, joints, inertias and Gazebo sensor
blocks for the following devices. UTM-30LX, D435, MID-360, ANN-MB and VN100
external visuals use pinned redistributable meshes; the fish-eye housing and
mount are project-generated CAD because the exact camera SKU remains pending.

- Hokuyo UTM-30LX: 270 degrees, 0.1-30 m, 40 Hz;
- Livox MID-360 approximation: 360 degrees horizontal and -7 to +52 degrees vertical, 0.1-40 m, 10 Hz;
- front and wrist Intel D435 depth cameras: 87 x 58 degrees at 30 Hz;
- two independent Arducam B0202 fisheye cameras: 150 x 129 degree physical
  envelope, independent frames and topics at 30 Hz;
- ZED-F9P/ANN-MB-00 GNSS at 10 Hz; and
- VN-100 IMU at 200 Hz.

Gazebo's camera sensor uses a pinhole projection for each fisheye image. The
150-degree visibility envelope is simulated; a real equidistant calibration and
distortion stage is not yet present and remains an explicit fidelity boundary. MID-360 is represented
by a dense raster GPU lidar; its real non-repeating Livox scan pattern remains a
sensor-plugin calibration task. The tower is a bolted twin-column load path with
independent UTM cantilever, MID-360 four-isolator top plate and side-lower ANN-MB
ground plane. Exact mesh ray occlusion remains subject to the 3 cm target-visibility gate.

## Manipulator and gripper module

`manipulator_stack.xacro` exports `hf_manipulator_stack(parent)` and is called
with `parent="arm_mount_link"`. It provides a complete six-axis UR5e chain with
the nominal 0.425 m upper arm, 0.3922 m forearm, 0.1333/0.0997/0.0996 m wrist
offsets, the official two-layer base frame, flange and `tool0`. Visual and
collision geometry comes from the pinned official UR description; the nominal
UR5e physical-link masses sum to 20.6 kg. The command joints are:

- `shoulder_pan_joint`
- `shoulder_lift_joint`
- `elbow_joint`
- `wrist_1_joint`
- `wrist_2_joint`
- `wrist_3_joint`

The Robotiq 2F-85 model uses the upstream adapter and full palm, left/right
outer/inner knuckles, finger bodies and fingertip meshes.
`robotiq_85_left_knuckle_joint` is the
single commanded closure joint. The right outer knuckle and both inner knuckles
use URDF `mimic` relations. This keeps the gripper one-DOF while retaining
separate contact geometry. Final full-space self-collision, vendor calibration
offsets, effort tuning and grasp-contact tuning remain formal simulation gates.
The formal controller configuration uses a six-joint `arm_controller` and an
independent one-joint `gripper_controller`; both reject partial goals and enforce
explicit path and terminal tolerances.
