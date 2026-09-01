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
0.1143 m; the controller uses the same 0.1651 m rolling radius as the physical
collision and vendor wheel mesh contract.
The model uses skid-steer wheel axes because the selected A300 is skid-steer.
Each of the four vendor motor links now has a separate inboard
`*_encoder_link`, fixed cap joint, collision and redistributed inertia.  A
read-only encoder publisher converts continuous wheel joint position into a
4096-count/revolution simulation stream.  Clearpath does not publish the A300
encoder package CAD or resolution in the selected public description/manual,
so 4096 is explicitly an engineering simulation parameter; the as-built model,
polarity and resolution remain a hardware gate rather than a fabricated vendor
specification.

The platform macro also installs the 12.0 kg, 0.460 x 0.254 x 0.449 m UR
e-Series standard control-box cabinet (the public W x D x H dimensions, mounted
upright) and the connected Journey 6P / RDK S100P V1P0 compute assembly.
`s100_board_reference_link` temporarily uses the earlier RDK S100 public
0.121 x 0.120 x 0.0524 m envelope inside a project-authored protective
enclosure; this is a provisional collision reference, not a measured S100P CAD
claim. The reference link carries only numerical epsilon mass because the
official board mass is not published. The frozen 2 kg enclosure/I/O allowance
therefore remains conservative without inventing mass, thermal, connector or
power-integration evidence.

## Sensor suite module

`sensor_suite.xacro` exports
`hf_sensor_suite(mast_parent, base_parent, wrist_parent)`. For the formal
assembly these arguments are `sensor_mast_link`, `base_link`, and `tool0`.
The macro provides separate physical mount links, device links, joints, inertias and Gazebo sensor
blocks for the following devices. UTM-30LX, D435, MID-360, ANN-MB and VN100
external visuals use pinned redistributable meshes; the fish-eye housing and
mount are project-generated CAD for the frozen Arducam B0202/M27195H15 envelope
because no suitably redistributable vendor metrology mesh was available.

- Hokuyo UTM-30LX: 270 degrees, 0.1-30 m, 40 Hz;
- Livox MID-360 approximation: 360 degrees horizontal and -7 to +52 degrees vertical, 0.1-40 m, 10 Hz;
- front and wrist Intel D435 depth cameras: registered RGB/depth plus explicit
  left/right monochrome IR imagers at the physical 50 mm stereo baseline,
  87 x 58 degrees at 30 Hz;
- two independent Arducam B0202 fisheye cameras: 150 x 129 degree physical
  envelope, independent frames and topics at 30 Hz;
- a separately modeled ZED-F9P-04B receiver package in an open mast-side
  service enclosure plus the remote ANN-MB-00 sky-view antenna, at 10 Hz; and
- VN-100 IMU at 200 Hz.

The wrist D435 installation is an explicit three-level mechanical chain:
`tool0 -> wrist_rgbd_mount_link -> wrist_rgbd_link`, followed by the distinct
`wrist_rgbd_depth_optical_frame`, `wrist_rgbd_infra1_optical_frame` and
`wrist_rgbd_infra2_optical_frame` datums.  The front unit exports the same
three-frame camera geometry.  Infra2 is 50 mm to camera-right of Infra1; both
units publish two `L_INT8` image streams and independent `CameraInfo` topics in
addition to the existing RGBD image/depth/point-cloud chain.
The first physical link is the separately inertialized machined bracket; the
second is the independently bolted camera housing. The optical frame is a
massless coordinate frame and is not used to conceal either physical part.
The 236.709 g bracket is a rear-plane dog-leg: every load-carrying member stays
at least behind the D435 rear housing plane while the outboard rail reaches the
tool adapter. The deterministic full-mesh audit therefore still ray-tests the
bracket and measures 96.6946779% clear pregrasp depth FOV plus 9/9 visible cube
targets; no bracket link is added to the sensor ignore set.

Each rear fisheye uses SDFormat's native `wideanglecamera` sensor with an
`equisolid_angle` lens and a 150-degree horizontal field of view. Its ROS
`CameraInfo` remains the standard `equidistant` model, but now carries the
non-zero Kannala-Brandt coefficients `[-1/24, 1/1920, -1/322560,
1/92897280]`. Those coefficients are the theta^9 Taylor representation of
the exact SDF mapping `r = 2 f sin(theta/2)`; at the 75-degree image edge the
normalized-radius approximation error is below `5e-10`. The nominal focal
length `788.486223 px` follows only from 1920 pixels, 150-degree HFOV and
`scale_to_hfov=true`. These are explicitly simulation-derived parameters, not
a calibration of an Arducam serial number. The public M27195H15 material freezes
the 150 x 129 degree lens envelope, while measured intrinsics, per-unit
distortion, image crop and as-built installation calibration remain external
deployment gates. The parameter audit checks the wide-angle sensor type,
equisolid-angle lens, HFOV scaling and pi/2 cutoff in addition to
topic/rate/frame/clip/image dimensions. MID-360 is represented
by a dense raster GPU lidar; its real non-repeating Livox scan pattern remains a
sensor-plugin calibration task. The tower is a bolted twin-column load path with
independent UTM cantilever, MID-360 four-isolator top plate and side-lower ANN-MB
ground plane. The committed deterministic mesh-ray report passes the frozen
transport, pregrasp, pick and deposit poses plus the 3 cm target work zones;
real lens calibration and as-built installation remain external gates.

The GNSS receiver is no longer hidden in the antenna link.  The explicit chain
is `sensor_mast_link -> zed_f9p_receiver_enclosure_link ->
zed_f9p_module_reference_link`; the public u-blox `17 x 22 x 2.4 mm` package is
visible in an open project tray, with separate coax/power service cues.  The
antenna remains a sibling installation because it is remotely connected, and
the NavSat datum correctly stays at `gnss_antenna_link`.  The tray and receiver
mass are split from the existing GNSS mounting allowance; the module itself has
epsilon mass because the selected public u-blox material does not publish mass.

All three Pololu 4694 cleaning drives likewise expose distinct encoder-cap
links.  Their quantized output contract uses the manufacturer 64 CPR motor
encoder and nominal 70:1 gearbox (`4480` counts/output revolution).  Exact gear
ratio tolerance, backlash, electrical polarity and cable pinout remain
as-built calibration boundaries.

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

## Physical service-contact chain

The left power-service bay contains a 90-degree, lockable rotary battery
isolator and a normally-open 4 mm main-contactor armature.  Both are physical
URDF joints with state-only ros2_control interfaces.  Operator main-power
intent moves the real handle through bounded joint force; the contactor may
close only after the measured handle reaches ON, safety power is present and
the physical E-stop latch is released.  Applied branch state comes from the
measured contactor travel, not the command topic.  Missing joint or bridge
feedback is therefore an open circuit.  Their 0.400 kg explicit mass is an
equal redistribution from the former 0.700 kg lumped PDU allocation, leaving
the vehicle's empty mass unchanged.

`power_service_hardware.xacro` gives the charge receptacle and wastewater drain
coupling dedicated named collisions and preserved fixed joints. Their 50 Hz
Gazebo contact sensors publish on:

- `/formal_vehicle/gazebo/charge_receptacle/contact`
- `/formal_vehicle/gazebo/wastewater_drain_coupling/contact`

The default simulation launch bridges these one way to the product raw ROS
topics `/formal_vehicle/service/raw/charge_plug_contact` and
`/formal_vehicle/service/raw/drain_hose_contact`. Charging and service drainage
derive connector presence only from non-empty `ros_gz_interfaces/msg/Contacts`;
there is no synthetic connector-present Boolean. The passive charge door,
charge lock and drain service cap have state-only ros2_control interfaces so
their physical joint positions reach `/joint_states` without exposing a command
interface. The four bodywork service doors follow the same manual-service rule:
each has a chassis-fixed hinge bracket, a mechanically limited vertical hinge
and an independent rotary latch. Zero latch angle is the locked transport state;
hinge and latch positions are observable but intentionally have no powered
command interface.

The dry-bin and wastewater-tank lids also use explicit passive service
mechanisms rather than fixed decorative hardware. Each compartment has a
three-body over-centre latch (tank-mounted base, 70-degree hand lever and
lid-mounted keeper) plus its existing lid hinge. The four moving service joints
are state-only in ros2_control. Zero radians is the closed / locked transport
state; the documented human sequence releases the latch before lifting the lid.
No electric lid or tipper actuator is claimed. Wastewater maintenance emptying
uses the separate pipe, normally-closed ball valve, removable cap and hose
coupling described above.

For simulation acceptance, `ServiceDoorSystem` exposes evaluator-only target
topics and applies bounded spring/damper force to those same physical joints.
The ROS bridge for those targets is disabled by default and is enabled only by
the dedicated service-door acceptance runner.
It rejects hinge opening until the measured latch is beyond the unlock
threshold and prevents latch relocking until the measured hinge is closed.
The formal collector accepts only `/joint_states` feedback; the plugin does not
teleport joints or add a production ros2_control command interface.

## Physical emergency stop and lighting

The red mushroom stop is not decorative: `emergency_stop_plunger_joint` is a
0--6 mm prismatic joint with a state-only ros2_control interface. Exactly one
`FormalAuxiliaryVisualSystem` drives the engineering press/release motion,
measures the physical joint, treats missing feedback as asserted, latches the
result and accepts reset only when the external request is false, the measured
plunger is below 5 mm and the safety-power branch is live. Its one-way bridge
is the sole formal ROS writer of `/emergency_stop`; `simulation_safety_inputs`
and the whole-vehicle safety manager consume that state.

The same plugin drives the named front work-lamp, rear tail-lamp and four-corner
warning-lamp materials. Front work lamps also own model-following spot lights;
tail lamps own two red point lights, and warning lamps own four amber point
lights. Requested power-qualified
states and physical applied acknowledgements remain separate ROS topics under
`/formal_vehicle/lighting/`.
