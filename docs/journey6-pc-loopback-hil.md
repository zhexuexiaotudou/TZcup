# Journey 6 PC loopback HIL

This lane separates the PC sensor/plant from the Journey 6 algorithm graph. It
freezes and executes the ROS 2 topic/QoS contract before hardware arrival. Two
runtime identities are deliberately distinct: `JOURNEY6_OE` is the official
board-runtime path, while `PC_ONNX` is an x86 emulation path that must always
emit `not_journey6_runtime=true`.

The readiness flags are independent and fail closed:

The current trusted, run-bound attestation collector is not implemented, so
both the raw report evaluator and final status generator keep all four flags
false regardless of hand-authored JSON. Each diagnostic run now carries a
fresh UUID, but formal promotion remains blocked until one reviewed collector
binds its monotonic window to model metrics, process command lines, ROS endpoint
GIDs, Gazebo provenance, and official runtime/board identity.

- `J6_LOOPBACK_TRANSPORT_READY` requires at least 1800 seconds and proves a real
  ROS 2 split process graph, monotonic sensor/clock timestamps, TF/static TF,
  image-depth synchronization, the fixed QoS endpoints, zero GT-control use,
  the PC placement audit, command authority, and the complete fault matrix.
- `J6_LOOPBACK_ALGORITHM_READY` additionally requires the selected required
  model, a SHA-bound qualification manifest covering PT/ONNX parity, real PC
  inference, and a prior 1800-second full-stack run, a declared ONNX Runtime
  provider, no provider fallback, and successful inference.
- `J6_LOOPBACK_HIL_EMULATION_READY` additionally requires a real Gazebo sensor
  source, at least 1800 seconds, and the complete control/fault matrix.
- `J6_LOOPBACK_HIL_READY` retains its old meaning and can only be set by
  official Journey 6 OE/runtime evidence. `PC_ONNX` can never set it.

## Placement and authority

The PC side may run Gazebo rendering, simulated sensors, vehicle dynamics,
collision/contact, actuator execution, an independent evaluator, recording, and
the final safety gateway. Perception, localization/mapping, Nav2 planning,
Coverage, cleaning intelligence, control, and safety-state algorithms belong on
the J6 host. The gateway audits the live PC ROS graph and latches safe stop if a
blacklisted duplicate or an oracle/ground-truth node appears. J6 nodes must use
the fixed `/j6` namespace; an algorithm node outside `/j6` is treated as a PC
duplicate and fails closed.

`/hil/vehicle/ackermann_command` is a JSON envelope carried by
`std_msgs/msg/String`. Its fixed fields are `stamp_s`, `sequence`, `speed_mps`,
`steering_angle_rad`, `acceleration_limit_mps2`, `source_id`, and
`valid_until_s`. The PC gateway reuses
`sanitation_perception.journey6_hil.HilCommandAuthority`; it does not duplicate
planning or command-generation logic. The only accepted non-zero source is
`j6-algorithm`. The PC can preserve an accepted J6 command or emit a zero command
identified as `pc-safety-gate-zero-only`.

Startup, E-stop, stale health, command expiry, sequence replay, clock rollback,
physical-envelope violation, network loss, or placement violation all produce a
zero. Network recovery alone does not re-arm motion: a fresh J6 health sequence,
a clean PC placement audit, an inactive E-stop, an operator resume, and then a
new J6 command sequence are required.

The authoritative topic list and QoS values are installed from
`starter_ws/src/journey6_hil_gateway/config/hil_topic_qos_contract.yaml`.
Sensor streams use sensor-data QoS; control uses reliable depth 1 with an 80 ms
deadline and 120 ms lifespan; health is reliable; static TF/boundaries are
transient local.

## Proprietary OE boundary

The repository does not contain or redistribute a Journey 6 OpenExplorer image.
`Dockerfile.oe-wrapper` requires a local `J6_OE_BASE_IMAGE` and verifies that the
configured OE root contains an actual HBDK/HUCP/runtime executable. An absent or
generic image fails the build. `Dockerfile.algorithm-host` then builds the ROS
packages over that wrapper. There is no ONNX/CPU fallback when a J6 runtime is
requested.

The algorithm container mounts only these runtime inputs:

- the board/runtime bundle, read-only;
- model artifacts, read-only;
- the fixed HIL contract, read-only;
- a writeable evidence directory.

It never mounts a Gazebo world, ground-truth directory, garbage instance
registry, evaluator output, or sealed data. A private Docker network plus a Fast
DDS discovery server links the PC and J6 sides. Network faults are applied only
inside the J6 container network namespace (`NET_ADMIN`); host networking is not
used.

## Start and stop

### PC_ONNX emulation

The PC path loads and executes a real ONNX model with ONNX Runtime; it is not a
pure-function loop. The launcher verifies the model SHA-256 before starting.
It refuses to label an alternate diagnostic model as the required D1 model.
The algorithm-host default is Ubuntu 22.04/ROS 2 Humble
(`ros:humble-ros-base`); the PC gateway and synthetic compatibility harness use
Jazzy. `J6_ALGORITHM_ROS_IMAGE` and `J6_ALGORITHM_ROS_SETUP` may override the
algorithm image/setup explicitly, but a Jazzy algorithm-host run is diagnostic
and does not satisfy the V2 platform contract.

```bash
export J6_MODEL_ARTIFACTS=/absolute/path/to/model-directory
export PC_ONNX_MODEL_FILENAME=model.onnx
export PC_ONNX_MODEL_ID=d1_littercam_yolov9c
export PC_ONNX_MODEL_SHA256=<lowercase-sha256>
export PC_ONNX_REQUIRED_MODEL_ID=d1_littercam_yolov9c
export HIL_APPLY_NETWORK_FAULTS=true
bash scripts/run_j6_loopback_hil.sh \
  --runtime-backend PC_ONNX --duration-seconds 1800 --sensor-source gazebo
```

On PowerShell, the equivalent invocation is:

```powershell
$env:J6_MODEL_ARTIFACTS = 'C:\absolute\model-directory'
$env:PC_ONNX_MODEL_FILENAME = 'model.onnx'
$env:PC_ONNX_MODEL_ID = 'd1_littercam_yolov9c'
$env:PC_ONNX_MODEL_SHA256 = '<lowercase-sha256>'
$env:PC_ONNX_REQUIRED_MODEL_ID = 'd1_littercam_yolov9c'
$env:HIL_APPLY_NETWORK_FAULTS = 'true'
powershell -NoProfile -ExecutionPolicy Bypass -File `
  .\scripts\run_j6_loopback_hil.ps1 -RuntimeBackend PC_ONNX `
  -DurationSeconds 1800 -SensorSource gazebo
```

`synthetic_transport_probe` is useful only as a transport endurance diagnostic.
Even at 1800 seconds it cannot set any formal readiness flag, because V2 formal
transport acceptance requires the PC Gazebo/Jazzy sensor graph.

`J6_MODEL_ARTIFACTS` must also contain
`model_qualification_manifest.json`. Its model ID/SHA and three referenced
evidence files are hashed and content-checked; an environment boolean cannot
qualify a model. The Gazebo lane separately requires a hashed
`HIL_GAZEBO_SENSOR_PROVENANCE.json` binding the audited sensor/plant-only
launch, live Gazebo processes, and publisher endpoints. In Gazebo mode the
harness creates no sensor/clock/TF publishers of its own.

The launcher does not substitute the repository's full product simulation for
that PC graph: the full launch contains algorithm nodes and evaluator/truth
components that are forbidden in this split. For `--sensor-source gazebo`, an
operator must first provide a dedicated Jazzy **sensor/plant-only** launch that
publishes the fixed `/hil/*` sensor topics and consumes only the validated
actuator topics. If that launch is absent, the harness records zero sensor
traffic and all readiness flags remain false.

### Official Journey 6 runtime

Set these values to real local resources. The algorithm command must start the
J6-side graph, not a PC duplicate:

```bash
export J6_OE_BASE_IMAGE=<local-proprietary-image:tag>
export J6_OE_ROOT=/opt/journey6
export J6_ROS_SETUP=/opt/ros/<actual-j6-ros-distro>/setup.bash
export J6_RUNTIME_BUNDLE=/absolute/path/to/runtime-bundle
export J6_MODEL_ARTIFACTS=/absolute/path/to/model-artifacts
export J6_ALGORITHM_COMMAND='<actual J6 algorithm launch command>'
export ROS_DOMAIN_ID=66
bash scripts/run_j6_loopback_hil.sh
```

The launch command must place every algorithm node under `/j6` (for example,
with a launch namespace or `--ros-args -r __ns:=/j6`). The container exports
`TZCUP_HIL_NODE_NAMESPACE=/j6` so a project launch can consume the same fixed
value. Every J6 algorithm node must retain the incoming sensor acquisition
timestamps and consume `/hil/clock` (or remap its ROS `/clock` subscription to
that topic). The PC gateway already runs with `use_sim_time=true` and that
remap; no wall-clock substitution is accepted for command expiry.

PowerShell uses the same environment variables:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_j6_loopback_hil.ps1
```

The launcher writes `HIL_PC_DDS_ENV.sh` into the evidence directory. Source it
before launching a PC **sensor/plant-only** graph. Do not use the full product
launch in HIL mode because it contains algorithm duplicates. Stop without
deleting images or evidence:

```bash
bash scripts/stop_j6_loopback_hil.sh
```

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\stop_j6_loopback_hil.ps1
```

After healthy J6 startup, publish a fresh health frame and perform an explicit
operator resume. A representative health payload is:

```json
{"source_id":"j6-algorithm","sequence":1,"stamp_s":0.1,"healthy":true}
```

## Fault injection and evidence

`scripts/j6_hil_network_faults.py` is dry-run unless `--apply` is present. Run
the image-baked copy inside the J6 container for `delay`, `loss`, `bandwidth`,
`disconnect`, or `normal` (restore). For example:

```bash
docker exec tzcup-j6-loopback-j6-algorithm-1 \
  python3 /opt/tzcup/bin/j6_hil_network_faults.py disconnect --apply \
  --evidence /evidence/HIL_NETWORK_DISCONNECT.json
```

Every loss or delay test must show the validated command becoming zero within
the command/health bound. Restore must not replay the old non-zero command;
motion resumes only after a fresh health sequence, operator resume, and a new
command sequence.

The gateway continuously updates:

- `HIL_NODE_PLACEMENT.json`;
- `HIL_PC_PROCESS_LIST.txt` (captured by the host launcher);
- `HIL_PC_GATEWAY_PROCESS_LIST.txt` (the gateway container namespace);
- `HIL_ROS_GRAPH.json`;
- `HIL_COMMAND_AUTHORITY.json`.
- `HIL_ROS_QOS_INFO.txt` (live DDS endpoint QoS captured with a Fast DDS super
  client);
- `HIL_ALGORITHM_RUNTIME.json` (runtime identity, model/hash/provider, actual
  inference counters, synchronization counters, and actual network actions);
- `J6_LOOPBACK_HIL_EMULATION_REPORT.json` (machine-derived V2 statuses and the
  safety/fault matrix).

The algorithm container writes `HIL_J6_PROCESS_LIST.txt` at graph startup. The
PC_ONNX harness exercises command timeout, an actual container-network
disconnect/restore, manual resume, stale-sequence injection, E-stop, and a live
blacklisted PC node injection. A 30-minute PC result remains emulation evidence,
never official Journey 6 runtime or board acceptance.
