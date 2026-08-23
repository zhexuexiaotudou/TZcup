# Journey 6 PC loopback HIL

This lane separates the PC sensor/plant from the Journey 6 algorithm graph. It
freezes the ROS 2 topic and QoS contract before hardware arrival without
claiming that proprietary OE, HBM execution, a 30-minute loop, or physical-board
acceptance has run.

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

The algorithm container writes `HIL_J6_PROCESS_LIST.txt` at graph startup. A
formal `J6_LOOPBACK_HIL_READY=true` decision additionally requires a real
30-minute loop, zero GT-control violations, zero PC duplicates, command-authority
pass, timeout/network safe-stop, and stale-replay rejection. Until those machine
results and a real OE image exist, this implementation is an interface-ready,
fail-closed lane rather than a passed HIL acceptance.
