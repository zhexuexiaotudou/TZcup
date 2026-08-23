# Journey 6 split-loopback HIL V2 report

Date: 2026-08-23

## Result

The PC_ONNX split graph is implemented and has executed in real Docker/ROS 2
processes. It remains fail closed: all four formal readiness states are false.
No result below is Journey 6 OE, HBM, board, Gazebo, or D1 acceptance.

The trusted run-bound attestation collector is not implemented. Reports now
emit a generated UUID `run_id`, `formal_attestation_evaluator_available=false`,
and `status_evaluator_blocked=trusted_run_bound_attestation_collector_not_implemented`.
Formal evaluation is therefore hard-disabled rather than trusting replayable
JSON booleans. Diagnostic counters remain available.

```text
J6_LOOPBACK_TRANSPORT_READY=false
J6_LOOPBACK_ALGORITHM_READY=false
J6_LOOPBACK_HIL_EMULATION_READY=false
J6_LOOPBACK_HIL_READY=false
runtime_backend=PC_ONNX
not_journey6_runtime=true
```

## 1800-second endurance diagnostic

Evidence directory:
`artifacts/j6_loopback_pc_onnx_v2_transport_1800s/`
(compact machine report SHA-256
`6c66f462fd9e4469b8a3d0c1faf3cc23f3866ee6e6ef595eaccf5f1e51e26f4a`).

- actual duration: `1800.0490624530066 s`;
- runtime: Ubuntu 24.04 / ROS 2 Jazzy PC_ONNX diagnostic;
- sensor source: `synthetic_transport_probe`, not Gazebo;
- model: `d2_suhan_yolo11n_diagnostic`, SHA-256
  `bce5b56cc825efaf4912a7137f74ad147634ac99ef6585662d003d39329ab100`;
- ONNX Runtime `CPUExecutionProvider`, no provider fallback;
- `17968` real inference calls, `0` inference errors;
- `17978` synchronized RGB/depth pairs, `0` rejected pairs, monotonic sensor
  and clock timestamps, TF `17977`, static TF `1`;
- live DDS endpoint evidence passed the fixed sensor/control/health QoS audit;
- `tc netem loss 100%` and restore both returned `0`;
- command timeout, network-loss stop, reconnect/manual-resume, stale replay,
  E-stop, blacklist detection/stop, non-zero J6-source authority, zero GT use,
  and steady-state zero PC duplicate algorithms all passed.

This remains diagnostic because the sensor source was synthetic, the algorithm
host was Jazzy rather than Humble, and D2 is neither the required nor a qualified
D1 contract model. The machine report was re-evaluated under the final V2 gates,
so every formal state is false.

## Humble algorithm-host smoke

`ros:humble-ros-base` (digest
`sha256:75dd3aba34a3838dadbb31a9f7bef769bdfa8713e6cec686fc868db2981b0987`)
built the two ROS packages successfully. An isolated 8-second host smoke proved
Ubuntu `22.04`, ROS 2 `humble`, ONNX Runtime `1.22.1`, D2 SHA verification, and
model loading on `CPUExecutionProvider`.

A real Jazzy-PC/Humble-algorithm split smoke ran for
`30.011304692015983 s`: `270` inference calls, `280` synchronized pairs, actual
network disconnect/restore, and the complete short safety sequence. Its live QoS
snapshot caught one cross-distribution color subscription before discovery had
settled, so `qos_contract_pass=false`. Independently, the message counters prove
the endpoint subsequently transported all 280 color frames. The run is short,
synthetic, and uses unqualified D2; all formal states are therefore false.
The compact machine report SHA-256 is
`9de4b75e0aad875f72de7356efd8cad6bf0970b58dedbd403ab9c1249fff5700`.

## Remaining hard blockers

- The canonical D1 ONNX is available, but it failed strict PT/ONNX parity and
  the fixed TRAIN semantic gate, so no contract-qualified D1 was eligible for
  the loopback.
- No dedicated PC Gazebo/Jazzy sensor-and-plant-only launch exists. The full
  product launch contains algorithm/evaluator/truth nodes and is intentionally
  not substituted.
- No proprietary Journey 6 OE image/runtime or physical board is available.
- Consequently no 1800-second Gazebo + Humble + qualified-D1 split run and no
  official Journey 6 runtime/board run has occurred.

## Verification

- HIL gateway pure-Python suite: `23 passed`.
- Ubuntu 22.04/Humble algorithm-host Docker build: passed.
- PowerShell parser, Bash `-n`, Python bytecode checks, Compose profile config,
  and `git diff --check`: passed.
- A full fast-CI attempt was deliberately interrupted while the real 1800-second
  CPU endurance run owned the machine; the parent integration lane must rerun it
  after all concurrent branches settle.
