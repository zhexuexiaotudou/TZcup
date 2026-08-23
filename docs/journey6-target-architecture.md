# Journey 6 PC-first target architecture

## Scope and truth boundary

The target family is Horizon Journey 6. Until a physical board and its official
SDK identify the actual device, the only valid target fields are:

```text
target_family = journey6
target_sku = auto
target_march = auto
profile = journey6_generic
```

`nash-e`, `nash-m`, and `nash-p` are supported compile-profile names, not a
guess about the board. `Journey6Target.resolve()` accepts SKU and march facts
only from board inventory or the official Journey 6 SDK. RDK S100/S100P, J5,
RDK OS, TROS Humble, and their precompiled HBM files are not compatible
substitutes.

## Split architecture

```text
PC / Gazebo                                      Journey 6 algorithm side
-------------                                    ------------------------
world and vehicle physics  -- HIL sensors ---->  perception provider
virtual RGB-D/LiDAR/IMU/GNSS                     tracking and projection
independent evaluator                             ActionVerifier and map
final physical safety gate  <--- commands ------ planning/control/task state
evidence recorder                                 provider and runtime health
```

The PC is the sensor and physics authority. The Journey 6 side is the sole
algorithmic source of non-zero vehicle commands in HIL mode. The PC may stop
motion for timeout, E-stop, collision, or invalid command input, but it may not
replace a missing J6 command or run a duplicate planner.

Production Target List and DynamicTrashMap start empty. Gazebo world state,
semantic IDs, target coordinates, sealed data, and evaluator output are never
mounted into or sent to the algorithm container.

## Model and provider boundary

The provider-neutral pipeline owns fixed tensor names, static shapes, dtypes,
strides, class order, thresholds, letterbox metadata, and graph-external
decode/NMS. The intended provider transition is:

```text
StrictOnnxProvider (PC reference)
    -> official Journey 6 x86 HUCP/DNN runtime
    -> Journey6HbmProvider (physical board)
```

`Journey6HbmProvider` validates HBM SHA-256, march, exact runtime version,
input format, tensor shape/dtype/stride, and output names. It has no ONNX or CPU
fallback lane. A missing official runtime fails lifecycle configuration.

The detector consumes RGB on PC and NV12 on the runtime lane. Depth,
CameraInfo, and timestamped TF remain mandatory for projection and safety; the
pretrained detector is not converted into a six-channel RGB-D network.

## Product state separation

The following states are independent:

- `J6_DEV_MODEL_AVAILABLE`: a real, SHA-locked, non-mock development model has
  verified checkpoint classes and PC inference. It does not imply a release
  license or competition acceptance.
- `J6_PC_DISCRETE_FUNCTIONAL_PASS` and `J6_PC_AREA_FUNCTIONAL_PASS`: the live
  no-GT Gazebo gates for discrete and Area targets pass independently.
- `J6_PC_FUNCTIONAL_PASS`: a real pretrained pipeline completes the live PC
  chain without GT, mocks, or preloaded targets; it requires both preceding
  functional states.
- `J6_X86_SIMULATION_READY`: the official J6 x86 runtime passes sanity and
  model parity.
- `J6_LOOPBACK_TRANSPORT_READY`: Ubuntu 22.04/Humble PC_ONNX algorithm-host and
  PC Gazebo complete the 30-minute topic/QoS/timestamp/authority/fault matrix.
- `J6_LOOPBACK_ALGORITHM_READY` and `J6_LOOPBACK_HIL_EMULATION_READY`: the same
  run additionally hosts the complete planned algorithm stack with no PC
  duplicate. Neither is official Journey 6 evidence.
- `J6_LOOPBACK_HIL_READY`: the legacy official-runtime Journey 6 HIL definition;
  PC_ONNX emulation can never set it.
- `J6_CALIBRATION_PACK_READY` and `J6_SOURCE_DEPLOYMENT_BUNDLE_READY`: audited
  non-sealed calibration records and a checksum-locked source-only board bundle
  exist. They do not imply HBM compilation.
- `J6_COMPILED_HBM_BUNDLE_READY`: the official toolchain, resolved march, model
  conversion/verifier, and runtime-load gates pass.
- `J6_DEPLOYMENT_BUNDLE_READY`: the checksum-bound, installable model bundle
  and rollback contract pass.
- `J6_COMPETITION_MODEL_READY` and `J6_LICENSE_RELEASE_READY`: competition and
  redistribution/release acceptance; development-only evidence cannot set them.

None of these implies `SIMULATION_PRODUCT_COMPLETE`,
`PRODUCT_INTEGRATION_READY`, or `PRODUCT_FIELD_READY`. Those remain governed by
the fixed V1 A-P acceptance contract.
