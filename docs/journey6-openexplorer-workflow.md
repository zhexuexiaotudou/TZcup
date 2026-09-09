# Journey 6 OpenExplorer workflow

## Discovery

The repository never vendors the proprietary Journey 6 SDK. Discover an
already mounted or extracted SDK root through explicit `--root` arguments or
the approved `J6_SDK_ROOT`, `JOURNEY6_SDK_ROOT`, or `J6_OE_ROOT` environment
variables. Docker images and WSL installations must first expose the SDK as an
explicit filesystem root; they are not scanned implicitly:

```powershell
py -3 scripts/j6_discover_sdk.py --output C:\tzcup-j6\J6_SDK_INVENTORY.json
py -3 scripts/j6_validate_sdk.py `
  --inventory C:\tzcup-j6\J6_SDK_INVENTORY.json `
  --output C:\tzcup-j6\J6_SDK_LOCK.yaml
```

Discovery looks for the official J6 OpenExplorer release identity,
`horizon_tc_ui`, `hmct`, `hbdk4_compiler`, `hb_compile`, `hb_model_info`,
`hb_verifier`, HUCP/DNN samples, AIBenchmark, sysroot, cross compiler, and
runtime libraries. Finding executable names alone is insufficient. The package
identity, version, archive or image digest, license boundary, and supported
march list must agree.

Any package identified as RDK S100/S100P, J5, X5, or another target is recorded
as a rejected candidate. It cannot make `J6_SDK_AVAILABLE=true`.

## Conversion sequence

After the official package is available, run its documented sequence without
inventing command flags from another Horizon generation:

```text
canonical static ONNX
  -> official checker / shape inference
  -> PTQ or non-training mixed precision
  -> HBDK compile for resolved march
  -> HBM metadata inspection
  -> official verifier
  -> official x86 HUCP/DNN runtime
  -> physical-board runtime
```

The project preflight requires batch 1, static dimensions, a known output
contract, no custom operators, no embedded dynamic NMS/TopK, and at least 1000
calibration records. Actual SDK constraints are authoritative when stricter.
QAT, distillation, fine-tuning, or any other training is outside this task.

## Parity evidence

Each output node is compared between float ONNX, optimized ONNX, quantized
intermediate, x86-simulated HBM, and later the physical board. Evidence records
shape, stride/padding, cosine similarity, maximum absolute error, and mean
absolute error. The preferred cosine threshold is 0.999 and the absolute
minimum is 0.99.

If the official SDK is absent, code, manifests, NV12 emulation, HIL, board
inventory, deployment, and rollback work continue, but:

```text
J6_SDK_AVAILABLE=false
BLOCKED_EXTERNAL_J6_SDK=true
J6_X86_SIMULATION_READY=false
```
