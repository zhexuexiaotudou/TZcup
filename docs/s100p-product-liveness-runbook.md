# S100P product-liveness incident runbook

This is the short retry checklist for the DOSOD -> EdgeSAM -> product-map
development lane.  It does not promote development artifacts to formal or
semantic acceptance.

## Before every attempt

- Use a new create-only board root and a new local evidence directory.
- Refuse an already existing result path; keep failed attempts intact.
- Confirm no older DOSOD, EdgeSAM, adapter, publisher, or witness process is
  alive, and record available board memory and disk space.
- Bind the candidate commit/tree/archive and every HBM/config input before
  upload.  A successful process exit is not enough.
- Require one positive source timestamp to join RGB, depth, CameraInfo, raw
  detections, product outputs, BPU diagnostics, and the map projection.

## Known failure signatures

| Signature | Established cause | Fast check and correction |
| --- | --- | --- |
| Product masks are published but every cell is zero | The development publisher used identity `map <- camera` while constant depth was 1 m, so every projected point was 1 m above the map ground plane. | Inspect mask dimensions/encoding/nonzero count and the exact-stamp TF.  For the synthetic liveness fixture only, use camera `z=1 m`, quaternion `x=1,w=0`, so 1 m depth reaches map `z=0`.  Never copy this synthetic pose into real-camera calibration. |
| BPU latency exists in diagnostics but no raw frame can join it | Diagnostics used publication time instead of the inference source-frame timestamp. | The adapter must put the positive source stamp in both `DiagnosticArray.header.stamp` and `source_stamp_ns`.  The witness joins on the header stamp. |
| Empty `latency_ms` is counted as BPU evidence | The witness checked only a latency-like key name. | Require BPU context, diagnostic level 0, `message=inference_ok`, `inference_ok=true`, and a finite positive numeric latency.  Reject empty, NaN, Inf, zero, and negative values. |
| `mono_edgesam` exits `-6` after useful inference | In R14c the failure occurred during group shutdown, not while BPU inference was producing outputs. | Signal the `ros2 launch` PID with INT first, allow a grace period, then escalate only if its process group survives.  Always scan `graph.log` for child non-zero exits and fail the attempt if one remains. |
| A remote run says PASS but local proof is incomplete | Optional downloads and required evidence were treated alike. | A successful run must retrieve non-empty receipt, asset inventory, manifest producer output, witness, graph log, and `zero-survivor.txt`; the receipt must contain `PRODUCT_LIVENESS_STATUS=PASS`. |

## R14d reference result

Evidence root:
`F:\Project\TZcup\.workspace\evidence\s100p-r14d-product-liveness-template-20260908T114500Z-28bbe2e\board_results`

R14d completed with four qualifying same-stamp frames.  A representative frame
contained 42 raw ROIs, 36 valid detections, three EdgeSAM prompts, four EdgeSAM
outputs, and 180 non-zero cells in a 100 x 100 mono8 map mask.  DOSOD latency
was 3 ms and EdgeSAM latency was 17 ms on the BPU.  The graph recorded no child
non-zero exit and cleanup recorded zero survivors.  The receipt remains
`NON_FORMAL_PRODUCT_LIVENESS_ONLY`, `SEMANTIC_STATUS=NOT_EVALUATED`, and
`MOTION_OR_PLANNING_STARTED=false`.

## Next gate

Do not rerun the synthetic liveness lane unless its source or artifacts change.
The next useful step is a no-motion real-camera shadow run: real RGB, depth,
CameraInfo, map, and `map <- camera` calibration must produce the same exact-
stamp chain and a non-empty map mask.  Real actuator commands remain prohibited
until the hardware bring-up contract and physical E-stop gates pass.
