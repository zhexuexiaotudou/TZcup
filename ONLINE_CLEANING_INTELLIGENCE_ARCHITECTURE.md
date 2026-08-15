# Online Cleaning Intelligence architecture

```text
Safety perception (continuous, authoritative)
  LiDAR / RGB-D / IMU / localization / costmap / collision monitor / Nav2
                               │
                               └── may veto every cleaning action

Cleaning intelligence (mission-scoped)
  onboard RGB-D + timestamped TF
    → adaptive low-rate discovery
    → higher-rate tracker_v2
    → current-FOV observation proof
    → DynamicTrashMap fusion and confirmation
    → CLEAN_NOW / DEFER / OBSERVE_AGAIN scheduler
    → safe Coverage pause and approach
    → clean
    → event-triggered post-clean visual verification
    → Coverage resume with brush off
```

The x86 product runtime retains strict RGB-D synchronization, CUDA ONNX Runtime I/O binding,
timestamped TF, projection, tracker v2, lifecycle health and watchdog code from PR #89. New code
adds the online observation boundary, dynamic map and task state machines. The product package does
not import research models.

`reference_vision/` defines one detector and tracker interface for FCOS-R50, Grounding DINO,
YOLO-World, SAM 2 and Grounded SAM 2. It lives in a separate pinned container. The reference lane
may benchmark or auto-label but cannot publish product targets directly. Current SAM 2 and
Grounded SAM 2 work is adapter-only and reference-only; product runtime remains tracker v2.

Coverage remains the default action. Only a confirmed, persistent, nearby target at a safe
boundary with healthy Nav2, keepout, obstacle, localization and perception inputs may interrupt it.
Low-confidence targets request another observation; distant targets are batched in the deferred
queue. Cleaning completion enters `POST_VERIFY`, not `CLEANED`: discrete targets require three
absent/low-confidence frames, and area targets require at least 90% visual area reduction. One
re-clean is allowed before manual attention.
