# PERCEPTION-ONLINE product operation guide

## Current runnable scope

The branch provides tested software contracts but no frozen product model. The checked-in
pipeline manifest remains intentionally fail-closed, so the product node must not become ACTIVE
with placeholder artifacts.

Run the deterministic local gate with `py -3 scripts/ci_fast.py`. Run the ROS package gate in the
project image with an external build/install/log base. Use `--packages-up-to` when workspace
dependencies are not already installed, then test `sanitation_learning`, `sanitation_perception`,
`sanitation_spot_cleaning`, and `sanitation_coverage`.

## Mission parameters

- `mission_id` is mandatory.
- `resume_same_mission=false` starts with an empty dynamic map.
- `resume_same_mission=true` additionally requires `dynamic_map_path` whose stored mission ID
  exactly matches.
- RGB, depth, CameraInfo and the RGB-timestamped camera-to-map TF are mandatory.

The node publishes `/perception/product/observations`, `/perception/product/tracks`,
`/perception/product/dynamic_trash_map`, `/perception/product/targets`,
`/perception/product/area_regions`, health and metrics. Spot cleaning consumes only
`/perception/product/targets`; `/ground_truth/*` topics and ground-truth source backends are denied.

## Fail-closed conditions

Missing/hash-mismatched model artifacts, absent CUDA provider, CPU fallback, invalid mission
restore, camera/CameraInfo mismatch, stale camera, missing timestamped TF, invalid depth, degraded
health, keepout/obstacle/localization failure, or absent post-clean evidence must prevent or defer
action. `MODEL_FREEZE_X86.json` currently says `NOT_CREATED`; do not open G5 or package a product
release until that changes through the formal development gate.
