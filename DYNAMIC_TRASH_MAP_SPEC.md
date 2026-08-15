# DynamicTrashMap specification

## Mission contract

A new mission is created only through `DynamicTrashMap.start_new(mission_id)` and must have
`count == 0`. Persisted targets may be loaded only through the explicit
`resume_same_mission(path, mission_id)` path, which rejects a different mission ID. There is no
Gazebo or evaluation-registry bootstrap API.

## Observation admission

Every `TargetObservation` carries the RGB timestamp, camera and image frame IDs, source model,
image bbox/mask reference, class probabilities, projected map measurement and covariance. Target
creation requires both `in_current_fov=true` and an exact timestamp/frame match in
`ObservedRegionMap`; a merely plausible coordinate is insufficient. Ground-truth and registry
sources are rejected at ingress.

## Fusion and lifecycle

Nearby observations associate class-agnostically by target type and map distance. Position uses
inverse-covariance weighted fusion, class probabilities accumulate as log evidence, confidence
uses an EMA, and bbox/mask, view direction, camera frame, physical size and polygon history remain
available for replay. Three observations plus class, confidence and covariance gates produce
`CONFIRMED`. The complete state vocabulary is `CANDIDATE`, `TRACKED`, `CONFIRMED`, `SCHEDULED`,
`APPROACHING`, `VERIFYING`, `CLEANING`, `POST_VERIFY`, `CLEANED`, `DEFERRED`, `REJECTED`, `LOST`
and `UNREACHABLE`; invalid transitions fail closed. Missing targets become `LOST` and then
`REJECTED` after bounded expiry.

## Persistence and replay

Snapshots include mission/config, target state/history, observed frusta, accepted and rejected
observations, and explicit `preknown_target_coordinates_used=false` /
`ground_truth_control_allowed=false`. Replay rebuilds the map solely by re-applying recorded online
observations against recorded camera frusta.
