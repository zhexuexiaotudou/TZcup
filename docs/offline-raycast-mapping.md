# Deterministic offline raycast mapping

## Claim boundary

`scripts/offline_raycast_mapping.py` is an offline reconstruction tool. It
parses the frozen `sanitation_campus_large` static collision geometry, uses it
to simulate known-pose 2D range observations, and reconstructs a PGM/YAML
occupancy grid with inverse sensor log odds.

The source geometry raster exists only inside the process to generate range
returns and evaluation metrics. It is not copied to the output map. This result
is not Gazebo live SLAM, not a physical mapping run, and not saved-map
localization, navigation, map reuse, or product acceptance.

## Fixed model

- Source world: `starter_ws/src/sanitation_worlds/worlds/sanitation_campus_large.sdf`
- Field: exactly `200 m x 100 m`, `20,000 m2`
- 2D scan height: `1.1621 m`, matching the formal UTM-30LX mount height
- Sensor idealization: deterministic full-circle range sensor, `1080` rays,
  `29 m` maximum range, zero noise
- Pose cadence: at most `1.0 m` between recorded poses
- Route: fixed source-world serpentine starting at `(-98, 0)`, with rows at
  `y = 48.5, 16, -16, -48.5 m`
- Reconstruction update: `log_odds_free = -0.40`, `log_odds_occupied = 0.85`
- Output: `trinary` PGM/YAML at `0.05 m`, with a `5 m` observation margin

The area verifier counts every known free or occupied cell. `field_known_area_m2`
is also recorded separately for the exact `200 m x 100 m` source field; building
interiors remain unknown because a 2D scan sees their surfaces rather than their
private volume.

## Reproduce

```powershell
py -3 scripts/offline_raycast_mapping.py `
  --world starter_ws/src/sanitation_worlds/worlds/sanitation_campus_large.sdf `
  --output-directory reports/mapping/offline_raycast_mapping

py -3 scripts/verify_map_area.py `
  --root reports/mapping/offline_raycast_mapping `
  --map-yaml reports/mapping/offline_raycast_mapping/occupancy.yaml `
  --minimum-area-m2 20000 `
  --output reports/mapping/offline_raycast_mapping/map_area_verification.json `
  --require-pass
```

The manifest records source and output hashes, the exact route and sensor
parameters, observation counts, area and topology metrics, runtime, and the
remaining live-SLAM evidence. A failed area or quality gate is retained as
`OFFLINE_RAYCAST_MAPPING_FAIL`.
