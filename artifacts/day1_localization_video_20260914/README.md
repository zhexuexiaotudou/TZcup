# Localization and tracking success video pack

This pack provides success-only material for the localization/tracking section
of a five-minute competition video.

## Primary outputs

- `rendered_final/localization_tracking_success_1080p.mp4`
  - 1920x1080, 30 fps, 60 seconds
- `rendered_final/localization_trajectory_success.png`
- `rendered_final/localization_error_success.png`
- `rendered_final/tracking_goals_success.png`
- `rendered_final/manifest.json`

## Success conclusion

- navigation results: `[4, 4]`
- completed goals: `2 / 2`
- causal localization replay:
  - RMSE: `29.2 mm`
  - P95: `40.3 mm`
  - max: `47.6 mm`

The animation is generated from the sealed run trajectory, map, route timeline
and the truth-isolated causal replay CSV. No Gazebo process is used.
The source run has no camera stream, so the visual is data-driven rather than
synthetic photography.

## Original materials

- `source/localization_focus.json`
- `source/candidate_paired_samples.csv`
- `source/recovery_receipt.json`
- `source/occupancy.pgm`
- `source/occupancy.yaml`
- `source/route.json`
- `source/route_timeline.jsonl`
- `source/effective_parameters.json`
- `source/tf_authority.json`

The raw 129.9 MiB MCAP remains at the bound rental-host evidence path and is
identified by:

```text
8e64c0b7dd6a29247ad0d2f61ac719ba7f8c4f6b1957edf8bffc88aa38308047
```

## Reproduce

```powershell
py -3.13 scripts\render_day1_localization_success_video.py `
  --mcap <bag_0.mcap> `
  --focus-result artifacts\day1_localization_video_20260914\source\localization_focus.json `
  --candidate-csv artifacts\day1_localization_video_20260914\source\candidate_paired_samples.csv `
  --receipt artifacts\day1_localization_video_20260914\source\recovery_receipt.json `
  --map-pgm artifacts\day1_localization_video_20260914\source\occupancy.pgm `
  --map-yaml artifacts\day1_localization_video_20260914\source\occupancy.yaml `
  --route-timeline artifacts\day1_localization_video_20260914\source\route_timeline.jsonl `
  --output-dir <fresh-output> `
  --duration-sec 60 `
  --fps 30
```
