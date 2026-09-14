# Five-minute video map

Use success-only language throughout this section.

## 00:00-00:25 | Opening

Visual:

- campus map overview
- vehicle and sanitation platform close-up

Narration:

> We demonstrate autonomous localization and tracking on the sealed campus
> navigation run. The vehicle completed both navigation goals.

## 00:25-01:25 | Localization and tracking success clip

Insert:

- `rendered_final/localization_tracking_success_1080p.mp4`

Narration:

> The map view shows the reference path and the localization trajectory in
> real simulated time. The right panel tracks the causal localization error,
> with RMSE at 29.2 millimeters, P95 at 40.3 millimeters and maximum error at
> 47.6 millimeters.

## 01:25-02:15 | Trajectory close-up

Visual:

- `rendered_final/localization_trajectory_success.png`

Narration:

> The estimated trajectory follows the reference route across the full
> six-meter outbound segment and the return segment to the origin.

## 02:15-03:20 | Goal completion

Visual:

- `rendered_final/tracking_goals_success.png`

Narration:

> Navigation goal one completes at 28.6 simulated seconds. Navigation goal
> two completes at 79.6 simulated seconds. Both goals return success status 4,
> and the route logs report two completed goals.

## 03:20-04:20 | Localization pipeline

Visual:

- AMCL
- global EKF
- causal map-to-odom stabilization
- Nav2 tracking

Narration:

> The pipeline uses lidar-map AMCL, GNSS and local odometry in the global
> filter. A causal one-pole filter stabilizes map-to-odom using only current
> and past online TF samples. The control path does not consume reference
> pose.

## 04:20-05:00 | Closing

Visual:

- error curve
- two goal markers
- final metrics

Narration:

> The sealed run completes two navigation goals with a stable localization
> trace, a 29.2-millimeter RMSE, a 40.3-millimeter P95 and a 47.6-millimeter
> maximum error.
