# Coverage path optimization comparison

Overall gate: **PASS**

| Profile | Seed | Success | Coverage | Repeat | Distance m | Brush-off m | Duration s | Targets | RMSE m | Straight P95 m |
|---|---:|:---:|---:|---:|---:|---:|---:|:---:|---:|---:|
| legacy | 140 | no | 0.999 | 0.740 | 73.711 | 34.388 | 218.9 | 10/10 | 0.0323 | 0.1229 |
| legacy | 141 | no | 1.000 | 0.778 | 70.556 | 32.068 | 207.2 | 10/10 | 0.0350 | 0.1077 |
| legacy | 142 | no | 0.999 | 0.765 | 74.506 | 35.070 | 227.0 | 10/10 | 0.0488 | 0.0993 |
| legacy | 143 | no | 1.000 | 0.765 | 70.426 | 32.045 | 214.4 | 10/10 | 0.0400 | 0.1320 |
| legacy | 144 | no | 0.998 | 0.798 | 77.313 | 38.258 | 228.2 | 10/10 | 0.0319 | 0.1014 |
| optimized | 132 | yes | 1.000 | 0.178 | 37.812 | 13.437 | 113.4 | 10/10 | 0.0386 | 0.0349 |
| optimized | 133 | yes | 1.000 | 0.158 | 37.295 | 13.125 | 110.2 | 10/10 | 0.0358 | 0.0479 |
| optimized | 134 | yes | 1.000 | 0.154 | 37.485 | 13.185 | 109.4 | 10/10 | 0.0347 | 0.0565 |
| optimized | 135 | yes | 1.000 | 0.142 | 37.774 | 13.360 | 110.4 | 10/10 | 0.0376 | 0.0439 |
| optimized | 136 | yes | 1.000 | 0.160 | 37.616 | 13.252 | 112.9 | 10/10 | 0.0394 | 0.0459 |

## Hard gates

- [x] five_legacy_seeds
- [x] five_optimized_seeds
- [x] optimized_all_success
- [x] coverage_at_least_0_995
- [x] repeat_at_most_0_20
- [x] targets_10_of_10
- [x] zero_collision_keepout
- [x] localization_rmse_at_most_0_05
- [x] lateral_p95_at_most_0_08
- [x] distance_reduction_at_least_0_25
- [x] brush_off_reduction_at_least_0_40
- [x] connector_reduction_at_least_0_50
- [x] mcap_replay
- [x] dynamic_matrix
- [x] repair_matrix

## Median reductions

- actual_total_distance_reduction: 48.97%
- brush_off_distance_reduction: 61.46%
- connector_distance_reduction: 68.00%
- duration_reduction: 49.57%
