# GPT review: coverage path optimization

## Delivered behavior

The independent 16 m × 12 m small Gazebo field now defaults to a skid-steer-aware hybrid area-fill planner. Fields2Cover supplies the area and swath geometry; TZcup chooses the swath angle and spacing, orders adjacent boustrophedon passes, and executes explicit `SWATH`, `ROTATE`, `SHIFT`, `BACKUP`, `OBSTACLE_BYPASS`, `REPAIR_SWATH`, and `RETURN_HOME` components through Nav2. Brush state, speed profile, component identity, planned geometry, actual cleaning/transit/repair traces, blocked intervals, and cleaned cells are published separately and rendered as semantic layers in the native Gazebo panel.

The selected small-field profile is 0 degrees and 0.52 m spacing for the 0.65 m brush. The legacy 0.35 m Dubins route remains available for A/B regression. `AREA_FILL`, SHA-sealed `TAUGHT_ROUTE`, and delegated `POINT_CLEAN` are explicit task modes; malformed or unsafe inputs fail closed.

## Formal evidence

- Optimized 5-seed matrix: 5/5 pass; 100% empirical coverage; 10/10 targets; 14.25%–17.83% repeat coverage; zero collision and keepout violations; localization RMSE 3.47–3.94 cm; straightness P95 3.49–5.65 cm.
- Median improvement over the retained legacy matrix: total executed distance -48.97%; brush-off distance -61.46%; connector distance -68.00%; duration -49.57%.
- Repair matrix: 10/10 pass; exactly one connected-component repair per run; final coverage at least 99.5%; repeat coverage 16.08%–19.50%; repair length within 10% of primary swaths; Gazebo truth not used for control.
- Dynamic matrix: three independent missions, 24/24 valid LiDAR-observed interactions; recovery and mission-resume rates 100%; collision count 0; minimum observed separation 0.467 m; repeated oscillation count 0; every mission completed with acceptable coverage and brush disabled on exit.
- MCAP replay: real `ros2 bag play` plus sequential reconstruction passed for optimized seed 132; 124,094 messages over 125.303 seconds on 14 required topics, with semantic plan/components, brush transitions, and terminal `COMPLETED` state.

The external auditable evidence root is `F:\Project\TZcup-coverage-evidence\coverage_path_optimization_20260804T073000Z_delivery_final`. Raw MCAP, trajectories, simulator logs, rejected candidates, unsafe dynamic trials, and tuning attempts remain outside Git. This repository only keeps compact review reports.

## Review boundaries and rollback

The evidence proves the ROS 2/Nav2/Gazebo SIL path, not real litter recognition, physical suction, real pedestrian braking, RTK multipath performance, wheel slip, thermal limits, J6 deployment, or 20,000 m² endurance. Before real-vehicle release, recalibrate brush width/offset and skid-steer dynamics under load, replace simulator-truth coverage with localization plus brush-contact evidence, and pass HIL, closed-field low-speed, and operator rollback tests.

Rollback is `-CoverageProfile legacy`, backed by the retained `0.35 m + Dubins` configuration and formal legacy matrix. The task branch and all acceptance evidence remain available until the user confirms the result.
