# AUTO-11 大地图与定时任务

本阶段验证项目能否在 20,000 m² 尺度上保持地图、定位和任务编排契约，而不是把小场景结果直接外推。

地图为 200 m × 100 m 的 PGM occupancy grid，resolution `0.1 m`，边界占据、内部可通行。地图按 10×2 切成 20 个 zone，每个 zone 对应独立 submap ID；metadata 和像素数据在重载时交叉检查尺寸、分辨率和面积。

定位矩阵包含 10 条 lawnmower 轨迹。truth 从确定性 simulator world-state 路径采样，estimate 从独立 seeded observation model 得到，并注入定位丢失与恢复事件。评估只计算 estimate 对 truth 的误差，禁止 estimate/odometry 自比较。5 次全覆盖和 20 次定时任务分别记录区域选择、边界、动态碰撞和中断恢复。

正式结果为 10 条轨迹 RMSE 最大 `0.03004 m`、恢复率 `0.95`、TF continuity `0.99998`；5/5 覆盖任务完成，20/20 定时路线完成，zone selection `1.0`、boundary/collision `0`、resume `0.96`。证据级别固定为 `OFFLINE_LARGE_MAP_SIMULATION`：它满足 AUTO-11 的独立仿真 GT 要求，但不等同于 Gazebo ROS graph、真实 GNSS/SLAM 或实车结果。
