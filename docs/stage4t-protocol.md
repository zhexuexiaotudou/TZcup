# Stage4T 瞬态、融合与定位恢复协议

## 目的与边界

Stage4T 不删除 Stage4S 的失败门槛。`0.60 rad/s` 固定时间开环试验仍是默认禁用的 stress lane；真实建图、定位和覆盖任务分别受 `precision_mapping` 与 `localization_coverage` 包络约束。ground truth 仅用于评分，除显式标记 `oracle_only=true` 的隔离通道外，不参与控制。

## 车辆与量测

- 物理轮半径：`0.14 m`。
- DiffDrive 有效轮距：`1.22 m`。
- 原始 `/odom/unfiltered` 与 `/imu/data` 永久保留，便于审计和 rosbag 回放。
- `sanitation_measurement_adapter` 将 Gazebo scoped frame 规范为 `odom/base_footprint` 与 `imu_link`，并发布 `/measurements/wheel_odom`、`/measurements/imu`。
- 非零 covariance 的来源、数值和实车迁移边界见 `sanitation_tasks/config/measurement_covariance.yaml`。

## 瞬态矩阵

固定时间阶跃角速度为 `±0.10、±0.25、±0.35、±0.45、±0.60 rad/s`。每个方向和速度各保存 10 次独立仿真冷启动 trial 与 10 次完整多速度预热序列后的 hot trial。闭环目标角为 `±90°、±180°、±360°`，每档同样保存 10 次 cold 与 10 次 hot trial。闭环反馈只使用 IMU 角速度积分，ground truth 只在 trial 结束后评分。

每个 trial 保存理论请求、实际 `/cmd_vel`、ground truth、raw odom、IMU、EKF 的逐样本 CSV 和 JSON 指标。聚合报告不会丢弃失败 trial，并报告 request→output delay、output→body delay、rise/settling time、overshoot、稳态增益、积分跟踪误差和重复性。

## 运行包络

| profile | max linear | max angular | 默认状态 | 用途 |
|---|---:|---:|---|---|
| `precision_mapping` | 0.30 m/s | 0.25 rad/s | enabled | SLAM 建图 |
| `localization_coverage` | 0.45 m/s | 0.35 rad/s | enabled | AMCL、Nav2、Coverage |
| `stress` | 0.45 m/s | 0.60 rad/s | disabled | 复现高速度开环边界 |

Nav2 controller、velocity smoother、rotate-to-heading、behavior server、Coverage turn、建图探针和最终 emergency-stop velocity gate 使用同一上限。最终 gate 对正负方向同时限幅，急停与 command timeout 的优先级高于包络限幅。

## EKF A/B/C/D

- A：wheel `vx` + IMU `vyaw`。
- B：wheel `vx` + 已验证 IMU orientation yaw/`vyaw`。
- C：wheel `vx/vy/vyaw` + IMU `vyaw`，作为 Stage4S 行为对照。
- D：wheel `vx/vyaw`，无 IMU，作为退化对照。

每个候选使用相同车辆、相同 14 段动作集和 seed 0–4。选择依次比较 ground-truth XY 误差、闭环残差、多速度稳定性、左右对称性；处于声明等价带时才使用实车可迁移性排序。最终选择及完整选择轨迹以 `ekf_ablation_report.json` 为准。

## Oracle / realistic 与停止条件

Oracle lane 由 `sanitation_oracle_odom_adapter` 隔离发布 Gazebo truth odom，只用于定位 SLAM、地图、Nav2 或 Coverage 的系统层故障，不能作为竞赛证据。realistic lane 使用 corrected wheel、corrected IMU、selected EKF 与 AMCL。

selected EKF 确定后才重新生成 `0.05 m` 与 `0.02 m` 地图，并在刚体配准后计算 occupancy IoU、boundary Chamfer/RMSE、直线角误差、loop ghosting、known area 与 unknown ratio。realistic AMCL 必须完成 10 seed 且所有 trial 的 XY RMSE 不大于 `0.05 m`；失败即停止完整 Coverage 和 Stage5A。

## 复现入口

Windows Docker 入口：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_stage4t_core_smoke_docker.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_stage4t_ekf_ablation_docker.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_stage4t_transient_matrix_docker.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File scripts/run_stage4t_mapping_docker.ps1
```

定位与 Coverage 脚本要求先通过 `TZCUP_MAP_DIR` 指向已生成 `selected_map.yaml` 的建图 artifact。所有脚本保留逐 trial 报告和日志；正式评审目录仅收敛可追溯摘要、逐 trial JSON、图片和 rosbag 元数据，原始大体积 bag 保留在本地直到用户确认。
