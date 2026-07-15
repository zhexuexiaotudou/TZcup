# GPT_REVIEW_STAGE4T

## 结论

`READY_FOR_GPT_REVIEW_STAGE4T=false`<br>
`READY_FOR_STAGE5A=false`

Stage4T 已完成转向瞬态矩阵、运行包络、measurement covariance、EKF A/B/C/D 消融、双分辨率建图和 Oracle 10-seed 定位。第一真实失败层是 `oracle_localization_pass`：最终最优 Oracle lane 10/10 次导航均成功，TF 全部连续且粒子退化为 0，但 XY RMSE 的 P50/P95/max 分别为 `0.08397 / 0.14848 / 0.16972 m`，未满足所有 trial `<= 0.05 m` 的硬门。

因此本轮按提示词停止，没有执行 realistic 全量 10-seed、完整 Coverage、20-seed 动态障碍、30 次急停或完整任务 rosbag 回放，也没有进入 Stage5A。阻断项均以 `executed=false` 写入复核证据，没有伪造通过结果。

## 基线、分支、PR 与 CI

- 基线：`b7734801d775740dccf6ce16a12f6e739b2e8136`
- 分支：`agent/stage4t-transient-ekf-localization`
- 独立工作树：`F:\Project\TZcup-stage4t`
- PR：[#7 advance Stage4T transient EKF and localization gates](https://github.com/zhexuexiaotudou/TZcup/pull/7)
- CI：本地 `py scripts/ci_fast.py` 已通过；PR 最新提交的远端 `fast-validation` 通过（[run 29427301500](https://github.com/zhexuexiaotudou/TZcup/actions/runs/29427301500)）
- main：merge commit `2412300192d6f4204e0049e55c06ba69353377ba`；回滚点 `b7734801d775740dccf6ce16a12f6e739b2e8136`
- 合并后验收：远端 main tree 的真实 Gazebo core smoke 再验通过 covariance、运行包络与最终速度门，实际速度越界 0；17.9 MiB MCAP 含 49,437 条消息且元数据可读

## Stage4S 旧失败与转向瞬态

- Stage4S 的 `0.60 rad/s` 正转整圈 `19.1825° > 18°` 失败被原样保留，没有删除旧 artifact，也没有把阈值抬到 20°。
- Stage4T 固定时长矩阵完整保存 `200/200` 个 trial：`±0.10/0.25/0.35/0.45/0.60 rad/s`，每档 10 次独立 cold 和 10 次完整预热后的 hot。
- 闭环航向矩阵完整保存 `120/120` 个 trial：`±90°/±180°/±360°`，每档 10 cold + 10 hot；反馈仅使用 IMU 角速度积分，`ground_truth_control_violation_count=0`。
- 闭环最大 GT 航向误差 `0.8995°`，`closed_loop_tracking_pass=true`。0.25 和 0.35 rad/s 的低速跟踪门通过；0.60 rad/s 开环 stress 仍为 false。
- 每个 trial 都积分实际 `/cmd_vel`，并保留 request/output/body 延迟、rise/settling、overshoot、稳态增益、积分跟踪误差和重复性；不是用 schedule 理论值代替实际输出。

## Operational envelope

- `precision_mapping`：`0.30 m/s, 0.25 rad/s`。
- `localization_coverage`：`0.45 m/s, 0.35 rad/s`。
- `stress`：最大 `0.60 rad/s`，默认禁用。
- Nav2 controller、velocity smoother、rotate-to-heading、behavior server、Coverage turn、建图探针和最终 emergency-stop velocity gate 使用同一包络；最终 gate 对正负方向同时限幅。
- 真实 core smoke 中 precision/coverage 均通过，`actual_cmd_limit_violations=0`；`high_speed_open_loop_stress_pass=false` 未被包络掩盖。

## Measurement covariance

- 原始 `/odom/unfiltered` 和 `/imu/data` 永久保留；审计确认 bridge 原始 covariance 为全 0。
- 项目内 `sanitation_measurement_adapter` 发布 `/measurements/wheel_odom` 和 `/measurements/imu`，统一 frame 为 `odom/base_footprint` 与 `imu_link`，并注入 YAML 化、非零、可解释 covariance。
- EKF 使用修正后的 measurement topic；审计记录 pose/twist/orientation/angular velocity covariance、frame、timestamp、rate、全零/奇异/有限性。
- 真实 covariance smoke 通过，17.4 MiB MCAP 正常生成 `metadata.yaml` 并可由 `ros2 bag info` 读取。

## EKF A/B/C/D

每个候选使用同一 14 段动作集和 seed 0–4，共保留 `20/20` 个完整 trial。

| 候选 | XY RMSE (m) | 闭环残差 RMSE (m) | 多速度 yaw 稳定性 (rad) | 左右对称性 (rad) |
|---|---:|---:|---:|---:|
| A | 0.17203 | 0.05357 | 0.02073 | 0.13799 |
| B | 0.17119 | 0.05291 | 0.01410 | 0.13231 |
| C | 0.17024 | 0.05525 | 0.01803 | 0.12394 |
| D | 0.30113 | 0.12105 | 0.07879 | 0.00898 |

选择顺序严格为 realistic GT XY、闭环残差、多速度稳定性、左右对称性、实车可迁移性。A/B/C 在误差等价带内逐层筛选，最终 B 在多速度稳定性等价带和可迁移性排序中胜出；D 明显退化。`selected_ekf.yaml` 为 wheel `vx` + 已验证 IMU yaw/`vyaw`。

## Optional chassis yaw-rate controller

0.25/0.35 包络和闭环航向均通过，因此记录 `optional_chassis_controller=not_needed`。没有在证据不足时增加 response controller；最终安全 gate 仍最后生效，急停和 timeout 优先级最高。

## 双分辨率地图与刚体几何评估

- 两套地图均在 selected EKF、修正 covariance、1.22 m effective separation 和自动闭环路线下重新生成，控制反馈为 `/odom`，GT 只评估。
- `0.05 m`：路线完成，known area `153.37 m²`，质量门通过并被选中。
- `0.02 m`：路线完成，但 known area `74.20 m²`，质量门失败，未选中。
- 几何脚本从 world SDF 提取固定障碍物，以 spawn 先验为初值执行 Powell 鲁棒刚体配准，再计算 IoU、Chamfer、boundary RMSE/P95、直线角误差、ghosting、known area 和 unknown ratio。
- 选中 0.05 m 地图配准后：IoU `0.16515`、Chamfer `0.09403 m`、boundary RMSE `0.23205 m`。这些指标显示地图仍存在非刚性误差，是后续 AMCL 精度失败的主要风险，而不是可忽略的显示问题。
- keepout 和 speed mask 已从选中地图生成；由于 localization 前置门失败，本轮没有把“生成成功”误报为运行时 filter 门通过。

## Oracle / realistic 定位

- Oracle lane 明确标记 `oracle_only=true, competition_evidence=false`，仅将 Gazebo truth 转换为 `/odom` 和 `odom→base_footprint` TF。
- 首轮通用 AMCL 噪声下只有 `2/10` 导航成功，XY RMSE max `2.864 m`。修复 initial-pose 时间戳未来外推、使用固定已知起点 covariance，并按 selected EKF 标定收紧运动模型后，正式最优轮达到 `10/10` 导航成功。
- 正式最优 Oracle：XY RMSE P50 `0.08397 m`、P95 `0.14848 m`、max `0.16972 m`；yaw RMSE P50 `0.01639 rad`、P95 `0.02342 rad`、max `0.02539 rad`；TF 10/10 连续、粒子退化 0、recovery 0。
- 同步 SLAM、异步低速 SLAM和不同 AMCL scan-weight 调优均保留失败 artifact。最接近门槛的单 seed 为 `0.05278 m`，仍未过门；没有将它四舍五入为成功。
- 因 Oracle 未过门，realistic 全量未执行；`realistic_localization_report.json` 明确记录 `executed=false` 和 `blocked_by=oracle_localization_pass`。

## Coverage、filters、动态障碍与急停

完整 Coverage 的前置条件是 realistic 10-seed 通过。本轮此前置条件不成立，因此以下项目均未执行：12 swath + 11 turn 全量任务、empirical coverage、collision/brush terminal gate、keepout/speed runtime gate、20-seed 动态障碍、30 次急停和完整任务 rosbag replay。`stage4t_coverage_report.json` 明确记录阻断原因。

## 效率边界

保持 `0.65 × 0.45 × 3600 = 1053 m²/h`，`competition_efficiency_pass=false`。没有声称达到 3500 m²/h；在当前 0.65 m 作业宽度下理论所需速度约 `1.496 m/s`，在 0.45 m/s 下理论所需宽度约 `2.160 m`，均超出当前已验证能力。

## Gazebo、rosbag 与 SHA-256 manifest

- 所有运行结果来自 `tzcup/sanitation-jazzy:stage0` 中的真实 Gazebo Harmonic headless 仿真，不是合成 JSON。
- 正式复核目录包含 320 个瞬态 trial JSON、20 个 EKF trial、10 个 Oracle 定位 trial、两套地图几何/质量/路线报告、Gazebo 帧、overlay、核心和建图 rosbag 元数据。
- 原始大体积 MCAP 保留在本地任务 artifact 中直至用户确认；复核目录收敛为可审计 JSON/YAML/PNG/rosbag info。
- `artifacts/stage4t_20260715_review/MANIFEST.json` 覆盖复核目录文件的 SHA-256；manifest 自身按声明排除，避免自引用。

## P0 / P1 / P2

- P0：Oracle 10-seed XY RMSE max `0.16972 m > 0.05 m`，阻断 realistic、完整 Coverage 和 Stage5A。
- P1：选中地图虽通过基础 span/known-area 门，但配准后 boundary RMSE `0.23205 m` 且存在非刚性漂移；下一轮应优先提高环境几何约束、闭环质量或采用更适合稀疏场景的定位地图，而非继续弱化 AMCL 传感器权重。
- P1：完整 Coverage、20-seed 动态障碍、30 次急停和任务 rosbag 回放因前置定位门失败而未执行。
- P2：0.60 rad/s 开环 stress 仍失败但默认禁用；0.25/0.35 闭环运行包络已通过，不应为非赛题 stress 过度设计 controller。
- P2：理论效率仍为 `1053 m²/h`，与 3500 m²/h 目标存在独立硬差距。

## 复核入口

- `artifacts/stage4t_20260715_review/stage4t_summary.json`
- `artifacts/stage4t_20260715_review/MANIFEST.json`
- `artifacts/stage4t_20260715_review/oracle_localization_report.json`
- `artifacts/stage4t_20260715_review/map_geometry_report.json`
- `artifacts/stage4t_20260715_review/transient_response_report.json`
- `artifacts/stage4t_20260715_review/ekf_ablation_report.json`
- `artifacts/stage4t_20260715_review/deployment_verification.json`
