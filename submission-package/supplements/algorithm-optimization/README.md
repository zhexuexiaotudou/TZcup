# 算法优化审查简报

本简报汇总当前能够复核的算法优化，按“问题、方法、可复核结果、适用边界”分组；不把受控候选或离线结果写成官方硬指标 PASS。

## 1. 定位因果稳定器

- 问题：原动态定位 run 的严格 max 误差为 `126.327803 mm`，全局 `map -> odom` 修正存在低频漂移。
- 方法：只对在线 `map -> odom` 使用一阶因果低通，再与原始 `odom -> base_footprint` 合成；不使用未来帧或 ground truth。
- 结果：同一 session、同一 `2711` 点分母，RMSE/P95/max 从 `39.271/83.780/126.328 mm` 降为 `29.192/40.285/47.566 mm`。
- 边界：离线因果复算候选；实时 TF 链和 Gazebo live 复跑仍为 `NOT_MEASURED`。

## 2. 动态避障反馈闭环修复

- 问题：Nav2 recovery 期间，global EKF 将 `navsat_transform` 基于自身融合输出生成的 `/odometry/gps` 再次作为全局测量，导致 `map -> odom` 漂移。
- 方法：增加 `global_gnss_odometry_topic` 覆盖点；生产默认仍为 `/odometry/gps`，单次恢复验收器切到无 publisher 的 `/odometry/gps_disabled`，仅断开错误闭环。
- 结果：根因、最小修复、验收器和失败回执已归档。
- 边界：尚无新的成功 Gazebo run；官方 `>=95%` 仍为 `NOT_MEASURED`，单次成功最多 `FUNCTIONAL_PASS_1_OF_1_NOT_OFFICIAL_95`。

## 3. 受控感知全帧复算与低置信后备

- 问题：旧运行时只保留 9 帧 presence mask，造成 `41/0/35` 和 `33/0/43` 被错误解释为完整模型能力。
- 方法：按完整 30 帧重新计算 raw 与 runtime-compatible policy；对低置信 metal_can 使用固定颜色原型后备规则，不使用真值调参。
- 结果：受控夹具 30 帧、76 个实例上五类 P/R 均为 `1.0`，TP/FP/FN=`76/0/0`。
- 边界：`reviewable_95_candidate`，官方 R01、真实域和 S100P 板端仍为 `NOT_MEASURED_DEFINITION_UNSPECIFIED`。

## 4. 任务分解与安全门控

- 方法：冻结转写经确定性 DSL 解析；任务、运动和安全状态分层，直接执行器访问计数为 0。
- 结果：开发集 `36/37=0.972973`，内部冻结 holdout `32/32=1.0`。
- 边界：内部冻结回归，不是公开盲测；语音识别和任务执行未测。

## 5. 可复用证据入口

- `evidence/day1/perception-score95/`
- `video/evidence/localization-causal-filter-60s/`
- `evidence/day1/avoidance-recovery/`
- `evidence/day1/competition-task-decomposition.json`
- `evidence/day1/competition-width-efficiency-estop.md`

## 结论

这些优化形成了可复核的工程增量，但只有定位离线候选、受控感知候选和避障根因修复；没有一条能够替代官方 live 硬门。后续提分优先级仍是 live 定位、一次连续完整任务和官方避障率。
