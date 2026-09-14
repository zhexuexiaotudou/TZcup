# 动态避障恢复根因与最小修复

本目录归档 `codex/day1-avoidance-recovery@99ee7f0` 的根因分析、最小修复证据和单次恢复验收器，用于支撑算法优化和下一轮闭证。

## 根因

Nav2 recovery 原地旋转期间，`global_ekf` 将 `navsat_transform` 基于自身 `/localization/fused_odom` 生成的 `/odometry/gps` 作为全局测量再次反馈，导致 `map -> odom` 漂移到异常位置。AMCL、局部 odometry、碰撞监视和安全心跳均保持正常。

## 最小修复

- 增加 `global_gnss_odometry_topic` 覆盖点。
- 生产默认仍为 `/odometry/gps`。
- 单次恢复验收器只把该输入切到无 publisher 的 `/odometry/gps_disabled`，断开错误闭环，同时保留 AMCL、local EKF、NavSat 和 global EKF 的原有职责。

## 边界

- 该修复和 harness 已通过离线回归，但本轮没有新的 Gazebo 成功运行。
- 单次成功最多写为 `FUNCTIONAL_PASS_1_OF_1_NOT_OFFICIAL_95`。
- 官方避障成功率仍为 `NOT_MEASURED`，不能把根因修复写成 95% 通过。
