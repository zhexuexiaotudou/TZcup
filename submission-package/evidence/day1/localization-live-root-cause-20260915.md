# 定位 live 根因复核

## 结论

run-04 的最终失败不是 `odom` frame 没有建立，而是 `map -> odom` 从未发布：

- `/odom -> base_footprint` 记录到 `2937` 条 TF。
- `map -> odom` 记录为 `0` 条。
- `/localization/raw_map_odom` 注册了 `/global_ekf` 发布者，但 bag 消息为 `0`。
- `/localization/map_odom_stabilizer/status` 保持 `WAITING`。
- Nav2 中 `bt_navigator` 和 `planner_server` 停在 inactive，`controller_server` 为 active。

## 可用输入

同一 run 中 `/scan`、`/odom`、`/imu/data`、`/gnss/fix` 和 `/odometry/gps`
都有消息，`/amcl_pose` 只有 `1` 条。因此现有证据不支持“缺少雷达或轮速输入”
这一解释。

## 判定边界

这是 live fail-closed 诊断，不是 live PASS。它只说明后续应优先检查 global EKF
的全局测量/bootstrap 链，不能评价 RMSE、P95 或 max 精度。
