# Stage4V 混合定位架构与验证约束

## 1. 生产数据链

Stage4V 将连续局部运动估计与全局定位拆开：

1. 既有 EKF-B 继续发布 `/odom`，并独占 `odom -> base_footprint`；
2. `sanitation_scan_refiner` 读取 `/map`、`/scan` 和先验位姿，在无真值参与的前提下发布 `/localization/refined_pose`；
3. `hybrid_global_fuser` 读取 `/odom`、标准 `sensor_msgs/NavSatFix` 和扫描精化位姿，发布 `/localization/fused_pose`；
4. `hybrid_global_fuser` 是混合定位模式下唯一的 `map -> odom` 发布者；
5. Nav2 只消费 TF、标准里程计和地图，不订阅任何 `/ground_truth/*` 话题。

扫描精化器采用占据栅格距离场、双线性查值、Huber 鲁棒损失、先验正则和三级 coarse-to-fine SE(2) 搜索。默认搜索窗口为 `±0.20 m / ±3 deg`，最细步长为 `0.005 m / 0.05 deg`。低点数、高残差、改善不足或局部不可观时不发布精化位姿，并在 `/localization/refiner_diagnostics` 给出原因和累计接受/拒绝计数。

## 2. 仿真传感器边界

`sanitation_gnss_sim` 只允许在仿真 launch 中启动。它可从 `/ground_truth/odom` 生成传感器读数，但输出必须经过固定偏置、白噪声、随机游走、延迟、丢包和多路径模型，且只通过标准 GNSS 消息进入融合器。诊断固定声明：

- `simulated_sensor=true`
- `ground_truth_direct_fusion=false`

内置剖面为：

| 剖面 | 频率 | 平面标准差 | 延迟 | 额外行为 |
|---|---:|---:|---:|---|
| `rtk_fixed` | 10 Hz | 0.02 m | 0.10 s | 固定偏置与随机游走 |
| `rtk_float` | 10 Hz | 0.12 m | 0.10 s | 固定偏置与随机游走 |
| `gnss_denied` | 10 Hz | - | 0.10 s | 不发布定位消息 |
| `multipath` | 10 Hz | 0.02 m | 0.10 s | 1% 概率注入 0.50 m 异常值 |

## 3. TF 所有权

混合模式的合法 TF 链为：

```text
map --(hybrid_global_fuser)--> odom --(EKF-B)--> base_footprint
```

`sanitation_tf_ownership_audit` 同时检查目标变换、运行时 `/tf` 发布端点、预期所有者存在性，以及 `amcl` / `slam_toolbox` 等禁止的第二全局所有者。Jazzy 的 Python 订阅回调不提供可用于逐消息归因的发布者 GID，因此审计明确使用“配置所有者 + 运行时端点图”方法，不把空 GID 当作发布者证据。

## 4. 车端边界

本阶段不实现 J6 专有驱动或硬件部署。可迁移部分仅包括：

- C++17 扫描匹配和融合节点；
- 标准 ROS 2 消息与 TF；
- x86/ARM 均可编译的 CPU 实现；
- 可复现的配置、单测和仿真门禁。

`sanitation_gnss_sim`、Gazebo 真值适配器和评估器不属于生产部署集合。上车前仍需补齐真实 GNSS/RTK 驱动、时间同步、传感器外参、ARM 交叉编译、实时资源测量与硬件在环验收。

## 5. 正式门禁

正式主定位门禁要求同一路线 10 个种子全部完成，并同时满足：

- 每个种子的 XY RMSE 不大于 `0.05 m`；
- 10 个种子 RMSE 的聚合 P95 不大于 `0.05 m`；
- 导航成功 `10/10`；
- TF 连续且单一所有者 `10/10`；
- 真值控制违规为 0。

只有在正式定位门禁通过后，才允许继续全量 Coverage、20 次动态障碍、30 次急停和 replay 回归。扫描精化累计接受数为 0 时，必须将结果标为 RTK 回退证据，不得宣称混合扫描修正已经生效。
