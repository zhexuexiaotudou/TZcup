# GPT Stage4R 复核报告

## 结论

`READY_FOR_GPT_REVIEW_STAGE4R=false`，`READY_FOR_STAGE5A=false`。

本轮完成了真值桥接、同帧定位评测、有效建图路线、组件化覆盖执行器、经验覆盖栅格、独立 keepout/speed mask、30 次急停统计、真实 Gazebo 俯视渲染、完整 MCAP 与回放验证。Stage4R 仍未通过，决定性阻塞是滑移里程计生成的地图发生明显几何畸变，AMCL 同帧定位 XY RMSE 为 14.0441 m，远高于 0.05 m 门槛；因此覆盖任务到首条作业带起点的两次 `NavigateToPose` 均以错误码 203 终止，实际清扫覆盖率为 0%。按提示词要求，本轮没有进入 Stage5A，也没有开展 J6 部署、量化或训练。

## 硬门逐项复核

| 门项 | 结果 | 证据 |
|---|---:|---|
| Stage3 旧 1.805570 m 指标 | 无效 | 旧值直接相减 `map` 下 AMCL 与 `odom` 下轮速里程计；新增单测强制拒绝跨帧直接比较 |
| Gazebo ground truth | 已实现 | `/ground_truth/odom`，`frame_id=map_gt`，显式记录 `world→map` 初始平移 `(+8,0)` |
| 时间同步 | 通过采样门 | 1068 对样本，丢弃 0；同步误差 P95=0.015 s、max=0.020 s |
| 同帧定位 | **失败** | XY RMSE=14.0441 m，P95=16.0237 m，max=16.1822 m；门槛 0.05 m |
| SLAM 数值质量 | 通过 | 0.05 m/px，40.85×33.15 m，known area=442.8175 m²，map_server 成功加载 817×663 地图 |
| SLAM 几何/可定位性 | **失败** | 地图 PNG 存在明显放射状畸变；AMCL 闭环轨迹与真值严重分离 |
| 组件化覆盖执行器 | 已实现 | 起点 `NavigateToPose`；12 个 swath 和 11 个 turn 分别使用 `FollowPath`，逐段等待终态，带有限重试；已删除 180/2140 点截断 |
| 完整覆盖实跑 | **失败** | 起点导航两次失败，错误码 203；未进入 23 个覆盖组件 |
| 经验覆盖 | **失败** | ground-truth cleaning footprint + brush-on 栅格；实际覆盖率 0%，规划覆盖率 97.5% 仅单列为计划值 |
| 碰撞/刷盘退出 | 当前尝试通过 | collision_count=0，退出时 brush=false；由于任务未开始，不能解释为完整任务安全通过 |
| keepout/speed mask | 配置通过、实跑失败 | 两张独立同尺寸 mask 已加载；定位失败导致路线试验未完成，不能报通过 |
| 动态障碍 20 seeds | 注入通过、交互失败 | 20/20 Gazebo SetPose 成功，但有效覆盖交互试验数为 0，不能报避障通过 |
| 急停 30 trials | 通过 | 独立速度门中 30/30 归零并恢复；P50=0.000650 s、P95=0.000797 s、max=0.001124 s；超时归零通过 |
| 3500 m²/h | **失败** | 当前 0.65 m×0.45 m/s 理论仅 1053 m²/h；0.65 m 宽至少需 1.4957 m/s，且未证明机械/稳定/制动可行 |
| 真实 Gazebo PNG | 通过 | 初始、建图、覆盖尝试、动态障碍、最终五张 Ogre2 俯视图均为相机话题实帧 |
| 完整 rosbag | 记录与回放通过 | MCAP 155.6 MiB、1264.096 s、499810 条消息；`/ground_truth/odom` 回放成功；`/brush_enabled` 为 0 条也准确反映覆盖未开始 |

## 关键修复

1. 新增 Gazebo `dynamic_pose/info → /ground_truth/odom` 真值链和 `map_gt` 坐标系，不再把协方差 trace 或跨帧终点差当作定位误差。
2. 新增带时间同步、XY/yaw RMSE/P50/P95/max、丢弃计数、CSV 和 PNG 的定位评测器，并加入 `map`/`odom` 直接相减必须抛错的回归测试。
3. 实际探索得到 817×663 地图，替换“只保存初始局部图”的证据模式；失败路线与绕开纸箱/积水后的闭环补全路线均保留。
4. 覆盖执行改成 23 个组件逐段终态判定，成功谓词要求起点、全部组件、经验覆盖、安全和刷盘退出同时满足。
5. 新增基于 Gazebo 真值与 `cleaning_footprint` 偏移的 brush-on 经验覆盖栅格，不再把 Fields2Cover 计划几何率写成实测率。
6. 修复 Nav2 全局 `/cmd_vel` 重映射造成的 collision-monitor/velocity-smoother 反馈环；现在链路为 Nav2 → smoother → collision monitor → `/cmd_vel_gate` → emergency-stop gate → `/cmd_vel`，任务结束后实测归零。
7. 新增独立 keepout/speed mask、动态红色障碍物、固定俯视 RGB 相机和完整证据清单。

## 证据入口

- 总结：`artifacts/stage4r_20260715_093931/stage4r_summary.json`
- 完整清单：`artifacts/stage4r_20260715_093931/MANIFEST.json`（61 个文件，逐文件 SHA256）
- 地图：`slam_quality_report.json`、`stage4r_slam_map.{yaml,pgm,png}`
- 定位：`localization_report.json`、`localization_trajectory.csv`、`localization_error.png`
- 覆盖：`coverage_report.json`、`coverage_path.json`、`coverage_trajectory.csv`
- 过滤器：`filter_report.json`、`filters/keepout_mask.*`、`filters/speed_mask.*`
- 动态障碍：`dynamic_obstacle_report.json`、`gazebo_dynamic_obstacle.png`
- 急停：`safety_latency_report.json`
- 效率：`efficiency_scan.json`
- rosbag：`full_mission_bag/metadata.yaml`、`rosbag_info.txt`、`replay_ground_truth.txt`
- 失败证据：`mapping_probe_attempt1_failed.json`、`invalid_feedback_loop/`、`localization_attempt1_failed/`

## 下一轮唯一主线

先修复“转向滑移导致轮速里程计闭环漂移、SLAM 地图畸变、AMCL 粒子退化”这一条根因链，再重跑同帧定位门。建议按顺序：校准四轮滑移/差速模型与轮距半径 → 对比 `/odom/unfiltered`、EKF、Gazebo 真值的分段误差 → 用已校准里程计重新建图 → 检查地图拓扑与闭环 → AMCL RMSE≤0.05 m 后再重跑组件覆盖、keepout/speed 和 20-seed 动态避障。不得绕过定位门直接宣称覆盖或效率通过。
