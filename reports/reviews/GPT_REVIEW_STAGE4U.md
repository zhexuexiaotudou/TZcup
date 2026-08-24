# TZcup Stage4U 独立复核入口

## 结论

Stage4U 已完成坐标系审计、粒子云修复、地图/定位器隔离、M1/M2/M3 对照、AMCL/SLAM Toolbox 筛查、LiDAR/AMCL 灵敏度试验和正式 Oracle 10-seed。当前仍不能进入 Stage5A：最优候选的 map-relative XY RMSE P50/P95/max 为 `0.06767/0.07983/0.08022 m`，高于 `0.05 m` 硬门。

`READY_FOR_GPT_REVIEW_STAGE4U=false`，`READY_FOR_STAGE5A=false`。

## 本轮修复

- 统一使用显式 `T_target_source` 的 SE(2) 变换，冻结一次性标定并校验哈希；禁止逐 seed 拟合。
- 将定位误差拆成 map-relative、地图地理配准误差、历史直接世界误差和标定后世界误差。
- 粒子云改为 Jazzy 实际类型 `nav2_msgs/msg/ParticleCloud`，匹配 best-effort QoS，并记录权重均值、协方差、spread、ESS、max weight、entropy 和退化次数。
- 地图生成、基础质量和 5 cm 定位几何质量分门统计；旧 `pass` 不再代表定位地图合格。
- AMCL 与 SLAM Toolbox 后端可切换；LiDAR 分辨率/频率、Nav2 参数、世界和固定标定可复现注入。
- 10-seed 聚合改为 fail-closed：0 样本、导航未完成、粒子仪器无效或缺少 RMSE 的 seed 不计为完成。

## 地图与后端矩阵

- M1 `slam_raw`：刚体标定残差不足以支撑定位，AMCL 单 seed RMSE `0.19926 m`。
- M2 `slam_refined`：已保存 posegraph，但未执行独立离线优化/重新渲染；地图定位几何门失败。AMCL `0.08744 m`，SLAM Toolbox localization `0.29788 m`。
- M3 `surveyed_reference`：仅作定位参考，不冒充 SLAM 建图成绩。稀疏场景 baseline `0.12427 m`；AMCL 调参后 360@10 Hz `0.11293 m`，720@20 Hz `0.12232 m`。
- 结构化 v2 场景加入可见建筑边界、灯杆、树干和垃圾箱，不含隐藏定位标记；0.05/0.02 m 参考图单 seed 分别为 `0.06473/0.06281 m`。

## 正式 Oracle 10-seed

候选：M3 结构化 v2、0.02 m surveyed reference、AMCL 精度配置、360 samples @ 10 Hz。

- 完整种子：`10/10`
- 导航成功：`10/10`
- TF 连续：`10/10`
- 粒子仪器有效：`10/10`
- 恢复次数：`0`
- XY RMSE P50/P95/max：`0.067669/0.079833/0.080218 m`
- worst seed：`seed_7`
- `oracle_localization_pass=false`

按主提示词停止条件，未运行 realistic 10-seed、完整 Coverage、20-seed 动态障碍、30 次急停和全任务 rosbag replay。理论效率仍为 `1053 m²/h`，没有写成目标 `3500 m²/h`。

## 建议 GPT 重点复核

1. 5 cm 指标是否应继续采用 AMCL+2D LiDAR，还是允许升级传感器/融合后端。
2. surveyed reference 的结构化特征是否符合比赛场景边界；它明确不计作 SLAM 建图成绩。
3. M2 posegraph 只证明序列化成功，是否值得新增独立离线优化和重渲染路线。
4. 当前 6.8–8.0 cm 稳态误差是否主要来自 2D scan matcher/AMCL 分辨率，而不是坐标硬编码或粒子仪器缺失。

## 证据入口

- `artifacts/stage4u_20260716_review/stage4u_summary.json`
- `artifacts/stage4u_20260716_review/oracle_10seed_compact.json`
- `artifacts/stage4u_20260716_review/map_lane_summary.json`
- `artifacts/stage4u_20260716_review/MANIFEST.json`

原始 MCAP、posegraph、逐 seed 图片和日志保留在本地工作树，未塞入仓库；压缩包内包含本复核入口、紧凑证据和可复现源码。
