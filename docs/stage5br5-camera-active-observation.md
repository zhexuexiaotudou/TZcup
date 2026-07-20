# Stage5BR5 相机机械重构与主动观察基础

## 目标与停止规则

本轮先修正 Stage5BR4 主动观察状态机的时间语义，再建立可验证的相机安装、运行时、人工可辨识与主动观察前置链。任何前置门失败都停止后续模型工作；不会通过降低阈值或把 view replay 改名为主动观察来获得通过。

## 时间语义

候选任务分别记录 `first_seen_s`、`last_seen_s`、`queued_at_s`、`preflight_started_s`、`approach_started_s`、`approach_deadline_s` 与 `last_observation_s`。传感器 stale 只依据最近一次观测，queue timeout 只依据排队时间。路径返回后以路径长度和最小接近速度计算动态 approach deadline。空间邻近候选可在模型 ID 变化时合并，但 class 不同或超过 merge radius 时不会误合并。

## 相机机械网格

四个配置同时记录 `base_link` 与前保险杠相对坐标。机械评价使用车辆 Xacro 中的 body、front bumper、双刷盘、机械臂预留体积、地面、最大安装高度、相机旋转 AABB 和 Stage5BR5 trial footprint。

| 配置 | base_link xyz (m) | pitch | 机械结果 | 运行结果 |
| --- | --- | ---: | --- | --- |
| V1 | `[0.67, 0, 0.30]` | -35° | 通过 | 六世界通过 |
| V2 | `[0.67, 0, 0.48]` | -50° | 通过 | 六世界通过 |
| V3 | `[0.69, 0, 0.70]` | -60° | trial footprint 冲突，剔除 | 未运行 |
| V4 | `[0.67, 0.34, 0.48]` | -50° | 通过 | 六世界通过 |

当前 production Nav2 footprint 仍是 `x ±0.40 m / y ±0.36 m`。所有前悬候选都超出该 footprint，因此本轮只使用 trial footprint 做碰撞验证；没有选择相机，也没有修改生产 footprint。

## 六世界运行时消融

V1、V2、V4 在六个真实 Gazebo Harmonic 世界分别运行 discovery 与 verification，每个 role 保存 10 组精确同步 RGB、depth、semantic 和 instance 帧，共 36 次 capture、360 帧。世界、asset、seed、车辆与目标 pose、轨迹命令在同一世界的相机对照中保持一致。

V1/V2/V4 均满足 verification self pixels P95 `<=0.05`、target/self overlap `<=0.05` 和机械 collision/envelope 门。v2 recognition-ready fraction 分别为 `0.13450 / 0.13636 / 0.30508`。V4 因 v2 ready fraction 最高而用于构造盲审集，但没有被选为生产或训练相机。

## 平衡盲审集

最终盲审集为 200 张 crop，五类各 40 张，覆盖六世界。评审字段为 class、target/no-target、suitable-for-recognition、self-occluded 和 confidence。`reviewer_a_responses.json` 与 `reviewer_b_responses.json` 均为空白模板；`truth_mapping_not_for_reviewers.json` 必须与评审者隔离。

当前没有两名独立评审者，因此 accuracy、Cohen kappa 与 self-occlusion failure 均不计算，manual gate 为 false。自动脚本只负责抽样、盲化、哈希和门禁汇总，不能替代人工判断。

## Policy v2

v1 文件不修改。v2 新增离散/区域类最小可见比例、最大 target/self overlap 与 boundary completeness；保留 shortest side、mask area 和 depth 条件。由于人工审计尚未完成，v2 明确标为候选草案：`frozen_before_model_training=false`、`training_permitted=false`。任何修改都必须使用新 policy id 和新 SHA-256。

## Observation pose planner

纯几何核心在目标周围按弧段和 standoff 采样，检查 cleanable/keepout polygon、footprint clearance、定位协方差、预期像素、ROI、视角、自遮挡、路径长度和转向代价。ROS 2 wrapper 使用 TF 获取当前位姿，并逐候选调用 Nav2 `/compute_path_to_pose`；无路径或未配置真实相机遮挡参数时 fail-closed。输出始终记录 `ground_truth_pose_used=false`。

相机尚未通过人工门，因此本轮没有把 planner 包装成“已执行的主动观察”：正式要求的 6 world、60 scene、200 matched candidate、每类 30，以及 Coverage boundary → ComputePath → Navigate → capture → return/resume 均未运行。

## 本地原始证据

仓库只提交紧凑复核材料。约 GB 级原始帧、失败的非平衡盲审尝试、Gazebo 日志、回归 rosbag 和运行时工作区保留在 `F:\Project\TZcup-stage5br5-data` 与 `F:\Project\TZcup-stage5br5-runtime`，等待用户确认后再决定清理。
