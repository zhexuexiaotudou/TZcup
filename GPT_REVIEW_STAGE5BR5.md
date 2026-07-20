# Stage5BR5 GPT 独立复核入口

## 结论先行

Stage5BR5 已完成 ActiveObservation 时间语义修复、verification 相机机械网格、六世界真实运行时消融、200 张平衡盲审集、evaluability policy v2 草案、观测位姿规划核心与 ROS 2 `ComputePathToPose` wrapper。复核材料完整，但相机尚未定型：盲审集虽达到五类各 40 张并覆盖六世界，当前没有两名独立人工评审结果，故 manual gate 必须为 false。

```text
first_blocking_layer=G2_camera_selection_blocked_two_independent_human_manual_reviewers_not_available
REVIEW_PACKET_COMPLETE=true
READY_FOR_GPT_REVIEW_STAGE5B=false
READY_FOR_STAGE5C=false
```

这不是“程序代替人工审核”。`manual_recognizability_audit_v2.json` 中准确率、Cohen kappa 与自遮挡失败率均保持 `null`。因此未执行正式 oracle-candidate 主动观察、detector/area micro-overfit、120/1200 screening、500/5000、正式 live、真实 30 次 Nav2 spot-clean 或 J6。

## 请优先复核

1. `artifacts/stage5br5_20260720_review/stage5br5_status.json`：机器可读总状态和停止边界。
2. `artifacts/stage5br5_20260720_review/camera_mechanics_report.json`：V1/V2/V4 机械可行，V3 因 trial footprint 冲突被剔除；生产 footprint 未修改。
3. `artifacts/stage5br5_20260720_review/camera_grid_report.json`：V1/V2/V4 × 6 world × discovery/verification × 10 帧，共 360 帧真实 Gazebo 采集。
4. `artifacts/stage5br5_20260720_review/manual_blind_audit_v2/`：200 张盲审 crop、两份空白独立评审模板和不提供给评审者的 truth mapping。
5. `artifacts/stage5br5_20260720_review/manual_recognizability_audit_v2.json`：人工门未执行而非失败指标被伪造。
6. `starter_ws/src/sanitation_spot_cleaning/sanitation_spot_cleaning/active_observation.py`：首见、末见、排队、preflight、approach 与 observation 时间分离。
7. `starter_ws/src/sanitation_spot_cleaning/sanitation_spot_cleaning/observation_pose_planner.py` 与 `observation_pose_node.py`：ROS-independent 几何核心和真实 Nav2 action wrapper；默认自遮挡参数为 1.0，未配置时 fail-closed。
8. `artifacts/stage5br5_20260720_review/artifact_manifest.json`：所有复核文件的大小与 SHA-256。

## 相机网格结果

- V1：低位前悬斜视；机械/运行门通过，v2 ready fraction `0.13450`。
- V2：中位前悬斜视；机械/运行门通过，v2 ready fraction `0.13636`。
- V3：高位向下；旋转后的相机碰撞盒超出 Stage5BR5 trial footprint，机械门失败，未进入 Gazebo 消融。
- V4：侧向 verification；机械/运行门通过，v2 ready fraction `0.30508`，仅作为盲审数据候选。

三项实际运行候选的 verification self pixels P95 最坏世界均为 `0.0`，target/self overlap 最大值均为 `0.0`。这些结果只证明相机运行时物理/遮挡门通过，不替代人工可辨识门。所有候选都位于当前 production Nav2 footprint 外；在相机未定型前，production footprint 保持原样。

报告中的 matched-candidate conversion 是同 pose/seed 的 discovery/verification view replay，仅有 22 个 matched candidates，不包含 Coverage boundary、`ComputePathToPose`、`NavigateToPose`、capture、return/resume 的真实主动观察闭环，因此明确标记为 `not_active_observation`，不得用于宣称 `>=0.90` 主动观察门通过。

## ActiveObservation 修复

- 重复 discovery 刷新 `last_seen_s`，不再用首次发现时间误判传感器 stale。
- sensor stale 与 queue timeout 分离；长 component 等待不会因首次发现时间过早而被拒绝。
- approach deadline 为 `base + path_length / minimum_speed`。
- 空间合并支持模型 ID 改变；每个终态保留 reason、额外里程和时间。
- 旧 `discovered_at_s` 记录可迁移；continuous refresh、stale、long wait、duplicate、two approaches、timeout、unreachable 与 Coverage resume 均有测试。

## 复核边界

回归结果：`ci_fast` 68/68；受影响 ROS 包 colcon build 通过、29 tests/0 failures；Stage5A 30/30 spot-clean、119 帧 live、GT control violation 0、rosbag 已录制；Stage4W seed0 完成 17/17 组件，经验覆盖率 `0.944`、定位 RMSE `0.03737 m`、碰撞/keepout/brush violation 0、MCAP replay 通过；生产默认运行时 GT 隔离通过。

固定结论保持：

```text
competition_perception_pass=false
real_domain_evaluation_executed=false
j6_runtime_pass=false
competition_efficiency_pass=false
theoretical_efficiency_m2_h=1053
target_efficiency_m2_h=3500
```

下一步只能先安排两名互相独立且看不到 truth mapping 的人工评审，回填两份 response JSON，再计算每类 recognition-ready accuracy、Cohen kappa 与 self-occlusion failure。只有 manual gate、相机选择和随后真正六世界 oracle active-observation 全部门通过，才允许进入模型微过拟合。
