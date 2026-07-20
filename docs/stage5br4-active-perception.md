# Stage5BR4：可观测性、相机消融与主动观察停止边界

## 结论

Stage5BR4 没有继续训练 Stage5BR3 的语义 TinyUNet。项目先冻结 `perception_evaluability_policy.yaml`，并对 Stage5BR3 的 3370 条真实可见实例重新读取原始 instance/depth 数组。C0 只有 875 条达到 recognition-ready，比例 `25.96%`；2495 条属于“可见但不可识别”，因此旧模型失败不能只归因于训练轮数。

相机 C0–C3 已在同一 `world_a_asphalt_campus`、scene seed 11、相同对象 pose 与 0.35 m/s 轨迹命令下实际采集。C0、C1、C2 各 10 帧；C3 的 forward discovery 与独立 downward-oblique verification 各 10 帧，所有 RGB、depth、semantic、instance 时间戳完全一致。首次 C1–C3 运行暴露 pitch 符号映射错误，失败证据保留；修正规划负俯角到 URDF 正 Y 旋转后重跑通过。

C3 verification 将该小规模审计中的 recognition-ready 比例提高到 `29.63%`，但 discovery 非 ready candidate 的 ready 转换只有 `2/4 = 50%`，未达到 `90%`；同时车辆自身像素比例 P50 为 `21.11%`，人工审计确认车体显著遮挡近场。没有配置同时通过可辨识性、主动转换和碰撞/安装审计，因此相机不允许定型。

首个阻断层为 `G2_camera_selection_blocked_active_observation_ready_conversion_below_0.90_and_manual_audit_failed`。按冻结的先后顺序，本轮没有启动 detector/area micro-overfit、120 scene/1200 native frame、三次新模型 screening、formal、正式 live、真实 active Nav2 或 J6。Stage5BR3 的旧三次失败结果保持逐字不变。

## 新增工程能力

- C0–C2 单相机 mount 参数化；C3 是物理独立的 forward discovery 与 downward-oblique verification 双 RGB-D/GT 训练相机。
- 生产默认仍只启用 C0；verification 相机、semantic/instance GT 和 vehicle self-mask 均默认关闭。
- 评测固定同时输出 `all_visible`、`recognition_ready`、`non_ready`，策略文件在训练前冻结 SHA-256。
- 主动观察状态机覆盖 `DISCOVERED → OBSERVATION_QUEUED → APPROACH_PREFLIGHT → APPROACHING → RECOGNITION_READY → CONFIRMED/REJECTED/UNREACHABLE`。
- candidate-id 去重，component 边界、路径、keepout、footprint、visibility、定位协方差、stale、timeout 与最大接近次数全部 fail-closed，并累计额外里程/时间。

## 证据入口

- `perception_observability_report.json`：完整 C0 3370 实例的可观测性与三分区统计。
- `camera_ablation_report.json`：C0–C3 真实运行、距离桶、自身像素、地面视场和 active conversion。
- `manual_recognizability_audit.json`：四张固定 SHA 图像的人工审计。
- `production_isolation/production_isolation_report.json`：生产默认 GT、双相机和 self-mask 隔离。
- `stage5br4_status.json`：当前门禁与固定 false 边界。

原始相机消融帧、首次 pitch 失败、Stage5A/Stage4W 完整回归、ROS 工作空间和旧 2.225 GB 数据均只保留本机，用户确认前不清理。

回归方面，快速 CI 68 项和受影响包 30 项测试通过；13 个 ROS 包完成 colcon build。Stage5A offline/live/bag 通过。Stage4W seed0 底层汇总为 `static_gate_pass=true`、17/17、覆盖率 93.4%、碰撞 0、定位 RMSE 0.0363 m、rosbag replay=true；外层 PowerShell 因本轮传入含 Windows 分隔符的嵌套 OutputName 在最后查找汇总时 exit 1，此收尾路径问题已单独披露，不改写底层运行结果。
