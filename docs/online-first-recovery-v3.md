# OPRV3 在线优先感知恢复

OPRV3 是 PR #90 上独立于 X1/X2/X3 与 MRV2-A/B/C 的新协议。历史静态失败保持原样，`MODEL_BLOCKED_INTERNAL=true` 也不会因测量口径变化而被改写。新协议先回答移动清扫车在目标进入安全可行动窗口后，能否在错过清扫机会前完成发现、分类、跟踪和地图定位；旧的单帧、`<18 px`、AP、FP 与 area 指标继续作为诊断并完整报告。

## 测量边界

- `ObservableTargetEncounter` 和可见/遮挡/深度状态只由独立 evaluator 使用；生产 observation 不含 GT identity 或 GT 坐标。
- `ActionableObservationWindow` 在看任何 OPRV3 移动模型结果前，从已冻结的 AUTO-05R 相机、15 Hz、Nav2 清扫速度/减速度、控制延迟、刷盘前向偏置、Spot Cleaning 三次确认规则和 G4 真实物理尺寸推导。
- 低置信度 observation 只能进入多帧跟踪；observation、track confirmation 与 clean action 使用严格递增的独立阈值。
- 所有 GT target 都必须落入 `never_in_camera_frustum`、`occluded_entirely`、`visible_but_never_actionable` 或 `entered_actionable_window`，不得按模型结果事后缩小分母。

## 门槛来源

[`oprv3_gate_provenance.yaml`](../starter_ws/src/sanitation_learning/config/oprv3_gate_provenance.yaml) 将门槛分为 `OFFICIAL_GATE`、`INTERNAL_DIAGNOSTIC_GATE` 和 `ONLINE_PRODUCT_GATE`。截至 2026-08-10，仓库与公开一手材料没有给出可核验的本项目官方感知统计定义；因此仓库内部的 `3500 m²/h`、`<18 px recall >= 0.70` 等规则不冒充当前官方赛题门。缺失官方原文时 `COMPETITION_PERCEPTION_PASS` 保持 false/未判定。

## 阶段边界

OPRV3-00/01 的代码和解析测试只建立门槛溯源、事件 schema 与解析几何。公式审计不能替代 Gazebo 移动车辆实测；在至少每类 20 个目标的经验探针完成前，不允许声称 OPRV3-01 通过。现有 MRV2-C/MRV2-A/X3 只有完成不少于 20 个移动任务的开发矩阵后，才可决定直接进入在线集成或启动 G6/新模型恢复。
