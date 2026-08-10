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

## OPRV3-01 移动相机经验探针

开发采集使用 AUTO-05R 产品相机姿态、`0.65 m/s` 指令速度、90 帧/任务、四路 RGB/depth/semantic/instance 严格同戳和 50 ms 内里程计匹配。采集回调在运动阶段采用有界内存缓冲，停车后再持久化，避免把 PNG/NPY 写盘速度误当相机节奏。通过集包含 24 条任务、2160 帧、4 个 world，其中 20 条正向任务为每类恰好 20 个 GT 目标，另有 4 条 negative-only 任务。全部通过任务均为 90/90 帧，最大传感器/里程计偏差 50 ms，实测速率中位数为 `0.65 m/s`。

严格四路 GT 的有效采样率为 `1.6425–8.4810 Hz`、中位数 `5.0898 Hz`，低于相机标称 15 Hz。100 个正向 GT 中 99 个取得至少 3 个实际可行动采样帧；1 个 metal-can 只有 1–2 个窗口采样，单列为 `insufficient_sampled_actionable_frames`，不计作模型漏检，也不从 `all_gt_targets` 删除。湿地长批次在共享 GPU 负载下多次墙钟超时，失败报告完整保留；同参数单场景湿地 smoke 曾 90/90 通过，只用于反射能力诊断，不冒充正式覆盖。

## OPRV3-02 现有模型前向开发矩阵

MRV2-C、MRV2-A 和 X3 在完全相同的 24 条任务上运行，G5 与 legacy D6 均未读取。GT 只在离线 evaluator 中构造窗口和匹配；生产模型只接收图像输入。冻结阈值分别来自各自 TRAIN holdout，低置信观察不会直接触发动作。

| route | eventual detection | eventual correct class | three-frame confirmation | actionable miss | wrong actionable rate |
|---|---:|---:|---:|---:|---:|
| MRV2-C | 1.0000 | 1.0000 | 0.9495 | 0.0000 | 0.01187 |
| MRV2-A | 1.0000 | 1.0000 | 0.9798 | 0.0000 | 0.00967 |
| X3 | 0.9899 | 0.9899 | 0.9495 | 0.0101 | 0.00458 |

MRV2-A 是当前前向开发候选：核心 eventual recall/class 与错误率满足 OPRV3-02 数值门；MRV2-C 的错误率超过 `0.01`，X3 漏掉一个 eligible metal-can。该矩阵仍不能通过完整 OPRV3-02，因为当前任务没有后方入视野、转弯、显式遮挡和正式反射覆盖，也没有完成在线地图/area 正式门。故不创建 freeze，不读取 sealed final，不启动 30-seed 或 Spot Cleaning 正式门，`MODEL_BLOCKED_INTERNAL=true` 保持不变。

原始外部证据：

- `F:\Project\TZcup\.workspace\artifacts\TZcup-oprv3-moving-benchmark-v1\OPRV3_MOVING_BENCHMARK.json`，SHA256 `4b0368dfe2ef4b9c4abd2fb2b997c12d088fb1279d95bb1af60e5c3024233298`。
- `F:\Project\TZcup\.workspace\artifacts\TZcup-oprv3-moving-benchmark-v1\PIXEL_DISTANCE_EMPIRICAL_REPORT.json`，SHA256 `1ae9660897117da3e23b486f58e3ba61930e1cc4259c78b6ed9755303586e6d9`。
