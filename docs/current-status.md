# 当前项目状态

本页只描述当前有效状态，不记录按日期或尝试次数排列的开发过程。历史变更由 Git、PR 和 `artifacts/` 中的紧凑证据承担。

## 产品成熟度

| 领域 | 当前结论 | 主要边界 |
|---|---|---|
| 仿真与车辆模型 | 可运行 | 已具备 Gazebo 园区、清扫车、传感器和基础清扫演示 |
| 定位、导航与覆盖清扫 | 可运行 | 已具备 SLAM、Nav2、安全控制、覆盖规划和补扫链路 |
| 人机监督与可视化 | 可运行 | 支持 Gazebo、RViz 和浏览器看板；不替代真实场地验收 |
| 学习感知 | 内部阻断 | 静态检测候选通过，但运动相机在线质量和严格性能门未通过 |
| J6 端侧交付 | 外部阻断 | 缺少当前可用工具链、冻结 student 和实体板验证 |
| 真实场地交付 | 外部阻断 | 缺少正式 RGB-D 录制、独立地图真值和现场验收 |

因此，仓库当前不能表述为已经完成实车产品部署。有效产品标志仍为：

- `MODEL_BLOCKED_INTERNAL=true`
- `PRODUCT_X86_PERCEPTION_READY=false`
- `PRODUCT_J6_TOOLCHAIN_READY=false`
- `PRODUCT_J6_BOARD_READY=false`
- `PRODUCT_FIELD_READY=false`

## 当前感知结论

TRCRV10 已证明三类近距目标在短边 `>=18 px` 时可辨识，并冻结 T3 Grounding-DINO proposal operating point：threshold `0.37`、2 帧 persistence、eventual/small proposal recall 均为 `1.0`、FP/frame `0.00926`。这不等于产品四分类通过；V10 ConvNeXt-Tiny targeted recovery 的 macro-F1 只有 `0.6318`，background specificity 只有 `0.0833`。

CRCRV11 的冻结审计确认 proposal crop 不是主要退化来源：GT tight 到 proposal tight 的 macro-F1 只下降 `0.0064`，context 反而提高 `0.0058`。真正的数据合同缺口是 TRAIN 只有 9 个 unique background crop、replacement sampler 期望重复约 `186.64` 次，且 runtime-faithful positive view 只占 `40%`。100 个 source→PNG→torchvision 样本全部通过像素与 RGB 通道一致性；V10 HOLDOUT context manifest 另有记录 proposal box 而非实际 1.6x context box 的元数据缺陷。

C11 只用 G10_TRAIN 重建 matched proposal tight/context positive、真实 FP、0.05–0.37 hard negatives 和 negative-only 固定地面 ROI，得到 7,491 个 TRAIN candidate pairs、6,576 个 unique background tight crop；与 298 个 HOLDOUT pairs 的 world、source frame、exact crop 和 pHash 交叉均为 0。R1 将 background specificity 恢复到 `1.0`，但 target macro-F1 只有 `0.5397`；R2 combined macro-F1 为 `0.6311`、background specificity `0.6333`；有 22.76% tight/context 互补证据的 R3 macro-F1 仍只有 `0.4561`。

因此 R1/R2/R3 已按唯一授权路线全部失败，当前停止条件为 B：`CLOSE_RANGE_CLASSIFIER_CONTRACT_BLOCKED=true`、`MODEL_BLOCKED_INTERNAL=true`、`SIMULATION_PRODUCT_COMPLETE=false`。ActionVerifier、integrated HOLDOUT、`G10_DEV_VAL_SEALED`、`VAL_NEW`、Tracker/Map、在线仿真、性能、x86 Freeze、`G5_V2`、30-seed、Spot Cleaning、soak、MCAP replay 和 release 均未执行；禁止追加 R4、重开 detector 搜索或降低安全门。

## 权威入口

- 系统目标与接口：[`PROJECT_SPEC.md`](../PROJECT_SPEC.md)
- 验收门定义：[`STAGE_GATES.md`](../STAGE_GATES.md)
- 当前机器可读状态：[`FINAL_AUTONOMOUS_STATUS.json`](../FINAL_AUTONOMOUS_STATUS.json)
- 当前阻塞项：[`FINAL_BLOCKER_REGISTER.json`](../FINAL_BLOCKER_REGISTER.json)
- DDRV4 最终证据：[`artifacts/detector_data_recovery_v4_20260811T134117Z/final/`](../artifacts/detector_data_recovery_v4_20260811T134117Z/final/)
- ODCV5 协议与当前阶梯：[ONLINE-DOMAIN-CLOSURE-V5](online-domain-closure-v5.md)
- 开发和交付规则：[开发工作流](development-workflow.md)

## 下一步解锁条件

后续工作按以下边界继续：

1. 收集或构建与真实 Gazebo/目标资产分布一致、严格 TRAIN/HOLDOUT/VAL 隔离的移动离散数据，形成新的受限研究协议；当前 CRV6 不允许用已读取的真实 Gazebo开发回放继续调参；
2. 使用现有 G6 Area candidate 做独立 integration 修复或补充未消费的边界/负样本开发集，使 boundary F1 与 negative FP/frame 达到 CRV6 门；
3. 获得当前 J6 工具链、冻结 student 和授权实体板，执行可追溯转换与实板验收；
4. 获得正式 RGB-D 录制、独立地图真值和现场授权，执行真实场地验收。

更新本页时应直接替换已经失效的结论，不追加日期、轮次或提交日志。
