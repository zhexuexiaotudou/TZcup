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

Detector Data Recovery V4 的 G7 静态候选达到 recall/precision `0.9778/0.9778`，但运动相机兼容回归的 eventual recall 为 `0.3898`，metal recall 为 `0.1053`，产品地图 precision 为 `0.2111`。性能回放达到 `9.9974 Hz`、P95 `155.83 ms`、掉帧率 `0`，仍未满足严格 `>=10 Hz` 门。

ODCV5-00 对同一 24 mission 回放建立了严格单调的在线损失阶梯。60 个离散 GT 全部进入视野，59 个进入 actionable window；随后 observation/action-threshold/correct-class 只剩 `52/23/10`。10 个 correct-class 且 depth-valid 的目标中，只有 3 个能与同类 product-map target 审计关联。当前首要损失是 detector score 与类别，projection 到 map 仍有复合损失；旧格式缺逐目标 scheduler attribution，也不能把 projection、tracker、map 三者独立归因。

细粒度根因决策将直接可证的损失归因为 `DETECTOR_SCORE_LOW=29`、`DETECTOR_WRONG_CLASS=13` 和 `OUTSIDE_ACTIONABLE_WINDOW=1`。7 个 observation miss 因旧报告未保留完整 proposal 列表而不能在 `DETECTOR_NO_PROPOSAL` 与 `DETECTOR_BOX_IOU_FAIL` 间伪拆；7 个 projection-to-map 复合 miss 和 3 个 scheduler 决策同样保持 legacy trace gap。

CRV6 已完成历史 D1-B 最终恢复审计，精确 SHA-256 `481374d4...a361` 仍不可恢复；历史通过事实不改写。R1 以已审计初始化重构出 hash-new candidate `0d6f4e83...aa8d`，static historical regression、150-frame P0/P1/P2 parity 均通过。原生 G7-MOVING gate 失败后，仅执行授权的 MA1；MA1 checkpoint `7e823494...0b9f3` 在 HOLDOUT 选阈值 `0.21`，独立 MOVING VAL 的 eventual/correct/per-class/small/precision 均为 `1.0`，wrong 与 negative 均为 `0`，AP50/AP50:95 为 `0.9988/0.9929`。

ODCV5-02 已建立独立的 G7-MOVING 开发包：`MOVING_TRAIN/HOLDOUT/VAL=30/10/15` missions、990 帧，完整保存同步 RGB-D、CameraInfo、timestamp、vehicle pose/TF 与 evaluator-only semantic/instance GT。world/seed split 交叉、RGB exact 重复和跨 split pHash 重复均为 0，规定覆盖在三个 split 中完整。TRAIN 每类 58 个、VAL 每类 28 个 actionable encounter，VAL 每类 14 个 first-visible `<18 px` encounter；`required_coverage_complete=true`、`G7_MOVING_PASS=true`。

MA1 随后在现存物理一致的真实 Gazebo 24-mission/2160-frame 开发回放上执行 RGB-D→projection→tracker→DynamicTrashMap→scheduler。Projection 误差门通过（median `0.0278 m`、P95 `0.0350 m`、map RMSE `0.0732 m`），但真实 Gazebo 离散 eventual correct-class recall 仅 `0.1356`，离散 map precision/coverage 为 `0.6667/0.0333`；behind-FOV、turn、occlusion、reflection coverage 也不完整。现有 G6 Area 的 mIoU `0.9502`，但 boundary F1 `0.7672 < 0.80`，negative actionable FP/frame `0.02051 > 0.02`。因此 CRV6-07 fail-closed，`MODEL_BLOCKED_INTERNAL=true`；freeze、G5_V2、30-seed、Spot Cleaning、soak、release、J6 student 和 field evaluation 未解锁。

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
