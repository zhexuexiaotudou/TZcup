# 当前项目状态

本页只保留当前有效结论，不记录尝试轮次、日期流水或中间产物。

## 最终状态

```text
SIMULATION_PRODUCT_COMPLETE=false
PRODUCT_X86_PERCEPTION_READY=false
PRODUCT_INTEGRATION_READY=false
PRODUCT_FIELD_READY=false
```

原因不是裁决工具缺失，而是当前没有一套绑定固定 V1 合同、覆盖 A–P 且通过全部硬门的正式证据。历史实验报告、旧分支和旧运行目录不迁移为新合同的通过证据。

## 已具备的产品基线

| 领域 | 当前代码能力 | 当前证据边界 |
|---|---|---|
| 车辆与仿真 | Ackermann Xacro、Gazebo steering plugin、wheel/steering state、Ackermann EKF、Nav2 与 Coverage profile；默认入口已切换为 Ackermann | 静态/单元合同通过；仍需正式运行矩阵 |
| 清扫机构 | 默认有效刷宽 1.32 m、40 L 箱体几何、真实 footprint 清扫足迹 | 几何合同存在；效率与长期运行未通过 |
| 定位/导航/覆盖 | SLAM/定位、双天线 GNSS 航向融合、Nav2、未知栅格 frontier 探索、地图保存/硬重启/加载/重定位/多航点导航、keepout、Collision Monitor、skip-lane Ackermann Coverage、repair 与可视化链 | 40 m × 20 m 整链烟测已通过，但不能替代 20,000 m²、30-seed 和 3500 m²/h 正式门 |
| 离散感知 | proposal、近距四分类接口、ActionVerifier、重观察、Tracking、RGB-D 投影、DynamicTrashMap | 当前已知近距分类结果低于 macro-F1 0.98，且新 V1 正式证据未建立 |
| Area 感知 | leaf/puddle runtime、训练评估与独立指标代码 | 需在固定 split 与完整产品链重新冻结/验收 |
| Spot/Post-Clean | Scheduler、Pre-Clean、执行协调、camera-backed Post-Clean 状态机 | 缺少 ≥30 seeds 的零错误清扫正式证据 |
| 交互 | HMI、Speech/任务接口与安全边界代码 | 缺固定集 ≥95% 与两个模态综合正式报告 |
| 冻结/发布 | sealed one-shot、manifest、hash、x86 packaging、SBOM/许可工具基础 | 尚未形成 V1 freeze、sealed final、release 与真实 rollback 证据 |

## 固定裁决能力

[`scripts/product_acceptance.py`](../scripts/product_acceptance.py) 已实现：

- 验证验收原文 SHA-256，阻止静默移动阈值；
- A–P 共 131 个机器检查和 14 个全局否决项；
- 缺失指标、错误类型、NaN/Infinity、失败阈值全部 fail closed；
- 每 Gate 的 commit/model/config/dataset/container/dependency/seed/command/exit-code 溯源；
- JSON/Markdown/raw-log 文件存在性、根目录约束和 SHA-256 校验；
- freeze、报告、SBOM、SHA256SUMS 与唯一 release ZIP 完整性检查；
- 原子生成最终状态、矩阵和证据索引，默认拒绝覆盖旧 final。

## 当前真实 Ackermann 基线

当前无头 Gazebo 开发基线已在完整 `200 m × 100 m` 地图的 108 m² 代表区完成全部 15 个 Ackermann Coverage 组件。运行时参数实测为物理车宽 `1.32 m`、规划间距 `1.12 m`；brush-swept coverage `1.0`、repeat coverage `0.1365`、直线度 P95 `0.0178 m`、横向误差 P95 `0.0614 m`、collision `0`、keepout violation `0`，但全任务效率仅 `267.4 m²/h`，所以总判定仍为 FAIL。

10,440 m² 长直道候选现使用 skip-lane 顺序、连续简单 Dubins 跟踪、目标清扫带刷盘关闭引导重叠和 `0.20 m/s` 绝对曲率限速；限时 Gazebo 诊断已连续跨过多个 U 形连接器。双天线 GNSS 航向融合也使若干单次诊断的 XY RMSE/P95 低于 `0.05 m`，但同步配对样本不足，且没有整场与 30-seed 报告，不能计作正式 B/C/D 证据。未知栅格 frontier 链已在 40 m × 20 m 烟测中完成 995.34 m² 建图、地图与位姿图保存、硬重启、加载重定位和 5 航点导航，两阶段 TF 断裂均为 0；该证据明确标记 `formal_scope=false`。全范围开发链现使用独立长前视控制、物理可达转弯、60 s 目标看门狗、180 s 失败记忆、边界耗尽恢复，以及仅排序在线 frontier 的边界/激光量程推导蛇形偏置。垂直换带在在线净空允许时跳过倒车，优先执行按曲率边界拆分的解析 forward Dubins；否则最多两次碰撞检查 BackUp，再降级到一次 Smac Hybrid 规划和显式 cusp 分段。车辆保持前轮转向、后轮驱动，轮胎圆柱轴、关节轴与 ODE 摩擦方向已统一为 Gazebo 官方参考语义；刚性刷盘碰撞体改为真实轮毂高度，校园铺装带改为单一地面接触平面上的视觉材质层，不再形成低于二维激光视线、却连续分割 20,000 m² 世界的不可规划物理墙。固定提交首轮正式候选运行到 9,562 m²、扫描目标索引 7 后，在空旷北部栅格边缘持续选择同一远端原始前沿对应的 0.8–2 m Ackermann 近端圆弧，约 17 分钟无有效地图增长，因此失效关闭并停止，不计作正式 PASS。根因修复为代价图拒绝/导航失败时同时冷却原始栅格前沿与实际下发端点，并新增 `raw_frontier_exclusion_count` 审计字段及回归测试；仍须从新提交重跑 20,000 m² 全程及后续保存、硬重启、加载重定位、多航点正式链，因此 B 门继续失效关闭。完整感知、Tracking、Spot Cleaning 与 re-observation 继续受下述分类器停止条件阻塞。

## 当前近距分类硬边界

[CRCRV11](close-range-classifier-contract-recovery-v11.md) 已完成协议允许的 R1/R2/R3 三条路线并触发停止条件 B。C11 虽将 unique background tight crop 从 9 扩展到 6,576，R1 background specificity 也恢复为 `1.0`，但最佳正式 candidate macro-F1 仍只有 `0.6311`。因此不得继续 R4/R5、搜索新 detector、读取 sealed 数据调参或降低 E 门；ActionVerifier 之后的感知产品链保持 dependency-blocked。

## 当前阻塞顺序

1. 近距四分类已按 CRCRV11 耗尽 R1/R2/R3 并失败；在新的、明确解除 V11 路线限制且不污染 sealed 数据的产品方案获批前，E 门是硬阻塞。
2. 在边界锚点蛇形 frontier 默认链上完成 7,200 s / 20,000 m² 建图闭环，再完成 B/C/D：≥30 navigation seeds、95% brush coverage、零碰撞/keepout 和 ≥3500 m²/h 全耗时效率。
3. 串联 E–I，完成动态插入/移除、Tracking/Map、≥30 Spot Cleaning seeds 和 camera-backed Post-Clean。
4. 完成 J/K：固定交互/LLM 集与完整 pipeline 的 10 Hz、10 min、P95/drop/资源门。
5. Freeze 后完成 2 h soak、全 fault matrix、≥5 MCAP replay；任何运行时修改都使 freeze 失效。
6. 仅在开发门全过后原子访问一次 SEALED_FINAL，随后生成 release、演练 rollback、完成供应链与比赛映射。

## 权威入口

- 固定自然语言标准：[product-acceptance-spec-v1.md](product-acceptance-spec-v1.md)
- 机器合同：[product_acceptance_v1.json](../config/product_acceptance_v1.json)
- 最终裁决器：[product_acceptance.py](../scripts/product_acceptance.py)
- 系统规范：[PROJECT_SPEC.md](../PROJECT_SPEC.md)
- 门禁导航：[STAGE_GATES.md](../STAGE_GATES.md)
- 开发交付：[development-workflow.md](development-workflow.md)

在 `FINAL_ACCEPTANCE_STATUS.json` 真实生成且所有必需字段为 true 前，不得使用“仿真产品级完成”表述。
