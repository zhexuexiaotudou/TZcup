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

10,440 m² 长直道候选现使用 skip-lane 顺序、连续简单 Dubins 跟踪、目标清扫带刷盘关闭引导重叠和 `0.20 m/s` 绝对曲率限速；限时 Gazebo 诊断已连续跨过多个 U 形连接器。双天线 GNSS 航向融合也使若干单次诊断的 XY RMSE/P95 低于 `0.05 m`，但同步配对样本不足，且没有整场与 30-seed 报告，不能计作正式 B/C/D 证据。未知栅格 frontier 链已在 40 m × 20 m 烟测中完成 995.34 m² 建图、地图与位姿图保存、硬重启、加载重定位和 5 航点导航，两阶段 TF 断裂均为 0；该证据明确标记 `formal_scope=false`。全范围链保持真实 Ackermann 前轮转向/后轮驱动、双天线 GNSS + wheel/IMU 定位、物理可达 frontier、失败双中心冷却和无 oracle 控制。当前固定候选在 7,200 s 内达到 15,349.33 m² 已知区域、687 个成功目标、11 个失败目标、39 次碰撞检查倒车恢复和 11/12 个扫描锚点，因未达到 20,000 m² 而诚实终止；后续保存地图用于诊断，不计作正式 PASS。真值叠加确认 11.99 m 无回波哨兵曾被旧 12.0 m 阈值误写成量程边缘伪墙；11.95 m 阈值已消除该结构性伪影。局部图仍出现平行重复边界，因此 RTK 全局位姿权威的产品 profile 关闭 Karto loop closure。运行诊断还确认 Nav2 可连续返回成功而已知面积不增长。探索器冻结下发时的原始前沿世界坐标，仅在收到新 OccupancyGrid 后评估面积；低于 2 m² 的成功计作低增益，每连续 3 个向当前 bounds-derived sweep anchor 构造 8/6/4/3/2 m staging 候选，累计 12 个时才长时冷却原始前沿与端点。只要锚点航向误差大于 0.15 rad，就连续生成单步不超过 0.70 rad 的前向 Ackermann 对准弧；对准弧和推进候选都须通过完整路径的实时 costmap 净空检查，对准弧使用 20 s、最长 8 m 推进使用 60 s 看门狗。实际运动与碰撞仍由生产 Nav2、controller 和 Collision Monitor 检查。所有恢复分支通过独立计数和逐目标字段审计。仍须从新提交通过同世界诊断，再重跑完整 7,200 s / 20,000 m²、保存、硬重启、加载重定位和多航点正式链，因此 B 门继续失效关闭。完整感知、Tracking、Spot Cleaning 与 re-observation 继续受下述分类器停止条件阻塞。

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
