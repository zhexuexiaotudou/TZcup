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
| 车辆与仿真 | Ackermann Xacro、Gazebo steering plugin、wheel/steering state、Ackermann EKF、Nav2 与 Coverage profile；产品入口默认 headless、独立 Gazebo Transport 分区和有界 1× 时钟 | 静态/单元合同及一次完整拓扑烟测通过；仍需正式运行矩阵 |
| 清扫机构 | 默认有效刷宽 1.32 m、40 L 箱体几何、真实 footprint 清扫足迹 | 几何合同存在；效率与长期运行未通过 |
| 定位/导航/覆盖 | SLAM/定位、Gazebo 内部 0.80 m 双 NavSat、RTK 误差/延迟模型、wheel/IMU 里程计、world→map 标定、Nav2、未知栅格 frontier 探索、地图保存/硬重启/加载/重定位/多航点导航、keepout、Collision Monitor、skip-lane Ackermann Coverage、repair 与可视化链；产品混合定位器独占 `map→odom` 并发布统一 `/localization/fused_pose`，ROS 图不桥接/订阅 evaluation GT | 双天线基线/位置/航向、唯一发布者、TF 链和短距安全运动已在完整产品拓扑实测通过；40 m × 20 m 整链烟测也已通过，但不能替代 20,000 m²、30-seed、定位精度和 3500 m²/h 正式门 |
| 离散感知 | proposal、近距四分类接口、Tracking、RGB-D 投影、独立 ActionVerifier、最多两次重观察与 DynamicTrashMap；Tracker/Map 均不能自行 CONFIRMED | 当前已知近距分类结果低于 macro-F1 0.98；placeholder manifest 不可激活，真实 ROS/Gazebo 链与 V1 正式证据未建立 |
| Area 感知 | leaf/puddle runtime、训练评估与独立指标代码 | 需在固定 split 与完整产品链重新冻结/验收 |
| Spot/Post-Clean | 产品入口已接真实 keepout/global costmap 全车 footprint、Coverage pause/resume acknowledgement、Nav2 path/approach、刷盘互锁、Pre-Clean、camera-frustum 离散后验与 Area 残余/单次重清 | 纯逻辑门已覆盖；仍缺 ROS build、完整 Gazebo 实链和 ≥30 seeds 零错误清扫正式证据 |
| 交互/安全 | HMI、Speech/任务接口；单一 E-stop 权威上电急停并心跳，产品监督器区分运动故障与清扫降级，速度/点清洁/重观察在权威失联时失效关闭；产品 HMI 不订阅垃圾 GT | 完整拓扑已实测“感知 ERROR、运动健康”可人工解除上电急停；杀死监督节点或双 NavSat 适配器都会重新急停并锁存，进程自动重启且健康恢复后仍不自动解锁，新的人工清除才成功。仍缺固定集 ≥95%、两个模态综合和正式全 fault/30 次延迟报告 |
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

未知栅格 frontier 正式入口已从真值派生 GNSS 模拟器切换到 Gazebo 内部双 NavSat 原始传感器、RTK 误差模型和 wheel/IMU 融合链。map/save/restart/reload 两阶段都必须收到双天线原始观测，并保存 adapter 与融合器的实时订阅图；裁决器要求两个阶段的输入完整且均无 `/ground_truth/*`，不再接受手写 provenance 布尔值。此前烟测和长时结果只作历史诊断，必须从当前链路重跑。

当前无头 Gazebo 开发基线已在完整 `200 m × 100 m` 地图的 108 m² 代表区完成全部 15 个 Ackermann Coverage 组件。运行时参数实测为物理车宽 `1.32 m`、规划间距 `1.12 m`；brush-swept coverage `1.0`、repeat coverage `0.1365`、直线度 P95 `0.0178 m`、横向误差 P95 `0.0614 m`、collision `0`、keepout violation `0`，但全任务效率仅 `267.4 m²/h`，所以总判定仍为 FAIL。

10,440 m² 长直道候选现使用 skip-lane 顺序、连续简单 Dubins 跟踪、目标清扫带刷盘关闭引导重叠和 `0.20 m/s` 绝对曲率限速；限时 Gazebo 诊断已连续跨过多个 U 形连接器。双天线 GNSS 航向融合也使若干单次诊断的 XY RMSE/P95 低于 `0.05 m`，但同步配对样本不足，且没有整场与 30-seed 报告，不能计作正式 B/C/D 证据。未知栅格 frontier 链已在 40 m × 20 m 烟测中完成 995.34 m² 建图、地图与位姿图保存、硬重启、加载重定位和 5 航点导航，两阶段 TF 断裂均为 0；该证据明确标记 `formal_scope=false`。全范围链保持真实 Ackermann 前轮转向/后轮驱动、双天线 GNSS + wheel/IMU 定位、物理可达 frontier、失败双中心冷却和无 oracle 控制。当前固定候选在 7,200 s 内达到 15,349.33 m² 已知区域、687 个成功目标、11 个失败目标、39 次碰撞检查倒车恢复和 11/12 个扫描锚点，因未达到 20,000 m² 而诚实终止；后续保存地图用于诊断，不计作正式 PASS。真值叠加确认 11.99 m 无回波哨兵曾被旧 12.0 m 阈值误写成量程边缘伪墙；11.95 m 阈值已消除该结构性伪影。局部图仍出现平行重复边界，因此 RTK 全局位姿权威的产品 profile 关闭 Karto loop closure。运行诊断还确认 Nav2 可连续返回成功而已知面积不增长。探索器冻结下发时的原始前沿世界坐标，仅在收到新 OccupancyGrid 后评估面积；低于 2 m² 的成功计作低增益，每连续 3 个向当前 bounds-derived sweep anchor 构造 8/6/4/3/2/1.5/1 m staging 候选，累计 12 个时才长时冷却原始前沿与端点。只要锚点航向误差大于 0.15 rad，就连续生成单步不超过 0.70 rad 的前向 Ackermann 对准弧；每次 staging 成功后继续重入同一 sweep anchor，直到锚点到达或实时 costmap 不再存在完整净空路径。对准弧使用 20 s、最长 8 m 推进使用 60 s 看门狗，实际运动与碰撞仍由生产 Nav2、controller 和 Collision Monitor 检查。所有恢复分支通过独立计数和逐目标字段审计。仍须从新提交通过同世界诊断，再重跑完整 7,200 s / 20,000 m²、保存、硬重启、加载重定位和多航点正式链，因此 B 门继续失效关闭。完整感知、Tracking、Spot Cleaning 与 re-observation 继续受下述分类器停止条件阻塞。

短距 frontier 投影被在线 SLAM 栅格或 Nav2 costmap 阻挡，或者普通短 frontier 无路、超时或中止时，当前实现为同一原始前沿排队一次全局路径 fallback，校验规划起终点并按两张在线栅格的较细分辨率加密检查整段净空，再沿验证路线截取最长 30 m 前视点。净空检查按每个规划姿态旋转真实非对称 footprint（前 0.82 m、后 0.575 m、半宽 0.66 m），四边增加 0.15 m 安全余量，并以半个栅格对角线覆盖所有相交单元；此前 0.70 m 圆盘会漏掉前角，而 1.21 m 外接圆又会过度拒绝车尾/侧向仍合法的 frontier。原始 SLAM 栅格承担 occupied 独立否决，稀疏射线间的 unknown 由 Nav2 融合 costmap 继续按 unknown/占据/膨胀成本失效关闭，从而不把原始图孔洞误当实体障碍。frontier 行为树使用 1 Hz PipelineSequence 持续对增长地图重规划，修复一次性路径在后续暴露障碍后长期被控制器拒绝的问题。ROS action 级回归覆盖贴墙路线拒绝、方向相关足迹、首次导航中止、fallback 入队、绕障规划和二次导航成功，但不能替代校园世界实跑；Mapping Gate 仍需同世界诊断与完整正式闭环。

水平 sweep 的安全 frontier 连续 5 次不可用后，会从在线已知自由栅格选择 30 m 内、朝当前 sweep 锚点推进的 staging 候选并复用全局路径门；route 失败时冷却该候选并立即尝试下一个，不等待普通 frontier TTL。连续 30 次仍无安全路线时仅允许碰撞检查倒车，倒车也不可用则明确失败，不再永久等待。探索报告的原子替换由进程内锁串行化，ROS 多回调回归不会争用同一临时文件。

costmap 临时排除绑定候选与当前地图几何，并由 AST 回归门约束全部调用签名，避免该真实运行分支再次出现参数缺失。

## 当前近距分类硬边界

[CRCRV11](close-range-classifier-contract-recovery-v11.md) 已完成协议允许的 R1/R2/R3 三条路线并触发停止条件 B。C11 虽将 unique background tight crop 从 9 扩展到 6,576，R1 background specificity 也恢复为 `1.0`，但最佳正式 candidate macro-F1 仍只有 `0.6311`。因此不得继续 R4/R5、搜索新 detector、读取 sealed 数据调参或降低 E 门；ActionVerifier/重观察/清洁闭环代码可继续验证，但产品模型激活与 E–I 正式运行保持 dependency-blocked。

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
