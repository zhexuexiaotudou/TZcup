# 当前项目状态

本页只保留当前有效结论，不记录尝试轮次、日期流水或中间产物。

## 最终状态

```text
SIMULATION_PRODUCT_COMPLETE=false
PRODUCT_X86_PERCEPTION_READY=false
PRODUCT_INTEGRATION_READY=false
PRODUCT_FIELD_READY=false
```

Journey 6 PC-first 状态同样保持失效关闭：

```text
J6_PC_FUNCTIONAL_PASS=false
J6_X86_SIMULATION_READY=false
J6_LOOPBACK_HIL_READY=false
J6_DEPLOYMENT_BUNDLE_READY=false
```

目标家族已固定为 `journey6`，但真实板卡 SKU 与 `march` 仍为 `auto`。当前本机未发现经过身份、版本和哈希验证的官方 Journey 6 OpenExplorer/HUCP SDK；历史 RDK S100/S100P 包明确拒绝复用。D1 `best.pt` 已按固定 SHA 以 non-root、只读输入、断网容器真实加载，并由官方 YOLOv9 E1 路线导出 canonical ONNX：静态 `[1,3,640,640]`、opset 17、IR 8、FP32、无自定义算子、无内嵌 NMS，精确十类从 checkpoint 读取。100 张 TRAIN 图的严格 PT/ONNX parity 历史实跑失败（主输出最大框误差 `1.6038 px`、分类分数误差 `0.003521`）；本轮 native PT 在同一 410 张 TRAIN 图、81 个标注上重新实跑，三类 TP 仍均为 `0`，proposal FP/frame 为 `2.0122`、negative FP/frame 为 `2.0152`，与 canonical ONNX 语义结果一致，主失败归因为 domain/semantic mismatch。发布者固定 revision 没有模型卡引用的 `road.jpg` 或其他 sample，固定集也没有 10 个明显大目标（最大短边 21 px），所以 A0 最终归因文件已生成但 `A0_COMPLETE=false`；D1 仍不得激活、调参或训练，15 个 moving Gazebo mission 与 Spot/Post-Clean 未启动。D1 second-pass 的 fail-closed provider、Tracker→ActionVerifier→DynamicTrashMap 合同与恢复的 development-only Area ONNX 已落地；Area 负样本回放暴露 puddle 全帧误报，正式 Area gate保持 false。1800.049 秒 PC_ONNX/Jazzy synthetic+D2 诊断真实执行了 ROS、同步、命令权威与 network fault，但因非 Gazebo、非 Humble、required-D1 mismatch 和 model contract false，transport/algorithm/emulation/official HIL 四状态均为 false；另有 30.011 秒 Humble split smoke，同样不满足正式门。校准盘点仅 `471` 个 TRAIN RGB、`0` ROI；source bundle、官方 SDK/x86、HBM、物理板端和产品状态均保持 false。所有板端 FPS、BPU/CPU/DDR、温度、功耗、HBM 与网络 HIL 时延、30-seed 字段保持 `null/not_run`。

d6 YOLOX-Tiny COCO ONNX 已作为首个新增 reference 在相同 410/81 TRAIN 上真实运行。`0.001–0.5` 全阈值 proposal recall 均为 `0`，阈值 `0.5` 的 FP/frame 仍为 `1.4439`；COCO 缺目标 can/paper 语义，semantic 指标强制 `not_applicable`，因此 d6 不能成为现存功能或产品候选。当前 fixed 数据没有独立 negative-only frame，partial HOLDOUT 又缺 plastic_bottle，完整 A4 与 background specificity 仍保持 blocked。

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

[CRCRV11](close-range-classifier-contract-recovery-v11.md) 已完成协议允许的 R1/R2/R3 三条路线并触发停止条件 B。C11 虽将 unique background tight crop 从 9 扩展到 6,576，R1 background specificity 也恢复为 `1.0`，但最佳正式 candidate macro-F1 仍只有 `0.6311`。该失败事实、R1/R2/R3 路线耗尽和 sealed 禁区保持不变；新的 `EMFJ6V3` 已完成有上限、可审计的现存模型发现并冻结 `6 detector / 6 classifier / 3 Area` 清单。`EMF_EXISTING_MODEL_INVENTORY_READY=true` 只证明发现和 source artifact intake 闭合；TACO 类序绑定、全部固定开发集筛选、非训练调整、functional/product 候选及训练授权仍为 false。

## 当前阻塞顺序

1. 近距四分类已按 CRCRV11 耗尽 R1/R2/R3 并失败；`EMFJ6V3` 的 D1 主失败已归因到域/语义不匹配，候选清单已冻结，但 A0 的发布者样图与明显大目标人审条件仍失败关闭，固定开发集 screening 和非训练调整尚未完成，因此 E 门继续硬阻塞，训练也继续禁止。
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
