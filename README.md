# TZcup 智慧环卫无人清扫车

TZcup 是基于 ROS 2 Jazzy 与 Gazebo Harmonic 的无人清扫车产品基线，覆盖 Ackermann 车辆与传感器数字孪生、定位建图、Nav2、覆盖清扫、离散/区域感知、动态垃圾地图、定点清扫、安全、人机交互和可审计发布。

仓库采用失效关闭的产品口径：代码存在、单元测试通过或演示可运行，都不等于产品验收通过。只有固定 V1 合同的 A–P 主门、全局否决项、正式证据哈希和发布包全部通过，才允许生成：

```text
SIMULATION_PRODUCT_COMPLETE=true
```

`PRODUCT_INTEGRATION_READY` 还要求目标计算平台部署与板端性能；`PRODUCT_FIELD_READY` 还要求真实车辆、传感器和道路的独立验收。

## 产品边界

- 正式车辆默认使用 Ackermann 前轮转向、后轮驱动；`skid_steer_legacy` 只用于显式历史回归。
- 有效清扫宽度默认 `1.32 m`，垃圾箱几何容量 `40 L`。
- 生产任务的 Target List 与 `DynamicTrashMap` 空启动；垃圾坐标、类别、instance ID、出现时间和真实清扫结果不得预载。
- Gazebo semantic/instance/world truth 只允许进入独立 evaluator，禁止进入生产感知、调度、导航、控制和安全链。
- `CANDIDATE` 不可行动；只有 Close-Range Classifier 经独立 `ActionVerifier` 确认后才能进入 `CONFIRMED` 和 Scheduler。
- Safety Plane 独立于 Autonomy Plane 与 Cleaning Intelligence Plane，后两者不能绕过 E-stop、Collision Monitor、keepout 或边界保护。

完整架构见 [PROJECT_SPEC.md](PROJECT_SPEC.md)，固定门槛见 [产品验收规范 V1](docs/product-acceptance-spec-v1.md) 与 [STAGE_GATES.md](STAGE_GATES.md)。

## 当前结论

底盘最终速度门拒绝超时或非有限数命令；解除 E-stop 后必须收到新的运动命令，不能重放停机前或停机期间缓存的速度。非零命令沿 `/cmd_vel_gate → velocity_gate → /cmd_vel_safe → actuator_command_gate → /cmd_vel` 串联，另一个独立 sentinel 只允许在最终命令中断 `0.080 s` 后补发零速；速度门以 20 ms 周期持续刷新安全命令，安全权威、速度门、串联门和 sentinel 均快速重启，从而避免任一单进程崩溃留下持续非零命令。正式 E-stop 探针和所有聚合器统一执行 30 次试验、P95 不超过 `0.200 s` 的产品硬门，并同时保留 median、P95 与 max 原始统计。

仓库已有 Ackermann 模型/控制、Nav2 与 Coverage profile，以及上电急停/权威心跳安全层、区分运动故障与清扫降级的产品监督器、GT 隔离的产品启动拓扑、统一全局位姿契约、生产感知、独立 ActionVerifier、最多两次主动重观察、DynamicTrashMap、真实 Coverage/Nav2 点清洁、受控滚刷所有权和 camera-backed Post-Clean 的失效关闭代码链。Coverage 的漏扫判定、补扫规划、产品任务终态和运行效率现统一由 `/localization/fused_pose` 积累的刷盘轨迹驱动；Gazebo 真值只在显式评估模式结束后生成独立评分，不能改变控制。Ackermann 五类任务速度现在是启动时失效关闭校验并实际下发的运行合同，1.32 m 刷宽的直线清扫链已贯通到 1.0 m/s 控制能力，为全耗时 3500 m²/h 留出物理余量。最后一条刷道后还必须等待重观察与点清扫队列保持新鲜并稳定排空，期间暂停、移动、清扫和复检仍计入任务总时间；状态丢失或超时会失败关闭。产品仿真以 Gazebo 内部双 NavSat 传感器形成 0.80 m 基线，经标准 `gps_msgs/GPSFix` 桥接、确定性 RTK 误差模型和 world→map 标定后，与 wheel/IMU 里程计共同产生唯一 `/localization/fused_pose` 和 `map→odom`；产品 ROS 图不订阅任何 GT 位姿。产品入口默认无 GUI、以 ROS domain 派生独立 Gazebo Transport 分区，并用有界 1× 物理时钟阻止并发试验串场。仓库内模型清单仍是不可激活的 placeholder，且当前尚无一套与本合同绑定的完整 A–P 正式证据，因此：

```text
SIMULATION_PRODUCT_COMPLETE=false
PRODUCT_INTEGRATION_READY=false
PRODUCT_FIELD_READY=false
```

目标计算平台已收敛为地平线 Journey 6，当前 SKU 与 `march` 均保持 `auto`。仓库新增 PC 先行的预训练模型、NV12、严格 provider、分离式 HIL 与板到即部署合同，并在 CI 中分别构建 Jazzy transport/gateway 与 Humble PC-ONNX algorithm-host 镜像；但真实 detector/classifier 激活与评测、官方 J6 OpenExplorer x86 仿真、30 分钟 loopback HIL 和物理板端证据尚未通过，因此所有 `J6_*_READY/PASS` 状态仍失效关闭。RDK S100/S100P 产物不得作为 Journey 6 证据。架构与到板流程见 [Journey 6 目标架构](docs/journey6-target-architecture.md) 和 [板卡到货手册](docs/journey6-board-arrival-runbook.md)。

当前已按 `EMFJ6V3` 完成有界现存模型发现并冻结 `6 detector / 6 classifier / 3 Area` 清单，全部来源 revision 与登记 artifact SHA 可审计。库存 READY 只表示发现阶段闭合：TACO 类序绑定仍隔离，pLitter/COCO detector 仅作 proposal/reference，四个新 classifier 均没有 background/unknown，eWaSR/SegFormer 只有域错位或 generic Area 代理语义；固定开发集筛选、非训练调整、功能/产品候选与训练授权仍全部为 false，sealed 数据继续禁止访问。

D1 native PT 已在同一 410 张 TRAIN 图、81 个目标上重新实跑，三类 TP 仍全为 0，negative FP/frame 仍为 `2.0152`，与历史 canonical ONNX 语义结果一致，因此主失败归因为 domain/semantic mismatch，禁止继续围绕 D1 调参或训练。发布者固定 revision 未提供模型卡所引用的样图，且当前固定集最大目标短边只有 21 px，A0 的 source-domain sanity 与 10 个明显目标人审条件仍失败关闭。

d6 官方 YOLOX-Tiny COCO ONNX 已按固定 SHA 在相同 410/81 TRAIN 上完成 proposal-only CPU 实跑：阈值 `0.001–0.5` 的 class-agnostic recall 始终为 `0`，阈值 `0.5` 仍为 `1.4439 FP/frame`。COCO 只有 generic `bottle` 且没有目标 `metal_can/paper_litter`，所以 semantic 指标保持 `not_applicable`，d6 明确拒绝为现存产品候选；原始推理外置锁定，独立 negative-only 与完整 HOLDOUT 仍缺失，不能把本次诊断写成完整 A4 screening。

C1 WasteWise 静态 ONNX 也已按固定 SHA 完成 183 个 development-only GT-crop native smoke：四类 macro-F1 `0.1369`、background specificity `0.6765`，plastic/metal/paper 三类召回均为 `0`。该结果只证明直接 mapped-argmax 使用失败；GT crop 不是 proposal-crop A4，且尚未完成目标概率质量/unknown rejection 的一次有界非训练调整，所以 C1 保持 adjustment pending，不能提前启动训练。

C4 SigLIP2 safetensors 已在 non-root、断网、只读挂载、`trust_remote_code=false` 的固定 Transformers 4.50.2/CUDA 容器完成同一 183 crop native smoke；processor 明确锁定 slow bilinear、RGB、`/255`、mean/std 0.5 和 channels-last。macro-F1 `0.1911`、background specificity `0.9706`，三目标召回仍全部为 `0`，因此 direct-use 同样失败并进入非训练调整待办；尚无 canonical ONNX/parity，不得声明 Journey 6 可用。

Journey 6 校准数据与源码部署包同样失效关闭：当前只读盘点两个明确 TRAIN 根得到 `471` 个 RGB PNG 候选和 `0` 个 ROI/crop，尚无逐文件 SHA 与分层元数据，因此 `J6_CALIBRATION_PACK_READY=false`。reference-only source bundle 已锁定 D1 E1 canonical ONNX、development-only Area ONNX、C++ graph-external 后处理和真实 TRAIN golden tensor lock；但模型选择/发布许可、正式校准、nash profile 与官方工具链仍未齐备，因此 `J6_SOURCE_DEPLOYMENT_BUNDLE_READY=false`。许可审计文件缺失本身也会显式保持 `model_license_not_release_clear`，不会因干净 CI 环境缺少本地 `.workspace` 证据而漏报。`G5_V2`、`SEALED_FINAL`、`DEV_VAL` 始终禁止进入校准链，详见 [Journey 6 校准与源码部署包](docs/journey6-calibration-source-bundle.md)。

主要缺口是合格且可冻结的产品近距四分类/Area 模型、完整感知清扫链的真实 ROS/Gazebo 集成验证、20,000 m² 正式范围建图闭环、30-seed 综合链、3500 m²/h 实测效率、完整 10 Hz/10 min 性能、2 h soak、故障矩阵、5-bag replay、一次性 sealed final 和最终 release/rollback。详情见 [当前状态](docs/current-status.md)，运行时不变量见 [产品运行时基础架构](docs/product-runtime-architecture.md)。

近距分类已完成协议限定的 [CRCRV11 R1/R2/R3](docs/close-range-classifier-contract-recovery-v11.md)，但三条路线全部失败并触发停止条件 B；sealed 数据保持未读，禁止以增加 R4/R5、重开 detector 搜索或降低门槛绕过该阻塞。强制要求的紧凑最终状态、blocker、model registry、release manifest、evidence index 与报告保存在 [`docs/evidence/crcrv11`](docs/evidence/crcrv11/PERCEPTION_CRCRV11_EVIDENCE_INDEX.md)，失败 checkpoint 和训练流水账不进入当前仓库。

当前 20,000 m² 地图上的 108 m² Ackermann 兼容性基线已实测完成全部 15 个覆盖组件，brush-swept coverage `1.0`、repeat `0.1365`、直线度 P95 `0.0178 m`、横向误差 P95 `0.0614 m`，碰撞与禁区侵入均为 `0`；但全耗时效率仅 `267.4 m²/h`，该单区运行仍为 FAIL。10,440 m² 长直道候选已经能够以 skip-lane 路由和显式低速曲率控制连续跨过多个 Ackermann U 形连接器，双天线 GNSS 航向也显著降低了长直道航向漂移；这些仍只是限时诊断，未完成整场、30-seed、充分同步的定位评分或 `3500 m²/h` 正式效率门，不能替代 B/C/D 证据。

产品建图入口执行未知栅格连续探索、地图与位姿图保存、全部进程硬重启、保存地图加载、重定位和多航点 Nav2 导航。seed 2028 的无诊断覆盖建图已真实达到 `20,011 m²`，保存栅格复算为 `20,017.68 m²`；按 LiDAR 扫描平面可观测几何评估，边界 RMSE `0.132 m`、可见真值边界召回 `0.9598`、ghost ratio `0.00947`。该次运行在一期结束时暴露了僵尸进程误判并在旧 runner 中提前终止，因此不是正式 PASS。修复后的保存地图重载诊断已完成 3/3 航点、收敛后 TF 跳变和断裂均为 `0`、5/5 长驻组干净退出；正式 Mapping Gate 仍须由同一干净提交重新跑完整两阶段后裁决。

建图与重载均使用 Gazebo 双 NavSat 原始传感器、确定性 RTK 误差模型和 wheel/IMU 链，不向控制图注入真值。建图阶段由 SLAM 唯一发布 `map→odom`，融合器提供 `odom→wheel_odom`；重载阶段 AMCL 保留地图重定位输出但关闭 TF 广播，由经延迟补偿与圆周航向创新平滑的融合器唯一发布稳定 `map→odom`，局部 EKF 发布 `odom→base_footprint`。runner 实采目标 TF 边、互斥定位节点、融合器父子帧参数及实时订阅图；缺文件、错误所有权、真值订阅或 AMCL 错误广播都会失败关闭。

costmap 临时排除同时绑定候选与对应地图几何，且由 AST 回归门保证所有调用都提供完整参数，避免只在真实 costmap 拒绝路径才暴露参数错误。

规划中心边界 margin 为 0.80 m，由产品 footprint 半宽 0.66 m、定位 P95 0.05 m 与 0.09 m 仿真/控制余量组成；Nav2 与 Collision Monitor 中的完整 1.32 m footprint 没有缩小。

被建筑阻挡的短距 frontier 投影，以及无路、超时或中止的普通短 frontier，现在会为同一原始前沿排队一次 Nav2 全局路径 fallback：返回路径需与当前位姿和已知侧接近点一致，并按在线 SLAM 栅格与 Nav2 costmap 的较细分辨率加密检查净空；远端路径尚未建图时只截取到第一个不安全姿态之前的连续安全前缀，达到最小前进距离才下发，随后随地图增长重规划，最长前视仍为 30 m。目标与路径前缀按每个规划姿态旋转并检查真实非对称 footprint（前伸 0.82 m、后伸 0.575 m、半宽 0.66 m），四边增加 0.15 m 安全余量，并以半个栅格对角线覆盖相交单元，避免漏掉前角碰撞，也不再用外接圆过度拒绝合法 frontier。原始 SLAM 图对任何 occupied 单元独立否决，但把稀疏射线间的 unknown 交给 Nav2 融合 costmap 裁决；后者对 unknown、占据和膨胀成本全部失效关闭。frontier 行为树还以 1 Hz PipelineSequence 持续按增长中的 SLAM/costmap 重规划，不再让一次性旧路径在新障碍出现后由控制器原地拒绝至 180 s 看门狗。导航看门狗以 0.5 m 位姿进展或 2 m² 新增地图刷新空闲窗口，同时保留三倍绝对硬上限；探索总预算在任何 Nav2 recovery 状态之前判定，内部状态不能绕过全局 fail-closed。该链已有 ROS action 级“首次导航失败→fallback→绕障导航成功”“远端未知→仅推进连续安全前缀”和“恢复中仍执行全局超时”回归，但尚未通过同一校园世界的全新长时诊断和 7,200 s 正式闭环，因此 Mapping Gate 继续为 FAIL。

TF 连续性探针在重定位收敛窗口后同时判定时间断裂、时间戳回退和空间跳变，并记录前 20 个跳变事件用于根因定位。进程存活判定忽略仅待父进程回收的 zombie，但任何可执行残留、缺失 shutdown 记录或 `SIGKILL` 仍使门禁失败；Gazebo 必须先接受 `/server_control stop`，随后所有长驻组以可审计退出码和信号阶段收口。

水平 sweep 没有安全 frontier 时不再无限等待：连续 5 次后从在线已知自由栅格选择朝 sweep 锚点推进的 30 m 内候选并复用上述全局路径门；局部 frontier 失败后若其直接 backoff endpoint 不可用于全局规划，会立即升级为朝 sweep anchor 的已知自由区 route recovery。局部 frontier、staging 或 alignment 弧一旦失败且仍存在被排除的原始候选，也必须先全局绕障，不能在同一姿态再次武断重排 staging。某个 route 候选失败后立即冷却并尝试下一个，不等待整段 frontier TTL。连续 30 次仍无安全路线则只尝试碰撞检查倒车，倒车也不可用即失败关闭。运行 evidence 的原子写入已串行化，避免 action/map 回调争用同一临时文件。

## 快速启动

Windows + WSLg 的产品默认演示：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_visual_demo.ps1
```

只打开 Gazebo 原生任务控制：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_gazebo_cleaning_demo.ps1
```

两个入口默认均为 `DriveModel=ackermann`、`CoverageProfile=ackermann`。环境安装、构建和显式 legacy 回归见 [README_FIRST.md](README_FIRST.md)。
Gazebo 中橙色外框是任务范围，青绿色区域是实际可清扫范围；规划、实际轨迹、转场和 brush-swept area 分层显示。

## 开发验证

快速仓库门：

```powershell
py -3 scripts/ci_fast.py
```

验证固定验收合同：

```powershell
py -3 scripts/product_acceptance.py validate-contract
```

生成待填充的正式证据 manifest：

```powershell
py -3 scripts/product_acceptance.py template --output C:\tzcup-evidence\acceptance_evidence_manifest.json
```

最终裁决只读取正式证据，不运行仿真、不补写指标：

```powershell
py -3 scripts/product_acceptance.py evaluate `
  --evidence-manifest C:\tzcup-evidence\acceptance_evidence_manifest.json `
  --evidence-root C:\tzcup-evidence `
  --output-dir C:\tzcup-evidence\final
```

缺少任一指标、溯源字段、原始日志、报告、SHA-256、release 文件或否决项不为安全值时，命令以非零退出并生成 `SIMULATION_PRODUCT_COMPLETE=false`。

## 仓库结构

| 路径 | 作用 |
|---|---|
| `starter_ws/src/` | 自研 ROS 2 功能包 |
| `starter_ws/src/sanitation_product_bringup/` | 严格安全、GT 隔离的完整产品仿真入口 |
| `reference_vision/` | 只用于开发参考的第三方感知适配，禁止进入产品控制 |
| `config/product_acceptance_v1.json` | 固定、机器可判定的 A–P 验收合同 |
| `scripts/product_acceptance.py` | 失效关闭的最终裁决器 |
| `scripts/` | 构建、运行、评测、冻结和发布工具 |
| `docs/` | 稳定设计、操作、验收口径和当前状态 |

原始数据、bag、逐帧 trace、缓存和运行中间产物不进入 Git；正式结果只保留紧凑报告、日志索引和哈希清单。规则见 [证据策略](docs/artifact-policy.md) 与 [开发工作流](docs/development-workflow.md)。

## 文档入口

- [首次安装与启动](README_FIRST.md)
- [系统技术规范](PROJECT_SPEC.md)
- [产品验收门](STAGE_GATES.md)
- [当前状态](docs/current-status.md)
- [产品运行时基础架构](docs/product-runtime-architecture.md)
- [Ackermann 车辆模型](docs/ackermann-vehicle-model.md)
- [Ackermann 导航](docs/ackermann-navigation.md)
- [操作指南](docs/operator-guide.md)
- [回滚](docs/rollback.md)
- [许可与第三方资产](MODEL_AND_ASSET_LICENSES.md)

## 许可证

项目代码与第三方组件边界见 [LICENSE.md](LICENSE.md) 和 [MODEL_AND_ASSET_LICENSES.md](MODEL_AND_ASSET_LICENSES.md)。
