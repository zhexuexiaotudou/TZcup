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

主要缺口是合格且可冻结的产品近距四分类/Area 模型、完整感知清扫链的真实 ROS/Gazebo 集成验证、20,000 m² 正式范围建图闭环、30-seed 综合链、3500 m²/h 实测效率、完整 10 Hz/10 min 性能、2 h soak、故障矩阵、5-bag replay、一次性 sealed final 和最终 release/rollback。详情见 [当前状态](docs/current-status.md)，运行时不变量见 [产品运行时基础架构](docs/product-runtime-architecture.md)。

近距分类已完成协议限定的 [CRCRV11 R1/R2/R3](docs/close-range-classifier-contract-recovery-v11.md)，但三条路线全部失败并触发停止条件 B；sealed 数据保持未读，禁止以增加 R4/R5、重开 detector 搜索或降低门槛绕过该阻塞。强制要求的紧凑最终状态、blocker、model registry、release manifest、evidence index 与报告保存在 [`docs/evidence/crcrv11`](docs/evidence/crcrv11/PERCEPTION_CRCRV11_EVIDENCE_INDEX.md)，失败 checkpoint 和训练流水账不进入当前仓库。

当前 20,000 m² 地图上的 108 m² Ackermann 兼容性基线已实测完成全部 15 个覆盖组件，brush-swept coverage `1.0`、repeat `0.1365`、直线度 P95 `0.0178 m`、横向误差 P95 `0.0614 m`，碰撞与禁区侵入均为 `0`；但全耗时效率仅 `267.4 m²/h`，该单区运行仍为 FAIL。10,440 m² 长直道候选已经能够以 skip-lane 路由和显式低速曲率控制连续跨过多个 Ackermann U 形连接器，双天线 GNSS 航向也显著降低了长直道航向漂移；这些仍只是限时诊断，未完成整场、30-seed、充分同步的定位评分或 `3500 m²/h` 正式效率门，不能替代 B/C/D 证据。

仓库现有一键式未知栅格 frontier 连续建图→保存地图/位姿图→硬停止→全新 Gazebo/定位/Nav2 进程→加载重定位→多航点导航入口。40 m × 20 m 烟测已跑通整条接线并保持两阶段 TF 断裂为 0，但 `formal_scope=false`。Ackermann frontier 已加入独立长前视控制、物理可达转弯、60 s 目标看门狗、180 s 失败记忆、边界耗尽恢复，以及仅排序在线 occupancy-grid frontier 的边界/激光量程推导蛇形偏置；当前 costmap 拒绝只在本轮排序中排除，不再伪装成已下发导航失败并占用 180 s 失败 TTL。车辆轮轴与轮胎接触坐标已按 Gazebo 官方 Ackermann 参考语义统一，校园世界的低矮刚性路缘也已改为单接触平面上的无碰撞铺装材质层。当前固定候选在 7,200 s 内完成 15,349.33 m² 已知区域、687 个成功目标和 11/12 个扫描锚点，仍因未达到 20,000 m² 而失效关闭。地图叠加确认 11.99 m 无回波哨兵曾被 12.0 m SLAM 阈值误写成量程边缘实体墙；随后使用低于哨兵的 11.95 m 阈值虽消除了假墙，却因 Karto 只用阈值内点计算栅格边界而阻止开阔区扩图。产品配置现将无回波值与 11.95 m 栅格化阈值严格对齐：该射线参与边界与自由空间 raytrace，但不会写入占据端点；RTK 全局位姿权威链上仍关闭会生成平行重复边界的 Karto loop closure。探索器冻结每个前沿下发时的原始世界坐标，只在收到新地图后评估面积增量；低于 2 m² 的成功计作低增益，每连续 3 个会向 bounds-derived sweep anchor 生成 8/6/4/3/2/1.5/1 m staging 候选。横向蛇形回程必然穿越已知区域，此时累计 12 个低增益成功只继续请求 staging，不再冷却仍可达的原始前沿；横向 frontier 暂时耗尽时也会先尝试同一条在线 costmap 验证的 staging 走廊，再等待下一次更新。真实导航失败仍保持独立的失败点冷却与碰撞检查 BackUp。垂直换带只有在在线地图包络和车体横向位置都到达目标车道时才完成，避免 LiDAR 先看到建筑另一侧就提前横穿障碍；位于 y 边界的车体目标车道还会内缩一个最小转弯半径，地图包络仍覆盖原始 ±50 m 锚点，从而为下一条横带保留真实 Ackermann 前向圆弧空间；掉头后的水平偏置钳在车体允许边界内，不再向不可达的 y=±50 漂移。水平端点仍允许车体或已知邻域任一到达。只要锚点航向误差大于 0.15 rad，就连续执行单步不超过 0.70 rad 的前向 Ackermann 对准弧；每次 staging 成功后继续重入同一 sweep anchor，直到锚点到达或实时 costmap 不再存在完整净空路径。对准弧受 20 s、最长 8 m 推进受 60 s 看门狗约束，实际运动仍由 Nav2 与 Collision Monitor 检查和执行。上述修复必须经过同世界诊断及全新 7,200 s 正式链验证，Mapping Gate 仍为 FAIL。完整感知、Tracking、Spot Cleaning 与 re-observation 流水线同样受近距分类停止条件 B 阻塞。

建图正式入口现已统一使用产品侧 Gazebo 双 NavSat 原始传感器、RTK 误差模型与 wheel/IMU 融合链，并在 map/save 与 restart/reload 两个阶段分别保存、机器判定 adapter 和融合器的实时订阅图；缺少任一阶段、缺少双天线/GNSS/里程计输入或出现真值订阅都会使建图裁决失败。真值只供独立后评估，不再生成定位观测。旧烟测和 7,200 s 候选都早于本次链路统一，必须从当前提交重跑，不能沿用为产品证据。

costmap 临时排除同时绑定候选与对应地图几何，且由 AST 回归门保证所有调用都提供完整参数，避免只在真实 costmap 拒绝路径才暴露参数错误。

规划中心边界 margin 为 0.80 m，由产品 footprint 半宽 0.66 m、定位 P95 0.05 m 与 0.09 m 仿真/控制余量组成；Nav2 与 Collision Monitor 中的完整 1.32 m footprint 没有缩小。

被建筑阻挡的短距 frontier 投影，以及无路、超时或中止的普通短 frontier，现在会为同一原始前沿排队一次 Nav2 全局路径 fallback：返回路径需与当前位姿和已知侧接近点一致，并按在线 SLAM 栅格与 Nav2 costmap 的较细分辨率加密检查净空；远端路径尚未建图时只截取到第一个不安全姿态之前的连续安全前缀，达到最小前进距离才下发，随后随地图增长重规划，最长前视仍为 30 m。目标与路径前缀按每个规划姿态旋转并检查真实非对称 footprint（前伸 0.82 m、后伸 0.575 m、半宽 0.66 m），四边增加 0.15 m 安全余量，并以半个栅格对角线覆盖相交单元，避免漏掉前角碰撞，也不再用外接圆过度拒绝合法 frontier。原始 SLAM 图对任何 occupied 单元独立否决，但把稀疏射线间的 unknown 交给 Nav2 融合 costmap 裁决；后者对 unknown、占据和膨胀成本全部失效关闭。frontier 行为树还以 1 Hz PipelineSequence 持续按增长中的 SLAM/costmap 重规划，不再让一次性旧路径在新障碍出现后由控制器原地拒绝至 180 s 看门狗。导航看门狗以 0.5 m 位姿进展或 2 m² 新增地图刷新空闲窗口，同时保留三倍绝对硬上限；探索总预算在任何 Nav2 recovery 状态之前判定，内部状态不能绕过全局 fail-closed。该链已有 ROS action 级“首次导航失败→fallback→绕障导航成功”“远端未知→仅推进连续安全前缀”和“恢复中仍执行全局超时”回归，但尚未通过同一校园世界的全新长时诊断和 7,200 s 正式闭环，因此 Mapping Gate 继续为 FAIL。

TF 连续性探针、激光与点云自过滤器，以及安全权威、速度门、产品监督器和两级 actuator timeout guard，统一捕获 ROS 2 launch 的外部关闭信号/失效上下文并使用幂等清理；运行期 context 仍有效时的真实异常继续上抛。建图专用 runner 也显式关闭与产品速度门重复发布的上游 `command_timeout`。每个长驻进程组都必须留下 wrapper 退出码、最终信号阶段和残留扫描的 shutdown 记录；Gazebo 先走 `/server_control stop` 再进入通用信号收口，缺记录、残留或被迫 `SIGKILL` 都会让 Mapping Gate 失败关闭。

水平 sweep 没有安全 frontier 时不再无限等待：连续 5 次后从在线已知自由栅格选择朝 sweep 锚点推进的 30 m 内候选并复用上述全局路径门；某个 route 候选失败后立即冷却并尝试下一个，不等待整段 frontier TTL。连续 30 次仍无安全路线则只尝试碰撞检查倒车，倒车也不可用即失败关闭。运行 evidence 的原子写入已串行化，避免 action/map 回调争用同一临时文件。

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
