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

仓库已有 Ackermann 模型/控制、Nav2 与 Coverage profile、生产感知状态机、DynamicTrashMap、Spot Cleaning/Post-Clean 组件、密封测试和发布工具的代码基线。当前尚无一套与本合同绑定的完整 A–P 正式证据，因此：

```text
SIMULATION_PRODUCT_COMPLETE=false
PRODUCT_INTEGRATION_READY=false
PRODUCT_FIELD_READY=false
```

主要缺口是产品近距四分类、20,000 m² 正式范围建图闭环、30-seed 综合链、3500 m²/h 实测效率、完整 10 Hz/10 min 性能、2 h soak、故障矩阵、5-bag replay、一次性 sealed final 和最终 release/rollback。详情见 [当前状态](docs/current-status.md)。

近距分类已完成协议限定的 [CRCRV11 R1/R2/R3](docs/close-range-classifier-contract-recovery-v11.md)，但三条路线全部失败并触发停止条件 B；sealed 数据保持未读，禁止以增加 R4/R5、重开 detector 搜索或降低门槛绕过该阻塞。强制要求的紧凑最终状态、blocker、model registry、release manifest、evidence index 与报告保存在 [`docs/evidence/crcrv11`](docs/evidence/crcrv11/PERCEPTION_CRCRV11_EVIDENCE_INDEX.md)，失败 checkpoint 和训练流水账不进入当前仓库。

当前 20,000 m² 地图上的 108 m² Ackermann 兼容性基线已实测完成全部 15 个覆盖组件，brush-swept coverage `1.0`、repeat `0.1365`、直线度 P95 `0.0178 m`、横向误差 P95 `0.0614 m`，碰撞与禁区侵入均为 `0`；但全耗时效率仅 `267.4 m²/h`，该单区运行仍为 FAIL。10,440 m² 长直道候选已经能够以 skip-lane 路由和显式低速曲率控制连续跨过多个 Ackermann U 形连接器，双天线 GNSS 航向也显著降低了长直道航向漂移；这些仍只是限时诊断，未完成整场、30-seed、充分同步的定位评分或 `3500 m²/h` 正式效率门，不能替代 B/C/D 证据。

仓库现有一键式未知栅格 frontier 连续建图→保存地图/位姿图→硬停止→全新 Gazebo/定位/Nav2 进程→加载重定位→多航点导航入口。40 m × 20 m 烟测已跑通整条接线并保持两阶段 TF 断裂为 0，但 `formal_scope=false`。Ackermann frontier 已加入独立长前视控制、物理可达转弯、60 s 目标看门狗、180 s 失败记忆、边界耗尽恢复，以及仅排序在线 occupancy-grid frontier 的边界/激光量程推导蛇形偏置。垂直换带在在线净空允许时直接使用解析 forward Dubins，并在曲率边界拆成三个 forward-only 原语；否则最多两次碰撞检查 BackUp，再降级到一次 Smac Hybrid 规划和显式前进/倒车 cusp 分段。车辆轮轴与轮胎接触坐标已按 Gazebo 官方 Ackermann 参考语义统一，同时保持目标实体车的前轮转向、后轮驱动；校园世界中低于二维激光视线却连续分割全图的刚性路缘已改为单接触平面上的无碰撞铺装材质层，刷毛视觉与刚性轮毂碰撞语义也已分离。开发复测中后驱车辆 8 秒航向响应与命令约 1:1，Nav2 三段 Dubins 换带和后续在线 frontier 均连续到达；但这些诊断不替代 7,200 s / 20,000 m²、保存、硬重启、加载重定位和多航点正式链，Mapping Gate 继续失效关闭。完整感知、Tracking、Spot Cleaning 与 re-observation 流水线同样因近距分类停止条件 B 未执行。

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
- [Ackermann 车辆模型](docs/ackermann-vehicle-model.md)
- [Ackermann 导航](docs/ackermann-navigation.md)
- [操作指南](docs/operator-guide.md)
- [回滚](docs/rollback.md)
- [许可与第三方资产](MODEL_AND_ASSET_LICENSES.md)

## 许可证

项目代码与第三方组件边界见 [LICENSE.md](LICENSE.md) 和 [MODEL_AND_ASSET_LICENSES.md](MODEL_AND_ASSET_LICENSES.md)。
