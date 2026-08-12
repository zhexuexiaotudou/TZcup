# TZcup 智慧环卫无人清扫车

TZcup 是一个基于 ROS 2 与 Gazebo 的智慧环卫无人清扫车工程，覆盖车辆与园区数字孪生、自主定位导航、全覆盖清扫、垃圾感知、定点清扫、安全控制、人机界面和可审计验收。

项目面向 Ubuntu 24.04、ROS 2 Jazzy 和 Gazebo Harmonic，支持在 Windows WSLg 中运行可视化演示，也支持通过 Docker 执行无头验证。

## 核心能力

- 四轮差速清扫车、刷盘、传感器与结构化园区场景建模；
- SLAM、融合定位、Nav2 导航、keepout 与安全速度控制；
- 基于 OpenNav Coverage / Fields2Cover 的全覆盖路径规划与补扫；
- RGB-D 垃圾检测、跟踪、地图融合和定点清扫任务调度；
- Gazebo、RViz 与浏览器看板组成的调试和监督界面；
- 多场景回归、证据清单、模型注册、回放与发布检查工具。

## 系统边界

正式任务启动时只预载道路、可清扫区域、静态障碍和安全约束，不预载垃圾坐标。垃圾目标必须由车载 RGB-D 在车辆运动中发现，经多帧确认、时间戳 TF 投影和动态地图融合后，才能进入清扫调度；Gazebo 真值仅供独立评测使用。

当前仿真、导航、覆盖清扫、安全链和可视化演示可运行。学习感知仍处于 fail-closed 验证阶段，尚未取得完整在线质量门、J6 实板和真实场地产品验收，因此本仓库不能被表述为已经完成实车产品部署。详细边界见 [当前状态](docs/current-status.md) 和 [Detector Data Recovery V4](docs/detector-data-recovery-v4.md)。

## 快速体验

在 Windows PowerShell 的仓库根目录运行完整可视化演示：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_visual_demo.ps1 -Video on
```

只启动 Gazebo 清扫演示：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_gazebo_cleaning_demo.ps1
```

界面中的橙色外框表示任务范围，青绿色区域表示实际可清扫范围；规划、实际轨迹与已清扫区域使用不同图层显示。

首次安装、WSLg 配置、ROS 工作空间构建和其他启动方式见 [README_FIRST.md](README_FIRST.md)。

## 开发与验证

快速执行不依赖 ROS 的仓库检查：

```powershell
py -3 scripts/ci_fast.py
```

ROS、Gazebo、导航或运行时变更还需要执行对应的 Stage 验收，不能用快速检查代替。分支、PR、CI、证据和交付约束见 [开发工作流](docs/development-workflow.md) 与 [证据策略](docs/artifact-policy.md)。

## 目录结构

| 路径 | 内容 |
|---|---|
| `starter_ws/src/` | 自研 ROS 2 功能包 |
| `scripts/` | 安装、构建、启动、训练、评测与证据工具 |
| `config/` | 系统、任务和验收配置 |
| `docs/` | 架构、操作、验证协议与当前状态 |
| `artifacts/` | 适合纳入 Git 的紧凑验收证据 |

## 文档入口

- [项目规格](PROJECT_SPEC.md)
- [当前状态](docs/current-status.md)
- [环境与启动](README_FIRST.md)
- [操作指南](docs/operator-guide.md)
- [车辆模型](docs/vehicle-model-guide.md)
- [覆盖路径优化](docs/coverage-path-optimization.md)
- [数字孪生场景](docs/gazebo-digital-twin-scene.md)
- [工业化与 Sim2Real](docs/industrialization-and-sim2real.md)
- [故障回滚](docs/rollback.md)

## 许可证

项目代码及第三方组件的使用边界见 [LICENSE.md](LICENSE.md)。

## CRV6 感知恢复进展

`CHECKPOINT-RECONSTITUTION-V6` 已关闭历史 D1-B checkpoint 的最后一次有界恢复搜索：历史 SHA-256 `481374...a361` 的字节仍未找回，历史 DDRV4 D1 通过事实不改写。当前按 R1 使用已审计的 D1-B 初始化、冻结 G7 static TRAIN/HOLDOUT 边界和相同六轮协议重构出新候选；新候选必须以自己的 SHA 和 provenance 标识，不得冒充历史 D1-B。

CRV6 工具链覆盖恢复审计、R1 provenance、static VAL 非门禁回归、golden native/runtime parity、G7-MOVING HOLDOUT/VAL 原生门、有界 MA1 moving-domain adaptation，以及严格分离 discrete/area/combined 指标的真实 Gazebo 在线审计。MA1 在独立 G7-MOVING VAL 通过，但真实 Gazebo 24-mission 回放的离散 detector/map 门未通过；现有 G6 Area 的 boundary F1 和 negative FP/frame 也未达到 CRV6 阈值。因此 `MODEL_BLOCKED_INTERNAL=true`，禁止 freeze、读取 G5_V2 或声明产品就绪。大型数据、checkpoint 和逐帧 trace 保存在仓库外 evidence 根目录。
