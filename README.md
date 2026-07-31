# TZcup 智慧环卫无人清扫车

TZcup 是一个面向智慧环卫无人清扫车的 ROS 2 仿真与自主任务工程。项目以 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic、Nav2、SLAM Toolbox、OpenNav Coverage 和 Fields2Cover 为基础，覆盖车辆建模、环境仿真、定位导航、全覆盖清扫、垃圾感知、定点清扫、安全控制、调试可视化和验收证据。

## 地图优先的人类监督台（2026-07-31）

新增 `sanitation_hmi` 地图监督模式：浏览器首屏以二维作业地图为主，明确分开
Gazebo 配置参考真值、SLAM `/map`、感知预测、全局/局部规划和 `/odom` 实际轨迹；
同时显示 Gazebo 全场相机、车载相机、来源新鲜度、任务/安全状态、事件时间线、
刷盘轨迹推导的未覆盖/已覆盖/重复覆盖网格以及当前会话真实回放。评委、学习、工程
三种界面密度可切换，地图支持平移、缩放、全图适配和图层开关。

一键启动入口为
`ros2 launch sanitation_hmi human_visualization_demo.launch.py operator_token:=<本地令牌>`，
操作与验收说明见 [`docs/human-visualization.md`](docs/human-visualization.md)。急停只在检测到
外部安全速度门订阅后启用；Coverage、暂停/恢复、返航等任务按钮在安全任务编排器尚未
接入时保持禁用，不把 DSL 校验冒充车辆执行。当前软件合同可以通过独立机器门，完整
`human_visualization_ready` 仍要求本机 live ROS 源、外部安全门和真实任务执行链同时通过。

## 实时可视化演示

项目现在提供一条 Windows 命令启动的真实 Gazebo 导航与全覆盖演示：Gazebo GUI 跟随清扫车，RViz 以 `base_footprint` 为目标坐标系跟随显示地图、激光、规划路径与代价地图，浏览器看板实时显示任务阶段、融合位姿、速度、17 个覆盖组件、刷盘、急停和车辆轨迹，并自动保存 MCAP、专用看板 MP4 与代表帧；代表帧从录像末尾抽取，用于直接展示完整轨迹和任务终态。演示复用 AUTO-02 冻结的 `autonomous_navigation_profile_v1` 与正式 Coverage 链，不使用预制动画。

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_visual_demo.ps1 -Video on
```

本机完整任务通过：`17/17` 组件、经验覆盖率 `93.67%`、碰撞 `0`、禁行区违规 `0`、定位 XY RMSE `0.03588 m`，MCAP 为 `205528` 条消息/18 个话题；看板终态为 `COMPLETED`，专用 MP4 为 `1.49 MB`。使用与证据边界见 [`docs/auto17-visual-demo.md`](docs/auto17-visual-demo.md)。AUTO-17 只提升可观察性和演示复现能力，不改变学习感知、真实域、J6 板端及综合竞赛矩阵仍为 false 的事实。

## 当前状态

| 范围 | 状态 | 说明 |
|---|---|---|
| 软件工程与发布 | 已完成 | `AUTO-16=PASS`，源码、配置、测试、文档和发布工具齐全 |
| 基础仿真与自主导航 | 已通过机器门 | 车辆、场景、定位、Nav2、覆盖规划、安全控制和回放链已验证 |
| 调试可视化 | 可用 | Gazebo 显示物理场景，RViz 显示目标、障碍、区域、路径、车辆和系统状态 |
| 学习感知 | 阻断 | AUTO-05 数据门通过，但三次跨世界模型 screening 未达到冻结阈值 |
| 综合竞赛矩阵 | 未通过 | 受 AUTO-08 学习感知与定点清扫依赖阻断，正式综合任务未启动 |
| 真实域 | 外部阻断 | 缺少满足数量、标定和独立真值要求的真实数据集 |
| J6 部署 | 未通过 | 官方工具链已准备，但正式模型尚未产生，本机也没有 J6 实板 |

权威机器状态见 [`FINAL_AUTONOMOUS_STATUS.json`](FINAL_AUTONOMOUS_STATUS.json) 和 [`FINAL_BLOCKER_REGISTER.json`](FINAL_BLOCKER_REGISTER.json)。详细阶段过程、指标和失败边界保存在 [`docs/progress.md`](docs/progress.md)，不在本页重复记录逐步变更。

## 主要能力

- 4WD 差速/滑移转向清扫车模型，含刷盘、尘箱、LiDAR、RGB-D 相机和 IMU；
- 道路、路缘、窄通道、积水、垃圾、落叶、静态/动态障碍等 Gazebo 场景；
- SLAM、AMCL、混合定位、Nav2、keepout/speed filter、碰撞监控和急停；
- 全覆盖规划、任务几何、覆盖率/定位/安全指标与 rosbag 回放审计；
- 五类清扫目标的数据生成、感知接口、跟踪、主动观察与定点清扫链；
- APP/API、语音入口和受限任务 DSL；
- RViz 调试图层与 Gazebo 三维物理界面；
- 分阶段验收、紧凑证据、SBOM、许可清单和发布打包工具。

## 快速开始

推荐使用 Ubuntu 24.04 原生环境或 WSLg。Windows 上的 Docker Desktop 适合运行无头门禁；需要交互图形界面时优先使用 WSLg，或为 Docker 配置 X11。

```bash
export SANITATION_WS=$HOME/sanitation_ws
mkdir -p "$SANITATION_WS/src"
rsync -a starter_ws/src/ "$SANITATION_WS/src/"

bash scripts/bootstrap_jazzy.sh
bash scripts/import_upstream.sh
bash scripts/build_ws.sh
bash scripts/run_baseline.sh
```

连接已运行的 Gazebo 并打开调试 RViz：

```bash
source /opt/ros/jazzy/setup.bash
source "$SANITATION_WS/install/setup.bash"
ros2 launch sanitation_debug_visualization debug_visualization.launch.py
```

同时启动基础 Gazebo 和调试 RViz：

```bash
ros2 launch sanitation_debug_visualization debug_sim.launch.py
```

界面操作、图层和坐标系说明见 [`docs/debug-visualization.md`](docs/debug-visualization.md)。完整安装、键盘控制和环境检查见 [`README_FIRST.md`](README_FIRST.md)。

## 项目结构

| 路径 | 内容 |
|---|---|
| `starter_ws/src/` | 项目自研 ROS 2 包 |
| `scripts/` | 环境、构建、运行、阶段门和证据工具 |
| `config/` | 自主阶段 registry 与冻结配置 |
| `docs/` | 架构补充、运行指南、阶段记录与开发流程 |
| `artifacts/` | 已评审的紧凑机器证据，不存放原始数据和构建日志 |
| `.github/` | PR 模板和快速 CI |

原始 rosbag、训练集、运行日志、构建目录和临时模型应留在 Git 忽略的本机工作区；仓库只提交可审计的摘要、清单和必要小型模型。规则见 [`docs/artifact-policy.md`](docs/artifact-policy.md)。

## 文档入口

- [`README_FIRST.md`](README_FIRST.md)：环境准备和启动步骤；
- [`PROJECT_SPEC.md`](PROJECT_SPEC.md)：系统架构与接口边界；
- [`STAGE_GATES.md`](STAGE_GATES.md)：Stage 与 AUTO 阶段验收条件；
- [`docs/progress.md`](docs/progress.md)：详细进度、指标和历史边界；
- [`docs/compatibility.md`](docs/compatibility.md)：Docker、WSLg、GPU 和 ROS/Gazebo 兼容性；
- [`docs/development-workflow.md`](docs/development-workflow.md)：分支、测试、PR、CI 和收尾流程；
- [`FINAL_EVIDENCE_INDEX.md`](FINAL_EVIDENCE_INDEX.md)：最终证据索引。

## 开发与验证

所有开发修改应在独立分支和 worktree 中完成。README 只在项目定位、使用方式、目录结构或当前能力边界发生变化时更新；逐阶段和逐提交记录写入对应文档或 Git/PR 历史。

最低快速门禁：

```powershell
py -3 scripts/ci_fast.py
```

Linux / CI：

```bash
python scripts/ci_fast.py
bash -n scripts/*.sh
```

涉及 ROS 包、URDF/Xacro、SDF、Nav2、SLAM、覆盖规划或运行时行为时，还必须执行受影响的 Docker/ROS Stage 门禁；轻量 CI 不能代替真实仿真验收。

## 能力边界

- 当前结论是仿真与软件工程结论，不代表实车、真实道路、真人审计或 J6 板端验收；
- 仿真真值只能用于训练和评估，不得进入生产控制链；
- 独立组件通过不能拼接成综合竞赛通过；
- 不提交服务器地址、密钥、令牌、个人数据或未经许可的第三方资产；
- `shumo` 仅用于用户明确提出的数学建模竞赛任务，不用于本项目常规 ROS 2 / Gazebo 开发。
