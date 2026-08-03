# TZcup 智慧环卫无人清扫车 · [覆盖路径优化](docs/coverage-path-optimization.md)

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

只希望在 Gazebo 一个窗口中观看完整清扫任务时，使用专用入口：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_gazebo_cleaning_demo.ps1
```

Gazebo 右侧原生卡片可控制开始、暂停、继续、停止和关闭；按钮通过 Coverage 服务驱动 Nav2，不直接发速度。`-MapSize small|medium|large` 选择独立 16 m × 12 m 演示场、80 m × 50 m 中型验证或严格 200 m × 100 m 比赛大图，详见 [`docs/gazebo-multiscale-control.md`](docs/gazebo-multiscale-control.md)。
Windows 启动器会准备 WSLg `/mnt/shared_memory`、恢复异常窗口，并在关闭 Gazebo 后立即停止运行链、释放端口和关闭已跟踪的 RemoteApp 窗口句柄，避免残留不可交互会话或无渲染内容的黑色外壳。

默认 `small` 是单独设计的 `16 m × 12 m` 竞赛功能演示场；橙色外框表示 30 m² 外部任务区，青色内框表示扣除安全回转带后的 12 m² 实际清扫区，覆盖率只统计内框。Gazebo 右侧同窗按语义分层显示规划条带、无刷连接、实际清扫、补扫和青绿色已清扫栅格，并实时显示目标状态。场内布置 10 个、五类可清扫目标，每类各 2 个；清扫完成后物品本体会从场景中移除。瓶、罐、A5 级纸张、小纸盒、落叶和积水不仅保持真实尺度，还具有瓶肩/瓶颈/标签、罐沿/拉环、纸张折角、纸盒翻盖、叶尖/叶柄/叶脉和不规则水斑等可辨识外形；起点/回库也是贴地薄标记，不再用悬浮大球冒充清扫物。手动演示入口使用不含 CameraTracking 的专用 GUI 配置，默认可自由旋转、平移和缩放，不会把拖动后的视角拉回小车。优化器默认用 `0.52 m` 条带间距配合 `0.65 m` 刷盘，按相邻弓字形条带执行原地转向与短距离横移；刷盘仅在清扫条带开启，残余区域最多补扫一轮。演示摘要同时硬验收实际覆盖率、重复覆盖率、安全状态和 `10/10` 目标清除。默认 `fast` 为 2x、`0.70 m/s`，`turbo` 为 3x、`0.90 m/s`；运行时同时更新控制器、安全速度门和 velocity smoother，避免旧版平滑器把演示速度仍夹在 `0.45 m/s`。顶部 World Stats 显示本机实际 RTF。
10 个可移除目标的中心全部位于青色实际清扫区内部；场景契约会拒绝任何只在橙色外任务区内、但覆盖路径无法遍历的目标配置。目标采用覆盖四个象限、彼此至少相距 `0.70 m` 且横纵坐标不成行列的错落均匀分布，既接近真实散落状态，也避免遮挡和重叠。薄纸张使用同尺寸超薄实体底层和紧贴本体的轮廓阴影，落叶堆由六片真实尺度、可双面观察的薄实体叶片和不规则贴地阴影组成；小场默认相机放大框住车辆与青色作业区，便于在开始前数清全部目标。这些细节不使用圆圈或悬浮光环，也不增加碰撞体。
条带执行坐标使用 seed 118/119/120/123 离线拟合的法向标定，为实际跟踪误差预留重复率余量；在线控制只读取融合位姿，Gazebo 真值仍仅用于任务结束后的覆盖和定位评分。
验收报告会分别记录刷盘开启、关闭和状态切换期间的实际里程，并以刷盘中心到最近主条带中心线的横向误差 P95 ≤ `0.08 m` 作为硬门，避免仅凭轨迹图片判断弓字形路径是否规整。
混合定位在里程计坐标中平滑 RTK 全局锚点，并对 RTK 与扫描修正执行 10 cm 创新一致性检查；这会抑制绝对定位噪声和错误地图匹配，同时保留轮速/IMU 的实时运动响应。
独立小场与冻结的 Stage4V 扫描地图不共用场景几何，因此小场明确使用 RTK+轮速+IMU，避免把地图不一致的扫描修正混入控制；中/大场仍保留混合扫描降级。Coverage 的最终 `success` 现在同时包含每 seed 定位 RMSE ≤5 cm，不能再出现路径通过但定位超限的假绿灯。
仿真 `rtk_fixed` 档将白噪声保持为 2 cm、固定偏置保持为 5 mm，并把长期随机游走标定为 1 mm/√s；较差的 RTK float、多路径和拒止档不随之放宽。该参数是明确的固定解传感器能力假设，实车必须用接收机日志重新标定，不能直接沿用仿真结论。
冷启动时，启动器会先验证定位话题与 `odom→base_footprint` TF，再由单个持续驻留的 ROS 图探针精确确认必需话题、Action 服务及 Nav2 controller/planner 均为 `active [3]`，之后才打开 Gazebo；探针结果保存为 `runtime_readiness.json`，避免逐轮重启 DDS 发现造成偶发假超时。若 GUI 在原生控制加载前提前退出，则只执行一次安全重启和同参数重试；任一就绪门重复失败都会明确返回错误，不会留下一个可见但不能行驶的误导窗口。
任务开始前的急停 false 可用性脉冲由单个持续驻留的 ROS 节点重复发布，并同时确认至少两个订阅者及仪表盘 `topics_seen`；只有安全链和人机界面都实际观察到该接口才继续，避免短生命周期 CLI 发布器的 DDS 发现竞态造成偶发假失败。
Nav2 转场与组件执行最多允许初次执行加 2 次有界重试；每次中止都保留终端位姿与错误证据，超过上限立即失败，不以无限恢复掩盖控制问题。
Coverage 进程由 `setsid --wait` 监督，启动器始终等待真实子进程终态，并以无缓冲日志和 `coverage_process_exit_code.txt` 保留退出证据；即使在终端、systemd 或 Windows 进程服务等不同父进程布局下，也不会把 `setsid` 的中间 fork 误判为任务完成并提前清理仿真。
在 WSLg 中，Gazebo 服务端和传感器继续使用 D3D12/NVIDIA，原生 GUI 自动改用已验证可见的 X11/llvmpipe 通道；启动器会抓取 `3D Scene` 实际像素，纯黑视口会返回错误而不再误报 READY。

它不打开浏览器或 RViz；车辆在独立小场内真实执行完整 Coverage，青绿清扫带随刷盘开启累积，默认任务结束后保留 Gazebo，按 `Ctrl+C` 才收尾并生成验收摘要。
长时无头多种子回归使用 `scripts/run_frozen_coverage_trial.ps1`：每轮固定独立 ROS domain、Gazebo partition 和小场任务配置，输出单独的启动日志与证据目录，避免测试编排器的会话寿命影响仿真任务本身。

本机完整任务通过：`17/17` 组件、经验覆盖率 `93.67%`、碰撞 `0`、禁行区违规 `0`、定位 XY RMSE `0.03588 m`，MCAP 为 `205528` 条消息/18 个话题；看板终态为 `COMPLETED`，专用 MP4 为 `1.49 MB`。使用与证据边界见 [`docs/auto17-visual-demo.md`](docs/auto17-visual-demo.md)。AUTO-17 只提升可观察性和演示复现能力，不改变学习感知、真实域、J6 板端及综合竞赛矩阵仍为 false 的事实。

工业化接口契约、故障档案和 SIL/HIL/封闭场准入见 [`docs/industrialization-and-sim2real.md`](docs/industrialization-and-sim2real.md)；演示目标按真值和刷盘足迹判定并从场景移除，不代表真实识别或物理吸入闭环。
### 竞赛尺度现场配置

`powershell -ExecutionPolicy Bypass -File scripts/run_visual_demo.ps1 -CompetitionProfile -GazeboOnly -ManualControl -KeepOpen`
在同一条 Gazebo/Nav2/Coverage 运行链加载 20,000 m² 完整地图、20 分区和
AUTO-12 的 1.32 m 刷盘/1.0 m/s 参数，并现场运行一个 108 m² 代表性分区；
Gazebo 状态标记使用显式浮点面积参数，避免 ROS 参数类型不匹配。需要诊断渲染器时可显式增加 `-GazeboGuiRenderer d3d12|software`；日常使用保留默认 `auto`。
这不等价于全场耐久通过；边界和剩余差距见
[`docs/competition-gazebo-profile.md`](docs/competition-gazebo-profile.md)。

## Gazebo 数字孪生场景

不需要浏览器控制台时，可直接启动人类可读的园区道路 Gazebo 场景和清扫车：

```bash
ros2 launch sanitation_bringup gazebo_scene.launch.py
```

结构化世界包含实体路面/路缘/人行道、道路标线、斑马线、绿化、建筑、树木、
路灯、垃圾桶、纸箱、行人障碍以及五类清扫目标；车辆补齐了上车体、保险杠、
轮毂、传感器外壳、尘箱和刷盘细节。冻结的导航锚点、二维 footprint、传感器外参、
话题和动力学参数保持不变。对象口径、几何差异和许可说明见
[`docs/gazebo-digital-twin-scene.md`](docs/gazebo-digital-twin-scene.md)。
车辆灯组、检修门、充电口、传感器、尘箱、刷盘和清扫机构说明见
[`docs/vehicle-model-guide.md`](docs/vehicle-model-guide.md)。

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
- 人类可读的园区道路、路缘、人行道、绿化、积水、垃圾、落叶和静态/动态障碍 Gazebo 场景；
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
