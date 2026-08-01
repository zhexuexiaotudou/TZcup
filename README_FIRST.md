# TZcup 环境与启动指南

本页只说明如何在本机准备和启动 TZcup。项目概览、当前能力和边界见根目录 [`README.md`](README.md)。

## 最快看到清扫车移动

在 Windows PowerShell 的仓库根目录运行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_visual_demo.ps1 -Video on
```

脚本使用 `TZcup-Ubuntu-24.04` WSL2/WSLg，自动启动 Gazebo、Nav2、Coverage、RViz 和 `http://127.0.0.1:8877` 实时看板；正常冷启动后车辆会自动驶向作业起点并执行 9 条清扫带和 8 个转弯。结果写入 `artifacts/auto17_visual_demo_<UTC>/`。再次启动前必须先停止旧实例；启动器会检测重复 Nav2/Coverage 节点并拒绝污染运行。完整说明见 [`docs/auto17-visual-demo.md`](docs/auto17-visual-demo.md)。

如果只看 Gazebo，不需要浏览器看板和 RViz：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_gazebo_cleaning_demo.ps1
```

默认运行一个约 `6 m × 5 m` 的完整小范围任务：蓝色边框和灰色底面是指定清扫区，车辆从
蓝色 `HOME` 出发并驶向绿色 `CLEANING START`，随后逐条覆盖。青绿色区域是刷盘实际经过的
已清扫带，琥珀色线是当前 Nav2 路径，车顶文字显示转场、对齐、清扫、转弯和完成状态。
需要原 17 段大范围任务时增加 `-FullArea`；使用 `-CloseOnComplete` 可在验收后自动关闭。

## 推荐环境

- Ubuntu 24.04（原生或 WSLg）；
- ROS 2 Jazzy Desktop；
- Gazebo Harmonic；
- Python 3.12；
- 可选 NVIDIA GPU；
- Windows 无头验收可使用 Docker Desktop，图形交互优先使用 WSLg。

项目使用 Linorobot2 4WD、Nav2、SLAM Toolbox、robot_localization、OpenNav Coverage 和 Fields2Cover，并在 `starter_ws/src/` 提供自研 ROS 2 包。

## 目录

- `starter_ws/src/`：车辆、场景、bringup、导航、安全、覆盖、感知、定点清扫、HMI 和调试可视化包；
- `scripts/`：依赖安装、构建、启动、阶段验收和证据工具；
- `config/`：自主阶段配置；
- `docs/`：运行指南、阶段说明和详细进度；
- `artifacts/`：紧凑验收证据，原始数据和日志不进入 Git。

## 首次安装

先安装 ROS 2 Jazzy Desktop，并确认：

```bash
source /opt/ros/jazzy/setup.bash
ros2 --help
```

在仓库根目录执行：

```bash
export SANITATION_WS=$HOME/sanitation_ws
mkdir -p "$SANITATION_WS/src"
rsync -a starter_ws/src/ "$SANITATION_WS/src/"

bash scripts/bootstrap_jazzy.sh
bash scripts/import_upstream.sh
bash scripts/build_ws.sh
```

如果使用 WSLg 并希望强制 NVIDIA D3D12 renderer：

```bash
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
glxinfo -B
```

## 启动基础仿真

```bash
bash scripts/run_baseline.sh
```

另开终端检查运行状态：

```bash
source /opt/ros/jazzy/setup.bash
source "$SANITATION_WS/install/setup.bash"

ros2 topic list
ros2 run sanitation_tasks sanitation_smoke_check --ros-args \
  -p timeout_sec:=30.0 \
  -p output_path:="$SANITATION_WS/artifacts/smoke_check.json"
```

## 打开可视化

先加载 ROS 2 和工作空间环境：

```bash
source /opt/ros/jazzy/setup.bash
source "$SANITATION_WS/install/setup.bash"
```

只打开 Gazebo 场地和车辆，不启动浏览器控制台、SLAM 或 RViz：

```bash
ros2 launch sanitation_bringup gazebo_scene.launch.py
```

该入口默认加载人类可读的园区道路结构化世界。场景对象、车辆模型和几何边界见
[`docs/gazebo-digital-twin-scene.md`](docs/gazebo-digital-twin-scene.md)。

`gazebo_scene.launch.py` 只负责静态场景和手动驾驶。需要车辆自动执行完整清扫任务时，
必须改用上面的 `run_gazebo_cleaning_demo.ps1`，它会同时启动定位、Nav2、Coverage 和
Gazebo 原生清扫可视化。

一键启动结构化 Gazebo 场景、SLAM、安全速度门和地图优先的浏览器监督台：

```bash
ros2 launch sanitation_hmi human_visualization_demo.launch.py \
  operator_token:=replace-with-a-local-token
```

Windows 浏览器打开 `http://127.0.0.1:8765`。若 Gazebo、SLAM 和安全门已经由其他
launch 启动，则改用 `human_visualization.launch.py` 只附着监督台，避免重复启动控制面。
数据口径、API、安全边界和验收命令见
[`docs/human-visualization.md`](docs/human-visualization.md)。

连接已经运行的 Gazebo：

```bash
ros2 launch sanitation_debug_visualization debug_visualization.launch.py
```

同时启动 Gazebo 与调试 RViz：

```bash
ros2 launch sanitation_debug_visualization debug_sim.launch.py
```

调试界面默认显示清扫区域、禁行区、五类目标、障碍、感知结果、车辆、LiDAR、覆盖路径和任务状态。鼠标与图层操作见 [`docs/debug-visualization.md`](docs/debug-visualization.md)。

## 键盘控制

```bash
source /opt/ros/jazzy/setup.bash
source "$SANITATION_WS/install/setup.bash"
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Windows Docker 阶段门

按改动范围选择对应脚本，例如：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage1_docker.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage2_docker.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage3_docker.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage4_docker.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage5a_docker.ps1 -OutputName stage5a_formal -RecordBag
```

完整门禁和证据要求见 [`STAGE_GATES.md`](STAGE_GATES.md) 与 [`docs/progress.md`](docs/progress.md)。

## 重要边界

- ROS 1 OpenPodcar 只适合作为车辆比例和模型结构参考，不作为主工程；
- 真实域数据、J6 板端和实车验收尚未完成；
- 仿真真值只用于训练/评估，不能进入控制链；
- 原始数据、bag、日志和构建产物应放在 Git 忽略目录，提交前阅读 [`docs/artifact-policy.md`](docs/artifact-policy.md)。
