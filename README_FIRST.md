# 无人清扫车仿真启动包

> 2026-07-17：Stage5A 已进入 GPT 独立复核边界。先读 `GPT_REVIEW_STAGE5A.md` 与 `docs/stage5a-garbage-perception.md`；本轮只声明 synthetic-domain 工程门，不声明真实数据、J6 实板、实车或竞赛效率达标。

本包用于把“智慧环卫无人清扫车”项目的仿真工作推进到可复现、可演示、可评测的第一阶段。

> 仓库总入口、当前状态和开发要求请先阅读根目录 [`README.md`](README.md)；本文件保留环境准备与启动细节。

## 1. 推荐基线

- **宿主系统**：Ubuntu 24.04（优先原生或双系统）
- **ROS 2**：Jazzy
- **仿真器**：Gazebo Harmonic
- **移动底盘基线**：Linorobot2 4WD（ROS 2、Nav2、SLAM Toolbox、robot_localization、Gazebo 已打通）
- **全覆盖任务套件**：OpenNav Coverage + Fields2Cover
- **项目自有包**：
  - `sanitation_vehicle_description`
  - `sanitation_worlds`
  - `sanitation_bringup`
  - `sanitation_tasks`
  - `sanitation_navigation`
  - `sanitation_safety`
  - `sanitation_coverage`
  - `sanitation_gnss_sim`
  - `sanitation_scan_refiner`
  - `sanitation_perception_interfaces`
  - `sanitation_perception`
  - `sanitation_ground_truth`
  - `sanitation_dataset`
  - `sanitation_spot_cleaning`

> 不建议把 ROS 1 OpenPodcar 直接作为主工程。它可用于参考车辆比例和模型结构，但其主线是 ROS Kinetic + Gazebo 7，迁移成本高。

## 2. 目录用途

- `README.md`：中文项目总入口、当前状态、快速开始和最近同步
- `CODEX_MASTER_PROMPT.md`：Stage 0–4 原始主提示词；当前阶段以根 README 和最新 `GPT_REVIEW_STAGE*.md` 为准
- `PROJECT_SPEC.md`：项目技术规范
- `COMPETITION_REQUIREMENTS.md`：赛题指标到仿真模块的映射
- `STAGE_GATES.md`：阶段、验收条件和 GPT 复核门
- `THIRD_PARTY_SELECTION.md`：第三方仓库选择理由和许可边界
- `AGENTS.md`：项目级 Agent 规则和开发门禁
- `docs/development-workflow.md`：统一命名的“开发工作流”，覆盖分支、PR、CI、部署、真实验收和收尾
- `docs/progress.md`：Stage 0–5A 的真实运行证据、当前边界和复现命令
- `scripts/`：环境检查、拉取依赖、构建、运行和证据采集脚本
- `starter_ws/src/`：可直接放进 ROS 2 工作空间的项目骨架

## 3. 首次使用

### 3.1 安装 ROS 2 Jazzy

先按 ROS 2 官方文档安装 ROS 2 Jazzy Desktop。确认：

```bash
source /opt/ros/jazzy/setup.bash
ros2 --help
```

### 3.2 创建工作空间并导入本包

```bash
export SANITATION_WS=$HOME/sanitation_ws
mkdir -p "$SANITATION_WS/src"

# 在本启动包根目录运行
rsync -a starter_ws/src/ "$SANITATION_WS/src/"

bash scripts/bootstrap_jazzy.sh
bash scripts/import_upstream.sh
bash scripts/build_ws.sh
```

### 3.3 启动基础仿真

```bash
bash scripts/run_baseline.sh
```

另开终端检查：

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/sanitation_ws/install/setup.bash"

ros2 topic list
ros2 run sanitation_tasks sanitation_smoke_check --ros-args \
  -p timeout_sec:=30.0 \
  -p output_path:="$HOME/sanitation_ws/artifacts/smoke_check.json"
```

### 3.4 键盘控制

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/sanitation_ws/install/setup.bash"
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 4. 当前项目已经包含

- 4WD 差速/滑移转向清扫车几何模型
- 0.65 m 清扫作业宽度可视化区域
- 40 L 尘箱几何体
- 2D 激光雷达、RGB-D 相机、IMU
- 道路、路缘、窄通道、垃圾、落叶堆、低摩擦积水区、静态障碍场景
- 一键启动入口
- ROS Topic 冒烟检查
- SLAM Toolbox、AMCL、Nav2、keepout/speed filter 和急停速度门
- OpenNav Coverage + Fields2Cover 覆盖规划、指标 JSON 和 rosbag 证据
- hybrid RTK/扫描精化定位、统一任务几何、可达 staging 和完整 17 组件执行
- 持久 ROS–Gazebo 动态障碍桥、20 次有效交互和动态清障证据
- raw measurement、非零 covariance adapter、EKF A/B/C/D 消融与双分辨率地图几何评估
- precision mapping、localization/coverage 和默认禁用 stress 三套运行包络
- Stage5A 五类 registry、仿真 GT、20-scene RGB-D/COCO 数据、ONNX Runtime 2D/3D/map 感知、多帧 tracker 与 synthetic task-state E2E
- J6、真实数据、实车和竞赛效率的独立 fail-closed 阶段门

## 5. 重要说明

当前 Windows 主机已通过 Docker Desktop、Ubuntu 24.04 / ROS 2 Jazzy 容器和 NVIDIA GPU passthrough 完成 Stage 0–5A 的 headless 构建与运行验证，真实日志、JSON、数据集和 rosbag 位于 `artifacts/`。Stage4W 正式定位/Coverage/安全回归通过；Stage5A synthetic perception、task-state E2E 与实时 RGB-D 链路已形成复核证据。竞赛感知、J6、竞赛效率和原生 Ubuntu/WSLg GUI 仍未通过。当前边界以 `docs/progress.md` 与 `GPT_REVIEW_STAGE5A.md` 为准。
