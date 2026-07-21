# 无人清扫车仿真启动包

> 2026-07-21：当前入口为 `GPT_REVIEW_STAGE5BR6.md`。Stage5BR6-A 已生成两个独立的 270 张人工盲审包并通过无 truth 泄漏审计；尚未收到两份真人 response，因此 `AWAITING_HUMAN_REVIEW=true`、`READY_FOR_STAGE5BR6_ORACLE=false`，V4、policy v2、candidate footprint 和 Oracle 主动观察均未越级推进。

> 2026-07-20：历史入口为 `GPT_REVIEW_STAGE5BR5.md`。Stage5BR5 已完成 ActiveObservation 时间语义修复、V1–V4 机械网格、V1/V2/V4 六世界 360 帧真实消融和五类各 40 张的 200 张盲审集；两名独立人工评审尚未完成，所以相机、policy v2、正式主动观察和模型训练均保持 fail-closed。

> 2026-07-20：历史入口为 `GPT_REVIEW_STAGE5BR4.md`。Stage5BR4 证明 C0 全量数据只有 `25.96%` recognition-ready；C0–C3 真实消融后 C3 主动观察转换仅 `50% < 90%` 且车体遮挡明显，因此相机没有定型，模型训练和 120/1200 数据扩充按门禁未启动。

> 2026-07-20：历史入口为 `GPT_REVIEW_STAGE5BR3.md` 和 `docs/stage5br3-g2-screening.md`。Stage5BR3 已完成真实车辆相机六世界运行时契约、80 scene/800 frame QA、四档分辨率扫描和三次 split-model screening；模型门失败后已停止。复核包完整不等于 Stage5B 通过，`READY_FOR_GPT_REVIEW_STAGE5B=false`、`READY_FOR_STAGE5C=false`。

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
  - `sanitation_learning`

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
- `docs/progress.md`：Stage 0–5B 的真实运行证据、当前边界和复现命令
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
- Stage5B 程序化多变体资产、学习模型训练/ONNX 评测、颜色压力测试、J6 预检与失败边界证据
- Stage5BR3 六个 world-isolated G2 世界、真实车辆 RGB-D/GT 同步采集、80/800 逐实例 QA、分辨率扫描与 split-model screening
- J6、真实数据、实车和竞赛效率的独立 fail-closed 阶段门

## 5. 重要说明

当前 Windows 主机已通过 Docker Desktop、Ubuntu 24.04 / ROS 2 Jazzy 容器和 NVIDIA GPU passthrough 完成 Stage 0–5A，并把 Stage5B 推进到 Stage5BR6-A。Stage4W 与 Stage5A 历史回归仍通过；V4 盲审双包已生成，但尚缺两名独立真人 response，D2、J6 与竞赛效率门也未通过。当前边界以 `docs/progress.md` 与 `GPT_REVIEW_STAGE5BR6.md` 为准。
