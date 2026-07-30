# 无人清扫车仿真启动包

> 2026-07-30：AUTO-15 已完成 18 类竞赛场景的需求/依赖/证据索引；因 AUTO-08 学习感知与定点清扫依赖阻断，正式 10-seed/30-mission 综合矩阵未启动，视频与 MCAP 均为 0。`AUTO-15=BLOCKED`、`SIMULATION_COMPETITION_MATRIX_PASS=false`。

> 2026-07-30：AUTO-14 已完成官方 D-Robotics OpenExplorer 3.7.0 工具链下载、哈希、版本、`hb_compile` 启动、ONNX 预检和 fail-closed runtime adapter；因 AUTO-06 正式模型未产生，量化/编译不得执行，本机亦无 J6 板卡。故 `AUTO-14=BLOCKED`、`J6_TOOLCHAIN_PASS=false`、`J6_RUNTIME_PASS=false`，不冒充模型编译或板端成绩。

> 2026-07-30：独立 AUTO-11 大地图与定时任务 lane 已通过离线仿真机器门：20,000 m² map、10 条真值分离定位轨迹、5 次全覆盖和 20 次定时路线全部满足冻结指标；证据等级不是 Gazebo 或实车。当前主依赖阶段仍为 AUTO-05。

> 2026-07-30：独立 AUTO-10 多模态任务入口已通过机器门。APP/API 288 用例、语音 500 用例和 DSL 1200 用例均达到冻结阈值；LLM/语言层只输出固定任务 DSL 并调用 allowlist，不直接访问 `/cmd_vel` 或关节命令。当前主依赖阶段仍为 AUTO-05。

> 2026-07-29：本机 `TZcup-Ubuntu-24.04` 已完成 ROS 2 Jazzy + Gazebo Harmonic + Nav2/RViz 的 WSLg 图形验收。Gazebo 三维场景与 Nav2 RViz 均实际渲染，D3D12 使用 RTX 4080 Laptop GPU；全工作空间 `449 tests / 0 failures`，运行中 smoke check 为 11/11 topic。证据见 `artifacts/wslg_gui_20260729_evidence/`，环境细节与边界见 `docs/compatibility.md`。

> 2026-07-30：AUTO-05 的 8-world、120-scene/1200-frame 原生 G3 数据门通过，但三次有界模型 screening 后仍有 7 个冻结门失败，故 `AUTO-05=BLOCKED`，AUTO-06/07/08 依赖阻断。production 默认、历史 evidence、真人/真实域/J6 和最终竞赛状态均未提升。

> 2026-07-30：独立 AUTO-13 真实域 lane 已实现采集、隐私、标定、接入与评测工具。资源发现只有本机相机，没有满足 20 scene/1000 frame 且带独立可审计 GT 的真实数据，因此 `AUTO-13=BLOCKED_EXTERNAL`、`REAL_DOMAIN_PASS=false`；fixture 不计真实域证据。

> 2026-07-21：历史工程入口为 `GPT_REVIEW_STAGE5BR6W.md`。工程豁免支线已完成 V4/policy/candidate-footprint opt-in 与规划器加固，但真实 Stage4W seed 0 因 `no_reachable_clean_route` 失败，故工程 Oracle 未启动；该失败结论保持不变。

> 2026-07-21：历史正式入口为 `GPT_REVIEW_STAGE5BR6.md`。Stage5BR6-A 已生成两个独立的 270 张人工盲审包并通过无 truth 泄漏审计；尚未收到两份真人 response，因此 `READY_FOR_STAGE5BR6_ORACLE=false`，原包与 sealed truth 保持不变。

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
- `STAGE_GATES.md`：历史 Stage 门与 AUTO-00–AUTO-16 自主阶段的验收条件
- `THIRD_PARTY_SELECTION.md`：第三方仓库选择理由和许可边界
- `AGENTS.md`：项目级 Agent 规则和开发门禁
- `docs/development-workflow.md`：统一命名的“开发工作流”，覆盖分支、PR、CI、部署、真实验收和收尾
- `docs/progress.md`：Stage 0 至当前自主阶段的真实运行证据、边界和复现命令
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

在 Ubuntu 24.04 WSLg 中需要强制选择 D3D12/NVIDIA renderer 时，可先设置：

```bash
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
glxinfo -B
```

本机 `zhexu` 用户已把这两个变量写入 shell 启动文件，并提供 `tzcup-gazebo`、`tzcup-navigation-rviz` 和 `tzcup-stop-visualization` 三个本地快捷命令；这些命令是主机便利入口，不是仓库可移植接口。

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

当前 Windows 主机已通过 Docker Desktop、Ubuntu 24.04 / ROS 2 Jazzy 容器和 NVIDIA GPU passthrough 完成 Stage 0–5A 与 AUTO-00–AUTO-04，并在本地 Ubuntu 24.04 WSLg 中补齐 Gazebo/RViz 图形复核和基础 topic smoke check；AUTO-05 数据门通过但模型门阻断。Stage5BR6W 的 V4 candidate-footprint 失败仍作为历史事实保留。正式人工门、真实域与 J6 板端门均未通过；AUTO-12 离线效率门已通过。当前边界以 `AUTONOMOUS_STATE.json`、`docs/progress.md` 与 `docs/auto05-g3-screening.md` 为准。
