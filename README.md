# TZcup 无人清扫车仿真项目

## Stage5BR2 G2 车载相机基础恢复（2026-07-20）

Stage5BR2 已纠正历史指标语义：G1 的 `cross_asset_world` 只能称为 `cross_asset_same_world`，单世界的真实 `cross_world` 与未隔离的 `cross_material` 均为 `null`；实例尺寸改为按 instance-id 掩码逐实例统计，零像素物体记为 `not_visible`。新增 G2 训练世界从当前车辆 Xacro/launch 提取生产相机契约，使用未放大的真实物理资产，在四种材料上生成 4 个不同 SHA 的 world，并由 Gazebo Harmonic 实际验证 RGB、深度、semantic GT、instance GT 话题全部可启动。GT 仅存在于训练世界，生产 launch 未修改。

当前尚未执行 G2 的 80 scene/800 frame 采集 QA、四分辨率实测和 detector/area segmenter 筛选，因此首个阻断层为 `G2_screening_dataset_80_scene_800_frame_not_executed`，`READY_FOR_GPT_REVIEW_STAGE5B=false`、`READY_FOR_STAGE5C=false`。复核入口见 [`GPT_REVIEW_STAGE5BR2.md`](GPT_REVIEW_STAGE5BR2.md)、[`docs/stage5br2-g2-vehicle-camera.md`](docs/stage5br2-g2-vehicle-camera.md) 与 [`artifacts/stage5br2_20260720_review/`](artifacts/stage5br2_20260720_review/)。

## Stage5BR Gazebo-camera 数据恢复与模型筛查（2026-07-19）

Stage5BR 已通过训练链自证：12 帧 micro-overfit 的 macro F1/mIoU 为 `0.98124/0.96333`，PyTorch/ONNX 最大 logit 误差 `6.866e-05`、argmax agreement `1.0`。项目新增真实 Gazebo Harmonic 共视场 RGB-D、semantic、instance 数据链，50 个独立 scene / 500 帧 smoke 的标注完整率为 100%，semantic-instance 一致性错误、asset leakage 和跨 split exact/pHash duplicate 均为 0。

三次 G1 模型筛查仍未同时通过 in-domain、跨资产/世界和颜色压力门；最佳尝试为 `0.84511 / 0.65804 / 0.47647`，未达到 `0.90 / 0.70 / 0.60`。因此没有扩成 500 scene/5000 帧正式 G1，也没有执行正式 live 或真实 Nav2 spot-clean。`REVIEW_PACKET_COMPLETE=true`，但 `READY_FOR_GPT_REVIEW_STAGE5B=false`、`READY_FOR_STAGE5C=false`。复核入口见 [`GPT_REVIEW_STAGE5BR.md`](GPT_REVIEW_STAGE5BR.md)、[`docs/stage5br-gazebo-camera-recovery.md`](docs/stage5br-gazebo-camera-recovery.md) 与 [`artifacts/stage5br_20260719_review/`](artifacts/stage5br_20260719_review/)。

## Stage5B 学习型感知筛查与停止边界（2026-07-19，Stage5BR 前历史基线）

Stage5B 已新增 30 个自研程序化垃圾资产（五类、每类六变体）、12 个硬负样本、按 scene/asset/texture/world 隔离的数据合同、两种实际梯度训练候选、ONNX Runtime 评测、颜色捷径压力测试、J6 fail-closed 预检和训练模型的真实 Gazebo RGB-D 接入诊断。三次结构性筛查后，最佳候选验证 macro F1 为 `0.38637`，但 100 个未见 scene / 1000 帧测试的离散类 macro P/R/F1 仅为 `0.00752/0.00784/0.00768`，颜色压力 aggregate macro F1 为 `0.05192`，均未过门。

该轮数据生成器还是程序化 D1 renderer，不是真实 Gazebo camera renderer；这是 Stage5BR 推进前的历史边界。Stage5BR 已补齐真实 Gazebo-camera G1 smoke 数据链，但模型 screening 仍失败；D2 真实数据为空，J6 官方工具链/实板不可用。依停止条件，仍未执行 500 seed/5000 帧正式 D1、30 seed/10 分钟正式实时门或 30 次真实 Nav2 spot-clean。因此 `REVIEW_PACKET_COMPLETE=true`，但 `READY_FOR_GPT_REVIEW_STAGE5B=false`、`READY_FOR_STAGE5C=false`、`competition_perception_pass=false`。Stage5B 历史复核入口见 [`GPT_REVIEW_STAGE5B.md`](GPT_REVIEW_STAGE5B.md)、[`docs/stage5b-learned-perception.md`](docs/stage5b-learned-perception.md) 与 [`artifacts/stage5b_20260719_review/`](artifacts/stage5b_20260719_review/)；当前结论以本页顶部 Stage5BR 段落为准。

## Stage4W 可达清扫域与完整任务闭环（2026-07-17）

Stage4W 已修复定位协方差/全局锚点传播、统一任务几何、可达 staging、完整组件执行、动态障碍清除与安全超时证据链。正式 hybrid 10-seed 全部通过，XY RMSE P50/P95/max 为 `0.02825/0.03726/0.03778 m`；静态 5-seed 全部通过，每次均为当前几何生成的 `17/17` 组件，经验覆盖率 `92.93%–94.53%`、覆盖期 RMSE `0.02930–0.04620 m`；动态交互 `20/20` 有效且碰撞 0。keepout、限速区、30 次急停和完整 MCAP 回放均通过，急停 P95 为 `0.188 s`，上游命令停止后 `1.694 s` 达到连续 5 帧稳定零输出。`READY_FOR_GPT_REVIEW_STAGE4W=true`、`READY_FOR_STAGE5A=true`；理论效率仍为 `1053 m²/h < 3500 m²/h`，因此竞赛效率门保持 false。复核入口见 [`GPT_REVIEW_STAGE4W.md`](GPT_REVIEW_STAGE4W.md) 与 [`artifacts/stage4w_20260717_review/`](artifacts/stage4w_20260717_review/)。

## Stage4V 混合定位与完整任务复核（2026-07-16）

Stage4V 已实现 C++ 扫描精化、标准 NavSatFix 仿真、局部/全局融合及 TF 单所有权审计。正式 hybrid 10-seed 全部通过：XY RMSE P50/P95/max 为 `0.03344/0.03792/0.03872 m`，导航、TF 与扫描实际参与均为 10/10，GT 控制违规 0。随后完整 Coverage 在 transit-to-start 失败，经验覆盖率 0%，动态障碍有效交互 0/20；但零碰撞、过滤器、30 次急停（P95 `0.171 s`）与 MCAP 回放通过。因此 `READY_FOR_GPT_REVIEW_STAGE4V=false`、`READY_FOR_STAGE5A=false`，理论效率仍为 `1053 m²/h < 3500 m²/h`。复核入口见 [`GPT_REVIEW_STAGE4V.md`](GPT_REVIEW_STAGE4V.md)。

## Stage4U 坐标标定、定位地图与 5 cm 闭环（2026-07-16）

Stage4U 已修复 map/map_gt 评测语义、`ParticleCloud` 类型/QoS 和地图质量假通过，并完成 M1/M2/M3、AMCL/SLAM Toolbox、LiDAR/AMCL 灵敏度及正式 Oracle 10-seed。最优候选为结构化 v2 surveyed reference 0.02 m + AMCL + 360@10 Hz；10/10 导航成功、TF 全连续、粒子仪器全有效、恢复 0 次，但 map-relative XY RMSE P50/P95/max 为 `0.06767/0.07983/0.08022 m`，未过 0.05 m 硬门。因此 `READY_FOR_GPT_REVIEW_STAGE4U=false`、`READY_FOR_STAGE5A=false`，realistic 与完整 Coverage 按停止条件未执行。复核入口见 [`GPT_REVIEW_STAGE4U.md`](GPT_REVIEW_STAGE4U.md) 与 [`artifacts/stage4u_20260716_review/`](artifacts/stage4u_20260716_review/)；原始 MCAP/posegraph 在用户确认前保留。

## Stage4T 瞬态、EKF 与定位恢复边界（2026-07-15）

Stage4T 已完成 200 组固定时长瞬态、120 组闭环航向、A/B/C/D 各 5 次 EKF 消融、非零 measurement covariance、双分辨率重建图和正式 Oracle 10-seed 定位。0.25/0.35 rad/s 运行包络与闭环航向通过，0.60 rad/s 开环 stress 失败被保留；最终选择 EKF-B。选中 0.05 m 地图后，Oracle 10/10 次导航成功，但 XY RMSE P50/P95/max 为 0.08397/0.14848/0.16972 m，未达到 0.05 m 硬门，因此 `READY_FOR_GPT_REVIEW_STAGE4T=false`、`READY_FOR_STAGE5A=false`，realistic 全量与完整 Coverage 按协议阻断。发布与审计见 [PR #7](https://github.com/zhexuexiaotudou/TZcup/pull/7)、[`GPT_REVIEW_STAGE4T.md`](GPT_REVIEW_STAGE4T.md) 与 [`artifacts/stage4t_20260715_review/`](artifacts/stage4t_20260715_review/)；原始 MCAP 和失败调优 artifact 在用户确认前保留。

本仓库用于构建、验证和交付基于 ROS 2 Jazzy、Gazebo Harmonic、Nav2、SLAM Toolbox、OpenNav Coverage 与 Fields2Cover 的智慧环卫无人清扫车仿真系统。项目强调可复现构建、真实运行证据、阶段门禁和明确的能力边界。

## 当前状态

- Stage 0–5A 已完成 Windows + Docker + NVIDIA GPU 的 headless 构建与运行验证；Stage5B 已实现学习型感知工具链并冻结在 D1 数据真实性、泛化与颜色鲁棒性失败边界。
- precision mapping 与 localization/coverage 包络分别限制为 0.30/0.25 和 0.45/0.35 m/s、rad/s；0.60 rad/s stress 默认禁用且仍失败。
- Stage4W hybrid 10-seed 的 XY RMSE P50/P95/max 为 0.02825/0.03726/0.03778 m，定位门禁通过且 GT 控制违规为 0。
- 完整 Coverage 静态 5/5 通过，每次均执行统一几何生成的 17/17 组件；动态障碍 20/20、碰撞 0，过滤器、30 次急停和 rosbag 回放全部通过。
- 原生 Ubuntu/WSLg 下的 Gazebo/RViz GUI 验收仍未完成；Stage5B 训练模型已接入真实 Gazebo RGB-D 链路，但该诊断不构成正式精度门，真实数据训练、J6 量化和实板部署仍未启动。
- 理论清扫效率仍为 1053 m²/h，未达到 3500 m²/h；不得用覆盖率或仿真实测净效率替代竞赛效率口径。
- 详细证据、复现命令和已知边界以 [`docs/progress.md`](docs/progress.md) 为准。

## 快速开始

推荐环境：Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic。Windows 可通过 Docker Desktop 执行 headless 阶段门禁，但不能替代原生 Ubuntu 或 WSLg 下的 GUI 验收。

```bash
export SANITATION_WS=$HOME/sanitation_ws
mkdir -p "$SANITATION_WS/src"
rsync -a starter_ws/src/ "$SANITATION_WS/src/"

bash scripts/bootstrap_jazzy.sh
bash scripts/import_upstream.sh
bash scripts/build_ws.sh
bash scripts/run_baseline.sh
```

Windows 上可按阶段运行 Docker 验收脚本：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage1_docker.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage2_docker.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage3_docker.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage4_docker.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage4t_core_smoke_docker.ps1
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage5a_docker.ps1 -OutputName stage5a_formal3 -RecordBag
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\scripts\run_stage5b_docker.ps1 -OutputName stage5b_screening
```

## 开发工作流

本项目所有开发修改统一使用“开发工作流”：核实问题，建立独立分支和 worktree，开发与测试，提交 PR，等待 CI，全绿后合并，部署并检查真实效果，最后用 `neat-freak` 同步文档和 Agent 记忆。汇报后必须等待用户确认，确认前不清理任务分支、worktree 或临时数据。

每个提交都必须复核并同步本文件。`最近同步` 只保留最新状态，不追加提交流水账。CI 会逐个检查 PR 中的提交是否包含 `README.md`。

开始修改前请阅读：

- [`AGENTS.md`](AGENTS.md)：项目级 Agent 规则和强制门禁。
- [`docs/development-workflow.md`](docs/development-workflow.md)：开发工作流的完整定义。
- [`PROJECT_SPEC.md`](PROJECT_SPEC.md)：系统架构、接口和技术约束。
- [`STAGE_GATES.md`](STAGE_GATES.md)：Stage 0–7 的任务与验收条件。
- [`README_FIRST.md`](README_FIRST.md)：环境准备、目录说明和启动细节。

所有修改至少运行快速门禁：

```powershell
py scripts/ci_fast.py
```

Linux 和 CI 使用：

```bash
python scripts/ci_fast.py
```

涉及 ROS 包、launch、URDF/Xacro、SDF、Nav2、SLAM、覆盖规划或运行时行为时，还必须执行受影响的 Stage 门禁。轻量 CI 不能替代真实仿真验收。

## 目录说明

- `starter_ws/src/`：项目自有 ROS 2 包。
- `scripts/`：环境、构建、阶段验收和证据采集脚本。
- `docs/`：进度、开发流程和补充说明。
- `artifacts/`：阶段运行证据；提交前需确认内容可追溯且不含敏感信息。
- `.github/`：PR 模板与“开发工作流”CI。

## 使用边界

- `shumo` 只用于用户明确提出的数学建模问题或数学建模竞赛任务，不用于常规 ROS 2 / Gazebo 工程开发。
- 服务器地址、SSH 私钥、令牌和其他凭据不得提交到仓库，只能放入受保护环境或 GitHub Secrets。
- 不能用“代码已合并”代替“运行时已部署”，也不能用命令返回 0 代替真实节点、话题、日志、JSON、rosbag 或页面效果验收。

## Stage5A 垃圾感知真值闭环、数据集与定点清扫（2026-07-17）

Stage5A 已建立五类垃圾的显式 semantic registry、稳定 UUID、仿真 GT、20-scene RGB-D/COCO 数据、固定契约 ONNX Runtime 后端、2D/3D/map 投影、多帧 tracker 和 deferred spot-clean 状态闭环。正式门包含 ROS 构建/测试、held-out synthetic perception、30-seed 状态闭环、Gazebo 实时 RGB-D 推理、压缩 MCAP，以及 Stage4W 单 seed 完整 Coverage 回归。

当前结论严格限定为 synthetic-domain 工程就绪；`competition_perception_pass=false`、`j6_quantization_pass=false`、`j6_runtime_pass=false`、`competition_efficiency_pass=false`，理论效率仍为 `1053 m²/h < 3500 m²/h`。复核入口为 [`GPT_REVIEW_STAGE5A.md`](GPT_REVIEW_STAGE5A.md)、[`docs/stage5a-garbage-perception.md`](docs/stage5a-garbage-perception.md) 和 `artifacts/stage5a_20260717_review/`。

## 最近同步

2026-07-20：Stage5BR2 已完成指标语义和逐实例尺寸统计纠正，建立从生产 Xacro/launch 派生的 G2 车载相机契约，生成 4 个不同 SHA/材料并按 2/1/1 world-isolated split 分配的训练世界；四世界 RGB-D/semantic GT/instance GT Gazebo Harmonic 话题烟测通过，G2 资产保持真实物理尺寸。80 scene/800 frame 数据、分辨率扫描、双模型筛选及后续正式/live/Nav2/J6 门尚未执行，首个阻断层为 `G2_screening_dataset_80_scene_800_frame_not_executed`，两个 readiness 均为 false；Stage5BR 原始 500 帧及两个任务 worktree 在用户确认前继续保留。
