# TZcup 无人清扫车仿真项目

## Stage4U 坐标标定、定位地图与 5 cm 闭环（2026-07-16）

Stage4U 已修复 map/map_gt 评测语义、`ParticleCloud` 类型/QoS 和地图质量假通过，并完成 M1/M2/M3、AMCL/SLAM Toolbox、LiDAR/AMCL 灵敏度及正式 Oracle 10-seed。最优候选为结构化 v2 surveyed reference 0.02 m + AMCL + 360@10 Hz；10/10 导航成功、TF 全连续、粒子仪器全有效、恢复 0 次，但 map-relative XY RMSE P50/P95/max 为 `0.06767/0.07983/0.08022 m`，未过 0.05 m 硬门。因此 `READY_FOR_GPT_REVIEW_STAGE4U=false`、`READY_FOR_STAGE5A=false`，realistic 与完整 Coverage 按停止条件未执行。复核入口见 [`GPT_REVIEW_STAGE4U.md`](GPT_REVIEW_STAGE4U.md) 与 [`artifacts/stage4u_20260716_review/`](artifacts/stage4u_20260716_review/)；原始 MCAP/posegraph 在用户确认前保留。

## Stage4T 瞬态、EKF 与定位恢复边界（2026-07-15）

Stage4T 已完成 200 组固定时长瞬态、120 组闭环航向、A/B/C/D 各 5 次 EKF 消融、非零 measurement covariance、双分辨率重建图和正式 Oracle 10-seed 定位。0.25/0.35 rad/s 运行包络与闭环航向通过，0.60 rad/s 开环 stress 失败被保留；最终选择 EKF-B。选中 0.05 m 地图后，Oracle 10/10 次导航成功，但 XY RMSE P50/P95/max 为 0.08397/0.14848/0.16972 m，未达到 0.05 m 硬门，因此 `READY_FOR_GPT_REVIEW_STAGE4T=false`、`READY_FOR_STAGE5A=false`，realistic 全量与完整 Coverage 按协议阻断。发布与审计见 [PR #7](https://github.com/zhexuexiaotudou/TZcup/pull/7)、[`GPT_REVIEW_STAGE4T.md`](GPT_REVIEW_STAGE4T.md) 与 [`artifacts/stage4t_20260715_review/`](artifacts/stage4t_20260715_review/)；原始 MCAP 和失败调优 artifact 在用户确认前保留。

本仓库用于构建、验证和交付基于 ROS 2 Jazzy、Gazebo Harmonic、Nav2、SLAM Toolbox、OpenNav Coverage 与 Fields2Cover 的智慧环卫无人清扫车仿真系统。项目强调可复现构建、真实运行证据、阶段门禁和明确的能力边界。

## 当前状态

- Stage 0–4T 已完成 Windows + Docker + NVIDIA GPU 的 headless 构建与运行验证；当前车辆参数为 0.14 m 轮半径、1.22 m 有效轮距，融合配置选择 EKF-B。
- precision mapping 与 localization/coverage 包络分别限制为 0.30/0.25 和 0.45/0.35 m/s、rad/s；0.60 rad/s stress 默认禁用且仍失败。
- 0.05 m 地图通过基础质量门，但 Oracle 10-seed 的 XY RMSE P50/P95/max 为 0.08397/0.14848/0.16972 m，第一失败层为 `oracle_localization_pass`。
- realistic 全量定位、完整 Coverage、动态障碍、完整任务急停与 rosbag 回放按停止条件未执行；理论清扫效率仍为 1053 m²/h，未达到 3500 m²/h。
- 原生 Ubuntu/WSLg 下的 Gazebo/RViz GUI 验收仍未完成；进入感知训练或 J6 量化前必须先解决定位硬门。
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

## 最近同步

2026-07-16：Stage4U 已由 [PR #9](https://github.com/zhexuexiaotudou/TZcup/pull/9) 合并到 `main@efd5e34cbb3c8ba1016118c63a6e35402704e787`，`fast-validation` 通过；合并树 `00f2b33c5866025421bc5e9bea224945b58eafbd` 的真实 Gazebo core smoke 再验通过 covariance 与运行包络。正式 Oracle 10-seed 虽 10/10 有效，但 XY RMSE P50/P95/max `0.06767/0.07983/0.08022 m` 未过 0.05 m，`READY_FOR_GPT_REVIEW_STAGE4U=false`。
