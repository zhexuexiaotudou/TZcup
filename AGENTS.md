# TZcup 项目规则

## 开发工作流

- 本项目的端到端研发流程统一命名为“开发工作流”，权威说明见 `docs/development-workflow.md`。
- 所有代码、配置、CI/CD 和部署修改必须遵守全局开发门禁以及本文件的项目约束。
- 当前主工作区可能保留竞赛阶段的未提交实验；开始新任务时先检查状态，并从最新远端基线建立独立分支和独立 worktree，禁止覆盖或夹带其他任务的改动。
- 发布分支使用 `agent/<task-slug>`，PR目标为 `main`。只显式暂存本任务文件。
- 每个提交都必须复核并同步中文 `README.md`；即使项目状态未变化，也要更新其中的“最近同步”以说明本次提交和当前门禁。该区域只保留最新状态，不追加提交流水账。
- `shumo` 仅在用户明确提出数学建模问题或数学建模竞赛任务时使用。本项目默认是 ROS 2 / Gazebo 机器人仿真工程，不自动触发 `shumo`。

## 开始任务前必读

- `README.md`：中文项目入口、当前状态、快速开始和最近同步。
- `README_FIRST.md`：环境、目录和启动入口。
- `PROJECT_SPEC.md`：系统架构与接口边界。
- `STAGE_GATES.md`：Stage 0–7 的任务与验收条件。
- `docs/progress.md`：当前阶段、证据和已知边界。
- `docs/development-workflow.md`：分支、PR、CI、部署、线上验收和清理流程。

## 验证要求

- 所有改动先运行 `py scripts/ci_fast.py`；Linux/CI 使用 `python scripts/ci_fast.py`。
- Bash 脚本改动运行 `bash -n scripts/*.sh`；PowerShell 脚本改动至少做解析检查。
- 纯算法改动运行对应 pytest；快速 CI 当前覆盖不依赖 ROS 的 coverage metrics 测试。
- ROS 包、launch、URDF/Xacro、SDF、Nav2、SLAM、覆盖规划或运行时改动，必须运行受影响的 `scripts/stageN_ci.sh`，Windows 优先使用对应 `scripts/run_stageN_docker.ps1`。完整仿真门禁不能被语法检查或轻量 CI 替代。
- UI/可视化改动必须检查真实渲染；服务或仿真改动必须等待就绪并验证真实话题、节点、日志和 JSON 证据。

## PR、部署与收尾

- PR必须通过 `.github/workflows/development-workflow.yml` 中的“开发工作流”CI；CI 会逐提交检查 `README.md` 是否同步，受影响的 Stage 运行证据写入 PR说明或可追溯的 artifact。
- CI全绿后才能合并。合并后确认远端 `main` 包含预期提交。
- 本仓库不是常驻 Web 服务。纯文档/CI变更可将部署标记为 `not_applicable`；运行时变更的“部署”是把合并后的修订部署到已批准的 ROS 2/Gazebo 运行环境或 CI服务器，并执行对应 Stage 门禁。服务器地址、密钥和令牌只放在受保护环境或 Secrets 中，不提交到仓库。
- 真实验收完成后运行 `neat-freak`（`/neat`），同步代码事实、README/docs、项目规则与持久记忆。若产生版本化修改，必须走补充PR和CI。
- 汇报后等待用户明确确认。确认前保留分支、worktree、临时数据库、仿真工作区和验收证据；确认后再安全清理。
