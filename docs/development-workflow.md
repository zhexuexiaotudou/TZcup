# 开发工作流

“开发工作流”是 TZcup 项目所有研发修改的默认交付流程。目标不是只把代码写完，而是把问题核实、隔离开发、测试、评审、合并、部署、真实验收、知识同步和安全清理串成一条可验证的证据链。

## 适用范围

适用于功能开发、缺陷修复、重构、ROS 2 包和 launch 修改、仿真世界或车辆模型修改、评测脚本、CI/CD、部署配置和项目规则修改。只读咨询、诊断、资料研究和写作任务不自动要求创建分支、PR或部署。

`shumo` 只用于用户明确提出的数学建模问题或数学建模竞赛。本项目的常规 ROS 2、Gazebo、导航、覆盖规划、评测和工程开发不调用 `shumo`。

## 流程总览

```text
发任务
→ 理解并核实问题
→ 新建独立分支和 worktree
→ 开发修改
→ 自动测试、必要的回测或页面/仿真检查
→ 推送分支并创建 PR
→ CI 自动验收
→ 合并到 main
→ 部署合并后的修订
→ 检查真实运行效果
→ 使用 neat-freak 同步代码、文档、规则和 Agent 记忆并复盘
→ 汇报最终结果
→ 等待用户确认没有问题
→ 清理开发分支、worktree 和任务专用临时数据
→ 任务结束
```

## 门禁定义

### 1. `problem_verified`：问题已核实

读取项目规则、当前分支和工作区状态，核对 `README_FIRST.md`、`PROJECT_SPEC.md`、`STAGE_GATES.md` 与 `docs/progress.md`。尽可能复现问题或确认现状，并把验收条件写清楚。缺少会改变结果的关键选择时才向用户询问。

### 2. `isolated_workspace_ready`：隔离工作区已建立

先执行：

```powershell
git fetch --prune origin
git status --short --branch
git worktree list
```

从最新、合适的远端基线创建 `agent/<task-slug>` 分支和独立 worktree。主工作区存在未提交实验时必须原样保留，禁止 stash、reset、checkout 或夹带到新任务。

### 3. `implementation_complete`：开发完成

只修改本任务范围内的文件。每个提交都必须复核并更新根目录中文 `README.md`；“最近同步”只保留最新状态，不追加提交流水账。接口、环境变量、数据格式、launch 参数或评测口径变化时，还要同步更新相应架构、运维或验收文档。

### 4. `local_verification_passed`：本地验证通过

所有任务先运行快速门禁：

```powershell
py scripts/ci_fast.py
```

快速门禁执行 Python 编译、JSON/YAML/XML/Xacro/SDF 解析和不依赖 ROS 的算法测试。根据改动类型追加验证：

| 改动 | 最低附加验证 |
|---|---|
| Bash / PowerShell 脚本 | Bash 语法检查、PowerShell 解析检查 |
| 纯算法与指标 | 对应 pytest 和边界用例 |
| ROS 包或构建配置 | `colcon build`、`colcon test`、测试结果检查 |
| URDF/Xacro/SDF/传感器 | Stage 2 Docker 门禁和真实 topic/TF 证据 |
| SLAM/Nav2/安全 | Stage 3 Docker 门禁、地图/导航/急停证据 |
| 覆盖规划与评测 | Stage 4 Docker 门禁、coverage JSON 和 rosbag 元数据 |
| 页面或可视化 | 打开真实页面或渲染结果检查 |

任何轻量检查都不能替代受影响 Stage 的运行时验收。

### 5. `pr_opened`：分支已推送并创建 PR

检查 diff 和敏感信息，只暂存本任务文件，确认本次每个提交都包含 `README.md`，使用简洁提交说明，推送远端分支并创建目标为 `main` 的 Draft PR。PR说明至少包含改动、原因、影响、验收命令、Stage 证据、部署计划与回滚点。

### 6. `ci_passed`：CI 自动验收通过

`.github/workflows/development-workflow.yml` 提供可立即运行的快速 CI，并逐个检查 PR 提交是否同步 `README.md`。失败时读取具体日志，在任务分支修复并重新运行。红灯、必须检查被跳过、状态未知或仍在排队都不能视为通过。

完整 ROS/Gazebo 门禁可在具备 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic、Docker及必要 GPU能力的受控服务器上执行。首次接入服务器前必须确认 SSH身份、主机指纹、运行目录、Runner标签、Secrets、健康检查、备份与回滚；这些信息不能写入公开仓库。

### 7. `main_updated`：已合并主分支

CI和必需的 Stage 证据全部通过后，按仓库策略合并PR。随后拉取或查询远端 `main`，确认合并提交与预期文件真实存在。

### 8. `deployed`：合并版本已部署

本项目按产物类型区分：

- 纯文档或CI规则：没有运行时产物，标记 `not_applicable`，以远端 `main` 和CI结果作为发布证据。
- ROS/Gazebo运行时：把合并后的精确提交部署到批准的仿真机、CI Runner或实车目标，记录提交SHA、镜像/工作空间、部署时间和回滚点。

不能因为“代码已合并”而宣称运行时已经部署。

### 9. `production_verified`：真实效果已检查

在目标环境检查受影响的节点、话题、action、service、TF、日志、JSON报告、rosbag元数据和关键用户路径。必须验证实际运行结果；部署命令返回0不是充分证据。非运行时变更以远端页面、PR、CI和文档渲染检查作为真实效果证据。

### 10. `knowledge_synced`：知识与规则已同步

线上或目标环境验收后运行 `neat-freak`（`/neat`）：

- 让 README/docs 与当前代码、接口、命令和证据一致；
- 审计 `AGENTS.md`、必备文件、路径引用和项目规则是否真实执行；
- 把可复用事实写入正确的项目文档，把跨会话偏好或非显然教训写入 Agent 记忆；
- 删除或修正过期、重复、矛盾的说明。

若同步产生仓库文件变化，必须另走补充PR与CI；只有运行时产物变化时才重新部署。

### 11. `final_reported`：最终结果已汇报

报告分支、PR、提交、CI、Stage证据、部署目标与修订、真实验收、知识同步、残余风险和回滚点。未完成的门禁必须写明阻塞原因。

### 12. `awaiting_user_confirmation`：等待用户确认

最终汇报后保留任务分支、worktree、任务专用仿真工作空间、临时数据库、日志和证据。没有用户明确确认“没有问题”时不得清理。

### 13. `cleanup_complete`：安全清理完成

用户确认后先检查是否存在未提交修改，再删除任务 worktree、本地和远端任务分支以及任务专用临时数据。不得删除共享数据、仍用于回滚的镜像或无法确认归属的证据。

## CI 与服务器接入边界

GitHub 托管 Runner 先执行快速、确定性的 CI。具备完整 ROS/Gazebo/GPU 条件的服务器可后续注册为带项目专用标签的 self-hosted Runner，或作为受保护部署环境。服务器接入属于独立基础设施任务，必须单独验证权限、Secrets、隔离、并发、磁盘清理和回滚，不能只凭公网 IP 宣称接入完成。

## 最终汇报模板

```text
开发工作流结果
- problem_verified:
- isolated_workspace_ready:
- implementation_complete:
- local_verification_passed:
- pr_opened:
- ci_passed:
- main_updated:
- deployed:
- production_verified:
- knowledge_synced:
- residual_risks:
- rollback_point:
- current_gate: awaiting_user_confirmation
```
