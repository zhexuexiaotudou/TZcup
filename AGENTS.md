# TZcup 项目规则

## 开发工作流

- 权威流程见 `docs/development-workflow.md`；代码、配置、CI/CD 和部署修改同时遵守全局开发门禁。
- 主工作区可能保留未提交实验。新任务先检查状态，从合适的远端基线建立 `codex/<task-slug>` 分支和独立 worktree，禁止覆盖或夹带其他任务改动。
- PR 目标为 `main`，只显式暂存本任务文件。CI 与受影响的 Stage 门全部通过后才允许合并。
- `shumo` 只在用户明确提出数学建模问题或竞赛任务时使用；常规 ROS 2 / Gazebo 开发不触发。

## 文档纪律

- README 只说明项目定位、主要能力、使用入口和当前产品边界，不记录逐次变更。
- `docs/current-status.md` 只保留当前有效状态；更新时直接替换失效结论，不追加日期、轮次或提交日志。
- 专题文档描述稳定的设计、协议、操作和验收口径。开发过程、失败尝试和提交历史由 Git、PR 与紧凑 evidence 记录。
- 接口、环境变量、数据格式、launch 参数、评测口径或能力边界变化时，同步更新对应文档，并检查引用是否仍然有效。

## 开始任务前必读

- `README.md`：项目入口和产品边界；
- `README_FIRST.md`：环境与启动；
- `PROJECT_SPEC.md`：系统架构和接口；
- `STAGE_GATES.md`：验收门定义；
- `docs/current-status.md`：当前状态和阻塞项；
- `docs/development-workflow.md`：分支、PR、CI、部署、验收和清理流程。

## 验证要求

- 所有改动先运行 `py -3 scripts/ci_fast.py`；Linux/CI 使用 `python scripts/ci_fast.py`。
- Bash 脚本改动执行语法检查；PowerShell 脚本改动至少执行解析检查；纯算法改动运行对应 pytest。
- ROS、launch、URDF/Xacro、SDF、Nav2、SLAM、覆盖规划或运行时改动必须运行受影响的 Stage 门。快速 CI 不能替代完整仿真验收。
- UI/可视化改动检查真实渲染；服务或仿真改动等待就绪后验证真实话题、节点、日志和 JSON 证据。

## 交付与清理

- CI 全绿后才能合并；合并后确认远端 `main` 包含预期提交。
- 纯文档或 CI 变更可将部署标为 `not_applicable`。运行时变更必须部署精确修订并验证真实路径。
- 真实验收后运行 `neat-freak`，使代码、README、专题文档和项目规则一致；产生的版本化修改继续走 PR 与 CI。
- 最终汇报后等待用户确认。确认前保留任务分支、worktree 和验收证据；确认后只清理归属明确的任务资源。
