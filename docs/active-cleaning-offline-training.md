# Active cleaning 离线训练与恢复

本入口只验证 Python 环境中的 belief-only 学习与配对评测，不代表 ROS/Gazebo、真实感知、机械臂或竞赛成绩通过。正式预算仍以 `starter_ws/src/sanitation_active_cleaning/config/formal_rl_budget_contract.yaml` 为唯一来源：Stage-A 为 10,000/500/1,000 task，多场地为 6,400/800/1,200 task，每套均使用五个冻结策略 seed。400 step 是单 episode 截断保护，不是 task 预算。

## 公开 smoke 与吞吐

在仓库根目录运行：

```powershell
py -3 scripts/smoke_active_cleaning_training.py --output-root .workspace/active-cleaning-smoke-new --max-steps 8
```

输出目录必须不存在。脚本仅物化公开 train/val 的 map 0、mission 0，执行一次训练及相同验证任务的全覆盖/学习策略对照；禁止选择 hidden。报告保存每次物化耗时、实际动作步数、吞吐、截断/失败结果及源码哈希。原始生成文件和日志保留在忽略目录；`formal_acceptance=false` 始终不变。允许的 smoke 上限为 40 step，不可用该入口申请正式成绩。

2026-09-11 Windows CPU 的诊断样本：

| 上限 | 训练墙钟 | 验证墙钟（基线+策略） | 单 task 物化 | 状态 |
|---|---:|---:|---:|---|
| 8 step | 1.24 s | 3.26 s | 6.81–7.20 s | 全部截断 |
| 40 step | 5.45 s | 59.52 s | 6.74–7.06 s | 全部截断 |

40 步样本约为训练 7.34 动作步/s、配对验证 1.34 动作步/s；这是共享主机上的有界观测。验证成本增长明显，且没有完成 episode，因此不提供全量 400 步或整个正式预算的预计完成时间。此测量也不包含完整预算所需的磁盘/内存开销。先保留失败与恢复证据，再申请专用资源执行完整公开预算，不能通过降低冻结预算得到正式通过。

## 训练和验证，不打开隐藏集

安装/配置 Python package 搜索路径后，可运行以下模块（`--help` 列出路径参数）。例如以下命令准备运行完整多场地公开预算，耗时远高于 smoke；本次没有执行该完整命令：

```powershell
$env:PYTHONPATH = (Get-ChildItem starter_ws/src -Directory | ForEach-Object FullName) -join [IO.Path]::PathSeparator
py -3 -m sanitation_active_cleaning.formal_training --train-validation-only --scenario-config starter_ws/src/sanitation_campus_scenario/config/default_scenario.yaml --motion-profile config/high_fidelity_vehicle/formal_motion_cleaning_profile.yaml --budget-contract starter_ws/src/sanitation_active_cleaning/config/formal_rl_budget_contract.yaml --policy-seeds 7,17,29,43,61 --work-root .workspace/public-training/tasks --evidence-root .workspace/public-training/evidence
```

- 多场地：`python -m sanitation_active_cleaning.formal_training --train-validation-only ...`。使用 `--budget-contract` 时必须同时提供 `--policy-seeds 7,17,29,43,61`、默认 400 step 和一次完整训练遍历；不可用 `--test` 混入隐藏任务。
- Stage-A：`python -m sanitation_active_cleaning.formal_stage_a_training --phase train-validation ...`。仍要求有效的 canonical snapshot/session，且完整消费冻结的公开 task；不要求 hidden receipt root，不提交 hidden freeze 或生成 hidden task。

两者结束只表示公开训练/验证已执行，不代表策略质量或最终正式验收通过。验证失败、截断与策略异常必须保留在结果中，不删除差样本。多场地每个 seed 的报告和 `training_state.json` 位于 `evidence-root/formal_planning/seed-N/`；Stage-A checkpoint 位于 `work-root/training_checkpoints/policy-N.json`。

## 断点与冻结

训练在每个完成 episode 边界原子替换 checkpoint，恢复时验证输入、源码、策略 seed、完整性摘要和已完成 episode 顺序。中断的 episode 从上一个完成边界重跑，不继续使用部分更新的 Q 表。Stage-A 同时绑定预算及 snapshot/session 内容。配置、源码或任务漂移必须使用新 run-root；不能修改旧 checkpoint 摘要绕过检查。失败记录保留类型和任务标识，不保存任意异常文本。

评测使用训练表的独立副本，不能因访问未见状态而修改冻结策略。多场地完整路径先完成全部策略 seed 的训练/验证，记录各 Q 表 SHA-256 和 checkpoint 选择规则，再通过既有受控隐藏物化入口。隐藏评测前要求完整且哈希一致的训练状态；缺失或损坏时拒绝，不能临时重训。最终隐藏入口不属于 smoke 或日常诊断；本次未执行。

当前恢复粒度仅覆盖训练 episode。验证中断会重新执行公开验证；隐藏消费仍受单次 receipt 约束，不能自动重开、重试或依据隐藏结果调参。完整预算物化目前仍集中准备任务对象，资源占用需在专用运行环境中实测。

## 评测与验收边界

配对评测拒绝重复/非整数 seed、重复策略名或空策略集合。策略 reset/action 异常计为失败并保存已执行轨迹；环境基础设施错误继续向调用方抛出，不能伪装为有效 rollout。汇总保留全部失败数、均值、样本标准差、置信区间和尾部指标；区间沿用 `1.96 × 样本标准差 / √n` 的正态近似，并不保证小样本精确覆盖。少于两个样本时标准差为 null，兼容数值区间字段仅为占位，`ci95_estimable=false`。距离的最差值为最大值。

正式多场地门必须覆盖每张地图的全部冻结 mission；只覆盖 32/8/12 张地图的 mission 0 仍是研究 smoke。独立 Python 单元测试与本机 smoke 不能替代当前 source/session 的 Gazebo 正式门，也不能替代最终隐藏集、S100P 或实车验证。
