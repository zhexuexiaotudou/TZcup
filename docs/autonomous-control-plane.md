# AUTO-00 自主控制面

本控制面把 `AUTO-00` 至 `AUTO-16` 表达为可验证的有向无环图（DAG），用于持续推进导航、感知、机械臂、交互、大地图、效率、真实域与 J6 等 lane。它只把机器实际通过的阶段写为 `PASS`，不会把缺少命令、缺少资源或尚未执行解释为成功。

## 权威文件

- `config/autonomous_stage_registry.yaml`：阶段、依赖、可选依赖、lane 和执行命令。
- `AUTONOMOUS_STATE.json`：当前运行状态和不可伪造的历史边界。
- `AUTONOMOUS_RUN_PLAN.json`：由 registry 推导的拓扑层级。
- `scripts/autonomous_runner.py`：校验、调度、锁、断点续跑、幂等复用和证据生成。
- `scripts/autonomous_git_adapter.py`：显式 `--execute` 保护下的 push、开 PR 和绿灯合并适配器。
- `scripts/verify_state_invariants.py`：状态/计划一致性与历史人工标志保护。
- `scripts/verify_evidence_manifest.py`：证据逐文件字节数、SHA-256 与覆盖率校验。
- `scripts/scan_secrets.py`：提交前常见凭据签名扫描。

规划输入基线为远端 `main` 的 `ac6d5697427425c438ff0f42780ff6ab772226f9`。历史 Stage4W 至 Stage5BR6W review evidence 是只读输入；Stage5BR6-A 的 `human_review_completed=false`、`manual_audit_pass=false` 也必须保持不变。

## 使用

Windows 当前安装的是 Python Launcher：

```powershell
py -3 scripts/autonomous_runner.py validate
py -3 scripts/autonomous_runner.py status
py -3 scripts/autonomous_runner.py plan
py -3 scripts/autonomous_runner.py finalize-auto00
py -3 scripts/verify_state_invariants.py
py -3 scripts/verify_evidence_manifest.py artifacts/<evidence-dir>
py -3 scripts/scan_secrets.py
```

Linux/CI 将 `py -3` 换成 `python`。

`run-stage` 只执行 registry 中显式配置为 argv 列表的命令。命令为空时返回 `NO_COMMAND` 并保留 `PENDING`；依赖未通过时返回 `DEPENDENCY_BLOCKED`；已有 `PASS` 且 manifest 完整时返回 `SKIPPED_EXISTING_PASS`，从而避免重复破坏证据。每次执行使用独占锁和原子状态写入。

## 证据合同

每阶段目录固定为 `artifacts/autonomous_<stage>_<UTC>_evidence/`，至少包含 `stage_status.json`、`stage_config.yaml`、`attempt_ledger.json`、`environment.json`、`commands.txt`、`metrics_summary.json`、`raw_metric_index.json`、`regression_summary.json`、`README.md` 和 `artifact_manifest.json`。

manifest 不自我哈希，其余文件必须 100% 列入并匹配精确字节数与 SHA-256。`null` 表示未执行或未知，不得改写成 `0`。仿真、真实域、J6、离线、在线和 Oracle 证据必须分级；Oracle 不能成为控制输入或竞赛证据。

## 失败与外部资源

失败只阻断依赖该阶段的后继，不阻断独立 lane。真实数据、J6 官方工具链、J6 实板、物理车和物理机械臂必须通过资源探测填写；不可用时使用 `BLOCKED_EXTERNAL` 或对应 final-state 标志，不能伪造结果。
