# 自主控制面状态

本目录只保存 AUTO-00–AUTO-16 控制面的机器可读计划与状态：

- `AUTONOMOUS_RUN_PLAN.json`：由 `../autonomous_stage_registry.yaml` 推导的 DAG 计划；
- `AUTONOMOUS_STATE.json`：阶段状态、证据索引和不可伪造的历史边界。

两份文件由 `scripts/autonomous_runner.py` 及各阶段 finalizer 读取或更新，不应复制到仓库根目录。修改 registry 后运行 `py -3 scripts/verify_state_invariants.py`，确保计划、状态与依赖合同一致。
