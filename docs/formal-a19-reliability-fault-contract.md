# A19 两小时长稳、故障注入与恢复合同

`config/high_fidelity_vehicle/formal_a19_reliability_fault_contract.json` 单独定义 A19，当前状态固定为 `BLOCKED_UNTIL_CURRENT_SESSION_RUNTIME_EVIDENCE`。它不改变 A12、AUTO-15 或任何标定结论。

候选采集报告须提供一个不少于 7200 秒的窗口，并绑定当前 `formal_final_acceptance_session.json`、冻结 runtime closure、原始遥测、启动日志和故障注入收据的 SHA-256。遥测中的覆盖率、碰撞数、定位 P95 和急停制动延迟必须分别满足已有 SIL/赛题口径；`transport_stress`、`wet_surface` 和 `degraded_drive` 均须记录 `STOPPED_SAFE` 与 `RECOVERED_SAFE`。

离线核验：

```bash
python3 scripts/validate_formal_a19_reliability_fault.py \
  --report <collector-report.json> \
  --snapshot reports/engineering/formal_vehicle_snapshot_manifest.json \
  --acceptance-session artifacts/formal_final_acceptance_session.json \
  --runtime-closure <final-runtime-closure-manifest.json> \
  --evidence-root <fresh-a19-evidence-directory> \
  --output <fresh-a19-validation.json>
```

它拒绝覆盖已有输出、会校验相对证据路径及哈希，并在缺失、会话/closure 漂移、短于两小时、注入记录不完整或安全结果不合格时输出 `A19_TWO_HOUR_RELIABILITY_FAULT_BLOCKED`。现有 65 秒 `run_formal_water_safety_soak.sh` 输出只能保留为独立水安全证据，不能作为 A19 输入或改标为两小时通过。
