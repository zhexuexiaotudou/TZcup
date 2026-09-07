# A19 两小时长稳、故障注入与恢复合同

`config/high_fidelity_vehicle/formal_a19_reliability_fault_contract.json` 单独定义 A19，当前状态固定为 `BLOCKED_MISSING_CANONICAL_A19_RUNTIME_PRODUCER`。它不改变 A12、AUTO-15 或任何标定结论。

产品标准 `docs/product-acceptance-spec-v1.md` §36–37（commit `e1d900f`，尚未合入当前 main）要求冻结后的 Coverage、Perception、Tracking、DynamicTrashMap、Spot Cleaning 和 Post-Clean Verification 连续运行不少于两小时。它要求 crash/deadlock/持续队列增长/意外模型重载/持续 TF 故障/不可恢复 watchdog/不安全清扫均为零、内存增长不超过 5%，定位 RMSE/P95 均不超过 50 mm，并覆盖 nominal、transport_stress、wet_surface、degraded_drive 以及 18 个明确故障。

当前 main 没有规范 A19 runtime producer 或 receipt schema：没有任何收据能同时绑定同提交的 snapshot/session/runtime closure、可解析的实时连续 wall-clock 序列、启动命令、退出码、零 survivor、全故障注入记录及基于时间戳的 STOPPED/RECOVERED 状态。因此校验器不会把任意“路径 + 哈希 + size”文件解释为上述语义，也刻意不会产生 PASS；手写 JSON、65 秒 water soak 或历史产物都只能得到 BLOCKED。这是缺少生产器的真实边界，不是运行通过结论。

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

它拒绝覆盖已有输出，并对 contract/report/snapshot/session/closure 及 output parent 做规范化路径、无 symlink ancestor、resolve 后仍 in-root、regular-file、稳定双读和最终重读检查。现有 65 秒 `run_formal_water_safety_soak.sh` 输出只能保留为独立水安全证据，不能作为 A19 输入或改标为两小时通过。
