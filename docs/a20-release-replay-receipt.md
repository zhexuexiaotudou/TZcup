# A20 冻结、回放、发布与回滚回执

机器可读合同是
[`config/high_fidelity_vehicle/a20_release_replay_receipt_contract.json`](../config/high_fidelity_vehicle/a20_release_replay_receipt_contract.json)，
静态校验入口是 `scripts/a20_release_replay_receipt.py`。它不取代正式运行编排，也不修改
AUTO-16 的历史状态文件。

一份 A20 receipt 必须同时提供：

- `formal_acceptance_session.py` 产生且已 `FORMAL_FINAL_ACCEPTANCE_SESSION_COMPLETE`
  的 sealed final session，以及该 session 的三项 frozen snapshot identity；
- `formal_final_runtime_closure.py verify` 的当前
  `FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED` record，且 digest 与 sealed session 的
  `runtime_closure_binding` 一致；
- source、model、config、dataset、dependency 五项 SHA-256；五个以上不同的 product
  MCAP bag SHA-256，且每一项均绑定上述 hashes、snapshot 与 closure，并携带
  `coverage_mcap_replay_audit.py` 的完整 passing audit；
- 已记录的 main release commit、ZIP SHA-256、SBOM SHA-256，以及按
  [`docs/rollback.md`](rollback.md) 执行并验证的精确 rollback commit 和验证报告 SHA-256。

例如，在所有真实记录已经由正式流程保留后：

```bash
python3 scripts/a20_release_replay_receipt.py \
  --receipt /external-evidence/a20_receipt.json \
  --output /external-evidence/a20_receipt_validation.json
```

同一个 bag digest 重复出现、少于五个 product replay、任一输入 hash 缺失、session 未封存、
closure 已漂移，或 rollback exercise 未验证，都会返回非零。校验器只检查提供 JSON 的一致性；
即使返回 `A20_RECEIPT_STATIC_VALID`，也明确输出 `release_runtime_pass: false`。因此它不能证明
ZIP 当前存在、Gazebo 实际运行过，或任何产品/竞赛门已经通过；这些结论仍由原始 artifact、CI 和
正式 runtime 验收分别给出。
