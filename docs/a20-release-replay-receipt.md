# A20 冻结、回放、发布与回滚回执

机器可读合同是
[`config/high_fidelity_vehicle/a20_release_replay_receipt_contract.json`](../config/high_fidelity_vehicle/a20_release_replay_receipt_contract.json)。
当前 A20 **BLOCKED**，不是静态 PASS：仓库没有一个 canonical formal-product MCAP
replay producer 能同时从 bag 重算 coverage、localization（关键指标误差不超过 1%）、
session/snapshot 与 runtime-closure binding。

现有 `auto02_replay_audit.py`、`auto03_replay_audit.py` 与
`coverage_mcap_replay_audit.py` 分别只覆盖历史 AUTO-02、AUTO-03 或 coverage 任务；它们不能
证明完整正式产品链。因此 A20 validator 对任何 receipt（包括五个手写 hash、嵌入 JSON、旧
AUTO-16 release/SBOM 或历史回放报告）均返回 `A20_RECEIPT_STATIC_BLOCKED`。这避免将缺失的
真实 artifacts、Gazebo runtime 或 release/rollback 演练伪装成通过。

未来解除阻断前，必须新增并独立验证一个 producer，至少要求五个不同的、非符号链接、仓库
受控路径内的真实 MCAP；每个 bag 都要由 canonical audit 实际读取并重算上述完整链。然后
receipt 才能绑定当前 snapshot/session/closure、source/model/config/dataset/dependency、ZIP、
SHA256SUMS、SBOM、容器/许可/依赖锁以及按 [`docs/rollback.md`](rollback.md) 真实演练的 rollback
报告。该 producer 还必须拒绝 historical AUTO-16 reuse、路径逸出、符号链接与 read/write TOCTOU。

CLI 目前仅为上述未完成 contract 保留安全的 fail-closed 边界：它只支持具备 `dir_fd` 与
`O_NOFOLLOW` 的 POSIX/WSL CPython；Windows CPython 会在解析后的第一项安全检查稳定地拒绝，
不会半执行。repository root、receipt 和 output 必须为绝对、仓库内、非符号链接路径；output
必须尚不存在，并以原子文件写入。它仍会以非零退出，直到 canonical producer 可用。
