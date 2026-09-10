# A20 冻结、回放、发布与回滚回执

机器可读合同是
[`config/high_fidelity_vehicle/a20_release_replay_receipt_contract.json`](../config/high_fidelity_vehicle/a20_release_replay_receipt_contract.json)。
当前 A20 的 canonical producer 已实现，但仍是 **NOT_RUN_CURRENT_SNAPSHOT**，不是 PASS。
[`formal_product_mcap_replay.py`](../scripts/formal_product_mcap_replay.py) 会在独立
`ROS_DOMAIN_ID` 且 `ROS_LOCALHOST_ONLY=1` 的环境实际执行 `ros2 bag play`，再由
`rosbag2_py.SequentialReader` 直接读取同一 bag，重算 coverage、localization（相对源报告
关键指标误差不超过 1%）及完整产品链消息汇总，并绑定当前 RUNNING formal session、snapshot、
source commit/tree 和 VERIFIED runtime closure。它不启动 Gazebo，也不取得全局 Gazebo 锁。

现有 `auto02_replay_audit.py`、`auto03_replay_audit.py` 与
`coverage_mcap_replay_audit.py` 分别只覆盖历史 AUTO-02、AUTO-03 或 coverage 任务；它们不能
证明完整正式产品链，仍不能作为 A20 replay。A20 validator 只接受至少五个不同 MCAP 的
canonical replay 文件引用，并重新核对 replay producer 源码哈希、bag 内容哈希、所有重算门、
session start、snapshot、closure 与 model/config/dataset/dependency 身份；手写嵌入 JSON、旧
AUTO-16 release/SBOM 或历史回放报告仍返回 `A20_RECEIPT_STATIC_BLOCKED`。

解除运行阻断仍至少要求五个不同的、非符号链接、仓库受控路径内的真实 MCAP；每个 bag 都要
由 canonical producer 实际读取并重算上述完整链。A20 receipt 还必须绑定当前 COMPLETE
session/snapshot/closure、source/model/config/dataset/dependency、ZIP、
SHA256SUMS、SBOM、容器/许可/依赖锁以及按 [`docs/rollback.md`](rollback.md) 真实演练的 rollback
报告。validator 会重新散列这些实际文件，并拒绝 historical AUTO-16 reuse、路径逸出、符号链接
与 receipt 输入/输出 TOCTOU。

receipt CLI 保留安全的 fail-closed 边界：它只支持同时具备
`O_NOFOLLOW`/`O_DIRECTORY`、`open`/`stat`/`link`/`unlink` 的 `dir_fd`，以及 `stat`/`link`
不跟随链接能力的 POSIX/WSL CPython；缺少任一能力（包括 Windows CPython）都会在路径处理前
稳定地拒绝，不会半执行。repository root、receipt 和 output 必须为绝对、仓库内、非符号链接
路径；output 必须尚不存在，并以原子文件写入。只有真实回放、发布包和 rollback 全部满足时
返回零；producer 存在本身不改变 A20 状态。
