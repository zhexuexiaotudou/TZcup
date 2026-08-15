# TZcup 仿真产品验收门 V1

本文件是固定 [产品验收规范 V1](docs/product-acceptance-spec-v1.md) 的导航页；机器真值是 [`config/product_acceptance_v1.json`](config/product_acceptance_v1.json)。门槛不得在看到测试结果后降低，任何修改必须创建显式新版本并在 sealed final 首次访问前完成。

## 总判定

下列 A–P 全部 PASS、14 个全局否决项全部为安全值、正式证据与 release 文件全部存在且 SHA-256 匹配，才允许 `SIMULATION_PRODUCT_COMPLETE=true`。缺失、未执行、未知、坏哈希和非零退出都按 FAIL 处理，门之间不能平均抵消。

| Gate | 主题 | 核心硬门 |
|---|---|---|
| A | Vehicle / Simulation Fidelity | 空目标/空 DynamicTrashMap 启动；Ackermann；point turn/零速 yaw/GT subscriber 为 0；清扫宽度 ≥600 mm；箱体 ≥40 L |
| B | Localization & Mapping | XY RMSE/P95 ≤0.05 m；连续建图 ≥20,000 m²；save/load/relocalize/Nav2 实跑 |
| C | Coverage & Efficiency | brush-swept coverage ≥95%；repeat ≤20%；碰撞/keepout 0；全耗时效率 ≥3500 m²/h |
| D | Navigation & Safety | ≥30 seeds；导航成功率 ≥95%；碰撞/越界 0；E-stop ≥30 次、100% safe、P95 ≤0.2 s |
| E | Discrete Perception | eventual proposal ≥0.98；四分类 macro-F1 ≥0.98；逐类 P/R、background、small、ActionVerifier 与零错误 CLEAN_NOW 全通过 |
| F | Area Perception | macro mIoU ≥0.80；boundary F1 ≥0.80；negative actionable FP/frame ≤0.02 |
| G | Tracking & DynamicTrashMap | track recall ≥0.98；identity/分类/投影/map precision/coverage/RMSE/duplicate/stale 全通过 |
| H | Spot Cleaning | ≥30 seeds；mission/confirmed-target cleaning ≥0.90；错误/虚假清扫 0；安全暂停 100% |
| I | Post-Clean Verification | camera-backed CLEANED 100%；false CLEANED 0；Area remaining ≤0.10；最多一次 retry |
| J | Multimodal / LLM | ≥2 模态；命令接收 ≥95%；固定集任务分解 ≥95%；unsafe bypass 0 |
| K | Performance | ≥1200 frames；完整链 ≥10 Hz；15 Hz 输入持续 ≥10 min；P95 ≤200 ms；drop ≤1%；queue bounded |
| L | Reliability / Soak | 完整冻结链 ≥2 h；crash/deadlock/queue growth/reload/TF/watchdog 为 0；memory growth ≤5% |
| M | Fault Injection | 规范列出的传感、时间、感知、provider、模型与 Nav2 故障；unsafe cleaning action 0 |
| N | Replay / Reproducibility | ≥5 bags 真实 `ros2 bag play`；全链重算；关键差异 ≤1%；数据 split 零泄漏 |
| O | Freeze / Release / Supply Chain | diff/CI/ROS/secret 全绿；freeze-before-final；one-shot sealed；release/rollback/license 全通过 |
| P | Competition Mapping | 官方 hard gate 全通过；≥30 integrated seeds；全部正式 domain；实时完整 demo 链 |

## 一票否决

以下任一出现即整体 FAIL：GT 控制、预载目标坐标、错误目标清扫、false-candidate 清扫、false/wrong-class CLEAN_NOW、碰撞、keepout 违规、sealed 泄漏、final 后调参重考、release 模型 hash 不匹配、未披露 CPU fallback、Safety bypass、无 hash 的正式证据。

## 证据要求

每个 Gate 必须有：

```text
machine-readable JSON
human-readable Markdown
raw log
artifact SHA-256
dataset / scenario
source commit
model / config / dataset SHA-256
container digest / dependency lock
seed / command / exit code
```

最终证据根还必须包含规范规定的 freeze、30-seed、post-clean、performance、soak、fault、replay、competition mapping、third-party notices、SBOM、SHA256SUMS 和唯一 release ZIP。

## 运行入口

```powershell
py -3 scripts/ci_fast.py
py -3 scripts/product_acceptance.py validate-contract
py -3 scripts/product_acceptance.py template --output C:\tzcup-evidence\acceptance_evidence_manifest.json
py -3 scripts/product_acceptance.py evaluate `
  --evidence-manifest C:\tzcup-evidence\acceptance_evidence_manifest.json `
  --evidence-root C:\tzcup-evidence `
  --output-dir C:\tzcup-evidence\final
```

`product_acceptance.py` 只裁决证据，不替代仿真运行。当前可执行能力与阻塞项见 [docs/current-status.md](docs/current-status.md)。
