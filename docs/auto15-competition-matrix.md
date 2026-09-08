# AUTO-15：全竞赛场景正式矩阵

## 结果

AUTO-15 已完成 18 类场景的机器可读需求、依赖和组件证据索引，但没有执行综合正式任务。首个阻断层为：

```text
dependency_AUTO-08_learned_spot_cleaning_blocked
```

AUTO-08 依赖 AUTO-07，而 AUTO-07 又因 AUTO-05 三次 G3 模型 screening 未通过而阻断。因此离散垃圾识别、落叶堆、积水和学习感知定点清扫不能进入综合矩阵。J6 runtime 也因 AUTO-14 未获得正式模型和实体板而阻断。

此外，AUTO-15 的 runtime receipt intake 为 `BLOCKED_NO_CANONICAL_PRODUCER`：仓库没有一个受总编排接线的 producer 能从当前 formal session 原子地产出新鲜、不可复用的每执行 video、MCAP、真实 replay/recalculation、provenance 和独立 mission-group receipt。因此任何手写 JSON、`ftyp` 填充文件或 MCAP magic bytes 都不会被当作运行证据。未来接线必须保留 `18×10=180` 个唯一 execution、每个 video/MCAP、以及至少 `30` 个有成员收据的独立 mission group；未实现前 AUTO-15 保持 BLOCKED。

## 证据口径

`competition_matrix.json` 覆盖全部 18 类场景，并分别记录：

- required stages 与当前状态；
- 组件证据是否存在；
- 综合执行状态；
- seed、mission、video、MCAP 数量；
- 首个阻断依赖；
- 不得由组件证据外推综合成绩的边界。

APP、语音、LLM DSL、大地图、定时任务、效率、安全导航和离线抓取的独立证据仍然有效，但它们没有被写成 AUTO-15 integrated mission。

## 冻结状态

```text
AUTO-15=BLOCKED
SIMULATION_COMPETITION_MATRIX_PASS=false
scenario_count=18
executed_seeds_per_scenario=0
executed_integrated_missions=0
formal_video_count=0
formal_mcap_count=0
```

紧凑证据位于
[`artifacts/autonomous_auto15_20260730_evidence/`](../artifacts/autonomous_auto15_20260730_evidence/)。
