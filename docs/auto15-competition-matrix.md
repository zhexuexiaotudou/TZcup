# AUTO-15：全竞赛场景正式矩阵

## 结果

AUTO-15 已完成 18 类场景的机器可读需求、依赖和组件证据索引，但没有执行综合正式任务。首个阻断层为：

```text
dependency_AUTO-08_learned_spot_cleaning_blocked
```

AUTO-08 依赖 AUTO-07，而 AUTO-07 又因 AUTO-05 三次 G3 模型 screening 未通过而阻断。因此离散垃圾识别、落叶堆、积水和学习感知定点清扫不能进入综合矩阵。J6 runtime 也因 AUTO-14 未获得正式模型和实体板而阻断。

AUTO-15 现有两个 canonical producer：`formal_product_mcap_replay.py` 在隔离 ROS domain 中实际执行 `ros2 bag play`，并直接读取 MCAP 重算 coverage、localization、observation、track/DynamicTrashMap、scheduler、clean decision 与 post-clean 汇总；`auto15_product_evidence.py` 对每次执行的视频做 `ffprobe` 审计，再按 fresh/no-replace 方式封存 execution、mission-group 与总账回执。`validate_product_acceptance_contract.py` 只接收当前 producer 哈希、当前 formal session/snapshot/runtime closure 和同一 run root 的引用式账本，继续拒绝手写嵌入 JSON、`ftyp` 填充、MCAP magic bytes、媒体复用和跨会话拼接。

producer 已闭合“无生产器”缺口，但未执行的运行门不会因此转绿：当前仍须在新鲜正式会话中保留 `18×10=180` 个唯一 execution、每个 execution 的独立 video/MCAP/replay，以及至少 `30` 个覆盖全部 execution 且不重叠的 mission group。完成前 AUTO-15 与产品状态仍保持 BLOCKED/false。

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

三个封存阶段均拒绝覆盖既有输出。先逐 bag 运行 `formal_product_mcap_replay.py`，再逐 execution 与 mission group 运行 `auto15_product_evidence.py`，最后用其 `ledger` 子命令生成总账，并把总账交给：

```bash
python3 scripts/validate_product_acceptance_contract.py \
  --execution-evidence "$RUN_ROOT/auto15/ledger.json" \
  --evidence-root "$RUN_ROOT"
```

紧凑证据位于
[`artifacts/autonomous_auto15_20260730_evidence/`](../artifacts/autonomous_auto15_20260730_evidence/)。
