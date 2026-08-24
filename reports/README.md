# 项目报告

`reports/` 收纳跨运行批次的汇总报告，避免生成文件散落在仓库根目录：

- `release/`：AUTO-16 最终状态、阻断清单、证据索引、SBOM 和发布清单；
- `reviews/`：各 Stage 的 GPT 复核包和配套工程 waiver。

单次运行的指标、清单和图表仍放在 `artifacts/<run>_review/` 或 `artifacts/<run>_evidence/`；原始 MCAP、日志、数据集和模型不进入 Git。控制面计划与状态位于 `config/autonomy/`。历史 `GPT_REVIEW_STAGE4V.md` 因被只读二进制清单固定引用而保留在根目录，不作为新文件布局的先例。
