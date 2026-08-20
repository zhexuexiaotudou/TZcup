# Close-Range Classifier Contract Recovery V11 最终结论

CRCRV11 永久保留 RGDRV8、TGARV9、TRCRV10 的失败结论，并保持 `G10_DEV_VAL_SEALED`、`VAL_NEW`、`G5_V2` 和正式 30-seed 数据未读。协议只允许在冻结的 T3 Grounding-DINO proposal gate 后尝试 R1、R2、R3 三条分类路线。

审计证明 proposal crop 几何不是主因：GT/proposal tight macro-F1 仅相差 `0.0064`。V10 TRAIN 只有 9 个 unique background crop，replacement sampler 期望重复约 `186.64` 次，runtime-faithful positive view 只占 `40%`；100/100 crop round-trip 通过像素和 RGB channel parity。C11 将背景扩展到 6,576 个 unique tight crop，且跨 split exact/pHash overlap 均为零。

R1 将 HOLDOUT background specificity 恢复到 `1.0`，但目标类别没有恢复；R2 的 binary 和 three-class 两阶段均未达到硬门；R3 因 tight/context complementary correctness `22.76%` 获准执行，但 shared ConvNeXt-Tiny fusion 仍失败。最佳正式 candidate macro-F1 仅 `0.6311`。

因此协议已触发停止条件 B：R1/R2/R3 全失败，无产品分类路线可选，`CLOSE_RANGE_CLASSIFIER_CONTRACT_BLOCKED=true`、`MODEL_BLOCKED_INTERNAL=true`、`SIMULATION_PRODUCT_COMPLETE=false`。ActionVerifier、sealed、online、performance、freeze、30-seed cleaning、soak、replay 与 release 均保持 dependency-blocked；禁止 R4/R5、新 detector 搜索、sealed-data tuning 或降低产品门槛。

历史来源为已关闭的 Draft PR #91 最终 commit `261f0d62d9e7bf6f844c0faf1fa72fe02486e0ce`。本页只保留最终结论，不恢复该 PR 的训练中间产物或失败模型。
