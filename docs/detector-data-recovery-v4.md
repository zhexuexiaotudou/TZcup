# Detector Data Recovery V4

DDRV4 是 PR #90 上继 OPRV3 后的新 detector 恢复协议。旧 X1/X2/X3、MRV2-A/B/C、OPR-A/B/C 结果保持不可变；旧 `G5_SEALED_FINAL` 已消费且只保留历史失败证据，`G5_V2_SEALED_FINAL` 在有效 DDRV4-07 freeze 前禁止读取。G6 只允许历史回归或对照，不得参与 DDRV4-D1/D2/D3 的 checkpoint、threshold、augmentation 或 route 选择。

DDRV4-00 的紧凑基线位于 [`artifacts/detector_data_recovery_v4_20260811T134117Z/baseline/`](../artifacts/detector_data_recovery_v4_20260811T134117Z/baseline/)。运行时边界由 `sanitation_learning.ddrv4_boundary` 统一执行，CI 覆盖旧 G5 永久拒绝、G5_V2 冻结前拒绝和 G6 选模拒绝。当前只授权新建独立 `G7_DETECTOR_DEVELOPMENT` 数据包；Area 继续复用已通过的 OPRV3-06 G6 结果，除非出现有证据的软件集成缺陷，不重新调参。

当前状态仍为 `MODEL_BLOCKED_INTERNAL=true`、`PRODUCT_X86_PERCEPTION_READY=false`。只有 G7 QA、失败分类和最多三条新路线中的某条静态门通过，才允许进入在线开发门；后续 freeze、G5_V2 one-shot、30-seed、Spot Cleaning、soak、release、J6 与 field 继续逐门 fail-closed。

G7 的实现入口为 `py -3 scripts/build_ddrv4_g7.py --output <NEW_EXTERNAL_ROOT>`。默认生成 13 个 `g7v4_` 世界、8 个固定 split、3200 帧，并按独立 asset namespace 记录 metal/bottle/paper 域、16 类全负样本、原生 bbox/mask/距离/材质/光照/遮挡元数据以及 exact/pHash 隔离。输出目录必须为空，生成器不接受任何现有数据根作为输入。

正式 v3 数据包包含 3200 帧、320 场、13 世界和 2810 个实例，metal/bottle/paper 分别为 `1235/757/818`，`<18 px` 与 `18–48 px` 分别为 `1046/1038`，全负样本为 800。独立审计重读 16,000 个像素/元数据文件和全部 2810 个实例，mismatch 与跨 split pHash duplicate 均为 0，`G7_DATASET_PASS=true`、`G7_INDEPENDENT_AUDIT_PASS=true`。v1 可见性失败与 v2 taxonomy 仅覆盖 4/16 类的失败目录原样保留；紧凑摘要见 [`G7_DEVELOPMENT_SUMMARY.json`](../artifacts/detector_data_recovery_v4_20260811T134117Z/g7/G7_DEVELOPMENT_SUMMARY.json)。
