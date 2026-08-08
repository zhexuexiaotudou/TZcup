# AUTO-05R P1：旧 discovery head 修复后对照复测

## 结论

历史 `small FPN + CenterNet-like full-frame head` 已正式淘汰。修正 bbox、split、
AMP、样本覆盖与有界后处理后，L1/L2/L3 三种 objectness loss 均不能同时达到：

- cross-world val candidate recall `>= 0.75`；
- false candidates/min `<= 20`；
- negative-only FP/frame `<= 0.20`。

冻结 holdout 阈值后，三个分支在 cross-world val 都退化为零检出；降低到 `0.05`
时，L2/L3 又分别产生约 `59,976/59,838` false candidates/min，负帧 FP rate
均为 `1.0`，召回仍只有 `0.0031/0.0213`。这是 proposal architecture/target
与 score separation 失败，不是继续增加 epoch 或调高阈值可以解决的问题。

```text
legacy_architecture_retired=true
additional_epoch_tuning_allowed=false
next_action=proceed_to_P2_teacher_then_FCOS_lite
```

## 协议

- 仅读取 raw `train` 与 `val`；从 train scenes 确定性派生
  `train_world_holdout`。
- 受限规模复测使用 world × positive/negative round-robin：600 train（8 worlds，
  300 negative-only）、100 holdout（8 worlds，38 negative-only）、100 val
  （2 worlds，50 negative-only）。不再使用 manifest 前缀切片。
- threshold 只在 train-world holdout 选择，cross-world val 只执行冻结阈值。
- legacy G4 test 与 G5 sealed final 均未读取。
- L1/L2/L3 均执行 per-epoch holdout validation、EMA、best checkpoint、AMP、
  gradient clipping、固定 seed 与 early stopping；同时记录正/负 loss、hard-negative
  contribution、各 head gradient norm 和正负 score histogram。
- decoder 在模型图外先做固定 `pre_nms_topk=1000`，再 NMS，最终最多 100 个
  detection；避免密集伪峰使 Python NMS 无界运行。

## 过程发现

P1 不只是得到一个失败分数，还修复了四类会污染后续产品训练的问题：

1. 明确恢复历史 small-FPN control，避免把“随机初始化 ResNet18”误标为旧头；
2. discovery dataset 不再读取完全未使用的 depth/semantic/instance 数组；
3. quality-focal 改用 AMP-safe `BCEWithLogits`；
4. 有限子集按 world × polarity 分层，且 NMS 前显式 top-K。

原始 checkpoint、完整报告和 24 张解码图保留在仓库外。紧凑指标、
checkpoint/report SHA-256 位于
`artifacts/auto05r_p1_evidence/P1_CONTROL_SUMMARY.json`。这些 checkpoint 全部是
diagnostic-only，不得进入 model freeze 或产品 registry。

## 下一步边界

P2 必须先用官方预训练 Torchvision FCOS ResNet50-FPN teacher 判断数据是否可学；
若 teacher 在 val 仍达不到 recall `>= 0.85`、false proposals/min `<= 10`，回到
数据/标注/相机尺度。只有 teacher 证明数据可学后，才进入最多三次的 FCOS-lite
ResNet18/MobileNetV3/distilled student 尝试。
