# AUTO-04 双模型 micro-overfit

## 当前状态

AUTO-04 已在实现提交 `152e7a55f89a83b395bca55445cf1c7c3353e8ba` 上完成正式 GPU 运行。第二轮通过全部冻结机器门，状态为 `AUTO-04=PASS`，自主状态推进到 AUTO-05。紧凑证据位于 `artifacts/autonomous_auto04_20260730_evidence/`；第一轮失败报告同时保存在该证据的 `prior_attempts/` 和 Git 忽略的原始运行目录。

## 离散目标 detector

离散目标采用直接的 anchor-free object detector：

```text
RGB
→ stride-4 feature map
→ 3-class object-centre heatmap
→ centre offset
→ bbox width/height regression
→ confidence-ranked decode
→ class-wise NMS
```

该模型不输出语义分割，不使用 connected-components 生成 detector 结果。训练目标直接由同步 Gazebo instance mask 的逐实例 bbox 编码，固定输入为 `1×3×192×192`、batch 为 1 的 ONNX；正式门同时检查 AP50、逐类 recall、negative-only FP、NMS、算子清单以及 PyTorch/ONNX 数值一致性。

## 区域目标 segmenter

`leaf_pile` 与 `puddle` 使用独立 RGB U-Net 风格二值输出 heads，固定输入为 `1×3×128×128`。训练门使用 binary cross-entropy、Dice 和负样本概率惩罚，正式报告分别计算两类 IoU、macro mIoU、最小面积候选过滤后的 negative-only area FP，以及加入背景基准后的 PyTorch/ONNX argmax agreement。该模型与 detector 分开导出，不能用一个任务的成绩替代另一个任务。

## 数据与边界

micro 数据来自 Stage5BR3 已留存的真实 Gazebo Harmonic 同步 RGB、semantic、instance 训练 split。detector 使用 20–40 个正样本、至少 10 个 negative-only 样本，并覆盖三类、多实例及小/中/大尺寸；area 数据要求 leaf/puddle 各至少 20 帧及至少 10 个 negative-only 样本。

本阶段只证明 task-specific train-set capacity。即使通过，也不能外推为 AUTO-05 跨世界 screening、AUTO-06 正式感知、live、真实域或竞赛感知通过。

第一轮正式运行中，detector 的 AP50 已达到 `0.99670`，但固定 `0.5` 阈值的 macro recall 为 `0.86087`；三分类 area head 的 macro mIoU 为 `0.46308`。该失败完整保留。第二轮只把 detector 阈值冻结为 `0.20`，并按预定义 fallback 将 area 改为两个独立二值 heads、收紧目标 crop；没有改样本身份或降低验收阈值。

第二轮结果为 detector AP50 `0.9966997`，三类 recall 均为 `1.0`，negative-only FP/frame 为 `0`；detector ONNX 最大数值误差 `1.1444e-05`、decoded agreement `1.0`。leaf/puddle IoU 为 `0.9810641/0.9691405`，macro mIoU `0.9751023`，negative-only area FP/frame 为 `0`；area ONNX 最大误差 `9.1553e-05`、argmax agreement `1.0`。这些结果只解除 AUTO-05 的依赖，不提升真实域、J6 或最终竞赛状态。

## 复现

```powershell
powershell -NoProfile -ExecutionPolicy Bypass `
  -File scripts/run_auto04_micro_overfit_docker.ps1 `
  -DataRoot F:\Project\TZcup-stage5br3-data\g2_screening_native `
  -OutputName autonomous_auto04_20260730_evidence `
  -ImplementationCommit <implementation-commit>
```
