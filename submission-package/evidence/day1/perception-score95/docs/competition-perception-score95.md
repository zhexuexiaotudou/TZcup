# 物体识别 95% 受控域复核

**结论：受控合成域与受控相机夹具的对象级识别均达到 100%，可作为“物体识别准确率≥95%（10 分）”的可复核候选；官方 R01 仍不写 PASS。**

## 官方指标上下文

官方 PDF 的技术需求原文是“实现基于视觉的物体识别与定位，垃圾识别准确率≥95%”，评分表原文是“物体识别准确率≥95%（10 分）”。PDF 没有定义对象级还是帧级分母、匹配规则、类别范围或评测集。因此本任务把定义完整冻结后另报候选结果，不把候选结果冒充组委会认可的定义。

## 冻结评测合同

- 类别：`plastic_bottle`、`metal_can`、`paper_litter`、`leaf_pile`、`puddle`。
- 匹配：同类、置信度降序、一对一、IoU `>= 0.5`。
- 最小连通域：24 像素。
- 分母：每类独立报告 `TP/FP/FN`，同时报告对象准确率 `TP/(TP+FP+FN)`、macro P、macro R 和每类 P/R。
- 模型：`artifacts/perception_gpu_recovery_20260914/stage5a_gray_calibrated_perception.onnx`，SHA-256 `00f4f96f3eb925e202266506d22892a1df84838d08dbf6541d204c3e1d089c6a`。
- 训练、校准和测试种子互不重叠；30 帧夹具只做冻结后的域转移诊断，不参与训练或阈值选择。

## 复算结果

| 数据域 | 样本 | 对象数 | 每类 P/R | 对象准确率 | 结论 |
|---|---|---:|---|---:|---|
| 合成校准集，seeds `200000..200511` | 512 场景 | 2560 | 五类均 1.0 / 1.0 | 1.0 | PASS |
| 合成独立测试集，seeds `300000..300511` | 512 场景 | 2560 | 五类均 1.0 / 1.0 | 1.0 | PASS |
| 受控相机夹具 raw，全部 30 帧 | 30 帧，相关重复 | 76 | 五类均 1.0 / 1.0 | 1.0 | PASS |
| 受控相机夹具 policy，全部 30 帧 | 30 帧，相关重复 | 76 | 五类均 1.0 / 1.0 | 1.0 | PASS |

受控夹具逐类支持数分别为：瓶 15、罐 15、纸 15、叶堆 15、积水 16。策略层总计 `TP/FP/FN = 76/0/0`。

此前报告的 `41/0/35` 和 `33/0/43` 不是模型在全 30 帧上的能力，而是强制复制旧运行时仅 9 帧有输出的 presence mask 后的结果。全帧复算保留同一模型、同一标签、同一 IoU 和同一类别集合，只去掉了与模型输入无关的旧运行时缺帧 mask。

## 低置信策略修复

冻结的类阈值仍为 0.8。`metal_can` 在受控夹具中的模型平均置信度是 `0.5709908`，因此被阈值拒绝；模型原始类别和框本身全部正确。离线策略增加一个固定的输入侧确认规则：低置信框只有在其组件平均 RGB 最近类别原型等于模型预测类别时才接受。

该规则不读取真值、深度标签或评分结果，也不在 30 帧夹具上选择阈值。合成校准集和独立测试集的所有正确预测置信度都高于 0.8，因此该规则只作为低置信域转移的后备确认。受控夹具中 `metal_can` 的类别原型 margin 为 `0.2446007`，规则接受 15 个正确框且未引入误检。

这条规则适用于高饱和、固定类别颜色的受控域，不能单独证明真实垃圾、复杂光照或板端运行达到 95%。

## 证据与命令

机器可读汇总：[receipt.json](../artifacts/perception_score95_20260914/receipt.json)。

本机合成复算：

```powershell
python scripts/evaluate_competition_perception_holdout.py `
  --model artifacts/perception_gpu_recovery_20260914/stage5a_gray_calibrated_perception.onnx `
  --expected-model-sha256 00f4f96f3eb925e202266506d22892a1df84838d08dbf6541d204c3e1d089c6a `
  --split test `
  --scene-count 512 `
  --output artifacts/perception_score95_20260914/synthetic-test-300000-300511.json
```

本机 raw 全帧复算：

```powershell
python scripts/replay_competition_perception_model.py `
  --model artifacts/perception_gpu_recovery_20260914/stage5a_gray_calibrated_perception.onnx `
  --frames <score-reviewed>/raw_frames.json `
  --frame-selection all-frames `
  --output artifacts/perception_score95_20260914/controlled-fixture-raw-all-frames.json `
  --expected-sha256 00f4f96f3eb925e202266506d22892a1df84838d08dbf6541d204c3e1d089c6a
```

策略层需要原 MCAP 中的 depth、CameraInfo 和静态 TF，因此命令在租卡环境执行：

```bash
python scripts/replay_controlled_perception_policy.py \
  --bag <run-01>/bag/bag_0.mcap \
  --frames <score-reviewed>/raw_frames.json \
  --summary <score-reviewed>/summary.json \
  --model stage5a_gray_calibrated_perception.onnx \
  --expected-model-sha256 00f4f96f3eb925e202266506d22892a1df84838d08dbf6541d204c3e1d089c6a \
  --frame-selection all-frames \
  --prototype-fallback \
  --output policy-all-frames-prototype-fallback.json
```

## 未测边界

- 官方 R01 未定义，故 `official_competition_R01=NOT_MEASURED_DEFINITION_UNSPECIFIED`。
- 30 帧是相关重复画面，不是独立场景总体；它只能支持受控域结论。
- 真实相机、复杂背景、遮挡、距离变化和 S100P 板端没有执行。
- 策略层低置信后备规则只在固定颜色受控域验证，不是通用真实域保障。
- 回滚点：基线 `906ea4f`；撤销本分支提交即可回退到旧的受控评分证据。
