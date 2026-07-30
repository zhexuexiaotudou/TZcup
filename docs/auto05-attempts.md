# AUTO-05 模型尝试记录

## Attempt 1：基础 direct detector + RGB-D area heads

假设：G3 的 world/asset/trajectory 隔离数据足以让轻量 direct detector 与独立 RGB-D area heads 从 micro-overfit 推进到跨世界 screening。

结果：执行器首次运行在 detector 解码时因缺少 `max_detections` 契约崩溃；补齐 score-ranked top-K 后用相同配置重跑并形成正式失败报告。validation cross-world detector macro F1 为 `0.08237`、small-object recall 为 `0.22321`、negative-only FP/frame 为 `0.39`；leaf/puddle IoU 为 `0.0000124/0.09949`。11 个模型门失败，两个 ONNX 数值门通过。原始报告位于 Git 忽略的 `artifacts/autonomous_auto05_attempt1_raw/`。

## Attempt 2：数据分布与训练稳定性修复

执行前假设：Attempt 1 对每张训练图在所有 epoch 使用同一轻量颜色增强，且 detector loss 有明显振荡，不能覆盖 val/test 的资产颜色、材质、光照和观察几何变化。保持 direct detector 与独立 area heads 类型不变，改为逐 epoch 确定性的强颜色/光照/仿射增强，增强后从 semantic/instance 重新计算检测框，并使用较低学习率、余弦退火与梯度裁剪。validation 仍只用于阈值选择，test 不参与训练或选参。

停止条件：仍使用规划包冻结的全部 AUTO-05 门；不降低阈值，不以 ONNX parity 代替精度。

首次启动在 epoch 1 前由随机灰度增强分支发现 float64/OpenCV dtype 契约错误，未形成模型尝试结果；增强链已统一强制为 contiguous float32，并以同一假设和配置重启。

正式结果：validation detector macro F1 提升到 `0.33476`，small-object recall 提升到 `0.86607`，puddle IoU 提升到 `0.68743`；test detector macro F1 为 `0.35201`，test negative-only FP/frame 为 `0.01`。small-object 与 same-color specificity 门转为通过，但 precision、validation negative-only、leaf 和 test area 仍失败，共 8 个门未通过。

## Attempt 3：硬负样本/类别重平衡与分辨率特征修复

执行前假设：Attempt 2 已证明分布增强能恢复召回，但训练集中纯负样本比例低于 held-out split，paper/leaf 类和小区域又被背景主导；仅继续增加相同 epoch 不足以解决 precision 和 leaf 泛化。最终方案把 pure-negative、无离散目标、paper、leaf 和 puddle 帧按预注册规则加权采样，将输入提高到 `512×384`，增加模型宽度，并给 direct detector/area heads 输入由 RGB-D、边缘、局部对比和饱和度组成的固定特征。验证阈值搜索范围扩展到高置信区间，但仍只使用 validation，test 保持冻结。

停止条件：Attempt 3 后不再无限盲调；仍有机器门失败时，AUTO-05 按规划包标记为 `BLOCKED`，AUTO-06/07/08 和 AUTO-15 的依赖部分不得伪造为通过。

首次执行完成 30 个 detector epoch 后，冻结评估函数因新增 RGB-D 特征却仍丢弃 depth 变量而触发 `NameError`，未生成指标，不计作第三个模型结果。评估解包已修复，并新增训练结束即保存 PyTorch checkpoint 的故障恢复边界；相同配置将重跑。
