# Stage5B 独立复核入口

## 请先确认的结论

本包是“可复核失败边界”，不是 Stage5B 通过声明。`REVIEW_PACKET_COMPLETE=true`，但 `READY_FOR_GPT_REVIEW_STAGE5B=false`、`READY_FOR_STAGE5C=false`。请重点审核停止条件是否执行正确、指标命名是否诚实，以及下一轮是否应先建设真正的 Gazebo-camera D1 数据流水线。

## 机器结果摘要

- 学习模型：是；候选 A/B 均由随机初始化经梯度训练，候选 B 被验证集选中，测试集不参与选择。
- D1 数据：50 seed/500 帧筛查，五类各六资产变体及 12 个硬负样本；但域为 `D1_procedural_rendered_not_gazebo_camera`。
- 未见测试：100 scene/1000 帧，离散 macro P/R/F1 `0.00752/0.00784/0.00768`；leaf/puddle IoU `0.00376/0.2494`；map RMSE `0.09731 m`。
- 颜色压力：aggregate macro F1 `0.05192`，失败。
- AP：未计算，字段为 null；没有用 IoU 匹配分数冒充 AP。
- Gazebo 诊断：真实 RGB-D/TF/ONNX Runtime 161 帧，分割与 map targets 有输出且不读取 GT；只证明接口可运行。
- 回归：快速测试 57 passed；Stage5A 基线通过；Stage4W seed 0 完整任务通过。
- D2/J6/效率：无真实数据；无官方 J6 工具链/实板；`1053 < 3500 m²/h`。

## 建议复核顺序

1. `artifacts/stage5b_20260719_review/stage5b_status.json` 与 `dataset_summary.json`
2. `training_attempts.json`、`model_selection_report.json` 与 `training_curves.png`
3. `d1_perception_report.json`、`d1_gate_matrix.png` 与 `color_shortcut_stress.png`
4. `stage5b_live_diagnostic_report.json` 及三个 regression 报告
5. `j6_preflight.json`、`D2_REAL_DATA_STATUS.md` 与 `MANIFEST.json`

详细复现、数据合同和下一步见 `docs/stage5b-learned-perception.md`。原始三次筛查、数据卷与 rosbag 因体积未提交，保留在本机直至用户确认。
