# TZcup Stage5BR GPT 独立复核入口

## 复核结论候选

本轮完成的是“真实 Gazebo-camera 数据链恢复成功，但学习模型 screening 仍失败”的可审计边界，不是 Stage5B 通过。

```text
REVIEW_PACKET_COMPLETE=true
READY_FOR_GPT_REVIEW_STAGE5B=false
READY_FOR_STAGE5C=false
competition_perception_pass=false
real_domain_evaluation_executed=false
j6_runtime_pass=false
competition_efficiency_pass=false
```

## 建议阅读顺序

1. `artifacts/stage5br_20260719_review/stage5br_status.json`
2. `docs/stage5br-gazebo-camera-recovery.md`
3. `artifacts/stage5br_20260719_review/micro_overfit_report.json`
4. `artifacts/stage5br_20260719_review/pipeline_parity_report.json`
5. `artifacts/stage5br_20260719_review/g1_annotation_qa.json`
6. 三份 `g1_model_screening_attempt_*.json`
7. Stage5A / Stage4W 回归摘要与 `visuals/`
8. `artifacts/stage5br_20260719_review/MANIFEST.json`

## 请重点复核

- micro-overfit 是否真实达到 F1 ≥0.98、mIoU ≥0.95，而不是评估器或标签泄漏造成；
- PyTorch/ONNX/ROS 预处理 parity 是否满足 1e-4 与 99.99% 门；
- G1 是否确由 Gazebo Harmonic RGB-D、SegmentationCamera 和 Label system 产生，而不是 P1 重命名；
- scene 与 asset split、timestamp、semantic/instance 解码和重复检查是否足以支持 50-scene screening；
- 三次模型筛查是否诚实保留失败，且没有用 test split 选型；
- 未执行 500-scene formal、30-seed live、真实 Nav2 spot-clean 与 J6 是否符合停止条件；
- 下一轮应优先增加世界/材质/几何多样性，还是切换 detector + high-resolution crop 路线。

## 关键数值

- Phase A：micro F1 `0.98124`，foreground mIoU `0.96333`；最大 logit error `6.866e-05`，argmax agreement `1.0`。
- G1 smoke：50 scene / 500 frame，annotation completeness `1.0`，label error `0.0`，跨 split exact/pHash duplicate `0/0`。
- 最佳 screening：in-domain F1 `0.84511`，cross asset/world F1 `0.65804`，leaf/puddle mIoU `0.86313`，color stress F1 `0.47647`；未过门。
- Stage5A live：186 帧、MCAP true、GT control violation 0。
- Stage4W seed 0：17/17、coverage `0.936`、RMSE `0.03260 m`、碰撞/keepout/brush violation 均 0。

## 固定边界

```text
1053 m²/h < 3500 m²/h
```

不得把 G1 smoke、Stage5A D0 回归或成功运行的 ROS 链路解释为竞赛感知精度通过。
