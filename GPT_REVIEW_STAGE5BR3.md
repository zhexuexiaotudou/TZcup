# Stage5BR3 独立复核入口

请独立检查本轮是否严格执行 G2 真实车辆相机、world-isolated 数据、逐实例 QA、分辨率筛选、split-model screening 与 fail-closed 停止条件。不要仅依据 README 的总结判定。

优先检查：

1. `artifacts/stage5br3_20260720_review/stage5br3_status.json`：总状态、三次尝试和固定 false 边界。
2. `runtime_contract/six_world_runtime_summary.json` 与六个逐世界 JSON：实际 RGB-D/GT/CameraInfo/同步/TF/车辆运动，而非只检查话题名。
3. `production_isolation/production_isolation_report.json`：生产 Xacro、launch、运行时 topic 和控制订阅均未泄漏 GT。
4. `g2_annotation_qa_attempt1_failed.json` 与 `g2_annotation_qa.json`：确认失败证据被保留，修复后确为全量重采，而非覆盖失败结论。
5. `g2_instance_qa_records.jsonl`：逐实例 bbox、短边、mask area、距离、遮挡、可见性与帧运动证据。
6. `g2_resolution_scan.json`：四档扫描与离散小目标依据；吞吐字段与模型字段没有互相冒充。
7. `model_attempt_1..3/model_screening_attempt_*.json`：随机初始化、train/val/test 隔离、真实 AP/F1/mIoU/negative-only FP、固定 shape ONNX、算子、时延、显存和三次上限。
8. `stage5br2_archive_integrity_report.json`：Git blob、working tree、git archive 和上一轮最终 ZIP 的四表面逐字节一致性。
9. `artifact_manifest.json`：复核包内文件的 SHA-256；2.225 GB 原始集只保留本机，不被误称为已打包。

建议复核问题：

- 六个世界是否真有不同几何、布局、材质和 SHA，而不是颜色换皮？
- 是否确由实际车辆运动采集，训练 GT 是否只在显式开关下存在？
- split 泄漏、重复帧、negative-only、逐实例对应和小目标统计是否可信？
- 三次模型尝试是否使用 test split 选型，是否存在把语义分割结果冒充 detector AP 的情况？
- 停止边界是否正确：screening 失败后没有启动 500/5000、live、真实 Nav2、真实域和 J6？
- 场景 manifest 中的曝光/白平衡/噪声/模糊/动态障碍请求是否有原生 Gazebo 逐项施加证据？本轮明确没有把该请求字段声称为已全部施加，建议作为下一轮补强项。

正确结论应保持：复核材料可审计不等于 Stage5B 通过；`READY_FOR_GPT_REVIEW_STAGE5B=false`、`READY_FOR_STAGE5C=false`，理论效率仍为 `1053 m²/h < 3500 m²/h`。
