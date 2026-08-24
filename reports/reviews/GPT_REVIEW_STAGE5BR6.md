# Stage5BR6-A 独立复核入口

## 结论先行

Stage5BR6-A 已完成两个隔离的人工盲审交付包，但没有收到真人答案。每包 270 张：Stage5BR5 V4 的 200 个正样本，加上 Stage5BR6 训练专用 label=0 几何世界通过真实 V4 Gazebo 相机采集的 70 个 no-target/hard-negative。七类负样本各 10 张，四传感器时间戳精确同步，semantic 目标像素总数为 0；生产世界未修改。

```text
first_blocking_layer=G2_camera_selection_blocked_awaiting_two_independent_human_reviews
AWAITING_HUMAN_REVIEW=true
READY_FOR_STAGE5BR6_ORACLE=false
READY_FOR_GPT_REVIEW_STAGE5BR6=false
READY_FOR_STAGE5BR7=false
READY_FOR_GPT_REVIEW_STAGE5B=false
READY_FOR_STAGE5C=false
```

## 优先复核

1. `artifacts/stage5br6_20260721_review/stage5br6_status.json`：确认人工回答为 0/2，所有后续门禁均未执行。
2. `human_handoff_manifest_redacted.json`：确认 A/B package ID、ZIP SHA、sample ID 摘要独立，正负样本为 200/70。
3. `human_handoff_integrity_report.json`：确认 CRC、图片数量、ID 集合、无 truth 泄漏和 PNG 元数据门通过。
4. `human_review_handoff.py`：确认硬负样本来自真实 RGB/semantic，负样本 crop 的目标像素必须为 0。
5. `stage5br6_validate_human_reviews.py`：确认脚本只校验真人 response，不自动生成、补全、修改或评分。
6. 确认 `external_review_handoff/` 与 `external_reviews/` 被 Git 忽略，sealed truth 和 reviewer ZIP 不进入仓库。

## 正确停止边界

V4 当前只允许标为 `pre_registered_camera_candidate=V4`、`camera_selected=false`。在两份独立真人 response 完整且人工指标过门前，不得冻结 V4/policy v2、修改 production footprint、运行 candidate-footprint Stage4W 回归、执行 Oracle active observation 或训练模型。`REVIEW_PACKET_COMPLETE=true` 仅表示 Stage5BR6-A 的机器可审计证据齐全，不表示 Stage5BR6 或 Stage5B 通过。
