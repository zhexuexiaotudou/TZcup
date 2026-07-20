# Stage5BR4 独立复核入口

请独立检查本轮是否真的先执行可观测性和相机消融，并在相机选择门失败后停止。不要把 `REVIEW_PACKET_COMPLETE=true` 解释为 Stage5B 通过。

优先检查：

1. `artifacts/stage5br4_20260720_review/stage5br4_status.json`：Stage5BR3 旧三次失败未改写，新首阻断层明确。
2. `perception_observability_report.json`：3370 个 all-visible 实例中仅 875 个 ready，non-ready 没有被隐藏。
3. `perception_evaluability_policy.yaml`：训练前冻结的三分区阈值与 SHA-256。
4. `camera_ablation_report.json`：五段真实采集均为 10/10 帧、同 world/seed/pose/命令；C3 conversion 为 0.50，不是 0.90。
5. `manual_recognizability_audit.json`：图像 SHA 与 C3 车体遮挡结论。
6. `production_isolation/production_isolation_report.json`：默认生产链没有 verification camera、训练 GT 或控制订阅泄漏。
7. `active_observation.py` 与测试：状态、去重、stale、timeout、unreachable、最大重试和任务代价。
8. 确认 detector/area micro-overfit、120/1200、screening、formal、正式 live、真实 active Nav2 和 J6 均未越级。

正确结论应保持：`REVIEW_PACKET_COMPLETE=true`，`READY_FOR_GPT_REVIEW_STAGE5B=false`，`READY_FOR_STAGE5C=false`；四个固定竞赛/真实域/J6 边界均为 false，`1053 m²/h < 3500 m²/h`。
