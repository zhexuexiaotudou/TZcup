# Stage4V 独立复核入口

## 结论

- `READY_FOR_GPT_REVIEW_STAGE4V=false`
- `READY_FOR_STAGE5A=false`
- `competition_efficiency_pass=false`

正式混合定位已经过门，但完整 Coverage 未过，因此不得提升总体就绪标志。

## 已通过

- hybrid 10-seed 完整 10/10；XY RMSE P50/P95/max 为 `0.033438/0.037916/0.038717 m`，所有 trial 均不超过 0.05 m。
- 导航成功 10/10，TF 单一 `map -> odom` 所有者 10/10，扫描精化参与 10/10。
- GT 控制违规 0；诊断声明 `ground_truth_direct_fusion=false`。
- 30 次急停 P95 `0.1705 s`，keepout 违规 0、速度区通过、碰撞 0。
- 完整 MCAP 已录制并成功回放 `/localization/fused_pose`。

## 未通过与停止边界

- Coverage 规划成功且计划覆盖率 97.5%，但 transit-to-start 超时/终止；完整执行 false、经验覆盖率 0%。
- Coverage 未进入有效执行，故动态障碍有效交互为 0/20，不能宣称动态避障通过。
- 理论效率 `0.65 × 0.45 × 3600 = 1053 m²/h`，低于 3500 m²/h。
- J6 真实 GNSS、时间同步、外参、ARM 交叉编译、实时资源与 HIL 均未执行。

## 复核文件

- `artifacts/stage4v_20260716_review/stage4v_localization_report.json`
- `artifacts/stage4v_20260716_review/stage4v_coverage_report.json`
- `artifacts/stage4v_20260716_review/MANIFEST.json`
- `docs/stage4v-hybrid-localization.md`

原始矩阵、Coverage 日志和 MCAP 保留在本地 artifact，待用户确认后再清理。
