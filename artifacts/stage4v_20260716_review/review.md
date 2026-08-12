# Stage4V 复核结论

`READY_FOR_GPT_REVIEW_STAGE4V=false`、`READY_FOR_STAGE5A=false`、`competition_efficiency_pass=false`。

混合定位 10-seed 全部完成，XY RMSE P50/P95/max 为 `0.033438/0.037916/0.038717 m`，导航、TF 所有权、扫描精化、GT 隔离、急停和 MCAP 回放通过。Coverage 在 transit-to-start 阶段超时，未形成完整执行或动态障碍有效交互；理论效率 `1053 m²/h` 低于 `3500 m²/h`。因此该证据只证明定位门，不产生 Coverage、J6 或产品 Ready。

详细指标见同目录的 `stage4v_localization_report.json`、`stage4v_coverage_report.json` 与 `MANIFEST.json`。
