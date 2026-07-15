# Stage4R GPT 复核包说明

请先阅读仓库根目录 `GPT_REVIEW_STAGE4R.md`，再核对 `artifacts/stage4r_20260715_093931/stage4r_summary.json` 与同目录 `MANIFEST.json`。

本包保留 Stage0–4 历史源码与证据，并新增 Stage4R 的源码、失败尝试、真实 Gazebo PNG、有效 SLAM 数值证据、同帧定位失败曲线、组件化覆盖失败报告、30 次急停报告、155.6 MiB 完整 MCAP 及回放输出。结论为 `READY_FOR_GPT_REVIEW_STAGE4R=false`，原因不是材料缺失，而是同帧定位和完整经验覆盖没有达到硬门；不得把规划覆盖率 97.5% 解释成实际覆盖率。

复核顺序：

1. `GPT_REVIEW_STAGE4R.md`
2. `artifacts/stage4r_20260715_093931/stage4r_summary.json`
3. `artifacts/stage4r_20260715_093931/localization_error.png`
4. `artifacts/stage4r_20260715_093931/stage4r_slam_map.png`
5. `artifacts/stage4r_20260715_093931/coverage_report.json`
6. `artifacts/stage4r_20260715_093931/safety_latency_report.json`
7. `artifacts/stage4r_20260715_093931/rosbag_info.txt`
8. `artifacts/stage4r_20260715_093931/MANIFEST.json`

原始 Stage4R 提示词已复制为 `artifacts/stage4r_20260715_093931/review_input_Stage4R.md`。压缩包不包含 `.git/`、本地 Docker 工作区 `.work/` 和旧 Stage4 复核 ZIP；这些内容不是本轮复核所需运行证据。
