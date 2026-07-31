# AUTO-17 紧凑证据

该目录保留 2026-07-31 本机 WSLg 正式演示的可审计紧凑证据。原始 MCAP、完整 MP4、逐帧录像与运行日志保留在任务工作树中，不提交到 Git；仓库不再生成或保存 GPT 复核压缩包。

- `acceptance_summary.json`：AUTO-17 fail-closed 机器汇总，状态 PASS。
- `coverage_report.json`：17/17 正式 Coverage 组件和完整指标。
- `dashboard_telemetry.json`：终态 `COMPLETED`、实时话题及结论边界。
- `rosbag_metadata.yaml`：205528 条消息、18 个话题、397.705 秒。
- `visual_demo_frame.png`：100% 终态代表帧。
- `sha256.txt`：本目录证据文件 SHA-256。

最终端到端复验由当前单命令启动器完成冷启动、17/17 任务、MCAP、录像编码、代表帧和 fail-closed 汇总，并自行返回 0。
