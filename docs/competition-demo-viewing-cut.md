# 竞赛演示观看版

`scripts/create_demo_viewing_cut.py` 将同一个正式 episode 的连续原片制作成
5--8 分钟观看版。它不是仿真器、视频录制器或证据生成器；只消费 T1/T2 交付目录中的
`manifest.json`、`checksums.sha256`、`timeline.csv` 和 manifest 指向的 MP4。

## 输入合同

manifest 必须使用 `tzcup.competition_demo_artifact_manifest.v1`，并包含同一
`run_root` 下的 `run_identity`（`session_id`、`runtime_id`、`episode_id`、
`session_start_epoch_ns`）、录制的起止 epoch nanoseconds，以及
`raw_collection`、`video_observation`、`formal_session_current`、`video`、`mcap`、
`event_reports`：前四项为非符号链接常规文件；`mcap` 是带逐文件哈希和目录树哈希的
非符号链接目录；`event_reports` 是同 run 的常规文件描述符数组。`checksums.sha256`
必须精确覆盖 manifest 与 timeline。工具还用 `ffprobe` 验证 MP4 可读及其时长。
上游 manifest 的 `delivery_status` 必须为 `READY_FOR_REVIEW`，且 timeline 状态必须
为 complete；MP4 时长必须与签名录制窗口相符（允许 2 秒封装误差）。

timeline 的列严格为：

```text
event,status,epoch_ns,source,reason
```

它必须恰好包含 `map_creation`、`hard_restart`、`cleaning`、`grasp_drop`、
`obstacle_avoidance`、`return_home`，并且全部为 `observed`。事件时间必须落在连续
原片内，六段各 50 秒不能重叠，总时长必须为 300--480 秒。任一缺失、校验漂移、跨
run-root 文件、无效 MP4、重叠时间窗或时长不足都会产生
`BLOCKED_MISSING_REQUIRED_EVIDENCE` 回执，且不产生 EDL、字幕或视频。

## 用法

先只生成可审阅 EDL 和字幕；这一模式不会渲染视频：

```powershell
py -3 scripts/create_demo_viewing_cut.py `
  --manifest <delivery>/manifest.json `
  --checksums <delivery>/checksums.sha256 `
  --timeline <delivery>/timeline.csv `
  --output-dir <delivery>/viewing_cut
```

在确认 EDL 后，增加 `--execute` 才调用系统 `ffmpeg`。它从原始 MP4 逐段重编码、拼接，
并将由 timeline 生成的可开关 SRT 作为 MP4 `mov_text` 字幕流；不生成动画、补帧、
历史片段、推断指标或未在 timeline 中出现的字幕。

```powershell
py -3 scripts/create_demo_viewing_cut.py ... --execute
```

输出包括 `viewing_cut.edl.json`（所有切点、倍速和来源事件）、`viewing_cut.srt`、
`competition_demo_viewing_cut.mp4`（仅 execute）和 `viewing_cut_receipt.json`。
EDL 当前始终记录 `speed=1.0`；若将来需要倍速，必须在 EDL 中逐段列出并保持对应的源
时间范围，不能静默加速。

## 证据边界

观看版只改善对同一真实 episode 的观看和审阅，不能使未通过的感知、控制、抓投或安全门
变为通过。完整连续原片保留在 run root，Git 只提交工具、合同、测试和紧凑回执。当前没有
合格的完整 episode 输入，因此本工具的可执行结果是 fail-closed blocked 回执，不是最终视频。
