# Day-1 完整任务 / 效率 / Demo 封装器

## 1. 目标

`scripts/day1_full_mission_acceptance.py` 把一次同源 full-mission run
封装为可复核的比赛 receipt，覆盖：

- 完整任务 sim 时长、wall 时长、RTF、阶段时间线和终态；
- coverage、brush、dirt delta、safety、return-home；
- 5-10 分钟 demo 视频、分镜、录制清单；
- 完整任务 MCAP 与必需话题；
- 软硬件完整性文件及 SHA-256；
- Demo 分镜、录制 checklist 和 shot index。

这是一个 **fail-closed wrapper**。只有所有机器门全部通过，receipt 才会是
`PASS`。缺失证据、旧 run、短时窗口、拼接视频、只有 coverage 没有完整任务，
都不会被提升为完整任务或官方得分。

## 2. 输入契约

### 2.1 Coverage run 目录

输入 `--run-dir` 必须包含：

| 文件/目录 | 作用 |
|---|---|
| `coverage_report.json` | coverage schema v2、质量/安全/定位门、有效面积和轨迹统计 |
| `coverage_trajectory.csv` | 至少一行同 run 轨迹 |
| `coverage_config.yaml` | 覆盖阈值来源 |
| `bounded_coverage_result.json` | 独立 bounded coverage 结论 |
| `cleaning_bridge.json` | brush permit、三工具 ready、roller 转速、dirt delta、退出状态 |
| `mission_timeline.json` | 完整任务 sim/wall 时长、RTF、阶段和终态 |
| `safety_receipt.json` | 碰撞/禁行区为零、急停延迟和最终速度 |
| `return_home_receipt.json` | 返航完成、终点误差、返航期间 brush 关闭 |
| `final_state.json` | 任务终态、运动归零、brush/dirt 关闭、急停释放 |
| `hardware_software_manifest.json` | 软硬件完整性证据与 SHA-256 |
| `bag/metadata.yaml` 和 `bag/*.mcap` | 完整任务 MCAP 与必需话题 |

`mission_timeline.json` 的 `duration_basis` 必须严格等于
`full_mission_sim_seconds`。coverage probe 自己的 `actual_duration_sec`
只保留为分阶段值，不参与总效率分母。

### 2.2 Demo 视频目录

输入 `--video-dir` 必须包含一个 MP4 和 `video_manifest.json`：

```json
{
  "schema_version": 1,
  "run_id": "same-run-id",
  "mission_id": "same-mission-id",
  "file": "demo.mp4",
  "file_sha256": "hex",
  "duration_sec": 480.0,
  "width": 1920,
  "height": 1080,
  "codec_name": "h264",
  "contains_operation_footage": true,
  "stitched": false
}
```

wrapper 使用同机 `ffprobe` 读取真实媒体信息，并要求：

- 5-10 分钟（默认 300-600 秒）；
- 至少 1280x720；
- H.264 或 H.265；
- 文件 SHA-256、run_id、mission_id、分辨率、时长和 codec 与 manifest 一致；
- 明确声明是连续操作画面且不是拼接占位。

## 3. 效率口径

唯一使用的正式效率公式：

```text
effective_cleaning_efficiency_m2_h
  = effective_cleaned_area_union_m2
  / full_mission_sim_duration_sec
  * 3600
```

receipt 同时给出：

- `coverage_stage_duration_sec` 和 `coverage_stage_efficiency_m2_h`；
- `wall_duration_sec` 和 `real_time_factor`，但只作独立报告；
- `wall_clock_used_for_efficiency=false`；
- `ten_second_steady_window_used=false`。

即使真实媒体文件是 10 秒稳态片段，也不能替换完整任务时长。当前完整性门
仍要求 5-10 分钟连续视频和接近完整 sim 时长的 MCAP。

## 4. 输出

指定 `--output-dir` 后生成：

| 文件 | 内容 |
|---|---|
| `full_mission_acceptance_receipt.json` | 总 receipt，包含每个门、证据、blockers |
| `demo_storyboard.json` | 8 分钟机器可读分镜 |
| `demo_storyboard.md` | 可交给人录制的分镜稿 |
| `demo_recording_checklist.json` | 录制与证据验收清单 |
| `demo_recording_checklist.md` | Markdown 勾选清单 |
| `demo_shot_index.csv` | 剪辑/验收使用的镜头索引 |

完整 receipt schema：

`schemas/day1_full_mission_acceptance_receipt.schema.json`

纯 fixture 样例：

`schemas/examples/day1_full_mission_acceptance_receipt.fixture.json`

样例带 `fixture_only=true`，不是项目运行结果，也不能复制进提交证据。

## 5. 运行命令

```powershell
py -3.13 scripts\day1_full_mission_acceptance.py `
  --run-dir F:\path\to\same-run `
  --video-dir F:\path\to\same-video `
  --output-dir F:\path\to\acceptance-output
```

只做单元测试：

```powershell
py -3.13 -m pytest -q scripts\test_day1_full_mission_acceptance.py
```

验证 fixture schema：

```powershell
py -3.13 -c "import json,jsonschema,pathlib; s=json.loads(pathlib.Path('schemas/day1_full_mission_acceptance_receipt.schema.json').read_text(encoding='utf-8')); x=json.loads(pathlib.Path('schemas/examples/day1_full_mission_acceptance_receipt.fixture.json').read_text(encoding='utf-8')); jsonschema.validate(x,s); print('PASS')"
```

## 6. 当前 coverage runner 的后续 patch 说明

当前 `scripts/run_day1_bounded_coverage_runner.sh` 只保存 coverage probe、
cleaning、safety/quality 和 probe 期间 MCAP；它尚未完成：


- 在 coverage 完成后通过正式任务接口请求 return-home，并等待完成；
- 记录完整任务 sim 时钟和 `mission_timeline.json`；
- 记录独立 `safety_receipt.json`、`return_home_receipt.json` 和
  `final_state.json`；
- 生成软硬件完整性 manifest；
- 把视频目录与同一 run 绑定。

建议的后续实现顺序，不直接修改当前共享 runner：

1. 在 coverage probe 返回后、清理 ROS/Gazebo 前，向正式任务接口发送
   return-home 请求；记录调用开始、完成、终点位姿和错误。
2. 在 coverage stage 之后追加 `return_home` 和 `shutdown` 阶段，写入
   `mission_timeline.json`；总时长从同 run `/clock` 或权威 sim-time
   recorder 取得，不使用 10 秒稳态窗口。
3. 把急停、collision monitor 和 keepout 原始证据写入
   `safety_receipt.json`，每个证据带 SHA-256。
4. 任务终态写入 `final_state.json`，必须证明 motion、brush、dirt 和急停
   均为安全释放状态。
5. 用同一 run 的源码索引、构建 receipt、测试 receipt、硬件组件清单、
   功率预算和板端运行 receipt 生成 `hardware_software_manifest.json`。
6. 录制 5-10 分钟连续操作视频，生成同 run `video_manifest.json`。
7. 按第 5 节命令运行 wrapper；只有 receipt 为 `PASS` 才进入提交包。

在上述 patch 完成前，对现有 bounded coverage run 运行 wrapper 应当返回
`FAIL`，这是预期行为，不得通过删除缺失门、复用旧 run、手工填写 PASS 或
降低阈值来绕过。

## 7. 提交边界

- 本 wrapper 不证明实车、HIL、真实道路或官方获奖；
- 本 wrapper 不把 coverage 面积自动扩张成 20,000 m2 建图结论；
- 本 wrapper 不做视频合成、不生成假 MCAP、不修改旧 run；
- 软硬件完整性 receipt 只证明提交证据齐备和哈希一致，最终判定仍由评委完成；
- 任何失败 run 都应保留原始目录和 blockers，供后续重跑和复盘。
