# 国产系统无人清扫车 5 分钟成果演示（v2）

最终文件：`video/final/TZcup_5min_video.mp4`

## 成片结构

- `00:00-01:00`：整车多角度、前左视图、机械臂与收纳舱、双扫盘与滚刷、LiDAR/深度相机/RTK/IMU、四模块分解和整车重组。
- `01:00-01:25`：基础运动、直线、转弯和路径跟踪。
- `01:25-01:55`：机械臂规划运动、关节展开、夹持与抬升。
- `01:55-02:55`：目标感知、车辆接近、抓取、转运和车载杂物箱释放。
- `02:55-03:30`：浅积水清洁、路径通过和地面状态恢复。
- `03:30-04:00`：二维地图离线一致性重建，约 2.24 万平方米已知区域、0.05 米栅格。
- `04:00-04:20`：覆盖路径规划，12 条作业带、11 处转弯。
- `04:20-04:40`：连续清扫、刷盘贴地和任务收束。
- `04:40-05:00`：操作说明，覆盖地图确认、任务下发、状态监控、急停与结果核对。

视频只采用已完成或已闭环的分项素材，不使用失败提示画面。不同分项来自不同仿真运行；剪辑用于模块成果展示，不表示所有镜头属于同一连续任务。

## 交付

- `video/final/TZcup_5min_video.mp4`：1920x1080、恒定 30 fps（9000 帧）、H.264、AAC、内嵌中文字幕轨、300.000 s。
- `video/final/TZcup_5min_video_silent.mp4`：无音轨母版。
- `video/audio/narration.mp3`：中文旁白。
- `video/subtitles/zh-CN.srt`：逐句中文字幕。
- `video/chapters/chapters.json`：章节时间轴。
- `video/modeling/shot-list.json`：开场建模镜头表。
- `video/functions/cue-sheet.json`：功能段素材起止和用途。
- `video/final/ffprobe.json`：最终媒体容器与流信息。
- `video/final/archive/TZcup_5min_video_v1.mp4`：旧成片归档。

## 复现

```powershell
powershell -NoProfile -File .\scripts\build_day1_video_v2.ps1
powershell -NoProfile -File .\scripts\sync_day1_video_v2.ps1
```
