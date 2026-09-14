# 国产系统无人清扫车 5 分钟成果演示

最终文件：`video/final/TZcup_5min_video.mp4`

## 成片结构

- `00:00-01:00`：整车多角度、底盘、刷盘与吸口、UR5e 机械臂、LiDAR/深度相机/RTK、干湿箱和模块分解展示。
- `01:00-02:10`：基础运动、路径跟踪、机械臂接近抓取和抬升。
- `02:10-03:00`：感知目标、接近、抓取和车载箱体收纳。
- `03:00-03:40`：水渍清洁。
- `03:40-04:20`：环境建图和定位成果。
- `04:20-05:00`：刷盘清扫、清除带、安全响应和全系统收束。

视频只采用已完成或已闭环的分项素材，不使用失败提示画面。不同分项来自不同仿真运行；剪辑用于模块成果展示，不表示所有镜头属于同一连续任务。

## 交付

- `video/final/TZcup_5min_video.mp4`：1920x1080、30 fps、H.264、AAC、内嵌中文字幕轨、300.000 s。
- `video/final/TZcup_5min_video_silent.mp4`：无音轨母版。
- `video/audio/narration.mp3`：中文旁白。
- `video/subtitles/zh-CN.srt`：逐句中文字幕。
- `video/chapters/chapters.json`：章节时间轴。
- `video/modeling/shot-list.json`：开场建模镜头表。
- `video/functions/cue-sheet.json`：功能段素材起止和用途。
- `video/final/ffprobe.json`：最终媒体容器与流信息。

## 复现

```powershell
powershell -NoProfile -File .\scripts\assemble_day1_video_final.ps1
```
