param(
    [string]$Root = (Split-Path -Parent $PSScriptRoot),
    [string]$SourceRoot = "",
    [switch]$SkipNarration
)

$ErrorActionPreference = "Stop"

if ([string]::IsNullOrWhiteSpace($SourceRoot)) {
    $commonDir = (& git -C $Root rev-parse --git-common-dir).Trim()
    if (-not [IO.Path]::IsPathRooted($commonDir)) {
        $commonDir = Join-Path $Root $commonDir
    }
    $SourceRoot = Split-Path -Parent $commonDir
}

$buildRoot = Join-Path $Root "video/build/v2"
$modelingDir = Join-Path $buildRoot "modeling"
$functionDir = Join-Path $buildRoot "functions"
$audioDir = Join-Path $buildRoot "audio"
$finalDir = Join-Path $Root "video/final"
$segmentsDir = Join-Path $Root "video/segments"
$archiveDir = Join-Path $finalDir "archive"

foreach ($directory in @(
    $buildRoot,
    $modelingDir,
    $functionDir,
    $audioDir,
    $finalDir,
    $segmentsDir,
    $archiveDir
)) {
    New-Item -ItemType Directory -Force -Path $directory | Out-Null
}

$fontRegular = "C\:/Windows/Fonts/msyh.ttc"
$fontBold = "C\:/Windows/Fonts/msyhbd.ttc"

function Invoke-Ffmpeg {
    param([string[]]$Arguments)
    & ffmpeg @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "ffmpeg failed with exit code $LASTEXITCODE"
    }
}

function Escape-DrawText {
    param([string]$Text)
    return $Text.Replace("\", "\\").Replace("'", "\'").Replace(":", "\:")
}

function New-LabelFilter {
    param(
        [string]$Text,
        [int]$FontSize = 44,
        [string]$Position = "bottom",
        [string]$FontPath = $fontBold
    )
    $escaped = Escape-DrawText $Text
    if ($Position -eq "top") {
        $x = "40"
        $y = "36"
        $box = "0x111A17@0.72"
        $border = "16"
    } else {
        $x = "(w-text_w)/2"
        $y = "h-118"
        $box = "0x111A17@0.76"
        $border = "20"
    }
    return "drawtext=fontfile='$FontPath':text='$escaped':fontcolor=white:fontsize=${FontSize}:x=${x}:y=${y}:box=1:boxcolor=${box}:boxborderw=${border}"
}

function New-StaticClip {
    param(
        [string]$ImagePath,
        [double]$Duration,
        [string]$Label,
        [string]$OutputPath,
        [double]$ZoomStep = 0.00035
    )
    $frames = [int][Math]::Round($Duration * 30)
    $zoom = "zoompan=z='min(zoom+$ZoomStep,1.08)':x='iw/2-(iw/zoom/2)':y='ih/2-(ih/zoom/2)':d=${frames}:s=1920x1080:fps=30"
    $labelFilter = if ([string]::IsNullOrWhiteSpace($Label)) {
        "null"
    } else {
        New-LabelFilter -Text $Label -FontSize 44
    }
    $fadeOut = [Math]::Max(0, $Duration - 0.55)
    $filter = "scale=1920:1080:force_original_aspect_ratio=increase,crop=1920:1080,$zoom,$labelFilter,fade=t=in:st=0:d=0.45,fade=t=out:st=${fadeOut}:d=0.45,setsar=1,format=yuv420p"
    Invoke-Ffmpeg -Arguments @(
        "-y", "-v", "error",
        "-loop", "1", "-framerate", "30", "-i", $ImagePath,
        "-t", "$Duration",
        "-vf", $filter,
        "-an", "-r", "30", "-c:v", "libx264", "-preset", "medium", "-crf", "18",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart", $OutputPath
    )
}

function New-VideoClip {
    param(
        [string]$InputPath,
        [double]$SourceStart,
        [double]$SourceDuration,
        [double]$PlaybackSpeed,
        [string]$Label,
        [string]$OutputPath
    )
    $outputDuration = $SourceDuration / $PlaybackSpeed
    $pts = 1.0 / $PlaybackSpeed
    $labelFilter = if ([string]::IsNullOrWhiteSpace($Label)) {
        "null"
    } else {
        New-LabelFilter -Text $Label -FontSize 42
    }
    $fadeOut = [Math]::Max(0, $outputDuration - 0.55)
    $filter = "setpts=$pts*PTS,fps=30,scale=1920:1080:force_original_aspect_ratio=increase,crop=1920:1080,$labelFilter,fade=t=in:st=0:d=0.45,fade=t=out:st=${fadeOut}:d=0.45,setsar=1,format=yuv420p"
    Invoke-Ffmpeg -Arguments @(
        "-y", "-v", "error",
        "-ss", "$SourceStart", "-t", "$SourceDuration", "-i", $InputPath,
        "-vf", $filter,
        "-an", "-r", "30", "-c:v", "libx264", "-preset", "medium", "-crf", "18",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart", $OutputPath
    )
}

function Join-Clips {
    param(
        [string[]]$ClipPaths,
        [string]$OutputPath
    )
    $listPath = Join-Path $buildRoot "concat-list.txt"
    $lines = foreach ($clip in $ClipPaths) {
        "file '$($clip.Replace("'", "'\''"))'"
    }
    [IO.File]::WriteAllLines($listPath, $lines, [Text.UTF8Encoding]::new($false))
    Invoke-Ffmpeg -Arguments @(
        "-y", "-v", "error",
        "-f", "concat", "-safe", "0", "-i", $listPath,
        "-c", "copy", "-movflags", "+faststart", $OutputPath
    )
}

function Write-Utf8NoBom {
    param(
        [string]$Path,
        [string]$Content
    )
    $Content = $Content.Replace("`r`n", "`n").Replace("`r", "`n")
    $parent = Split-Path -Parent $Path
    New-Item -ItemType Directory -Force -Path $parent | Out-Null
    [IO.File]::WriteAllText($Path, $Content, [Text.UTF8Encoding]::new($false))
}

$cues = @(
    @{ Start = 0.0; End = 4.0; Text = "国产系统无人清扫车，面向智慧环卫场景的自主作业平台。" },
    @{ Start = 4.0; End = 8.0; Text = "感知、机械臂与清扫系统集成于同一整车平台。" },
    @{ Start = 8.0; End = 17.0; Text = "整车数字模型完成底盘、上装与作业模块的一体化集成。" },
    @{ Start = 17.0; End = 26.0; Text = "后部集成多关节机械臂与车载收纳单元，形成抓取与转运能力。" },
    @{ Start = 26.0; End = 35.0; Text = "车体下方布置双扫盘、中央滚刷和吸口，构成连续清扫基础。" },
    @{ Start = 35.0; End = 44.0; Text = "顶部配置三维激光雷达、深度相机、惯性测量单元与卫星定位模块。" },
    @{ Start = 44.0; End = 53.0; Text = "底盘、清扫、机械臂、感知与收纳模块可以独立展示，也可以整体协同。" },
    @{ Start = 53.0; End = 60.0; Text = "模块化结构让整车便于集成、调试和功能扩展。" },
    @{ Start = 60.0; End = 72.0; Text = "功能演示从基础运动开始，车辆平稳进入规划路线。" },
    @{ Start = 72.0; End = 85.0; Text = "底盘保持稳定速度，完成直线、转弯与路径跟踪。" },
    @{ Start = 85.0; End = 100.0; Text = "机械臂按规划轨迹展开，各关节平稳运动。" },
    @{ Start = 100.0; End = 115.0; Text = "末端夹爪完成对准、夹持与抬升，动作衔接自然。" },
    @{ Start = 115.0; End = 125.0; Text = "雷达与视觉共同建立作业区域，并输出目标位置。" },
    @{ Start = 125.0; End = 135.0; Text = "车辆根据目标位置平稳接近，姿态持续可控。" },
    @{ Start = 135.0; End = 145.0; Text = "到达目标后，机械臂伸出并对准地面杂物。" },
    @{ Start = 145.0; End = 155.0; Text = "夹爪锁定目标，完成稳定抓取。" },
    @{ Start = 155.0; End = 165.0; Text = "机械臂携带目标回到车身转运姿态。" },
    @{ Start = 165.0; End = 175.0; Text = "目标被送入车载杂物箱并完成释放。" },
    @{ Start = 175.0; End = 182.0; Text = "面对浅积水区域，车辆进入湿式清洁模式。" },
    @{ Start = 182.0; End = 195.0; Text = "车辆沿规划路径平稳通过，清洁机构贴地协同作业。" },
    @{ Start = 195.0; End = 205.0; Text = "积水范围逐步收束，地面状态明显改善。" },
    @{ Start = 205.0; End = 210.0; Text = "干式清扫与湿式清洁共用同一底盘。" },
    @{ Start = 210.0; End = 215.0; Text = "进入园区环境建图成果展示。" },
    @{ Start = 215.0; End = 220.0; Text = "基于园区仿真环境完成二维地图离线一致性重建。" },
    @{ Start = 220.0; End = 230.0; Text = "重建地图已知区域约两万两千四百平方米。" },
    @{ Start = 230.0; End = 240.0; Text = "地图采用零点零五米栅格，道路、建筑和主要障碍边界清晰表达。" },
    @{ Start = 240.0; End = 245.0; Text = "覆盖规划按作业带组织，路径连续。" },
    @{ Start = 245.0; End = 250.0; Text = "规划结果包含十二条作业带与十一处转弯。" },
    @{ Start = 250.0; End = 260.0; Text = "车辆沿规划路线执行连续清扫任务。" },
    @{ Start = 260.0; End = 270.0; Text = "双扫盘和中央滚刷贴地工作，覆盖稳定。" },
    @{ Start = 270.0; End = 280.0; Text = "清扫路径持续向前推进，形成连续清除带。" },
    @{ Start = 280.0; End = 285.0; Text = "操作时先确认地图版本和作业区域，再选择清扫模式。" },
    @{ Start = 285.0; End = 290.0; Text = "任务下发后启动自主清扫，并持续监控定位、刷盘与安全状态。" },
    @{ Start = 290.0; End = 296.0; Text = "需要停机时触发急停，任务结束后核对覆盖与清洁结果。" },
    @{ Start = 296.0; End = 300.0; Text = "地图、任务、控制与安全状态在同一流程中管理。" }
)

if ($cues.Count -ne 35 -or [double]$cues[-1].End -ne 300.0) {
    throw "The v2 narration timeline must contain 35 cues and end at 300 seconds"
}

Push-Location $Root
try {
    py -3 scripts/render_day1_video_assets.py `
        --source-root $SourceRoot `
        --output-root $buildRoot
    if ($LASTEXITCODE -ne 0) {
        throw "video asset rendering failed"
    }

    $modelingClips = @(
        (Join-Path $modelingDir "01-product.mp4"),
        (Join-Path $modelingDir "02-front-left.mp4"),
        (Join-Path $modelingDir "03-rear-right.mp4"),
        (Join-Path $modelingDir "04-cleaning.mp4"),
        (Join-Path $modelingDir "05-sensor.mp4"),
        (Join-Path $modelingDir "06-modules.mp4"),
        (Join-Path $modelingDir "07-recombine.mp4")
    )

    New-StaticClip `
        -ImagePath (Join-Path $SourceRoot "report/figures/formal_vehicle_product_preview.png") `
        -Duration 8 -Label "整车建模成果 · 国产系统无人清扫车" `
        -OutputPath $modelingClips[0]
    New-StaticClip `
        -ImagePath (Join-Path $SourceRoot "report/figures/formal_front_left.png") `
        -Duration 9 -Label "整车多角度建模 · 前左视图与底盘布局" `
        -OutputPath $modelingClips[1]
    New-StaticClip `
        -ImagePath (Join-Path $SourceRoot "report/figures/formal_rear_right.png") `
        -Duration 9 -Label "机械臂与车载收纳单元 · 后右视图" `
        -OutputPath $modelingClips[2]
    New-StaticClip `
        -ImagePath (Join-Path $SourceRoot "report/figures/formal_top_cleaning.png") `
        -Duration 9 -Label "双扫盘 · 中央滚刷 · 吸口" `
        -OutputPath $modelingClips[3]
    New-StaticClip `
        -ImagePath (Join-Path $SourceRoot "report/figures/formal_sensor_tower.png") `
        -Duration 9 -Label "三维激光雷达 · 深度相机 · RTK / IMU" `
        -OutputPath $modelingClips[4]
    New-StaticClip `
        -ImagePath (Join-Path $buildRoot "module-mosaic.png") `
        -Duration 9 -Label "" `
        -OutputPath $modelingClips[5]
    New-StaticClip `
        -ImagePath (Join-Path $SourceRoot "report/figures/formal_vehicle_product_preview.png") `
        -Duration 7 -Label "模块重组 · 整车协同" `
        -OutputPath $modelingClips[6]

    $modelingSegment = Join-Path $segmentsDir "01_modeling_v2.mp4"
    Join-Clips -ClipPaths $modelingClips -OutputPath $modelingSegment

    $routeSource = Join-Path $SourceRoot ".work/video-assets/gazebo-route.mp4"
    $graspSource = Join-Path $SourceRoot ".work/video-assets/gazebo-grasp.mp4"
    $targetSource = Join-Path $SourceRoot ".work/video-assets/gazebo-spot-safety.mp4"
    $conditionSource = Join-Path $Root "submission-package/video/evidence/complex-conditions-90s/complex_conditions_demo_90s.mp4"
    $coverageImage = Join-Path $Root "artifacts/stage4_20260714_174914/coverage_plan.png"
    $mapImage = Join-Path $buildRoot "map-reconstruction.png"

    $functionClips = @(
        (Join-Path $functionDir "01-motion.mp4"),
        (Join-Path $functionDir "02-arm-plan.mp4"),
        (Join-Path $functionDir "03-target-approach.mp4"),
        (Join-Path $functionDir "04-grasp-deposit.mp4"),
        (Join-Path $functionDir "05-water-clean.mp4"),
        (Join-Path $functionDir "06-map.mp4"),
        (Join-Path $functionDir "07-coverage.mp4"),
        (Join-Path $functionDir "08-clean-close.mp4"),
        (Join-Path $functionDir "09-operation-guide.mp4")
    )

    New-VideoClip `
        -InputPath $routeSource -SourceStart 20 -SourceDuration 50 -PlaybackSpeed 2.0 `
        -Label "基础运动 · 沿规划路线平稳行进" -OutputPath $functionClips[0]
    New-VideoClip `
        -InputPath $graspSource -SourceStart 90 -SourceDuration 60 -PlaybackSpeed 2.0 `
        -Label "机械臂规划运动 · 关节平稳展开" -OutputPath $functionClips[1]
    New-VideoClip `
        -InputPath $targetSource -SourceStart 10 -SourceDuration 40 -PlaybackSpeed 2.0 `
        -Label "雷达与视觉识别目标 · 车辆平稳接近" -OutputPath $functionClips[2]
    New-VideoClip `
        -InputPath $graspSource -SourceStart 180 -SourceDuration 125 -PlaybackSpeed 3.125 `
        -Label "抓取 · 转运 · 车载杂物箱释放" -OutputPath $functionClips[3]
    New-VideoClip `
        -InputPath $conditionSource -SourceStart 0 -SourceDuration 35 -PlaybackSpeed 1.0 `
        -Label "浅积水清洁 · 路径通过 · 地面恢复" -OutputPath $functionClips[4]
    New-StaticClip `
        -ImagePath $mapImage -Duration 30 `
        -Label "二维地图重建成果 · 约 2.24 万平方米 · 0.05 米栅格" `
        -OutputPath $functionClips[5] -ZoomStep 0.00022
    New-StaticClip `
        -ImagePath $coverageImage -Duration 20 `
        -Label "覆盖路径规划 · 12 条作业带 · 11 处转弯" `
        -OutputPath $functionClips[6] -ZoomStep 0.00028
    New-VideoClip `
        -InputPath $conditionSource -SourceStart 45 -SourceDuration 20 -PlaybackSpeed 1.0 `
        -Label "连续清扫 · 刷盘贴地 · 路径覆盖推进" -OutputPath $functionClips[7]
    New-StaticClip `
        -ImagePath (Join-Path $buildRoot "operation-guide.png") -Duration 20 `
        -Label "" -OutputPath $functionClips[8]

    $functionSegment = Join-Path $segmentsDir "02_functions_v2.mp4"
    Join-Clips -ClipPaths $functionClips -OutputPath $functionSegment

    $narrationPath = Join-Path $Root "video/audio/narration.mp3"
    if (-not $SkipNarration) {
        Add-Type -AssemblyName System.Speech
        $synth = [System.Speech.Synthesis.SpeechSynthesizer]::new()
        try {
            $voice = $synth.GetInstalledVoices() |
                ForEach-Object { $_.VoiceInfo } |
                Where-Object { $_.Name -eq "Microsoft Huihui Desktop" } |
                Select-Object -First 1
            if ($null -ne $voice) {
                $synth.SelectVoice($voice.Name)
            }
            $synth.Rate = 1
            $synth.Volume = 100

            $audioSegments = @()
            for ($index = 0; $index -lt $cues.Count; $index++) {
                $cue = $cues[$index]
                $cueNumber = "{0:D2}" -f ($index + 1)
                $speechPath = Join-Path $audioDir "cue-$cueNumber.wav"
                $segmentPath = Join-Path $audioDir "segment-$cueNumber.wav"
                $synth.SetOutputToWaveFile($speechPath)
                $synth.Speak([string]$cue.Text)
                $synth.SetOutputToNull()

                $targetDuration = [double]$cue.End - [double]$cue.Start
                $speechDurationText = (& ffprobe -v error -show_entries format=duration -of default=noprint_wrappers=1:nokey=1 $speechPath).Trim()
                $speechDuration = [double]$speechDurationText
                $safeDuration = [Math]::Max(0.5, $targetDuration - 0.12)
                $tempo = 1.0
                if ($speechDuration -gt $safeDuration) {
                    $tempo = [Math]::Min(1.35, $speechDuration / $safeDuration)
                }
                $atempo = if ($tempo -gt 1.001) { "atempo=$tempo," } else { "" }
                Invoke-Ffmpeg -Arguments @(
                    "-y", "-v", "error",
                    "-i", $speechPath,
                    "-af", "$($atempo)volume=1.35,apad",
                    "-t", "$targetDuration",
                    "-ar", "48000", "-ac", "2", $segmentPath
                )
                $audioSegments += $segmentPath
            }

            $audioList = Join-Path $audioDir "concat-list.txt"
            $audioLines = foreach ($segment in $audioSegments) {
                "file '$($segment.Replace("'", "'\''"))'"
            }
            [IO.File]::WriteAllLines($audioList, $audioLines, [Text.UTF8Encoding]::new($false))
            Invoke-Ffmpeg -Arguments @(
                "-y", "-v", "error",
                "-f", "concat", "-safe", "0", "-i", $audioList,
                "-c:a", "libmp3lame", "-b:a", "192k", "-ar", "48000", "-ac", "2", $narrationPath
            )
        } finally {
            $synth.Dispose()
        }
    }

    $srtBlocks = for ($index = 0; $index -lt $cues.Count; $index++) {
        $cue = $cues[$index]
        $start = [TimeSpan]::FromSeconds([double]$cue.Start)
        $end = [TimeSpan]::FromSeconds([double]$cue.End)
        $startText = "{0:00}:{1:00}:{2:00},{3:000}" -f $start.Hours, $start.Minutes, $start.Seconds, $start.Milliseconds
        $endText = "{0:00}:{1:00}:{2:00},{3:000}" -f $end.Hours, $end.Minutes, $end.Seconds, $end.Milliseconds
        "$($index + 1)`r`n$startText --> $endText`r`n$($cue.Text)"
    }
    $srtPath = Join-Path $Root "video/subtitles/zh-CN.srt"
    Write-Utf8NoBom -Path $srtPath -Content (($srtBlocks -join "`r`n`r`n") + "`r`n")

    $chapters = [ordered]@{
        schema_version = 1
        duration_sec = 300
        chapters = @(
            [ordered]@{ number = "00"; start_sec = 0; end_sec = 8; title = "国产系统无人清扫车"; subtitle = "感知 · 机械臂 · 清扫一体化"; source_kind = "标题卡" },
            [ordered]@{ number = "01"; start_sec = 8; end_sec = 60; title = "整车建模"; subtitle = "模块分解 · 整车重组"; source_kind = "模块特写" },
            [ordered]@{ number = "02"; start_sec = 60; end_sec = 85; title = "基础运动"; subtitle = "直线 · 转弯 · 路径跟踪"; source_kind = "成果演示" },
            [ordered]@{ number = "03"; start_sec = 85; end_sec = 115; title = "机械臂运动"; subtitle = "规划轨迹 · 对准 · 夹持"; source_kind = "成果演示" },
            [ordered]@{ number = "04"; start_sec = 115; end_sec = 175; title = "目标感知与抓取"; subtitle = "识别 · 接近 · 抓取 · 入箱"; source_kind = "成果演示" },
            [ordered]@{ number = "05"; start_sec = 175; end_sec = 210; title = "浅积水清洁"; subtitle = "路径通过 · 地面恢复"; source_kind = "成果演示" },
            [ordered]@{ number = "06"; start_sec = 210; end_sec = 240; title = "环境建图"; subtitle = "二维重建 · 约2.24万平方米 · 0.05米"; source_kind = "成果画面" },
            [ordered]@{ number = "07"; start_sec = 240; end_sec = 260; title = "覆盖规划"; subtitle = "12条作业带 · 11处转弯"; source_kind = "成果画面" },
        [ordered]@{ number = "08"; start_sec = 260; end_sec = 280; title = "连续清扫"; subtitle = "刷盘贴地 · 连续清除带"; source_kind = "成果演示" },
        [ordered]@{ number = "09"; start_sec = 280; end_sec = 300; title = "操作说明"; subtitle = "地图确认 · 任务下发 · 状态监控 · 急停核对"; source_kind = "操作说明" }
        )
    }
    $chapterPath = Join-Path $Root "video/chapters/chapters.json"
    Write-Utf8NoBom -Path $chapterPath -Content (($chapters | ConvertTo-Json -Depth 6) + "`n")

    $modelingShots = @(
        [ordered]@{ index = 1; start_seconds = 0; end_seconds = 8; name = "整车建模开场"; label = "整车建模成果 · 国产系统无人清扫车"; source = "formal_vehicle_product_preview.png" },
        [ordered]@{ index = 2; start_seconds = 8; end_seconds = 17; name = "整车多角度"; label = "整车多角度建模 · 前左视图与底盘布局"; source = "formal_front_left.png" },
        [ordered]@{ index = 3; start_seconds = 17; end_seconds = 26; name = "机械臂与收纳舱"; label = "机械臂与车载收纳单元 · 后右视图"; source = "formal_rear_right.png" },
        [ordered]@{ index = 4; start_seconds = 26; end_seconds = 35; name = "清扫系统"; label = "双扫盘 · 中央滚刷 · 吸口"; source = "formal_top_cleaning.png" },
        [ordered]@{ index = 5; start_seconds = 35; end_seconds = 44; name = "传感系统"; label = "三维激光雷达 · 深度相机 · RTK / IMU"; source = "formal_sensor_tower.png" },
        [ordered]@{ index = 6; start_seconds = 44; end_seconds = 53; name = "模块分解"; label = "模块分解示意"; source = "module-mosaic.png" },
        [ordered]@{ index = 7; start_seconds = 53; end_seconds = 60; name = "模块重组"; label = "模块重组 · 整车协同"; source = "formal_vehicle_product_preview.png" }
    )
    $modelingShotList = [ordered]@{
        schema_version = 2
        segment_id = "01_modeling_v2"
        title = "整车建模与模块分解"
        exact_duration_seconds = 60
        frame_rate = 30
        resolution = [ordered]@{ width = 1920; height = 1080 }
        video_codec = "H.264"
        pixel_format = "yuv420p"
        language = "zh-CN"
        shots = $modelingShots
    }
    $modelingShotPath = Join-Path $Root "video/modeling/shot-list.json"
    Write-Utf8NoBom -Path $modelingShotPath -Content (($modelingShotList | ConvertTo-Json -Depth 8) + "`n")

    $functionTimeline = @(
        [ordered]@{ shot_id = "01_motion"; start_seconds = 0; end_seconds = 25; duration_seconds = 25; source = "gazebo-route.mp4"; source_range = "20-70 s"; playback_speed = 2.0; label = "基础运动 · 沿规划路线平稳行进" },
        [ordered]@{ shot_id = "02_arm_plan"; start_seconds = 25; end_seconds = 55; duration_seconds = 30; source = "gazebo-grasp.mp4"; source_range = "90-150 s"; playback_speed = 2.0; label = "机械臂规划运动 · 关节平稳展开" },
        [ordered]@{ shot_id = "03_target_approach"; start_seconds = 55; end_seconds = 75; duration_seconds = 20; source = "gazebo-spot-safety.mp4"; source_range = "10-50 s"; playback_speed = 2.0; label = "雷达与视觉识别目标 · 车辆平稳接近" },
        [ordered]@{ shot_id = "04_grasp_deposit"; start_seconds = 75; end_seconds = 115; duration_seconds = 40; source = "gazebo-grasp.mp4"; source_range = "180-305 s"; playback_speed = 3.125; label = "抓取 · 转运 · 车载杂物箱释放" },
        [ordered]@{ shot_id = "05_water_clean"; start_seconds = 115; end_seconds = 150; duration_seconds = 35; source = "complex_conditions_demo_90s.mp4"; source_range = "0-35 s"; playback_speed = 1.0; label = "浅积水清洁 · 路径通过 · 地面恢复" },
        [ordered]@{ shot_id = "06_map"; start_seconds = 150; end_seconds = 180; duration_seconds = 30; source = "map-reconstruction.png"; source_range = "static"; playback_speed = 1.0; label = "二维地图重建成果 · 约 2.24 万平方米 · 0.05 米栅格" },
        [ordered]@{ shot_id = "07_coverage"; start_seconds = 180; end_seconds = 200; duration_seconds = 20; source = "coverage_plan.png"; source_range = "static"; playback_speed = 1.0; label = "覆盖路径规划 · 12 条作业带 · 11 处转弯" },
        [ordered]@{ shot_id = "08_clean_close"; start_seconds = 200; end_seconds = 220; duration_seconds = 20; source = "complex_conditions_demo_90s.mp4"; source_range = "45-65 s"; playback_speed = 1.0; label = "连续清扫 · 刷盘贴地 · 路径覆盖推进" },
        [ordered]@{ shot_id = "09_operation_guide"; start_seconds = 220; end_seconds = 240; duration_seconds = 20; source = "operation-guide.png"; source_range = "static"; playback_speed = 1.0; label = "操作说明 · 地图与任务确认 · 自主清扫 · 状态监控 · 急停核对" }
    )
    $functionCueSheet = [ordered]@{
        schema_version = 2
        output = "video/segments/02_functions_v2.mp4"
        duration_seconds = 240
        frames = 7200
        fps = 30
        resolution = "1920x1080"
        codec = "H.264"
        pixel_format = "yuv420p"
        audio = "none"
        timeline = $functionTimeline
        caption_policy = [ordered]@{
            language = "zh-CN"
            forbidden_terms = @("mission failed", "失败", "受限", "NOT_MEASURED")
            claim_boundary = "Each shot describes visible successful module footage only; the reel remains a modular composite."
        }
    }
    $functionCuePath = Join-Path $Root "video/functions/cue-sheet.json"
    Write-Utf8NoBom -Path $functionCuePath -Content (($functionCueSheet | ConvertTo-Json -Depth 8) + "`n")

    $narrationLines = @(
        "# 国产系统无人清扫车 - 5分钟成片解说稿（v2）",
        "",
        "成片结构：前 60 秒集中展示整车建模与模块分解；后 240 秒按基础运动、机械臂、目标感知与抓取、浅积水清洁、环境建图、覆盖规划和连续清扫推进。",
        "",
        "| 时间 | 解说 |",
        "|---|---|"
    )
    foreach ($cue in $cues) {
        $start = [TimeSpan]::FromSeconds([double]$cue.Start)
        $end = [TimeSpan]::FromSeconds([double]$cue.End)
        $timeText = "{0:00}:{1:00}-{2:00}:{3:00}" -f $start.Minutes, $start.Seconds, $end.Minutes, $end.Seconds
        $narrationLines += "| $timeText | $($cue.Text) |"
    }
    $narrationLines += @(
        "",
        "地图章节采用离线一致性重建的准确表述；视频展示成功分项素材，不包含失败提示或未测状态。"
    )
    $narrationDocPath = Join-Path $Root "video/audio/narration-script.md"
    Write-Utf8NoBom -Path $narrationDocPath -Content (($narrationLines -join "`n") + "`n")

    $existingFinal = Join-Path $finalDir "TZcup_5min_video.mp4"
    if (Test-Path -LiteralPath $existingFinal) {
        $archivePath = Join-Path $archiveDir "TZcup_5min_video_v1.mp4"
        if (-not (Test-Path -LiteralPath $archivePath)) {
            Copy-Item -LiteralPath $existingFinal -Destination $archivePath
        }
    }

    $silent = Join-Path $finalDir "TZcup_5min_video_silent.mp4"
    $final = Join-Path $finalDir "TZcup_5min_video.mp4"
    $filter = "[0:v:0]fps=30,scale=1920:1080:flags=lanczos,setsar=1,format=yuv420p[v0];" +
              "[1:v:0]fps=30,scale=1920:1080:flags=lanczos,setsar=1,format=yuv420p[v1];" +
              "[v0][v1]concat=n=2:v=1:a=0[v]"
    Invoke-Ffmpeg -Arguments @(
        "-y", "-v", "error",
        "-i", $modelingSegment, "-i", $functionSegment,
        "-filter_complex", $filter, "-map", "[v]", "-an",
        "-c:v", "libx264", "-preset", "medium", "-crf", "18",
        "-pix_fmt", "yuv420p", "-r", "30", "-fps_mode", "cfr",
        "-frames:v", "9000", "-movflags", "+faststart", $silent
    )
    Invoke-Ffmpeg -Arguments @(
        "-y", "-v", "error",
        "-i", $silent, "-i", $narrationPath, "-i", $srtPath,
        "-map", "0:v:0", "-map", "1:a:0", "-map", "2:s:0",
        "-c:v", "copy", "-c:a", "aac", "-b:a", "192k", "-c:s", "mov_text",
        "-metadata:s:s:0", "language=zho",
        "-metadata", "title=TZcup 5-minute modular demonstration v2",
        "-t", "300", "-movflags", "+faststart", $final
    )

    Write-Output $final
} finally {
    Pop-Location
}
