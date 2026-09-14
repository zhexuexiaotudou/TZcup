param(
    [string]$Root = (Split-Path -Parent $PSScriptRoot)
)

$ErrorActionPreference = "Stop"

$sourceVideoRoot = Join-Path $Root "video"
$packageVideoRoot = Join-Path $Root "submission-package/video"
$sourceFinal = Join-Path $sourceVideoRoot "final/TZcup_5min_video.mp4"
$sourceSilent = Join-Path $sourceVideoRoot "final/TZcup_5min_video_silent.mp4"
$packageFinal = Join-Path $packageVideoRoot "final/TZcup_5min_video.mp4"
$packageSilent = Join-Path $packageVideoRoot "final/TZcup_5min_video_silent.mp4"

function Write-Utf8NoBom {
    param([string]$Path, [string]$Content)
    $Content = $Content.Replace("`r`n", "`n").Replace("`r", "`n")
    [IO.File]::WriteAllText($Path, $Content, [Text.UTF8Encoding]::new($false))
}

function FileRecord {
    param([string]$Path, [string]$RelativePath, [string]$Role)
    $item = Get-Item -LiteralPath $Path
    $hash = (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant()
    return [ordered]@{
        path = $RelativePath
        role = $Role
        bytes = $item.Length
        sha256 = $hash
    }
}

function Get-Probe {
    param([string]$Path)
    return (& ffprobe -v error -count_frames -show_streams -show_format -of json $Path) | ConvertFrom-Json
}

$copyMap = @(
    @{ Source = $sourceFinal; Destination = $packageFinal },
    @{ Source = $sourceSilent; Destination = $packageSilent },
    @{ Source = (Join-Path $sourceVideoRoot "audio/narration.mp3"); Destination = (Join-Path $packageVideoRoot "audio/narration.mp3") },
    @{ Source = (Join-Path $sourceVideoRoot "audio/narration-script.md"); Destination = (Join-Path $packageVideoRoot "audio/narration-script.md") },
    @{ Source = (Join-Path $sourceVideoRoot "subtitles/zh-CN.srt"); Destination = (Join-Path $packageVideoRoot "subtitles/zh-CN.srt") },
    @{ Source = (Join-Path $sourceVideoRoot "chapters/chapters.json"); Destination = (Join-Path $packageVideoRoot "chapters/chapters.json") },
    @{ Source = (Join-Path $sourceVideoRoot "modeling/shot-list.json"); Destination = (Join-Path $packageVideoRoot "modeling/shot-list.json") },
    @{ Source = (Join-Path $sourceVideoRoot "functions/cue-sheet.json"); Destination = (Join-Path $packageVideoRoot "functions/cue-sheet.json") }
)

foreach ($entry in $copyMap) {
    $parent = Split-Path -Parent $entry.Destination
    New-Item -ItemType Directory -Force -Path $parent | Out-Null
    Copy-Item -LiteralPath $entry.Source -Destination $entry.Destination -Force
}

$probe = Get-Probe $packageFinal
Write-Utf8NoBom -Path (Join-Path $packageVideoRoot "final/ffprobe.json") -Content (($probe | ConvertTo-Json -Depth 12) + "`n")
Write-Utf8NoBom -Path (Join-Path $sourceVideoRoot "final/ffprobe.json") -Content (($probe | ConvertTo-Json -Depth 12) + "`n")

$videoStream = $probe.streams | Where-Object codec_type -eq "video" | Select-Object -First 1
$audioStream = $probe.streams | Where-Object codec_type -eq "audio" | Select-Object -First 1
$subtitleStream = $probe.streams | Where-Object codec_type -eq "subtitle" | Select-Object -First 1
$duration = [double]$probe.format.duration
$frameCount = [int]$videoStream.nb_read_frames

$finalRecord = FileRecord $packageFinal "final/TZcup_5min_video.mp4" "final_master_with_narration_and_embedded_subtitles"
$silentRecord = FileRecord $packageSilent "final/TZcup_5min_video_silent.mp4" "silent_video_master"
$audioRecord = FileRecord (Join-Path $packageVideoRoot "audio/narration.mp3") "audio/narration.mp3" "chinese_narration"
$narrationRecord = FileRecord (Join-Path $packageVideoRoot "audio/narration-script.md") "audio/narration-script.md" "narration_and_shot_script"
$subtitleRecord = FileRecord (Join-Path $packageVideoRoot "subtitles/zh-CN.srt") "subtitles/zh-CN.srt" "sidecar_chinese_subtitles"
$chapterRecord = FileRecord (Join-Path $packageVideoRoot "chapters/chapters.json") "chapters/chapters.json" "chapter_timeline"
$modelingRecord = FileRecord (Join-Path $packageVideoRoot "modeling/shot-list.json") "modeling/shot-list.json" "modeling_shot_list"
$functionRecord = FileRecord (Join-Path $packageVideoRoot "functions/cue-sheet.json") "functions/cue-sheet.json" "function_timeline"
$validatorRecord = FileRecord (Join-Path $packageVideoRoot "tools/validate_video_package.py") "tools/validate_video_package.py" "verification_tool"
$onePageRecord = FileRecord (Join-Path $packageVideoRoot "一页说明.md") "一页说明.md" "score_boundary_brief"
$indexDocRecord = FileRecord (Join-Path $packageVideoRoot "视频索引.md") "视频索引.md" "human_readable_index"

$manifest = [ordered]@{
    schema_version = 2
    artifact_kind = "tzcup_day1_five_minute_submission_video_v2"
    duration_s = $duration
    width = [int]$videoStream.width
    height = [int]$videoStream.height
    frame_rate = [string]$videoStream.r_frame_rate
    average_frame_rate = [string]$videoStream.avg_frame_rate
    frame_count = $frameCount
    video_codec = [string]$videoStream.codec_name
    audio_codec = [string]$audioStream.codec_name
    subtitle_codec = [string]$subtitleStream.codec_name
    narration_voice = "Microsoft Huihui Desktop"
    chapters = 9
    source_segments = @(
        "video/segments/01_modeling_v2.mp4",
        "video/segments/02_functions_v2.mp4"
    )
    files = @($finalRecord, $silentRecord, $audioRecord, $narrationRecord, $subtitleRecord, $chapterRecord, $modelingRecord, $functionRecord)
    claim_boundary = "Modular composite demonstration. It combines successful module footage and does not establish official full-mission PASS, live SLAM, avoidance rate, board perception or vehicle validation."
}

Write-Utf8NoBom -Path (Join-Path $sourceVideoRoot "final/manifest.json") -Content (($manifest | ConvertTo-Json -Depth 12) + "`n")
Write-Utf8NoBom -Path (Join-Path $packageVideoRoot "final/manifest.json") -Content (($manifest | ConvertTo-Json -Depth 12) + "`n")

$manifestRecord = FileRecord (Join-Path $packageVideoRoot "final/manifest.json") "final/manifest.json" "build_manifest"
$ffprobeRecord = FileRecord (Join-Path $packageVideoRoot "final/ffprobe.json") "final/ffprobe.json" "media_probe"
$files = @(
    $finalRecord,
    $silentRecord,
    $audioRecord,
    $narrationRecord,
    $subtitleRecord,
    $chapterRecord,
    $modelingRecord,
    $functionRecord,
    $manifestRecord,
    $ffprobeRecord,
    $indexDocRecord,
    $onePageRecord,
    $validatorRecord
)

$videoIndex = [ordered]@{
    schema_version = 2
    artifact_id = "tzcup-day1-modular-demo-video-v2"
    status = "DELIVERED_MODULAR_DEMO"
    official_full_mission_pass = $false
    modular_composite_demo = $true
    same_continuous_mission = $false
    official_item = [ordered]@{
        id = "COMP-DEMO"
        item = "演示视频清晰流畅"
        max_points = 4
        status = "DELIVERED_MODULAR_DEMO"
        recommended_points_range = @(2, 3)
        review_discretion_max_points = 4
        official_points_claimed = 0.0
        rationale = "The reel follows the requested simple-to-complex progression and includes modeling, motion, arm grasp, target approach, deposit, water cleaning, mapping, coverage planning and cleaning. It remains a modular composite rather than one continuous mission."
    }
    media_contract = [ordered]@{
        duration_s = $duration
        width = [int]$videoStream.width
        height = [int]$videoStream.height
        frame_rate = [string]$videoStream.r_frame_rate
        average_frame_rate = [string]$videoStream.avg_frame_rate
        frame_count = $frameCount
        video_codec = [string]$videoStream.codec_name
        audio_codec = [string]$audioStream.codec_name
        subtitle_codec = [string]$subtitleStream.codec_name
        sidecar_subtitle_cues = 35
        narration_language = "zh-CN"
        subtitle_language = "zh-CN"
    }
    demonstrated_content = @(
        "vehicle modeling with separate chassis, cleaning, arm, sensor and storage modules",
        "basic motion and path following",
        "planned robotic-arm motion",
        "target perception and vehicle approach",
        "grasp, transfer and onboard-bin release",
        "shallow-water cleaning",
        "offline deterministic 2D map reconstruction with 22399.99 square meters known area",
        "coverage planning with 12 swaths and 11 turns",
        "continuous cleaning execution"
    )
    source = [ordered]@{
        branch = "codex/day1-video-report-sync"
        builder = "../../scripts/build_day1_video_v2.ps1"
        asset_renderer = "../../scripts/render_day1_video_assets.py"
        synchronization = "../../scripts/sync_day1_video_v2.ps1"
    }
    evidence = [ordered]@{
        map_manifest = "evidence/day1/offline-raycast-mapping/offline_raycast_manifest.json"
        water_and_dirt = "evidence/complex-conditions-90s/complex_conditions_demo_90s.mp4"
        continuous_motion_safety = "evidence/continuous-cleaning-safety-450s/gazebo-cleaning-raw.mp4"
    }
    files = $files
    verification = [ordered]@{
        command = "py -3 tools\\validate_video_package.py --write"
        report = "validation/video-package-validation.json"
        checks = @(
            "paths_sizes_hashes",
            "json_parse",
            "srt_and_embedded_subtitles",
            "ffprobe_media_contract",
            "full_decode",
            "presentation_forbidden_terms"
        )
    }
    claim_boundary = "Modular composite demonstration only. It does not establish official full-mission PASS, live SLAM, avoidance rate, board perception or vehicle validation."
    cannot_claim = @(
        "official full-mission PASS",
        "all footage belongs to one continuous run",
        "official 20000 square meter live SLAM",
        "official avoidance >=95 percent",
        "physical vehicle validation"
    )
}

Write-Utf8NoBom -Path (Join-Path $packageVideoRoot "video-index.json") -Content (($videoIndex | ConvertTo-Json -Depth 12) + "`n")

$manifestLines = Get-ChildItem -LiteralPath $packageVideoRoot -Recurse -File |
    Where-Object { $_.Name -ne "MANIFEST.sha256" } |
    Sort-Object FullName |
    ForEach-Object {
        $relative = [IO.Path]::GetRelativePath($packageVideoRoot, $_.FullName).Replace("\", "/")
        $hash = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash.ToLowerInvariant()
        "$hash  $relative"
    }
Write-Utf8NoBom -Path (Join-Path $packageVideoRoot "MANIFEST.sha256") -Content (($manifestLines -join "`n") + "`n")

Push-Location $packageVideoRoot
try {
    py -3 tools/validate_video_package.py --write
    if ($LASTEXITCODE -ne 0) {
        throw "video package validation failed"
    }
} finally {
    Pop-Location
}

Write-Output (Join-Path $packageVideoRoot "final/TZcup_5min_video.mp4")
