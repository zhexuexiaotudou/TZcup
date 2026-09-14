param(
    [string]$Root = (Split-Path -Parent $PSScriptRoot)
)

$ErrorActionPreference = "Stop"

$segments = Join-Path $Root "video/segments"
$finalDir = Join-Path $Root "video/final"
$silent = Join-Path $finalDir "TZcup_5min_video_silent.mp4"
$final = Join-Path $finalDir "TZcup_5min_video.mp4"
$audio = Join-Path $Root "video/audio/narration.mp3"
$subtitles = Join-Path $Root "video/subtitles/zh-CN.srt"
$modeling = Join-Path $segments "01_modeling.mp4"
$functions = Join-Path $segments "02_functions.mp4"

New-Item -ItemType Directory -Force -Path $finalDir | Out-Null

$filter = "[0:v:0]fps=30,scale=1920:1080:flags=lanczos,setsar=1,format=yuv420p[v0];" +
          "[1:v:0]fps=30,scale=1920:1080:flags=lanczos,setsar=1,format=yuv420p[v1];" +
          "[v0][v1]concat=n=2:v=1:a=0[v]"

& ffmpeg -y -v error -i $modeling -i $functions `
    -filter_complex $filter -map "[v]" -an `
    -c:v libx264 -preset medium -crf 18 -pix_fmt yuv420p `
    -r 30 -fps_mode cfr -movflags +faststart $silent

& ffmpeg -y -v error -i $silent -i $audio -i $subtitles `
    -map 0:v:0 -map 1:a:0 -map 2:s:0 `
    -c:v copy -c:a aac -b:a 192k -c:s mov_text `
    -metadata:s:s:0 language=zho `
    -metadata title="TZcup 5-minute modular demonstration" `
    -t 300 -movflags +faststart $final

if ($LASTEXITCODE -ne 0) {
    throw "Final video assembly failed"
}

Write-Output $final
