param(
    [string]$Root = (Split-Path -Parent $PSScriptRoot)
)

$ErrorActionPreference = "Stop"

$segments = Join-Path $Root "video/segments"
$finalDir = Join-Path $Root "video/final"
$silent = Join-Path $finalDir "TZcup_5min_video_silent.mp4"
$final = Join-Path $finalDir "TZcup_5min_video.mp4"
$concat = Join-Path $segments "concat.txt"
$audio = Join-Path $Root "video/audio/narration.mp3"
$subtitles = Join-Path $Root "video/subtitles/zh-CN.srt"

New-Item -ItemType Directory -Force -Path $finalDir | Out-Null

@("file '01_modeling.mp4'", "file '02_functions.mp4'") |
    Set-Content -LiteralPath $concat -Encoding ascii

& ffmpeg -y -v error -f concat -safe 0 -i $concat -c copy `
    -movflags +faststart $silent

& ffmpeg -y -v error -i $silent -i $audio -i $subtitles `
    -map 0:v:0 -map 1:a:0 -map 2:s:0 `
    -c:v copy -c:a aac -b:a 192k -c:s mov_text `
    -metadata:s:s:0 language=zho `
    -metadata title="国产系统无人清扫车 5分钟成果演示" `
    -movflags +faststart -shortest $final

if ($LASTEXITCODE -ne 0) {
    throw "Final video assembly failed"
}

Write-Output $final
