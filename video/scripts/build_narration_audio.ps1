param(
    [string]$Timeline = (Join-Path $PSScriptRoot "narration_timeline.json"),
    [string]$OutputDirectory = (Join-Path (Split-Path (Split-Path $PSScriptRoot -Parent) -Parent) "video\audio")
)

$ErrorActionPreference = "Stop"
Add-Type -AssemblyName System.Speech

$timelinePath = (Resolve-Path -LiteralPath $Timeline).Path
$timelineData = Get-Content -LiteralPath $timelinePath -Raw -Encoding UTF8 | ConvertFrom-Json
$outputRoot = [System.IO.Path]::GetFullPath($OutputDirectory)
$tempRoot = Join-Path $outputRoot ".tts-work"
New-Item -ItemType Directory -Path $outputRoot -Force | Out-Null
if (Test-Path -LiteralPath $tempRoot) {
    Remove-Item -LiteralPath $tempRoot -Recurse -Force
}
New-Item -ItemType Directory -Path $tempRoot -Force | Out-Null

$synth = [System.Speech.Synthesis.SpeechSynthesizer]::new()
$voices = @($synth.GetInstalledVoices() | Where-Object { $_.Enabled })
$selectedVoice = $null
foreach ($preferred in @($timelineData.voice_preference)) {
    $selectedVoice = $voices | Where-Object { $_.VoiceInfo.Name -eq $preferred } | Select-Object -First 1
    if ($null -ne $selectedVoice) { break }
}
if ($null -eq $selectedVoice) {
    $selectedVoice = $voices | Where-Object { $_.VoiceInfo.Culture.Name -eq "zh-CN" } | Select-Object -First 1
}
if ($null -eq $selectedVoice) {
    $synth.Dispose()
    throw "No Chinese SAPI voice is installed."
}

$synth.SelectVoice($selectedVoice.VoiceInfo.Name)
$synth.Volume = 100

function Get-MediaDurationSeconds {
    param([string]$Path)
    $raw = & ffprobe -v error -show_entries format=duration -of default=noprint_wrappers=1:nokey=1 -- $Path
    if ($LASTEXITCODE -ne 0) { throw "ffprobe failed for $Path" }
    return [double]::Parse($raw.Trim(), [System.Globalization.CultureInfo]::InvariantCulture)
}

function Invoke-FFmpeg {
    param([string[]]$Arguments)
    & ffmpeg -hide_banner -loglevel error -y @Arguments
    if ($LASTEXITCODE -ne 0) { throw "ffmpeg failed: $($Arguments -join ' ')" }
}

$chunkPaths = [System.Collections.Generic.List[string]]::new()
$metadata = [System.Collections.Generic.List[object]]::new()

try {
    foreach ($segment in @($timelineData.segments)) {
        $id = [string]$segment.id
        $durationSec = [double]$segment.end_sec - [double]$segment.start_sec
        $durationMs = [int][math]::Round($durationSec * 1000)
        $rawPath = Join-Path $tempRoot "$id.raw.wav"
        $chunkPath = Join-Path $tempRoot "$id.chunk.wav"
        $speechDurationSec = $null
        $selectedRate = $null

        foreach ($rate in 0, 1, 2, 3, 4, 5, 6, 7) {
            $synth.Rate = $rate
            $synth.SetOutputToWaveFile($rawPath)
            $synth.Speak([string]$segment.speech)
            $synth.SetOutputToNull()
            $candidateDuration = Get-MediaDurationSeconds -Path $rawPath
            if ($candidateDuration -le ($durationSec - 0.30)) {
                $speechDurationSec = $candidateDuration
                $selectedRate = $rate
                break
            }
        }
        if ($null -eq $selectedRate) {
            throw "Narration segment $id exceeds its cue window."
        }

        $leadMs = [int][math]::Round([math]::Max(80, ($durationMs - $speechDurationSec * 1000) / 2))
        $leadMs = [math]::Min($leadMs, 350)
        $fadeFilter = "adelay=${leadMs}|${leadMs},apad,atrim=0:${durationSec}"
        Invoke-FFmpeg -Arguments @(
            "-i", $rawPath,
            "-af", $fadeFilter,
            "-t", $durationSec.ToString("0.###", [System.Globalization.CultureInfo]::InvariantCulture),
            "-ar", "48000",
            "-ac", "2",
            "-c:a", "pcm_s16le",
            $chunkPath
        )

        $chunkPaths.Add($chunkPath.Replace("\", "/"))
        $metadata.Add([pscustomobject]@{
            id = $id
            start_sec = [double]$segment.start_sec
            end_sec = [double]$segment.end_sec
            speech_duration_sec = [math]::Round($speechDurationSec, 3)
            lead_silence_ms = $leadMs
            tts_rate = $selectedRate
            speech = [string]$segment.speech
        })
    }

    $concatFile = Join-Path $tempRoot "concat.txt"
    $concatLines = foreach ($path in $chunkPaths) { "file '$path'" }
    [System.IO.File]::WriteAllLines($concatFile, $concatLines, [System.Text.UTF8Encoding]::new($false))

    $joinedWav = Join-Path $tempRoot "narration.joined.wav"
    Invoke-FFmpeg -Arguments @(
        "-f", "concat",
        "-safe", "0",
        "-i", $concatFile,
        "-c:a", "pcm_s16le",
        $joinedWav
    )

    $finalWav = Join-Path $outputRoot "narration.wav"
    Invoke-FFmpeg -Arguments @(
        "-i", $joinedWav,
        "-af", "loudnorm=I=-16:LRA=11:TP=-1.5",
        "-ar", "48000",
        "-ac", "2",
        "-c:a", "pcm_s16le",
        $finalWav
    )

    $finalMp3 = Join-Path $outputRoot "narration.mp3"
    Invoke-FFmpeg -Arguments @(
        "-i", $finalWav,
        "-c:a", "libmp3lame",
        "-b:a", "192k",
        $finalMp3
    )

    $wavItem = Get-Item -LiteralPath $finalWav
    $mp3Item = Get-Item -LiteralPath $finalMp3
    $metadataPath = Join-Path $outputRoot "tts-metadata.json"
    $metadataPayload = [ordered]@{
        schema_version = 1
        status = "PASS"
        engine = "Windows SAPI"
        voice = $selectedVoice.VoiceInfo.Name
        culture = $selectedVoice.VoiceInfo.Culture.Name
        duration_sec = Get-MediaDurationSeconds -Path $finalWav
        segments = @($metadata)
        outputs = @(
            [ordered]@{
                path = "narration.wav"
                bytes = $wavItem.Length
                sha256 = (Get-FileHash -LiteralPath $finalWav -Algorithm SHA256).Hash.ToLowerInvariant()
            },
            [ordered]@{
                path = "narration.mp3"
                bytes = $mp3Item.Length
                sha256 = (Get-FileHash -LiteralPath $finalMp3 -Algorithm SHA256).Hash.ToLowerInvariant()
            }
        )
    }
    $metadataPayload | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $metadataPath -Encoding UTF8
    $metadataPayload | ConvertTo-Json -Depth 8
}
finally {
    $synth.Dispose()
    if (Test-Path -LiteralPath $tempRoot) {
        Remove-Item -LiteralPath $tempRoot -Recurse -Force
    }
}
