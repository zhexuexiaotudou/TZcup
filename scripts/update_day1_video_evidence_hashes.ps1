param(
    [string]$Root = (Split-Path -Parent $PSScriptRoot)
)

$ErrorActionPreference = "Stop"
$package = Join-Path $Root "submission-package"
$evidenceIndexPath = Join-Path $package "evidence/index.json"
$reportBaselinePath = Join-Path $package "docs/report-baseline.json"
$integrationManifestPath = Join-Path $package "evidence/day1/integration-manifest.json"
$day1EvidenceIndexPath = Join-Path $package "evidence/day1/evidence-index.json"
$now = (Get-Date).ToUniversalTime().ToString("o")

function Read-Json {
    param([string]$Path)
    return Get-Content -LiteralPath $Path -Raw -Encoding UTF8 | ConvertFrom-Json
}

function Write-Json {
    param([string]$Path, [object]$Value)
    $json = $Value | ConvertTo-Json -Depth 30
    $json = ($json + "`n").Replace("`r`n", "`n").Replace("`r", "`n")
    [IO.File]::WriteAllText($Path, $json, [Text.UTF8Encoding]::new($false))
}

function Set-JsonProperty {
    param([object]$Object, [string]$Name, [object]$Value)
    if ($Object.PSObject.Properties.Name -contains $Name) {
        $Object.$Name = $Value
    } else {
        $Object | Add-Member -NotePropertyName $Name -NotePropertyValue $Value
    }
}

function Get-FileInfo {
    param([string]$Path)
    $item = Get-Item -LiteralPath $Path
    return [ordered]@{
        bytes = $item.Length
        sha256 = (Get-FileHash -LiteralPath $Path -Algorithm SHA256).Hash.ToLowerInvariant()
    }
}

$reportPath = Join-Path $package "docs/技术方案报告.pdf"
$reportInfo = Get-FileInfo $reportPath
$reportBaseline = Read-Json $reportBaselinePath
$reportBaseline.current_report.pages = 24
$reportBaseline.current_report.bytes = [int]$reportInfo.bytes
$reportBaseline.current_report.sha256 = [string]$reportInfo.sha256
$reportBaseline.current_report.updated_utc = "2026-09-15"
Write-Json $reportBaselinePath $reportBaseline

$integrationManifest = Read-Json $integrationManifestPath
$integrationManifest.integrated_report.pages = 24
$integrationManifest.integrated_report.bytes = [int]$reportInfo.bytes
$integrationManifest.integrated_report.sha256 = [string]$reportInfo.sha256

$videoIntegration = $integrationManifest.integrations |
    Where-Object id -eq "day1-modular-demo-video" |
    Select-Object -First 1
if ($null -ne $videoIntegration) {
    $videoIntegration.source_branch = "codex/day1-video-report-sync"
    $videoIntegration.claim_level = "DELIVERED_MODULAR_DEMO"
    $videoIntegration.official_points_claimed = 0.0
    $videoIntegration.recommended_points_range = @(2, 3)
    $videoIntegration.review_discretion_max_points = 4
    $videoIntegration.unmeasured_boundary = "The 300 second reel is a modular composite of successful module footage, not one continuous mission. It does not establish official full-mission PASS, live SLAM, avoidance rate, board perception, or vehicle validation."
    foreach ($file in $videoIntegration.files) {
        $relative = [string]$file.package_path
        $path = Join-Path $package $relative
        if (Test-Path -LiteralPath $path) {
            $info = Get-FileInfo $path
            if ($file.PSObject.Properties.Name -contains "bytes") {
                $file.bytes = [int]$info.bytes
            }
            if ($file.PSObject.Properties.Name -contains "sha256") {
                $file.sha256 = [string]$info.sha256
            }
        }
    }
}
Write-Json $integrationManifestPath $integrationManifest

$mediaFiles = [ordered]@{
    "video/final/TZcup_5min_video.mp4" = "final master"
    "video/final/TZcup_5min_video_silent.mp4" = "silent master"
    "video/audio/narration.mp3" = "narration"
    "video/audio/narration-script.md" = "narration script"
    "video/subtitles/zh-CN.srt" = "subtitles"
    "video/chapters/chapters.json" = "chapters"
    "video/modeling/shot-list.json" = "modeling shot list"
    "video/functions/cue-sheet.json" = "function cue sheet"
    "video/final/manifest.json" = "build manifest"
    "video/final/ffprobe.json" = "ffprobe"
    "video/video-index.json" = "video index"
    "video/validation/video-package-validation.json" = "validation report"
    "video/MANIFEST.sha256" = "video manifest"
    "metrics/video-score-status.json" = "video score status"
    "code/README.md" = "submission code README"
    "code/CODE_MANIFEST.sha256" = "submission code manifest"
    "metrics/score-estimate.md" = "internal official score estimate"
    "metrics/score-estimate.json" = "machine-readable internal official score estimate"
}

$evidenceIndex = @(Read-Json $evidenceIndexPath)
foreach ($entry in $evidenceIndex) {
    $relative = [string]$entry.file
    if ($mediaFiles.Contains($relative)) {
        $path = Join-Path $package $relative
        $info = Get-FileInfo $path
        Set-JsonProperty $entry "bytes" ([int]$info.bytes)
        Set-JsonProperty $entry "sha256" ([string]$info.sha256)
        Set-JsonProperty $entry "source" "codex/day1-video-report-sync: $($mediaFiles[$relative])"
        Set-JsonProperty $entry "source_branch" "codex/day1-video-report-sync"
        Set-JsonProperty $entry "copied_utc" $now
    }
}

foreach ($relative in $mediaFiles.Keys) {
    if (-not ($evidenceIndex | Where-Object file -eq $relative)) {
        $path = Join-Path $package $relative
        $info = Get-FileInfo $path
        $evidenceIndex += [pscustomobject][ordered]@{
            file = $relative
            source = "codex/day1-video-report-sync: $($mediaFiles[$relative])"
            source_branch = "codex/day1-video-report-sync"
            bytes = [int]$info.bytes
            sha256 = [string]$info.sha256
            copied_utc = $now
        }
    }
}

$repoRoot = Join-Path $package ".."
$reportRelative = "docs/技术方案报告.pdf"
$reportEntry = $evidenceIndex | Where-Object file -eq $reportRelative | Select-Object -First 1
if ($null -eq $reportEntry) {
    $reportEntry = [pscustomobject][ordered]@{
        file = $reportRelative
        source = "codex/day1-video-report-sync: regenerated report"
        source_branch = "codex/day1-video-report-sync"
        copied_utc = $now
    }
    $evidenceIndex += $reportEntry
}
Set-JsonProperty $reportEntry "bytes" ([int]$reportInfo.bytes)
Set-JsonProperty $reportEntry "sha256" ([string]$reportInfo.sha256)

foreach ($relative in @("docs/report-baseline.json", "evidence/day1/integration-manifest.json")) {
    $entry = $evidenceIndex | Where-Object file -eq $relative | Select-Object -First 1
    if ($null -ne $entry) {
        $info = Get-FileInfo (Join-Path $package $relative)
        Set-JsonProperty $entry "bytes" ([int]$info.bytes)
        Set-JsonProperty $entry "sha256" ([string]$info.sha256)
        Set-JsonProperty $entry "copied_utc" $now
    }
}

Write-Json $evidenceIndexPath $evidenceIndex

$day1Index = @(Read-Json $day1EvidenceIndexPath)
$videoEntry = $day1Index |
    Where-Object { $_.media -eq "video/final/TZcup_5min_video.mp4" } |
    Select-Object -First 1
if ($null -ne $videoEntry) {
    Set-JsonProperty $videoEntry "source_branch" "codex/day1-video-report-sync"
    Set-JsonProperty $videoEntry "status" "DELIVERED_MODULAR_DEMO"
    Set-JsonProperty $videoEntry "recommended_points_range" @(2, 3)
    Set-JsonProperty $videoEntry "review_discretion_max_points" 4
    Set-JsonProperty $videoEntry "official_points_claimed" 0.0
    Set-JsonProperty $videoEntry "accepted_facts" @(
        "the final media is 300.000 seconds, 1920x1080, H.264 with AAC audio and embedded Chinese subtitles",
        "the repaired master is constant 30 fps with 9000 decoded frames",
        "the reel follows a modeling-to-function progression including module decomposition, motion, arm motion, target approach, grasp and bin deposit, shallow-water cleaning, offline 2D map reconstruction, coverage planning, continuous cleaning and chaptered operation instructions"
    )
    Set-JsonProperty $videoEntry "limitations" @(
        "the reel is a modular composite and does not show one continuous mission",
        "the mapping section is an offline deterministic reconstruction rather than live SLAM",
        "it does not establish official full-mission, live-SLAM, avoidance-rate, board-perception or vehicle acceptance",
        "official points are not self-awarded; the bounded review band is 2-3 of 4"
    )
}
Write-Json $day1EvidenceIndexPath $day1Index

Write-Output "updated evidence hashes"
