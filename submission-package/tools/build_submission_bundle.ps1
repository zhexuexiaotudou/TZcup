param(
    [Parameter(Mandatory = $true)][string]$OutputZip
)

$root = Resolve-Path (Join-Path $PSScriptRoot "..")
$output = [System.IO.Path]::GetFullPath($OutputZip)
$checksum = "$output.sha256"

Compress-Archive -Path (Join-Path $root "*") -DestinationPath $output -CompressionLevel Optimal -Force
$hash = (Get-FileHash -Algorithm SHA256 -LiteralPath $output).Hash.ToLower()
"$hash  $([System.IO.Path]::GetFileName($output))" | Set-Content -LiteralPath $checksum -Encoding ascii

Add-Type -AssemblyName System.IO.Compression.FileSystem
$archive = [System.IO.Compression.ZipFile]::OpenRead($output)
try {
    Write-Output "entries=$($archive.Entries.Count)"
    $reportPdfEntries = @(
        $archive.Entries |
            Where-Object {
                $normalized = $_.FullName.Replace("\", "/")
                $normalized.StartsWith("docs/") -and $normalized.ToLower().EndsWith(".pdf")
            }
    ).Count
    Write-Output "report_pdf_entries=$reportPdfEntries"
    foreach ($required in @(
        "video/final/TZcup_5min_video.mp4",
        "video/evidence/continuous-cleaning-safety-450s/gazebo-cleaning-raw.mp4",
        "video/evidence/localization-causal-filter-60s/localization_tracking_success_1080p.mp4",
        "video/evidence/complex-conditions-90s/complex_conditions_demo_90s.mp4",
        "evidence/day1/avoidance-recovery/day1-avoidance-recovery-root-cause.md"
    )) {
        $present = @(
            $archive.Entries |
                Where-Object { $_.FullName.Replace("\", "/") -eq $required }
        ).Count -gt 0
        Write-Output "$required=$present"
    }
}
finally {
    $archive.Dispose()
}

Write-Output $hash
