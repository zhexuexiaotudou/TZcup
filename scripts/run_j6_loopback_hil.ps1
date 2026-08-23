[CmdletBinding()]
param(
    [string]$EvidenceDirectory = ""
)

$ErrorActionPreference = "Stop"
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$composeFile = Join-Path $repoRoot "docker\compose.journey6-loopback.yaml"
$projectName = "tzcup-j6-loopback"

function Require-EnvironmentVariable {
    param([Parameter(Mandatory = $true)][string]$Name)
    $value = [Environment]::GetEnvironmentVariable($Name)
    if ([string]::IsNullOrWhiteSpace($value)) {
        throw "Required environment variable is missing: $Name"
    }
    return $value
}

function Assert-AllowedMount {
    param(
        [Parameter(Mandatory = $true)][string]$Name,
        [Parameter(Mandatory = $true)][string]$Path
    )
    if (-not (Test-Path -LiteralPath $Path -PathType Container)) {
        throw "$Name is not an existing directory: $Path"
    }
    $resolved = (Resolve-Path -LiteralPath $Path).Path
    $tokens = ($resolved -replace '\\', '/').ToLowerInvariant().Split('/')
    $forbidden = @("ground_truth", "world", "worlds", "sealed", "evaluator")
    foreach ($token in $tokens) {
        if ($forbidden -contains $token) {
            throw "$Name resolves to a forbidden algorithm-container mount: $resolved"
        }
    }
    return $resolved
}

@(
    "J6_OE_BASE_IMAGE",
    "J6_ROS_SETUP",
    "J6_RUNTIME_BUNDLE",
    "J6_MODEL_ARTIFACTS",
    "J6_ALGORITHM_COMMAND"
) | ForEach-Object { Require-EnvironmentVariable -Name $_ | Out-Null }

$env:J6_RUNTIME_BUNDLE = Assert-AllowedMount `
    -Name "J6_RUNTIME_BUNDLE" -Path $env:J6_RUNTIME_BUNDLE
$env:J6_MODEL_ARTIFACTS = Assert-AllowedMount `
    -Name "J6_MODEL_ARTIFACTS" -Path $env:J6_MODEL_ARTIFACTS

if ([string]::IsNullOrWhiteSpace($EvidenceDirectory)) {
    $timestamp = [DateTime]::UtcNow.ToString("yyyyMMddTHHmmssZ")
    $EvidenceDirectory = Join-Path $repoRoot "artifacts\j6_loopback_hil_$timestamp"
}
[System.IO.Directory]::CreateDirectory($EvidenceDirectory) | Out-Null
$env:HIL_EVIDENCE_DIR = (Resolve-Path -LiteralPath $EvidenceDirectory).Path

& docker compose -f $composeFile -p $projectName --profile build-only `
    build j6-oe-wrapper
if ($LASTEXITCODE -ne 0) { throw "Journey 6 OE wrapper build failed." }
& docker compose -f $composeFile -p $projectName build discovery pc-gateway
if ($LASTEXITCODE -ne 0) { throw "PC HIL gateway build failed." }
& docker compose -f $composeFile -p $projectName build j6-algorithm
if ($LASTEXITCODE -ne 0) { throw "Journey 6 algorithm host build failed." }
& docker compose -f $composeFile -p $projectName up -d `
    discovery pc-gateway j6-algorithm
if ($LASTEXITCODE -ne 0) { throw "Journey 6 loopback startup failed." }
Start-Sleep -Seconds 2
& docker compose -f $composeFile -p $projectName exec -T j6-algorithm `
    /bin/bash -lc "ps -ef > /evidence/HIL_J6_PROCESS_LIST.txt"
if ($LASTEXITCODE -ne 0) { throw "Journey 6 process evidence capture failed." }
Get-CimInstance Win32_Process |
    Sort-Object ProcessId |
    ForEach-Object { "$($_.ProcessId)`t$($_.ParentProcessId)`t$($_.CommandLine)" } |
    Set-Content -LiteralPath (Join-Path $env:HIL_EVIDENCE_DIR "HIL_PC_PROCESS_LIST.txt") `
        -Encoding UTF8
& docker compose -f $composeFile -p $projectName ps
if ($LASTEXITCODE -ne 0) { throw "Journey 6 loopback status check failed." }

$rosDomainId = if ($env:ROS_DOMAIN_ID) { $env:ROS_DOMAIN_ID } else { "66" }
$pcEnvironment = @(
    "export ROS_DOMAIN_ID=$rosDomainId",
    "export ROS_DISCOVERY_SERVER=127.0.0.1:11811",
    "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
) -join "`n"
[System.IO.File]::WriteAllText(
    (Join-Path $env:HIL_EVIDENCE_DIR "HIL_PC_DDS_ENV.sh"),
    $pcEnvironment + "`n",
    [System.Text.UTF8Encoding]::new($false)
)

Write-Host "Journey 6 loopback infrastructure started."
Write-Host "Evidence: $env:HIL_EVIDENCE_DIR"
Write-Host "Source HIL_PC_DDS_ENV.sh before the PC sensor/plant-only graph."
