[CmdletBinding()]
param(
    [string]$EvidenceDirectory = "",
    [ValidateSet("JOURNEY6_OE", "PC_ONNX")]
    [string]$RuntimeBackend = "JOURNEY6_OE",
    [ValidateRange(10, 86400)]
    [int]$DurationSeconds = 30,
    [ValidateSet("synthetic_transport_probe", "gazebo")]
    [string]$SensorSource = "synthetic_transport_probe"
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

if ($RuntimeBackend -eq "JOURNEY6_OE") {
    @(
        "J6_OE_BASE_IMAGE",
        "J6_ROS_SETUP",
        "J6_RUNTIME_BUNDLE",
        "J6_MODEL_ARTIFACTS",
        "J6_ALGORITHM_COMMAND"
    ) | ForEach-Object { Require-EnvironmentVariable -Name $_ | Out-Null }
    $env:J6_RUNTIME_BUNDLE = Assert-AllowedMount `
        -Name "J6_RUNTIME_BUNDLE" -Path $env:J6_RUNTIME_BUNDLE
} else {
    @(
        "J6_MODEL_ARTIFACTS",
        "PC_ONNX_MODEL_FILENAME",
        "PC_ONNX_MODEL_ID",
        "PC_ONNX_MODEL_SHA256"
    ) | ForEach-Object { Require-EnvironmentVariable -Name $_ | Out-Null }
    if (-not $env:PC_ONNX_REQUIRED_MODEL_ID) {
        $env:PC_ONNX_REQUIRED_MODEL_ID = "d1_littercam_yolov9c"
    }
    if (-not $env:HIL_APPLY_NETWORK_FAULTS) {
        $env:HIL_APPLY_NETWORK_FAULTS = "true"
    }
}
$env:J6_MODEL_ARTIFACTS = Assert-AllowedMount `
    -Name "J6_MODEL_ARTIFACTS" -Path $env:J6_MODEL_ARTIFACTS
if ($RuntimeBackend -eq "PC_ONNX") {
    $modelPath = Join-Path $env:J6_MODEL_ARTIFACTS $env:PC_ONNX_MODEL_FILENAME
    if (-not (Test-Path -LiteralPath $modelPath -PathType Leaf)) {
        throw "PC_ONNX model is missing: $modelPath"
    }
    $actualSha = (Get-FileHash -LiteralPath $modelPath -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($actualSha -ne $env:PC_ONNX_MODEL_SHA256.ToLowerInvariant()) {
        throw "PC_ONNX model SHA-256 mismatch."
    }
}

if ([string]::IsNullOrWhiteSpace($EvidenceDirectory)) {
    $timestamp = [DateTime]::UtcNow.ToString("yyyyMMddTHHmmssZ")
    $EvidenceDirectory = Join-Path $repoRoot "artifacts\j6_loopback_hil_$timestamp"
}
[System.IO.Directory]::CreateDirectory($EvidenceDirectory) | Out-Null
$env:HIL_EVIDENCE_DIR = (Resolve-Path -LiteralPath $EvidenceDirectory).Path
$env:HIL_RUNTIME_BACKEND = $RuntimeBackend
$env:HIL_RUN_ID = [Guid]::NewGuid().ToString("D").ToLowerInvariant()
$env:HIL_NOT_JOURNEY6_RUNTIME = if ($RuntimeBackend -eq "PC_ONNX") { "true" } else { "false" }
$env:HIL_DURATION_S = "$DurationSeconds"
$env:HIL_SENSOR_SOURCE = $SensorSource

& docker compose -f $composeFile -p $projectName build discovery pc-gateway
if ($LASTEXITCODE -ne 0) { throw "PC HIL gateway build failed." }
if ($RuntimeBackend -eq "JOURNEY6_OE") {
    & docker compose -f $composeFile -p $projectName --profile build-only `
        build j6-oe-wrapper
    if ($LASTEXITCODE -ne 0) { throw "Journey 6 OE wrapper build failed." }
    & docker compose -f $composeFile -p $projectName --profile journey6 `
        build j6-algorithm
    if ($LASTEXITCODE -ne 0) { throw "Journey 6 algorithm host build failed." }
    & docker compose -f $composeFile -p $projectName --profile journey6 up -d `
        discovery pc-gateway j6-algorithm
    if ($LASTEXITCODE -ne 0) { throw "Journey 6 loopback startup failed." }
    $algorithmService = "j6-algorithm"
} else {
    & docker compose -f $composeFile -p $projectName --profile pc-onnx `
        build pc-onnx-algorithm pc-harness
    if ($LASTEXITCODE -ne 0) { throw "PC_ONNX algorithm/harness build failed." }
    & docker compose -f $composeFile -p $projectName --profile pc-onnx up -d `
        discovery pc-gateway pc-onnx-algorithm pc-harness
    if ($LASTEXITCODE -ne 0) { throw "PC_ONNX split-loopback startup failed." }
    $algorithmService = "pc-onnx-algorithm"
}
Start-Sleep -Seconds 2
& docker compose -f $composeFile -p $projectName exec -T $algorithmService `
    /bin/bash -lc "ps -ef > /evidence/HIL_J6_PROCESS_LIST.txt"
if ($LASTEXITCODE -ne 0) { throw "Journey 6 process evidence capture failed." }
$qosEvidenceCommand = @'
source "${TZCUP_ROS_SETUP}"
source "${TZCUP_WS_SETUP}"
sleep 5
{
  echo ROS_SUPER_CLIENT=TRUE
  ros2 topic list --no-daemon -t
  for topic in /hil/camera/color /hil/camera/depth /hil/camera/camera_info /hil/tf /hil/tf_static /hil/vehicle/ackermann_command /hil/vehicle/validated_ackermann_command /hil/health; do
    echo "TOPIC=$topic"
    ros2 topic info -v "$topic"
  done
} > /evidence/HIL_ROS_QOS_INFO.txt
'@
& docker compose -f $composeFile -p $projectName exec -T `
    -e ROS_SUPER_CLIENT=TRUE pc-gateway /bin/bash -lc $qosEvidenceCommand
if ($LASTEXITCODE -ne 0) { throw "ROS topic/QoS evidence capture failed." }
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
Write-Host "runtime_backend=$RuntimeBackend"
Write-Host "run_id=$env:HIL_RUN_ID"
if ($RuntimeBackend -eq "PC_ONNX") {
    Write-Host "not_journey6_runtime=true"
    & docker compose -f $composeFile -p $projectName --profile pc-onnx wait pc-harness
    if ($LASTEXITCODE -ne 0) { throw "PC_ONNX loopback harness failed." }
    $report = Join-Path $env:HIL_EVIDENCE_DIR "J6_LOOPBACK_HIL_EMULATION_REPORT.json"
    if (-not (Test-Path -LiteralPath $report -PathType Leaf)) {
        throw "PC_ONNX harness exited without its report: $report"
    }
    Write-Host "PC_ONNX harness report: $report"
}
Write-Host "Evidence: $env:HIL_EVIDENCE_DIR"
Write-Host "Source HIL_PC_DDS_ENV.sh before the PC sensor/plant-only graph."
