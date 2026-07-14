[CmdletBinding()]
param(
    [string]$Image = "tzcup/sanitation-jazzy:stage0"
)

$ErrorActionPreference = "Stop"
$packRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot "..")).Path
$stamp = Get-Date -Format "yyyyMMdd_HHmmss"
$outputDirectory = Join-Path $packRoot "artifacts\stage0_$stamp"
[void](New-Item -ItemType Directory -Path $outputDirectory -Force)

$preflightLog = & powershell.exe -NoProfile -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot "run_docker_preflight.ps1") -Image $Image -SkipBuild 2>&1
$preflightExitCode = $LASTEXITCODE
[IO.File]::WriteAllLines(
    (Join-Path $outputDirectory "preflight_run.log"),
    [string[]]$preflightLog,
    [Text.UTF8Encoding]::new($false)
)
if ($preflightExitCode -ne 0) {
    throw "Stage 0 preflight failed with exit code $preflightExitCode"
}

$hostInventory = [ordered]@{
    generated_at = (Get-Date).ToUniversalTime().ToString("o")
    operating_system = Get-CimInstance Win32_OperatingSystem |
        Select-Object Caption, Version, BuildNumber, OSArchitecture
    computer = Get-CimInstance Win32_ComputerSystem |
        Select-Object Manufacturer, Model, TotalPhysicalMemory
    gpu = Get-CimInstance Win32_VideoController |
        Select-Object Name, DriverVersion, AdapterRAM, CurrentHorizontalResolution, CurrentVerticalResolution
    nvidia_smi = (& nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader 2>&1) -join "`n"
    workspace_drive = Get-PSDrive -Name ([IO.Path]::GetPathRoot($packRoot).Substring(0, 1)) |
        Select-Object Name, Used, Free
    docker = (& docker info --format '{{json .}}' | ConvertFrom-Json) |
        Select-Object ServerVersion, OperatingSystem, OSType, Architecture, NCPU, MemTotal
    image = (& docker image inspect $Image | ConvertFrom-Json) |
        Select-Object Id, RepoTags, Created, Architecture, Os, Size
}

[IO.File]::WriteAllText(
    (Join-Path $outputDirectory "host_inventory.json"),
    (($hostInventory | ConvertTo-Json -Depth 8) + "`n"),
    [Text.UTF8Encoding]::new($false)
)

Copy-Item -LiteralPath (Join-Path $packRoot "artifacts\preflight.json") -Destination (Join-Path $outputDirectory "preflight.json")

Write-Output $outputDirectory
