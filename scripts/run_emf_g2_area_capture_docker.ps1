param(
    [string]$DataRoot = "F:\Project\TZcup-emfj6v3-artifacts\captures\emf-g2-area",
    [string]$RuntimeRoot = "F:\Project\TZcup-emfj6v3-artifacts\runtime\emf-g2-area",
    [Parameter(Mandatory = $true)]
    [string]$UpstreamLinorobot2Root,
    [ValidateNotNullOrEmpty()]
    [string]$Image = "tzcup/sanitation-jazzy@sha256:418550f48916d794bc0aff144c60a3b1353d0bb0bb1dcf086cda0ec8e2a5aadc"
)

$ErrorActionPreference = "Stop"
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path

function Assert-NonsealedValue {
    param([Parameter(Mandatory = $true)][string]$Value, [string]$Field)

    $normalized = ($Value.ToUpperInvariant() -replace "[^A-Z0-9]+", "_").Trim("_")
    $padded = "_{0}_" -f $normalized
    foreach ($marker in @("G5", "G5_V2", "G5V2", "VAL_NEW", "DEV_VAL", "SEALED")) {
        if ($padded.Contains("_${marker}_")) {
            throw "Forbidden dataset marker in ${Field}: ${marker}"
        }
    }
}

function Assert-OutsideRepository {
    param([Parameter(Mandatory = $true)][string]$PathValue, [string]$Field)

    $absolute = [System.IO.Path]::GetFullPath($PathValue)
    $repoPrefix = $repo.TrimEnd("\") + "\"
    if ($absolute.Equals($repo, [System.StringComparison]::OrdinalIgnoreCase) -or
        $absolute.StartsWith($repoPrefix, [System.StringComparison]::OrdinalIgnoreCase)) {
        throw "${Field} must be outside the read-only repository: ${absolute}"
    }
    return $absolute
}

Assert-NonsealedValue -Value $DataRoot -Field "DataRoot"
Assert-NonsealedValue -Value $RuntimeRoot -Field "RuntimeRoot"
Assert-NonsealedValue -Value $UpstreamLinorobot2Root -Field "UpstreamLinorobot2Root"
Assert-NonsealedValue -Value $Image -Field "Image"

$data = Assert-OutsideRepository -PathValue $DataRoot -Field "DataRoot"
$runtime = Assert-OutsideRepository -PathValue $RuntimeRoot -Field "RuntimeRoot"
$upstream = Assert-OutsideRepository -PathValue $UpstreamLinorobot2Root -Field "UpstreamLinorobot2Root"
if ($data.Equals($runtime, [System.StringComparison]::OrdinalIgnoreCase)) {
    throw "DataRoot and RuntimeRoot must be distinct external directories"
}
if (-not (Test-Path -LiteralPath (Join-Path $upstream "linorobot2_description\package.xml") -PathType Leaf)) {
    throw "UpstreamLinorobot2Root must contain linorobot2_description/package.xml"
}

New-Item -ItemType Directory -Force -Path $data | Out-Null
New-Item -ItemType Directory -Force -Path $runtime | Out-Null
Get-Command docker -ErrorAction Stop | Out-Null

$dockerArguments = @(
    "run", "--rm", "--gpus", "all", "--shm-size", "2g",
    "--env", "NVIDIA_DRIVER_CAPABILITIES=all",
    "--env", "ROS_DOMAIN_ID=121",
    "--env", "GZ_PARTITION=emf_g2_area_capture",
    "--env", "IGN_PARTITION=emf_g2_area_capture",
    "--volume", "${repo}:/repo:ro",
    "--volume", "${data}:/data:rw",
    "--volume", "${runtime}:/runtime:rw",
    "--volume", "${upstream}:/upstream/linorobot2:ro",
    $Image,
    "bash", "/repo/scripts/emf_g2_area_capture.sh"
)

& docker @dockerArguments
if ($LASTEXITCODE -ne 0) {
    throw "EMF G2 Area capture failed with exit code $LASTEXITCODE"
}

Write-Output "EMF G2 Area capture completed under $data"
