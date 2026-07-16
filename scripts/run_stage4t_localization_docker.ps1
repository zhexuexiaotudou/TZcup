$ErrorActionPreference = "Stop"
$packRoot = Split-Path -Parent $PSScriptRoot; $mainCheckout = "F:\Project\TZcup"
if (-not $env:TZCUP_MAP_DIR) { throw "Set TZCUP_MAP_DIR to the artifact directory name containing selected_map.yaml" }
$seeds = if ($env:TZCUP_LOCALIZATION_SEEDS) { $env:TZCUP_LOCALIZATION_SEEDS } else { "1" }
$lane = if ($env:TZCUP_LOCALIZATION_LANE) { $env:TZCUP_LOCALIZATION_LANE } else { "realistic" }
$argsList = @(
  "run", "--rm", "--gpus", "all", "--env", "NVIDIA_DRIVER_CAPABILITIES=all",
  "--env", "SANITATION_WS=/work/.work/stage1_20260714_154523",
  "--env", "TZCUP_MAP_ROOT=/stage4t/artifacts/$($env:TZCUP_MAP_DIR)",
  "--env", "TZCUP_LOCALIZATION_SEEDS=$seeds", "--env", "TZCUP_LOCALIZATION_LANE=$lane",
  "--volume", "${mainCheckout}:/work", "--volume", "${packRoot}:/stage4t",
  "--workdir", "/stage4t", "tzcup/sanitation-jazzy:stage0", "bash", "scripts/stage4t_localization_ci.sh"
)
foreach ($name in @("TZCUP_MAP_YAML", "TZCUP_MAP_CALIBRATION", "TZCUP_FILTER_ROOT", "TZCUP_LOCALIZATION_BACKEND", "TZCUP_SLAM_PARAMS", "TZCUP_POSEGRAPH", "TZCUP_LIDAR_SAMPLES", "TZCUP_LIDAR_UPDATE_RATE", "TZCUP_NAV2_PARAMS", "TZCUP_WORLD_FILE")) {
  $value = [Environment]::GetEnvironmentVariable($name)
  if ($value) {
    $insertAt = $argsList.IndexOf("--volume")
    $argsList = $argsList[0..($insertAt - 1)] + @("--env", "$name=$value") + $argsList[$insertAt..($argsList.Count - 1)]
  }
}
docker @argsList
if ($LASTEXITCODE -ne 0) { throw "Stage4T localization failed: $LASTEXITCODE" }
