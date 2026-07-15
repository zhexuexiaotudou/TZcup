$ErrorActionPreference = "Stop"
$packRoot = Split-Path -Parent $PSScriptRoot; $mainCheckout = "F:\Project\TZcup"
$argsList = @(
  "run", "--rm", "--gpus", "all", "--env", "NVIDIA_DRIVER_CAPABILITIES=all",
  "--env", "SANITATION_WS=/work/.work/stage1_20260714_154523",
  "--volume", "${mainCheckout}:/work", "--volume", "${packRoot}:/stage4t",
  "--workdir", "/stage4t", "tzcup/sanitation-jazzy:stage0", "bash", "scripts/stage4t_mapping_ci.sh"
)
if ($env:TZCUP_STAGE4T_MAPPING_DIR) {
  $insertAt = $argsList.IndexOf("--volume")
  $argsList = $argsList[0..($insertAt - 1)] + @("--env", "STAGE4T_OUT=/stage4t/artifacts/$($env:TZCUP_STAGE4T_MAPPING_DIR)") + $argsList[$insertAt..($argsList.Count - 1)]
}
docker @argsList
if ($LASTEXITCODE -ne 0) { throw "Stage4T mapping failed: $LASTEXITCODE" }
