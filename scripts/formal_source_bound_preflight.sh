#!/usr/bin/env bash
# Shared fail-closed source/runtime admission for formal mapping and perception.
# Source this helper after run_formal_runtime_isolation.sh and before ROS setup.

formal_source_bound_preflight() {
  local repo_root="$1"
  local runtime_ws="$2"
  local closure_manifest="$3"
  local session="$4"
  local snapshot="$5"
  local binding="$6"
  local runtime_install="${runtime_ws}/install"

  for required in \
    "${runtime_install}/setup.bash" \
    "${closure_manifest}" \
    "${session}" \
    "${snapshot}"; do
    [[ -f "${required}" ]] || {
      echo "formal source-bound preflight input is missing: ${required}" >&2
      return 2
    }
  done
  [[ ! -e "${binding}" ]] || {
    echo "refusing to overwrite retained formal runtime binding: ${binding}" >&2
    return 2
  }

  # formal_runtime_gate_binding verifies one non-symlink frozen install, the
  # complete recorded closure, a RUNNING session and the current canonical
  # source snapshot before any runner can source an overlay.
  /usr/bin/python3 "${repo_root}/scripts/formal_runtime_gate_binding.py" \
    --repository-root "${repo_root}" \
    --install-root "${runtime_install}" \
    --closure-manifest "${closure_manifest}" \
    --session "${session}" \
    --snapshot "${snapshot}" \
    --output "${binding}"
}

formal_source_bound_verify_overlay() {
  local runtime_install="$1"
  /usr/bin/python3 - "${runtime_install}" <<'PY'
import os
import sys
from pathlib import Path

expected = Path(sys.argv[1]).resolve(strict=True)
if expected.is_symlink():
    raise SystemExit("frozen runtime install must not be a symbolic link")
prefixes = [
    Path(value).resolve()
    for value in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
    if value
]
packages = (
    "sanitation_active_cleaning", "sanitation_campus_scenario",
    "sanitation_coverage", "sanitation_formal_campus_integration",
    "sanitation_gazebo_auxiliary", "sanitation_gazebo_control",
    "sanitation_localization", "sanitation_manipulation",
    "sanitation_navigation", "sanitation_perception",
    "sanitation_perception_interfaces", "sanitation_power_system",
    "sanitation_product_demo_integration", "sanitation_safety",
    "sanitation_service_acceptance", "sanitation_vehicle_description",
)
for package in packages:
    marker = Path("share/ament_index/resource_index/packages") / package
    resolved = next((prefix for prefix in prefixes if (prefix / marker).is_file()), None)
    if resolved != expected:
        raise SystemExit(
            f"project package resolved outside the one frozen overlay: {package} -> {resolved}"
        )
print("FORMAL_SOURCE_BOUND_OVERLAY_OK")
PY
}

formal_source_bound_perception_roots() {
  local closure_manifest="$1"
  /usr/bin/python3 - "${closure_manifest}" <<'PY'
import json
import sys
from pathlib import Path

manifest = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
closure = manifest.get("closure")
if not isinstance(closure, dict):
    raise SystemExit("frozen runtime closure has no closure object")
for key in ("perception_artifact_root", "onnx_pythonpath"):
    value = closure.get(key)
    if not isinstance(value, str) or not value:
        raise SystemExit(f"frozen runtime closure has no {key}")
    root = Path(value).resolve(strict=True)
    if not root.is_dir():
        raise SystemExit(f"frozen runtime closure {key} is not a directory")
    print(root)
PY
}
