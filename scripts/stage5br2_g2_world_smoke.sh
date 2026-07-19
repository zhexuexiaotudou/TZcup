#!/usr/bin/env bash
set -euo pipefail

PACK_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD_DIR="${STAGE5BR2_WORLD_DIR:-${PACK_ROOT}/artifacts/stage5br2_20260720_review/g2_worlds}"
OUT="${STAGE5BR2_SMOKE_OUT:-${PACK_ROOT}/artifacts/stage5br2_20260720_review/g2_world_smoke}"
mkdir -p "${OUT}"

worlds=(world_a_asphalt_campus world_b_concrete_sidewalk world_c_wet_dark_ground world_d_mixed_curb_vegetation)
for world in "${worlds[@]}"; do
  log="${OUT}/${world}.gz.log"
  topics="${OUT}/${world}.topics.txt"
  gz sim -r -s "${WORLD_DIR}/${world}.sdf" >"${log}" 2>&1 &
  pid=$!
  ready=0
  for _ in $(seq 1 40); do
    if gz topic -l >"${topics}" 2>/dev/null && \
       grep -q "/g2/${world}/rgbd/image" "${topics}" && \
       grep -q "/g2/${world}/rgbd/depth_image" "${topics}" && \
       grep -q "/g2/${world}/semantic_gt/labels_map" "${topics}" && \
       grep -q "/g2/${world}/instance_gt/labels_map" "${topics}"; then
      ready=1
      break
    fi
    sleep 0.5
  done
  kill -INT "${pid}" 2>/dev/null || true
  wait "${pid}" 2>/dev/null || true
  if [[ "${ready}" -ne 1 ]]; then
    echo "${world}: topic readiness failed" >&2
    exit 2
  fi
done

python3 - "${OUT}" <<'PY'
import json
from pathlib import Path
import sys

out = Path(sys.argv[1])
worlds = ["world_a_asphalt_campus", "world_b_concrete_sidewalk", "world_c_wet_dark_ground", "world_d_mixed_curb_vegetation"]
report = {
    "schema_version": 1,
    "stage": "Stage5BR2 G2 four-world sensor smoke",
    "worlds": [],
}
for world in worlds:
    topics = (out / f"{world}.topics.txt").read_text(encoding="utf-8").splitlines()
    expected = [
        f"/g2/{world}/rgbd/image", f"/g2/{world}/rgbd/depth_image",
        f"/g2/{world}/semantic_gt/labels_map", f"/g2/{world}/instance_gt/labels_map",
    ]
    report["worlds"].append({"world_id": world, "expected_topics": expected, "all_topics_present": all(item in topics for item in expected)})
report["four_world_sensor_smoke_pass"] = all(item["all_topics_present"] for item in report["worlds"])
(out / "g2_world_smoke.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
print(json.dumps(report, indent=2))
PY
