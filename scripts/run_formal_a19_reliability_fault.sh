#!/usr/bin/env bash
# Canonical A19 two-hour product-soak/fault-injection entry point.
set -euo pipefail

repo_root="${TZCUP_REPOSITORY_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
contract="${FORMAL_A19_CONTRACT:-${repo_root}/config/high_fidelity_vehicle/formal_a19_reliability_fault_contract.json}"
runtime_ws="${FORMAL_VEHICLE_RUNTIME_WS:?set FORMAL_VEHICLE_RUNTIME_WS to the fresh final frozen workspace}"
runtime_install="${runtime_ws}/install"
runtime_closure="${FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST:-${runtime_ws}/final_runtime_closure_manifest.json}"
session="${FORMAL_ACCEPTANCE_SESSION_STATUS:-${repo_root}/artifacts/formal_final_acceptance_session.json}"
snapshot="${FORMAL_VEHICLE_SNAPSHOT_MANIFEST:-${repo_root}/reports/engineering/formal_vehicle_snapshot_manifest.json}"
evidence_root="${FORMAL_A19_EVIDENCE_ROOT:-${repo_root}/artifacts/formal_a19_reliability_fault_raw}"
output="${FORMAL_A19_OUTPUT:-${repo_root}/artifacts/formal_a19_reliability_fault_acceptance.json}"
runtime_binding_sidecar="${output}.runtime_binding.json"
adapter_argv_json="${FORMAL_A19_ADAPTER_ARGV_JSON:?set FORMAL_A19_ADAPTER_ARGV_JSON to the frozen product adapter argv JSON array}"
domain="${ROS_DOMAIN_ID:-82}"

common=(
  --repository-root "${repo_root}"
  --contract "${contract}"
  --runtime-install "${runtime_install}"
  --runtime-closure "${runtime_closure}"
  --session "${session}"
  --snapshot "${snapshot}"
  --evidence-root "${evidence_root}"
  --adapter-argv-json "${adapter_argv_json}"
)

if [[ "${1:-}" == "--preflight" ]]; then
  [[ $# -eq 1 ]] || { echo "A19 preflight accepts no additional arguments" >&2; exit 2; }
  exec /usr/bin/python3 "${repo_root}/scripts/produce_formal_a19_reliability_fault.py" --preflight "${common[@]}"
fi
[[ $# -eq 0 ]] || { echo "unknown A19 runner argument: $1" >&2; exit 2; }

for required in "${contract}" "${runtime_install}/setup.bash" "${runtime_closure}" "${session}" "${snapshot}"; do
  [[ -f "${required}" && ! -L "${required}" ]] || { echo "formal A19 prerequisite missing or linked: ${required}" >&2; exit 2; }
done
[[ ! -e "${evidence_root}" && ! -L "${evidence_root}" ]] || { echo "Refusing stale A19 evidence root: ${evidence_root}" >&2; exit 2; }
[[ ! -e "${output}" && ! -L "${output}" && ! -e "${runtime_binding_sidecar}" && ! -L "${runtime_binding_sidecar}" ]] || {
  echo "Refusing stale A19 final receipt or runtime binding: ${output}" >&2; exit 2;
}

source "${repo_root}/scripts/run_formal_runtime_isolation.sh"
formal_runtime_configure "${domain}"
export ROS_DOMAIN_ID="${domain}"
export GZ_PARTITION="${GZ_PARTITION:-tzcup_formal_a19_${domain}_$$}"
formal_runtime_register_evidence_paths "${evidence_root}" "${output}" "${runtime_binding_sidecar}"
cleanup() { return 0; }
formal_runtime_install_traps cleanup

/usr/bin/python3 "${repo_root}/scripts/generate_formal_vehicle_snapshot.py" --check >"${repo_root}/artifacts/formal_a19_snapshot_preflight.json"
set +e
/usr/bin/python3 "${repo_root}/scripts/produce_formal_a19_reliability_fault.py" --execute "${common[@]}" \
  >"${repo_root}/artifacts/formal_a19_producer.log" 2>&1
producer_status=$?
set -e

if [[ ! -f "${evidence_root}/raw_receipt.json" ]]; then
  echo "A19 producer did not retain a raw receipt; see artifacts/formal_a19_producer.log" >&2
  exit "${producer_status}"
fi
binding_pending="${runtime_binding_sidecar}.pending.$$"
cp -- "${evidence_root}/runtime_gate_binding.json" "${binding_pending}"
mv -- "${binding_pending}" "${runtime_binding_sidecar}"
/usr/bin/python3 "${repo_root}/scripts/generate_formal_vehicle_snapshot.py" --check >"${repo_root}/artifacts/formal_a19_snapshot_postflight.json"
set +e
/usr/bin/python3 "${repo_root}/scripts/validate_formal_a19_reliability_fault.py" \
  --report "${evidence_root}/raw_receipt.json" \
  --snapshot "${snapshot}" \
  --acceptance-session "${session}" \
  --runtime-closure "${runtime_closure}" \
  --evidence-root "${evidence_root}" \
  --contract "${contract}" \
  --repository-root "${repo_root}" \
  --output "${output}"
validator_status=$?
set -e
if (( producer_status != 0 || validator_status != 0 )); then
  echo "formal A19 failed closed: producer=${producer_status} validator=${validator_status} receipt=${output}" >&2
  exit 4
fi
echo "Published formal A19 acceptance: ${output}"
