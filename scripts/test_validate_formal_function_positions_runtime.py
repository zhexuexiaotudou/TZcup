import ast
import hashlib
import json
import time
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
VALIDATOR = ROOT / "scripts" / "validate_formal_function_positions_runtime.py"


def _binding_helpers() -> tuple[object, object]:
    """Load pure binding helpers without importing ROS on Windows."""

    tree = ast.parse(VALIDATOR.read_text(encoding="utf-8"), filename=str(VALIDATOR))
    helpers = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name in {"snapshot_binding", "bound_runtime_evidence"}
    ]
    namespace: dict[str, object] = {
        "Path": Path,
        "hashlib": hashlib,
        "json": json,
        "time": time,
    }
    exec(
        compile(ast.Module(body=helpers, type_ignores=[]), str(VALIDATOR), "exec"),
        namespace,
    )
    return namespace["snapshot_binding"], namespace["bound_runtime_evidence"]


def _bound_files(tmp_path: Path) -> tuple[Path, Path, Path, dict[str, object]]:
    snapshot_binding, _ = _binding_helpers()
    snapshot = tmp_path / "snapshot.json"
    session = tmp_path / "session.json"
    sidecar = tmp_path / "runtime_binding.json"
    snapshot.write_text(
        json.dumps(
            {
                "source_inventory_sha256": "a" * 64,
                "outputs": {
                    "reports/engineering/formal_competition_vehicle.urdf": {
                        "sha256": "b" * 64
                    }
                },
            }
        ),
        encoding="utf-8",
    )
    started_epoch_ns = time.time_ns() - 1_000_000_000
    session.write_text(
        json.dumps(
            {
                "status": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
                "started_epoch_ns": started_epoch_ns,
            }
        ),
        encoding="utf-8",
    )
    sidecar.write_text("{}", encoding="utf-8")
    source = snapshot_binding(snapshot)  # type: ignore[operator]
    binding: dict[str, object] = {
        "verified_epoch_ns": time.time_ns(),
        "acceptance_session_binding": {
            "snapshot": source,
            "session_manifest_sha256": hashlib.sha256(session.read_bytes()).hexdigest(),
            "session_started_epoch_ns": started_epoch_ns,
            "session_status_at_gate": "FORMAL_FINAL_ACCEPTANCE_SESSION_RUNNING",
            "snapshot_current_source_verified": True,
        },
        "runtime_closure_binding": {
            "status": "FORMAL_FINAL_RUNTIME_CLOSURE_VERIFIED",
            "runtime_install_root": "/frozen/runtime/install",
            "manifest_sha256": "d" * 64,
            "closure_sha256": "e" * 64,
            "symbolic_link_count": 0,
        },
    }
    return snapshot, session, sidecar, binding


def test_validator_requires_active_session_and_verified_runtime_closure() -> None:
    source = VALIDATOR.read_text(encoding="utf-8")
    assert "from formal_runtime_gate_binding import load_binding" in source
    assert "from formal_preembedded_sensor_world_binding import" in source
    assert 'parser.add_argument("--session", type=Path, required=True)' in source
    assert 'parser.add_argument("--runtime-binding", type=Path, required=True)' in source
    assert 'parser.add_argument("--preembedded-report", type=Path)' in source
    assert 'parser.add_argument("--preembedded-world", type=Path)' in source
    assert '"--diagnostic-skip-preembedded-binding"' in source
    assert '"formal acceptance requires --preembedded-report' in source
    assert '"formal_acceptance_eligible": False' in source
    assert '"acceptance_session_binding": acceptance_session_binding' in source
    assert '"runtime_gate_binding": runtime_gate_binding' in source
    assert '"runtime_identity": runtime_gate_binding["runtime_closure_binding"]' in source
    assert '"preembedded_world_binding": preembedded_world_binding' in source


def test_binding_helper_accepts_exact_identity_and_rejects_drift(tmp_path: Path) -> None:
    _, bound_runtime_evidence = _binding_helpers()
    snapshot, session, sidecar, binding = _bound_files(tmp_path)
    namespace = bound_runtime_evidence.__globals__  # type: ignore[attr-defined]
    namespace["load_binding"] = lambda _: binding

    source, session_binding, observed = bound_runtime_evidence(  # type: ignore[operator]
        snapshot, session, sidecar
    )
    assert source["source_inventory_sha256"] == "a" * 64
    assert session_binding["snapshot"] == source
    assert observed is binding

    drifted = json.loads(json.dumps(binding))
    drifted["acceptance_session_binding"]["snapshot"]["expanded_urdf_sha256"] = (
        "c" * 64
    )
    namespace["load_binding"] = lambda _: drifted
    with pytest.raises(ValueError, match="snapshot differs"):
        bound_runtime_evidence(snapshot, session, sidecar)  # type: ignore[operator]

    invalid_closure = json.loads(json.dumps(binding))
    invalid_closure["runtime_closure_binding"]["status"] = "BLOCKED"
    namespace["load_binding"] = lambda _: invalid_closure
    with pytest.raises(ValueError, match="closure is not VERIFIED"):
        bound_runtime_evidence(snapshot, session, sidecar)  # type: ignore[operator]
