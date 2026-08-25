from __future__ import annotations

from pathlib import Path

import pytest

from generate_formal_vehicle_snapshot import (
    CANONICAL_CONTROLLER_URI,
    ENTRYPOINT,
    SnapshotError,
    _canonical_digest,
    _inventory,
    _validate_expanded_urdf_paths,
    authoritative_source_paths,
    generate_snapshot,
    verify_snapshot,
)


ROOT = Path(__file__).resolve().parents[1]


def test_committed_formal_vehicle_snapshot_manifest_is_current() -> None:
    manifest = verify_snapshot(ROOT)
    assert manifest["profile"]["entrypoint"] == ENTRYPOINT.as_posix()
    assert manifest["profile"]["use_sim"] is True
    assert manifest["profile"]["xacro_version"] != "unreported"


def test_authoritative_inventory_covers_all_formal_xacros_and_contracts() -> None:
    sources = set(authoritative_source_paths(ROOT))
    xacro_root = ROOT / ENTRYPOINT.parent / "high_fidelity"
    expected_xacros = {path.relative_to(ROOT) for path in xacro_root.glob("*.xacro")}
    assert expected_xacros
    assert expected_xacros <= sources
    assert ENTRYPOINT in sources
    assert Path("config/high_fidelity_vehicle/formal_vehicle_layout.yaml") in sources
    assert Path("config/high_fidelity_vehicle/formal_vehicle_component_register.yaml") in sources
    assert Path("scripts/generate_formal_vehicle_snapshot.py") in sources


def test_inventory_digest_detects_source_or_output_mutation(tmp_path: Path) -> None:
    relative = Path("source.xacro")
    (tmp_path / relative).write_text("first\n", encoding="utf-8")
    before = _inventory(tmp_path, (relative,))
    (tmp_path / relative).write_text("second\n", encoding="utf-8")
    after = _inventory(tmp_path, (relative,))
    assert before != after
    assert _canonical_digest(before) != _canonical_digest(after)


def test_path_gate_preserves_sim_plugin_and_accepts_canonical_uri() -> None:
    raw = (
        '<robot><gazebo><plugin filename="gz_ros2_control-system">'
        f"<parameters>{CANONICAL_CONTROLLER_URI}</parameters>"
        "</plugin></gazebo></robot>\n"
    )
    validated = _validate_expanded_urdf_paths(raw, ROOT)
    assert "gz_ros2_control-system" in validated
    assert f"<parameters>{CANONICAL_CONTROLLER_URI}</parameters>" in validated


@pytest.mark.parametrize("path", ["/mnt/f/work/formal_vehicle_controllers.yaml", "C:/work/formal_vehicle_controllers.yaml"])
def test_path_gate_rejects_machine_specific_controller_path(path: str) -> None:
    raw = f"<robot><gazebo><plugin><parameters>{path}</parameters></plugin></gazebo></robot>"
    with pytest.raises(SnapshotError, match="canonical controller parameter URI"):
        _validate_expanded_urdf_paths(raw, ROOT)


def test_generation_fails_closed_without_xacro(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr("generate_formal_vehicle_snapshot.shutil.which", lambda _: None)
    with pytest.raises(SnapshotError, match="xacro is unavailable"):
        generate_snapshot(ROOT)
