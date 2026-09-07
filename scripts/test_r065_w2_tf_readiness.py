import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]
HELPER = ROOT / "scripts" / "r065_w2_tf_readiness.py"


def _module():
    spec = importlib.util.spec_from_file_location("r065_w2_tf_readiness", HELPER)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _transform(*, parent="map", child="base_footprint", stamp_ns=2_000_000_000):
    return SimpleNamespace(
        header=SimpleNamespace(
            frame_id=parent,
            stamp=SimpleNamespace(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000),
        ),
        child_frame_id=child,
    )


def test_validates_exact_fresh_nonfuture_map_to_base_footprint() -> None:
    helper = _module()
    actual = helper.validate_transform(_transform(), 2_100_000_000, 200_000_000)
    assert actual == {
        "parent_frame": "map", "child_frame": "base_footprint", "stamp_ns": 2_000_000_000,
        "clock_ns": 2_100_000_000, "age_ns": 100_000_000,
    }


@pytest.mark.parametrize(
    ("transform", "now_ns", "reason"),
    [
        (_transform(stamp_ns=0), 2_000_000_000, "zero_stamp"),
        (_transform(stamp_ns=2_000_000_001), 2_000_000_000, "future"),
        (_transform(stamp_ns=1_000_000_000), 2_000_000_000, "stale"),
        (_transform(parent="odom"), 2_000_000_000, "parent_frame_mismatch"),
        (_transform(child="base_link"), 2_000_000_000, "child_frame_mismatch"),
    ],
)
def test_rejects_bad_frame_or_clock_boundary(transform, now_ns, reason) -> None:
    helper = _module()
    with pytest.raises(helper.ReadinessError, match=reason):
        helper.validate_transform(transform, now_ns, 500_000_000)


def test_report_write_is_atomic_and_regular(tmp_path) -> None:
    helper = _module()
    output = tmp_path / "readiness.json"
    helper.atomic_write_json(output, {"passed": False, "status": "BLOCKED"})
    assert output.is_file() and not output.is_symlink()
    assert output.read_text(encoding="utf-8").startswith("{")


def test_report_writer_rejects_leaf_symlink_without_touching_external_target(tmp_path) -> None:
    helper = _module()
    external = tmp_path / "external.json"
    external.write_text("KEEP", encoding="utf-8")
    output = tmp_path / "readiness.json"
    try:
        output.symlink_to(external)
    except OSError as exc:
        pytest.skip(f"host cannot create symlink fixture: {exc}")
    with pytest.raises(helper.ReadinessError, match="symlink"):
        helper.atomic_write_json(output, {"passed": False})
    assert external.read_text(encoding="utf-8") == "KEEP"


def test_report_writer_rejects_ancestor_symlink_without_creating_external_file(tmp_path) -> None:
    helper = _module()
    external = tmp_path / "external"
    external.mkdir()
    linked_parent = tmp_path / "linked"
    try:
        linked_parent.symlink_to(external, target_is_directory=True)
    except OSError as exc:
        pytest.skip(f"host cannot create symlink fixture: {exc}")
    output = linked_parent / "readiness.json"
    with pytest.raises(helper.ReadinessError, match="symlink"):
        helper.atomic_write_json(output, {"passed": False})
    assert not (external / "readiness.json").exists()


def test_helper_ast_has_no_publish_or_control_api() -> None:
    import ast

    tree = ast.parse(HELPER.read_text(encoding="utf-8"))
    call_names = {
        node.func.attr for node in ast.walk(tree)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    assert "create_publisher" not in call_names
    assert "create_client" not in call_names
    assert "create_subscription" not in call_names
