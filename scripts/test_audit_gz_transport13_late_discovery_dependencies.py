import importlib.util
import json
import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "scripts/audit_gz_transport13_late_discovery_dependencies.py"
SPEC = importlib.util.spec_from_file_location("late_discovery_dependencies", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def write_config(prefix: Path, package: str, dependencies: list[str]) -> None:
    path = prefix / "lib/cmake" / package / f"{package}-config.cmake"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "\n".join(f"find_dependency({dependency})" for dependency in dependencies),
        encoding="utf-8",
    )


def make_closure(tmp_path: Path):
    runtime = tmp_path / "runtime"
    write_config(runtime, "gz-transport13", ["gz-cmake3", "gz-utils2", "gz-msgs10"])
    prefixes = {}
    for package, vendor in MODULE.ALLOWED_VENDOR_BY_PACKAGE.items():
        prefix = tmp_path / vendor
        prefixes[vendor] = prefix
        dependencies = {"gz-msgs10": ["gz-math7"], "gz-math7": ["gz-utils2"]}.get(package, [])
        write_config(prefix, package, dependencies)
    return runtime, prefixes


def test_audits_minimal_transitive_allowlist(tmp_path: Path) -> None:
    runtime, prefixes = make_closure(tmp_path)
    report = MODULE.audit(runtime, list(prefixes.values()))
    assert report["passed"] is True
    assert set(report["discovered_gz_dependencies"]) == MODULE.REQUIRED_TRANSITIVE
    assert {row["package"] for row in report["configs"]} == {
        "gz-transport13",
        *MODULE.REQUIRED_TRANSITIVE,
    }


def test_ignores_generated_config_usage_comments_without_mutating_hash_syntax(
    tmp_path: Path,
) -> None:
    runtime, prefixes = make_closure(tmp_path)
    root = runtime / "lib/cmake/gz-transport13/gz-transport13-config.cmake"
    root.write_text(
        "# find_package(gz-transport13) is documentation, not a dependency\n"
        "set(example \"literal # remains present\")\n"
        "find_dependency(gz-cmake3)\n"
        "find_package(gz-utils2)\n"
        "find_package(gz-msgs10)\n",
        encoding="utf-8",
    )
    report = MODULE.audit(runtime, list(prefixes.values()))
    assert MODULE.ROOT_PACKAGE not in report["discovered_gz_dependencies"]
    assert report["cycle_edges"] == []


def test_records_only_root_component_template_and_rejects_variable_unconditional_dependency(
    tmp_path: Path,
) -> None:
    runtime, prefixes = make_closure(tmp_path)
    root = runtime / "lib/cmake/gz-transport13/gz-transport13-config.cmake"
    root.write_text(
        "find_dependency(gz-cmake3)\n"
        "find_package(gz-utils2)\n"
        "find_package(gz-msgs10)\n"
        "# Find each of the components requested by find_package\n"
        "find_dependency(gz-transport13-${component})\n",
        encoding="utf-8",
    )
    report = MODULE.audit(runtime, list(prefixes.values()))
    root_row = next(row for row in report["configs"] if row["package"] == MODULE.ROOT_PACKAGE)
    assert root_row["conditional_template_dependencies"] == [
        "gz-transport13-${component}"
    ]

    root.write_text(
        "find_dependency(gz-cmake3)\n"
        "find_package(gz-utils2-${variable})\n",
        encoding="utf-8",
    )
    with pytest.raises(MODULE.AuditError, match="variable GZ dependency in unconditional"):
        MODULE.audit(runtime, list(prefixes.values()))


def test_accepts_jazzy_gz_cmake_share_layout(tmp_path: Path) -> None:
    runtime, prefixes = make_closure(tmp_path)
    original = prefixes["gz_cmake_vendor"] / "lib/cmake/gz-cmake3/gz-cmake3-config.cmake"
    legacy = prefixes["gz_cmake_vendor"] / "share/cmake/gz-cmake3/gz-cmake3-config.cmake"
    legacy.parent.mkdir(parents=True)
    legacy.write_bytes(original.read_bytes())
    original.unlink()
    report = MODULE.audit(runtime, list(prefixes.values()))
    row = next(row for row in report["configs"] if row["package"] == "gz-cmake3")
    assert Path(row["config"]) == legacy.resolve()


def test_records_only_a_real_root_self_edge_and_rejects_other_cycles(tmp_path: Path) -> None:
    runtime, prefixes = make_closure(tmp_path)
    write_config(
        runtime,
        "gz-transport13",
        ["gz-transport13", "gz-cmake3", "gz-utils2", "gz-msgs10"],
    )
    report = MODULE.audit(runtime, list(prefixes.values()))
    assert report["cycle_edges"] == [
        {"from": "gz-transport13", "to": "gz-transport13", "kind": "root_self_edge"}
    ]

    write_config(prefixes["gz_math_vendor"], "gz-math7", ["gz-msgs10"])
    with pytest.raises(MODULE.AuditError, match="non-self GZ dependency cycle"):
        MODULE.audit(runtime, list(prefixes.values()))


def test_rejects_new_unreviewed_gz_dependency(tmp_path: Path) -> None:
    runtime, prefixes = make_closure(tmp_path)
    write_config(runtime, "gz-transport13", ["gz-cmake3", "gz-widget9"])
    with pytest.raises(MODULE.AuditError, match="outside minimal allowlist"):
        MODULE.audit(runtime, list(prefixes.values()))


def test_rejects_transport_vendor_even_if_other_prefixes_are_complete(tmp_path: Path) -> None:
    runtime, prefixes = make_closure(tmp_path)
    forbidden = tmp_path / "gz_transport_vendor"
    forbidden.mkdir()
    with pytest.raises(MODULE.AuditError, match="forbidden"):
        MODULE.audit(runtime, [*prefixes.values(), forbidden])


def test_refuses_missing_required_transitive_dependency(tmp_path: Path) -> None:
    runtime, prefixes = make_closure(tmp_path)
    write_config(runtime, "gz-transport13", ["gz-cmake3", "gz-utils2"])
    with pytest.raises(MODULE.AuditError, match="did not expose required"):
        MODULE.audit(runtime, list(prefixes.values()))
