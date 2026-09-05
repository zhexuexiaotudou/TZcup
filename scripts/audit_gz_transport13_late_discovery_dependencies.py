#!/usr/bin/env python3
"""Fail-closed CMake-config closure audit for the transport-only smoke.

The smoke deliberately avoids sourcing the full ROS environment.  This helper
reads the actual frozen ``gz-transport13`` config and its declared GZ package
dependencies, proving that every one resolves through the small, explicit
Jazzy-vendor allowlist.  ``gz_transport_vendor`` is intentionally absent.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path


REPORT_ID = "tzcup_gz_transport13_late_discovery_dependency_closure_v1"
ROOT_PACKAGE = "gz-transport13"
REQUIRED_TRANSITIVE = frozenset(
    {"gz-cmake3", "gz-utils2", "gz-msgs10", "gz-math7"}
)
ALLOWED_VENDOR_BY_PACKAGE = {
    "gz-cmake3": "gz_cmake_vendor",
    "gz-utils2": "gz_utils_vendor",
    "gz-msgs10": "gz_msgs_vendor",
    "gz-math7": "gz_math_vendor",
}
DEPENDENCY_CALL_PATTERN = re.compile(
    r"\b(?:find_dependency|find_package|gz_find_package)\s*\(\s*([^\s)]+)",
    flags=re.IGNORECASE,
)
COMPONENT_SECTION_MARKER = "# Find each of the components requested by find_package"


class AuditError(RuntimeError):
    pass


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def regular_directory(path: Path, label: str) -> Path:
    if path.is_symlink() or not path.is_dir():
        raise AuditError(f"{label} must be a real directory: {path}")
    return path.resolve()


def config_path(prefix: Path, package: str) -> Path:
    expected = (
        prefix / "lib" / "cmake" / package / f"{package}-config.cmake",
        prefix / "lib" / "cmake" / package / f"{package}Config.cmake",
        # gz-cmake3 is installed by the Jazzy vendor in this legacy layout.
        prefix / "share" / "cmake" / package / f"{package}-config.cmake",
        prefix / "share" / "cmake" / package / f"{package}Config.cmake",
    )
    matches = [path for path in expected if path.is_file() and not path.is_symlink()]
    if len(matches) != 1:
        raise AuditError(
            f"expected one regular {package} config under {prefix}, found {matches}"
        )
    return matches[0].resolve()


def dependencies(config: Path, package: str) -> tuple[list[str], list[str]]:
    try:
        source = config.read_text(encoding="utf-8", errors="strict")
    except OSError as error:
        raise AuditError(f"cannot read config {config}: {error}") from error
    # The generated configs describe their normal usage in leading ``#``
    # comments (including ``# find_package(gz-transport13)``).  Ignore only
    # whole-line comments, preserving literal ``#`` characters in commands or
    # quoted strings instead of attempting unsafe CMake parsing.
    unconditional_source, marker, component_source = source.partition(
        COMPONENT_SECTION_MARKER
    )
    executable_source = "\n".join(
        line
        for line in unconditional_source.splitlines()
        if not line.lstrip().startswith("#")
    )
    direct: list[str] = []
    for match in DEPENDENCY_CALL_PATTERN.finditer(executable_source):
        token = match.group(1)
        if not token.startswith("gz-"):
            continue
        if "${" in token:
            raise AuditError(
                f"variable GZ dependency in unconditional config section: {package}: {token}"
            )
        direct.append(token)

    conditional: list[str] = []
    if package == ROOT_PACKAGE and marker:
        for match in DEPENDENCY_CALL_PATTERN.finditer(component_source):
            token = match.group(1)
            if not token.startswith("gz-") or "${" not in token:
                continue
            expected = f"{ROOT_PACKAGE}-${{component}}"
            if token != expected:
                raise AuditError(
                    f"unexpected variable GZ component template in frozen root: {token}"
                )
            conditional.append(token)
    return list(dict.fromkeys(direct)), list(dict.fromkeys(conditional))


def audit(runtime_prefix: Path, vendor_prefixes: list[Path]) -> dict[str, object]:
    runtime_prefix = regular_directory(runtime_prefix, "frozen runtime prefix")
    vendor_by_name: dict[str, Path] = {}
    for raw in vendor_prefixes:
        prefix = regular_directory(raw, "Jazzy vendor prefix")
        if "gz_transport_vendor" in prefix.as_posix() or prefix.name == "gz_transport_vendor":
            raise AuditError("gz_transport_vendor is forbidden in the dependency allowlist")
        if prefix.name in vendor_by_name:
            raise AuditError(f"duplicate vendor prefix name: {prefix.name}")
        vendor_by_name[prefix.name] = prefix
    expected_vendor_names = set(ALLOWED_VENDOR_BY_PACKAGE.values())
    if set(vendor_by_name) != expected_vendor_names:
        raise AuditError(
            "vendor allowlist must be exactly "
            + ", ".join(sorted(expected_vendor_names))
        )

    root_config = config_path(runtime_prefix, ROOT_PACKAGE)
    stack: list[tuple[str, Path, str, tuple[str, ...]]] = [
        (ROOT_PACKAGE, root_config, "frozen_runtime", (ROOT_PACKAGE,))
    ]
    seen: set[str] = set()
    rows: list[dict[str, object]] = []
    discovered: set[str] = set()
    cycle_edges: list[dict[str, str]] = []
    while stack:
        package, config, origin, ancestry = stack.pop()
        if package in seen:
            continue
        seen.add(package)
        direct, conditional = dependencies(config, package)
        rows.append(
            {
                "package": package,
                "config": str(config),
                "config_sha256": sha256(config),
                "origin": origin,
                "declared_gz_dependencies": direct,
                "conditional_template_dependencies": conditional,
            }
        )
        for dependency in direct:
            if dependency == ROOT_PACKAGE:
                if package == ROOT_PACKAGE:
                    cycle_edges.append(
                        {"from": package, "to": dependency, "kind": "root_self_edge"}
                    )
                    continue
                raise AuditError(
                    f"unexpected dependency cycle reaches frozen transport: {package} -> {dependency}"
                )
            if dependency in ancestry:
                raise AuditError(
                    "non-self GZ dependency cycle: "
                    + " -> ".join((*ancestry, dependency))
                )
            discovered.add(dependency)
            vendor_name = ALLOWED_VENDOR_BY_PACKAGE.get(dependency)
            if vendor_name is None:
                raise AuditError(
                    f"undeclared GZ dependency outside minimal allowlist: {dependency}"
                )
            prefix = vendor_by_name[vendor_name]
            stack.append(
                (dependency, config_path(prefix, dependency), vendor_name, (*ancestry, dependency))
            )

    missing = sorted(REQUIRED_TRANSITIVE - discovered)
    if missing:
        raise AuditError(
            "frozen transport config closure did not expose required dependencies: "
            + ", ".join(missing)
        )
    return {
        "schema_version": 1,
        "report_id": REPORT_ID,
        "status": "GZ_TRANSPORT13_LATE_DISCOVERY_DEPENDENCY_CLOSURE_PASSED",
        "passed": True,
        "frozen_runtime_prefix": str(runtime_prefix),
        "root_package": ROOT_PACKAGE,
        "allowed_vendor_prefixes": {
            package: str(vendor_by_name[vendor])
            for package, vendor in sorted(ALLOWED_VENDOR_BY_PACKAGE.items())
        },
        "discovered_gz_dependencies": sorted(discovered),
        "cycle_edges": cycle_edges,
        "configs": rows,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-prefix", type=Path, required=True)
    parser.add_argument("--vendor-prefix", type=Path, action="append", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists() or args.output.is_symlink():
        print(f"refusing stale output: {args.output}", file=sys.stderr)
        return 2
    try:
        report = audit(args.runtime_prefix, args.vendor_prefix)
    except (AuditError, OSError, ValueError) as error:
        print(f"dependency closure rejected: {error}", file=sys.stderr)
        return 1
    args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(report["status"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
