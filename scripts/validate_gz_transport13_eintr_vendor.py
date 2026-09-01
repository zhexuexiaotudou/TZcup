#!/usr/bin/env python3
"""Validate the pinned gz-transport13 frame-local EINTR vendor build."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import subprocess
import time
from pathlib import Path, PurePosixPath
from typing import Any, Iterable, Mapping, Sequence


ROOT = Path(__file__).resolve().parents[1]
PATCH_ROOT = ROOT / "patches" / "upstream" / "gz_transport13"
MANIFEST_PATH = PATCH_ROOT / "manifest.json"
SYSTEM_PROTOBUF_VERSION = "3.21.12"
SYSTEM_PROTOBUF_HEADER_VERSION = 3021012
SYSTEM_PROTOBUF_INCLUDE = Path("/usr/include")
SYSTEM_PROTOBUF_PROTOC = Path("/usr/bin/protoc")
ORTOOLS_VENDOR_PREFIX = "/opt/ros/jazzy/opt/ortools_vendor"
PROTOBUF_BINDING_KEYS = {
    "schema_version",
    "status",
    "passed",
    "protobuf_version",
    "protobuf_header_version",
    "config_mode_protobuf_disabled",
    "forbidden_prefix",
    "compile_command_count",
    "resolved",
}
PROTOBUF_RESOLVED_KEYS = {
    "Protobuf_INCLUDE_DIR",
    "Protobuf_LIBRARY_RELEASE",
    "Protobuf_LITE_LIBRARY_RELEASE",
    "Protobuf_PROTOC_LIBRARY_RELEASE",
    "Protobuf_PROTOC_EXECUTABLE",
}


class ValidationError(RuntimeError):
    pass


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def run(*args: str, cwd: Path | None = None) -> str:
    result = subprocess.run(
        args,
        cwd=cwd,
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise ValidationError(
            f"command failed ({result.returncode}): {' '.join(args)}: "
            f"{result.stderr.strip()[:400]}"
        )
    return result.stdout.strip()


def load_manifest() -> dict[str, Any]:
    data = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    required = {
        "schema_version",
        "component",
        "upstream_repository",
        "upstream_tag",
        "upstream_commit",
        "upstream_tree",
        "upstream_node_shared_sha256",
        "patched_node_shared_sha256",
        "patch",
        "patch_sha256",
        "eintr_retry_limit",
        "license",
    }
    if set(data) != required:
        raise ValidationError("vendor manifest keys differ from the frozen schema")
    if data["schema_version"] != 1 or data["component"] != "gz-transport13":
        raise ValidationError("unsupported vendor manifest identity")
    if data["upstream_tag"] != "gz-transport13_13.5.0":
        raise ValidationError("the vendor fix must remain pinned to gz-transport13 13.5.0")
    for field in (
        "upstream_commit",
        "upstream_tree",
        "upstream_node_shared_sha256",
        "patched_node_shared_sha256",
        "patch_sha256",
    ):
        expected_length = 40 if field in {"upstream_commit", "upstream_tree"} else 64
        if not re.fullmatch(rf"[0-9a-f]{{{expected_length}}}", str(data[field])):
            raise ValidationError(f"invalid {field}")
    if data["eintr_retry_limit"] != 3:
        raise ValidationError("EINTR retry limit drifted from the reviewed bound")
    patch_path = PATCH_ROOT / data["patch"]
    if not patch_path.is_file() or sha256(patch_path) != data["patch_sha256"]:
        raise ValidationError("vendor patch is missing or has drifted")
    return data


def validate_patch_contract(manifest: dict[str, Any]) -> dict[str, Any]:
    patch_path = PATCH_ROOT / manifest["patch"]
    patch = patch_path.read_text(encoding="utf-8")
    additions = "\n".join(
        line[1:]
        for line in patch.splitlines()
        if line.startswith("+") and not line.startswith("+++")
    )
    deletions = "\n".join(
        line[1:]
        for line in patch.splitlines()
        if line.startswith("-") and not line.startswith("---")
    )
    required_additions = (
        "constexpr unsigned int kPublishEintrRetryLimit = 3u;",
        "static void sendPublishFrameWithEintrRetry",
        "_publisher.send(_frame, _flags);",
        "_error.num() != EINTR || retryCount >= kPublishEintrRetryLimit",
        "sendPublishFrameWithEintrRetry(*this->dataPtr->publisher",
    )
    for text in required_additions:
        if text not in additions:
            raise ValidationError(f"patch lacks reviewed frame-local contract: {text}")
    if additions.count("_publisher.send(_frame, _flags);") != 1:
        raise ValidationError("the retry helper must contain exactly one frame send")
    if additions.count("sendPublishFrameWithEintrRetry(*this->dataPtr->publisher") != 12:
        raise ValidationError("not every conditional Publish frame is routed through the helper")
    if "NodeShared::Publish(" in additions or "NodeShared::Publish(" in deletions:
        raise ValidationError("the patch must not wrap or replay the complete publication")
    if deletions.count("this->dataPtr->publisher->send(") != 12:
        raise ValidationError("the reviewed patch must replace exactly ten conditional sends")
    if "catch(const zmq::error_t& ze)" in deletions:
        raise ValidationError("the existing outer non-EINTR/exhaustion handler must remain")
    return {
        "helper_send_count": additions.count("_publisher.send(_frame, _flags);"),
        "publish_frame_callsite_count": additions.count(
            "sendPublishFrameWithEintrRetry(*this->dataPtr->publisher"
        ),
        "replaced_raw_send_count": deletions.count(
            "this->dataPtr->publisher->send("
        ),
        "complete_publication_retry_present": False,
    }


def validate_source(source_dir: Path, manifest: dict[str, Any]) -> dict[str, Any]:
    if not source_dir.is_dir():
        raise ValidationError(f"source directory is missing: {source_dir}")
    commit = run("git", "rev-parse", "HEAD", cwd=source_dir)
    tree = run("git", "rev-parse", "HEAD^{tree}", cwd=source_dir)
    if commit != manifest["upstream_commit"] or tree != manifest["upstream_tree"]:
        raise ValidationError("checked-out upstream identity does not match the manifest")
    node_shared = source_dir / "src" / "NodeShared.cc"
    if sha256(node_shared) != manifest["patched_node_shared_sha256"]:
        raise ValidationError("patched NodeShared.cc hash mismatch")
    run("git", "diff", "--check", cwd=source_dir)
    status = run("git", "status", "--short", cwd=source_dir).splitlines()
    if status != ["M src/NodeShared.cc"]:
        raise ValidationError(f"patched source has unexpected changes: {status}")
    return {
        "source_dir": str(source_dir.resolve()),
        "commit": commit,
        "tree": tree,
        "node_shared_sha256": sha256(node_shared),
        "git_status": status,
    }


def system_protobuf_resolved_paths() -> dict[str, str]:
    multiarch = run("dpkg-architecture", "-qDEB_HOST_MULTIARCH")
    if not re.fullmatch(r"[A-Za-z0-9_.-]+", multiarch):
        raise ValidationError(f"invalid system multiarch identity: {multiarch!r}")
    library_dir = Path("/usr/lib") / multiarch
    candidates = {
        "Protobuf_INCLUDE_DIR": SYSTEM_PROTOBUF_INCLUDE,
        "Protobuf_LIBRARY_RELEASE": library_dir / "libprotobuf.so",
        "Protobuf_LITE_LIBRARY_RELEASE": library_dir / "libprotobuf-lite.so",
        "Protobuf_PROTOC_LIBRARY_RELEASE": library_dir / "libprotoc.so",
        "Protobuf_PROTOC_EXECUTABLE": SYSTEM_PROTOBUF_PROTOC,
    }
    missing = [str(path) for path in candidates.values() if not path.exists()]
    if missing:
        raise ValidationError(
            "pinned system Protobuf inputs are missing: " + ", ".join(missing)
        )
    return {key: str(path.resolve()) for key, path in candidates.items()}


def validate_protobuf_binding(
    path: Path,
    *,
    expected_resolved: Mapping[str, str] | None = None,
) -> dict[str, Any]:
    """Bind the configured build to Ubuntu Protobuf 3.21.12, not OR-Tools."""

    if not path.is_file() or path.is_symlink():
        raise ValidationError(f"Protobuf binding report is not a regular file: {path}")
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or set(data) != PROTOBUF_BINDING_KEYS:
        raise ValidationError("Protobuf binding report keys differ from schema v1")
    if (
        data.get("schema_version") != 1
        or data.get("status") != "SYSTEM_PROTOBUF_3_21_12_BINDING_PASSED"
        or data.get("passed") is not True
        or data.get("protobuf_version") != SYSTEM_PROTOBUF_VERSION
        or data.get("protobuf_header_version") != SYSTEM_PROTOBUF_HEADER_VERSION
        or data.get("config_mode_protobuf_disabled") is not True
    ):
        raise ValidationError("Protobuf binding identity or passing status drifted")
    if data.get("forbidden_prefix") != ORTOOLS_VENDOR_PREFIX:
        raise ValidationError("Protobuf binding does not forbid the OR-Tools prefix")
    command_count = data.get("compile_command_count")
    if isinstance(command_count, bool) or not isinstance(command_count, int) or command_count < 1:
        raise ValidationError("Protobuf binding has no audited compile commands")
    resolved = data.get("resolved")
    if not isinstance(resolved, dict) or set(resolved) != PROTOBUF_RESOLVED_KEYS:
        raise ValidationError("Protobuf resolved-path set differs from schema v1")
    expected = dict(
        system_protobuf_resolved_paths()
        if expected_resolved is None
        else expected_resolved
    )
    if set(expected) != PROTOBUF_RESOLVED_KEYS:
        raise ValidationError("internal expected Protobuf path set is incomplete")
    forbidden = ORTOOLS_VENDOR_PREFIX.lower()
    for key in sorted(PROTOBUF_RESOLVED_KEYS):
        if forbidden in str(resolved.get(key, "")).lower():
            raise ValidationError(f"Protobuf binding {key} references OR-Tools")
        actual_path = Path(str(resolved.get(key, ""))).resolve()
        expected_path = Path(str(expected[key])).resolve()
        if actual_path != expected_path:
            raise ValidationError(
                f"Protobuf binding {key} resolved to {actual_path}, expected {expected_path}"
            )
    return {
        "path": str(path.resolve()),
        "sha256": sha256(path),
        "schema_version": 1,
        "status": data["status"],
        "protobuf_version": data["protobuf_version"],
        "protobuf_header_version": data["protobuf_header_version"],
        "config_mode_protobuf_disabled": True,
        "forbidden_prefix": data["forbidden_prefix"],
        "compile_command_count": command_count,
        "resolved": {key: str(resolved[key]) for key in sorted(resolved)},
    }


def dynamic_symbols(path: Path) -> set[str]:
    output = run("nm", "-D", "--defined-only", str(path))
    symbols: set[str] = set()
    for line in output.splitlines():
        fields = line.split()
        if fields:
            symbols.add(fields[-1])
    return symbols


def demangle_symbols(symbols: Iterable[str]) -> dict[str, str]:
    """Demangle a dynamic-symbol set without changing its exact ABI spelling."""

    ordered = sorted(set(symbols))
    if not ordered:
        return {}
    result = subprocess.run(
        ["c++filt"],
        input="\n".join(ordered) + "\n",
        check=False,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise ValidationError(
            f"c++filt failed ({result.returncode}): {result.stderr.strip()[:400]}"
        )
    demangled = result.stdout.splitlines()
    if len(demangled) != len(ordered):
        raise ValidationError("c++filt returned an incomplete dynamic-symbol set")
    return dict(zip(ordered, demangled, strict=True))


def validate_public_gz_transport_v13_abi(
    reference_symbols: set[str],
    patched_symbols: set[str],
    *,
    reference_demangled: Mapping[str, str] | None = None,
    patched_demangled: Mapping[str, str] | None = None,
) -> dict[str, Any]:
    """Require full coverage of the reference v13 public namespace ABI.

    Different compiler flags can add inline / template instantiations or omit
    unrelated libstdc++ weak definitions.  Those are not a public
    gz-transport ABI break.  The installed 13.5.0 library remains the ABI
    reference, and every symbol it exports in ``gz::transport::v13`` must
    still exist with the exact mangled spelling in the patched library.
    """

    reference_names = dict(
        demangle_symbols(reference_symbols)
        if reference_demangled is None
        else reference_demangled
    )
    patched_names = dict(
        demangle_symbols(patched_symbols)
        if patched_demangled is None
        else patched_demangled
    )
    if set(reference_names) != reference_symbols:
        raise ValidationError("reference demangled symbol set is incomplete")
    if set(patched_names) != patched_symbols:
        raise ValidationError("patched demangled symbol set is incomplete")

    namespace_marker = "gz::transport::v13::"
    required_public = {
        symbol
        for symbol, demangled in reference_names.items()
        if namespace_marker in demangled
        and not demangled.startswith("guard variable for ")
    }
    if not required_public:
        raise ValidationError("system ABI reference exposes no gz::transport::v13 API")
    patched_public = {
        symbol
        for symbol, demangled in patched_names.items()
        if namespace_marker in demangled
        and not demangled.startswith("guard variable for ")
    }
    missing = sorted(required_public - patched_symbols)
    if missing:
        raise ValidationError(
            "patched core is missing public gz::transport v13 ABI symbols: "
            f"{missing[:10]}"
        )

    helper_exports = sorted(
        symbol
        for symbol, demangled in patched_names.items()
        if "sendPublishFrameWithEintrRetry" in demangled
    )
    if helper_exports:
        raise ValidationError(
            "patch-private EINTR helper leaked into the dynamic ABI: "
            f"{helper_exports[:10]}"
        )

    return {
        "reference_dynamic_symbol_count": len(reference_symbols),
        "patched_dynamic_symbol_count": len(patched_symbols),
        "reference_public_v13_symbol_count": len(required_public),
        "patched_public_v13_symbol_count": len(patched_public),
        "missing_public_v13_symbol_count": 0,
        "patch_private_helper_export_count": 0,
        "public_v13_abi_covered": True,
    }


def validate_protobuf_dynamic_linkage(
    dynamic_section: str,
    ldd_output: str,
    defined_symbols: str,
) -> dict[str, Any]:
    needed = re.findall(r"\(NEEDED\).*Shared library: \[([^\]]+)\]", dynamic_section)
    protobuf_needed = [name for name in needed if name.lower().startswith("libprotobuf")]
    if protobuf_needed != ["libprotobuf.so.32"]:
        raise ValidationError(
            "patched core must directly need exactly libprotobuf.so.32; "
            f"observed={protobuf_needed}"
        )
    forbidden_needed = [
        name for name in needed if "ortools" in name.lower() or "absl" in name.lower()
    ]
    if forbidden_needed:
        raise ValidationError(
            f"patched core has forbidden OR-Tools/Abseil dependency: {forbidden_needed}"
        )
    protobuf_matches = re.findall(
        r"^\s*libprotobuf\.so\.32\s+=>\s+(\S+)\s+\(",
        ldd_output,
        flags=re.MULTILINE,
    )
    if len(protobuf_matches) != 1:
        raise ValidationError("ldd did not resolve exactly one libprotobuf.so.32")
    runtime_path_raw = protobuf_matches[0]
    runtime_path = PurePosixPath(runtime_path_raw)
    if not runtime_path.is_absolute() or "ortools" in runtime_path_raw.lower():
        raise ValidationError(
            f"libprotobuf.so.32 resolved outside the system runtime: {runtime_path}"
        )
    static_vendor_symbols = [
        line.strip()
        for line in defined_symbols.splitlines()
        if "operations_research::" in line or "absl::" in line
    ]
    if static_vendor_symbols:
        raise ValidationError(
            "patched core exposes obvious statically linked OR-Tools/Abseil symbols"
        )
    return {
        "dynamic_needed": needed,
        "protobuf_needed": "libprotobuf.so.32",
        "protobuf_runtime_path": runtime_path.as_posix(),
        "forbidden_vendor_needed": [],
        "obvious_static_ortools_abseil_symbol_count": 0,
    }


def validate_install(install_prefix: Path) -> dict[str, Any]:
    library_dir = install_prefix / "lib"
    versioned = library_dir / "libgz-transport13.so.13.5.0"
    soname_alias = library_dir / "libgz-transport13.so.13"
    linker_alias = library_dir / "libgz-transport13.so"
    for path in (versioned, soname_alias, linker_alias):
        if not path.is_file() or path.is_symlink():
            raise ValidationError(f"installed core library is not a regular file: {path}")
    hashes = {path.name: sha256(path) for path in (versioned, soname_alias, linker_alias)}
    if len(set(hashes.values())) != 1:
        raise ValidationError("flattened core runtime library aliases differ")
    dynamic = run("readelf", "-d", str(versioned))
    if "Library soname: [libgz-transport13.so.13]" not in dynamic:
        raise ValidationError("patched library SONAME is not ABI-compatible")
    protobuf_linkage = validate_protobuf_dynamic_linkage(
        dynamic,
        run("ldd", str(versioned)),
        run("nm", "-C", "--defined-only", str(versioned)),
    )
    system_library = Path(
        "/opt/ros/jazzy/opt/gz_transport_vendor/lib/libgz-transport13.so.13.5.0"
    )
    if not system_library.is_file():
        raise ValidationError("system gz-transport13 ABI reference is missing")
    patched_symbols = dynamic_symbols(versioned)
    system_symbols = dynamic_symbols(system_library)
    abi = validate_public_gz_transport_v13_abi(system_symbols, patched_symbols)
    all_links = [path for path in install_prefix.rglob("*") if path.is_symlink()]
    if all_links:
        raise ValidationError(f"vendor install still contains symlinks: {all_links[:5]}")
    return {
        "install_prefix": str(install_prefix.resolve()),
        "core_library": str(versioned.resolve()),
        "core_library_sha256": hashes[versioned.name],
        "core_library_soname": "libgz-transport13.so.13",
        "dynamic_symbol_count": len(patched_symbols),
        "public_abi": abi,
        "protobuf_needed": protobuf_linkage["protobuf_needed"],
        "protobuf_runtime_path": protobuf_linkage["protobuf_runtime_path"],
        "dynamic_needed": protobuf_linkage["dynamic_needed"],
        "forbidden_vendor_needed": protobuf_linkage["forbidden_vendor_needed"],
        "obvious_static_ortools_abseil_symbol_count": protobuf_linkage[
            "obvious_static_ortools_abseil_symbol_count"
        ],
        "symlink_count": 0,
    }


def validate_runtime_activation(
    install_prefix: Path,
    runtime_plugins: Sequence[Path],
    environment: Mapping[str, str] | None = None,
) -> dict[str, Any]:
    """Prove dynamic consumers resolve to the patched in-prefix library.

    This check is deliberately run only after the final merged overlay has
    been sourced.  A successful vendor compilation in a separate prefix is
    not runtime activation evidence.
    """

    environment = os.environ if environment is None else environment
    install_prefix = install_prefix.resolve()
    library_dir = (install_prefix / "lib").resolve()
    ld_entries = [
        Path(row).resolve()
        for row in environment.get("LD_LIBRARY_PATH", "").split(os.pathsep)
        if row
    ]
    if not ld_entries or ld_entries[0] != library_dir:
        raise ValidationError(
            "patched runtime lib must be the first LD_LIBRARY_PATH entry"
        )
    if not runtime_plugins:
        raise ValidationError("runtime activation requires at least one consumer")

    expected_library = library_dir / "libgz-transport13.so.13"
    if not expected_library.is_file() or expected_library.is_symlink():
        raise ValidationError(
            f"patched runtime SONAME alias is not a regular file: {expected_library}"
        )
    expected_hash = sha256(expected_library)
    consumers: dict[str, dict[str, Any]] = {}
    for plugin in runtime_plugins:
        plugin = plugin.resolve()
        if not plugin.is_file() or plugin.is_symlink():
            raise ValidationError(
                f"runtime consumer is missing, linked, or non-regular: {plugin}"
            )
        try:
            plugin.relative_to(install_prefix)
        except ValueError as exc:
            raise ValidationError(
                f"runtime consumer escapes the merged install prefix: {plugin}"
            ) from exc
        output = run("ldd", str(plugin))
        matches = re.findall(
            r"^\s*libgz-transport13\.so\.13\s+=>\s+(\S+)\s+\(",
            output,
            flags=re.MULTILINE,
        )
        if len(matches) != 1:
            raise ValidationError(
                f"runtime consumer does not resolve exactly one gz-transport13: {plugin}"
            )
        resolved_library = Path(matches[0]).resolve()
        if resolved_library != expected_library.resolve():
            raise ValidationError(
                f"runtime consumer resolved system/unexpected gz-transport13: "
                f"{plugin} => {resolved_library}"
            )
        consumers[str(plugin)] = {
            "sha256": sha256(plugin),
            "resolved_library": str(resolved_library),
            "resolved_library_sha256": sha256(resolved_library),
        }
        if consumers[str(plugin)]["resolved_library_sha256"] != expected_hash:
            raise ValidationError("resolved runtime library hash drifted")
    return {
        "install_prefix": str(install_prefix),
        "ld_library_path_first": str(ld_entries[0]),
        "patched_soname_alias": str(expected_library.resolve()),
        "patched_soname_alias_sha256": expected_hash,
        "consumer_count": len(consumers),
        "consumers": consumers,
    }


def atomic_json(path: Path, payload: dict[str, Any]) -> None:
    if path.exists() or path.is_symlink():
        raise ValidationError(f"refusing stale validation output: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    pending = path.with_name(f"{path.name}.pending.{os.getpid()}")
    pending.write_text(
        json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    pending.replace(path)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path)
    parser.add_argument("--install-prefix", type=Path)
    parser.add_argument("--memory-preflight", type=Path)
    parser.add_argument("--protobuf-binding", type=Path)
    parser.add_argument("--parallel-workers", type=int)
    parser.add_argument("--runtime-plugin", type=Path, action="append", default=[])
    parser.add_argument("--require-active-runtime", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        manifest = load_manifest()
        result: dict[str, Any] = {
            "report_id": "tzcup_gz_transport13_eintr_vendor_v1",
            "status": "GZ_TRANSPORT13_EINTR_VENDOR_CONTRACT_PASSED",
            "passed": True,
            "recorded_epoch_ns": time.time_ns(),
            "manifest": manifest,
            "manifest_sha256": sha256(MANIFEST_PATH),
            "patch_contract": validate_patch_contract(manifest),
        }
        if args.protobuf_binding is None:
            raise ValidationError("--protobuf-binding is required")
        result["protobuf_binding"] = validate_protobuf_binding(args.protobuf_binding)
        if args.source_dir is not None:
            result["source"] = validate_source(args.source_dir, manifest)
        if args.install_prefix is not None:
            result["install"] = validate_install(args.install_prefix)
        if args.memory_preflight is not None:
            memory = json.loads(args.memory_preflight.read_text(encoding="utf-8"))
            if not memory.get("passed"):
                raise ValidationError("Windows memory preflight did not pass")
            result["memory_preflight"] = {
                "path": str(args.memory_preflight.resolve()),
                "sha256": sha256(args.memory_preflight),
                "status": memory.get("status"),
            }
        if args.parallel_workers is not None:
            if args.parallel_workers not in {1, 2}:
                raise ValidationError("parallel workers exceeded the reviewed bound")
            result["parallel_workers"] = args.parallel_workers
        if args.require_active_runtime or args.runtime_plugin:
            if args.install_prefix is None:
                raise ValidationError(
                    "runtime activation validation requires --install-prefix"
                )
            result["runtime_activation"] = validate_runtime_activation(
                args.install_prefix, args.runtime_plugin
            )
        if args.output is not None:
            atomic_json(args.output, result)
        else:
            print(json.dumps(result, indent=2, sort_keys=True))
        return 0
    except (OSError, ValidationError, subprocess.SubprocessError, json.JSONDecodeError) as exc:
        print(f"gz-transport13 EINTR vendor validation failed: {exc}", file=os.sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
