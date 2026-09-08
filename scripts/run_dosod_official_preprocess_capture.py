#!/usr/bin/env python3
"""Run a supplied official Y/UV producer, then seal only its real outputs.

This is intentionally a board/Linux producer wrapper, not an RGB/NV12
implementation.  The official tool and its verified identity remain external.
"""
from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path

from hbm_evidence_common import fresh_directory, normal_file, run_owned_process
import capture_dosod_official_preprocess as sealer


def _dpkg(package: str, path_role: str) -> tuple[str, int]:
    result = subprocess.run(
        ["dpkg-query", "-W", "-f=${Package}\t${Version}\n", package],
        capture_output=True, text=True, check=False,
    )
    line = result.stdout.strip()
    return (f"{line}\t{path_role}\n" if result.returncode == 0 and line else "", result.returncode)


def _command(items: list[str], values: dict[str, str]) -> list[str]:
    required = {"{raw_rgb}", "{images_y}", "{images_uv}"}
    if not items or not required.issubset(set(items)):
        raise ValueError("producer_command_must_contain_raw_rgb_images_y_images_uv_placeholders")
    return [values.get(item, item) for item in items]


def run(*, pilot_manifest: Path, pilot_record_index: int, adapter_binary: Path,
        adapter_source: Path, dpkg_package: str, dpkg_path_role: str,
        producer_command: list[str], output: Path, timeout_seconds: int) -> dict:
    normal_file(adapter_binary, "official_adapter_binary")
    normal_file(adapter_source, "official_adapter_source")
    if not dpkg_package or not dpkg_path_role or timeout_seconds <= 0:
        raise ValueError("official_producer_arguments_invalid")
    fresh_directory(output, "official_preprocess_producer_output")
    raw = sealer._pilot_binding(pilot_manifest, pilot_record_index)
    y, uv = output / "images_y.bin", output / "images_uv.bin"
    command = _command(producer_command, {
        "{raw_rgb}": raw["path"], "{images_y}": str(y), "{images_uv}": str(uv),
        "{width}": str(raw["width"]), "{height}": str(raw["height"]),
        "{step}": str(raw["step"]), "{encoding}": str(raw["encoding"]),
    })
    if command[0] != str(adapter_binary.resolve()):
        raise ValueError("producer_command_must_start_with_adapter_binary")
    returncode, stdout, stderr, execution = run_owned_process(
        command, timeout_seconds=timeout_seconds
    )
    stdout_path, stderr_path, dpkg_path = output / "official.stdout.txt", output / "official.stderr.txt", output / "official.dpkg.txt"
    stdout_path.write_text(stdout, encoding="utf-8")
    stderr_path.write_text(stderr, encoding="utf-8")
    dpkg_text, dpkg_returncode = _dpkg(dpkg_package, dpkg_path_role)
    dpkg_path.write_text(dpkg_text, encoding="utf-8")
    return sealer.capture(
        pilot_manifest=pilot_manifest, pilot_record_index=pilot_record_index,
        images_y=y, images_uv=uv, adapter_binary=adapter_binary,
        adapter_source=adapter_source, dpkg_output=dpkg_path, stdout=stdout_path,
        stderr=stderr_path, command=command, returncode=returncode,
        output=output / "sealed", dpkg_returncode=dpkg_returncode,
        execution=execution,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pilot-manifest", required=True, type=Path)
    parser.add_argument("--pilot-record-index", required=True, type=int)
    parser.add_argument("--adapter-binary", required=True, type=Path)
    parser.add_argument("--adapter-source", required=True, type=Path)
    parser.add_argument("--dpkg-package", required=True)
    parser.add_argument("--dpkg-path-role", required=True)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--timeout-sec", type=int, default=120)
    parser.add_argument("--producer-command", required=True, nargs=argparse.REMAINDER)
    args = parser.parse_args()
    try:
        receipt = run(**vars(args))
    except Exception as error:
        print(f"official_preprocess_producer_blocked:{type(error).__name__}:{error}")
        return 2
    print(json.dumps(receipt, indent=2))
    return 0 if receipt["status"] == "OFFICIAL_PREPROCESS_CAPTURED" else 2


if __name__ == "__main__":
    raise SystemExit(main())
