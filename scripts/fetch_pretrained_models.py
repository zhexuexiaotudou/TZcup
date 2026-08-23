#!/usr/bin/env python3
"""Fetch pinned Hugging Face ONNX artifacts into a non-Git artifact root."""

from __future__ import annotations

import argparse
import hashlib
import os
import shutil
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REGISTRY = ROOT / "config" / "pretrained_model_sources.yaml"
DEFAULT_ARTIFACT_ROOT = ROOT / ".workspace" / "models"
LOCK_FILENAME = "PRETRAINED_MODEL_DOWNLOAD_LOCK.yaml"
SHA256_LENGTH = 64


class FetchBlocked(RuntimeError):
    """A fail-closed source or integrity boundary blocked the fetch."""


def file_sha256(path: Path, chunk_size: int = 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(chunk_size), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_registry(path: Path) -> dict[str, Any]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or payload.get("schema_version") != 1:
        raise FetchBlocked("unsupported or malformed pretrained model registry")
    models = payload.get("models")
    if not isinstance(models, dict) or not models:
        raise FetchBlocked("registry contains no models")
    return payload


def _safe_filename(filename: str) -> str:
    path = Path(filename)
    if not filename or path.is_absolute() or ".." in path.parts:
        raise FetchBlocked(f"unsafe source filename: {filename!r}")
    return path.as_posix()


def _safe_model_id(model_id: str) -> str:
    if (
        not model_id
        or model_id in {".", ".."}
        or "/" in model_id
        or "\\" in model_id
    ):
        raise FetchBlocked(f"unsafe model id: {model_id!r}")
    return model_id


def validate_fetchable(model_id: str, entry: dict[str, Any]) -> dict[str, Any]:
    source = entry.get("source")
    if not isinstance(source, dict):
        raise FetchBlocked(f"{model_id}: missing source metadata")
    if source.get("provider") != "huggingface":
        raise FetchBlocked(f"{model_id}: unsupported provider")
    revision = str(source.get("revision", ""))
    if len(revision) != 40 or any(char not in "0123456789abcdef" for char in revision):
        raise FetchBlocked(f"{model_id}: revision must be a full lowercase commit SHA")
    filename = _safe_filename(str(source.get("filename", "")))
    if Path(filename).suffix.lower() != ".onnx":
        raise FetchBlocked(f"{model_id}: source artifact is not ONNX")
    if source.get("file_present_at_revision") is not True:
        raise FetchBlocked(
            f"{model_id}: pinned source revision does not contain {filename}; "
            "local conversion cannot be represented as a source ONNX"
        )
    expected_sha = source.get("expected_sha256")
    if (
        not isinstance(expected_sha, str)
        or len(expected_sha) != SHA256_LENGTH
        or any(char not in "0123456789abcdef" for char in expected_sha)
    ):
        raise FetchBlocked(f"{model_id}: source ONNX has no valid expected SHA-256")
    size = source.get("expected_size_bytes")
    if not isinstance(size, int) or size <= 0:
        raise FetchBlocked(f"{model_id}: source ONNX has no valid expected size")
    repo_id = str(source.get("repo_id", ""))
    if repo_id.count("/") != 1:
        raise FetchBlocked(f"{model_id}: invalid Hugging Face repo_id")
    return source


def huggingface_url(source: dict[str, Any]) -> str:
    filename = urllib.parse.quote(_safe_filename(str(source["filename"])), safe="/")
    repo_id = urllib.parse.quote(str(source["repo_id"]), safe="/")
    return (
        f"https://huggingface.co/{repo_id}/resolve/"
        f"{source['revision']}/{filename}?download=true"
    )


def _verified(path: Path, expected_sha256: str, expected_size: int) -> bool:
    return (
        path.is_file()
        and path.stat().st_size == expected_size
        and file_sha256(path) == expected_sha256
    )


def copy_from_offline_cache(
    cache_dir: Path,
    model_id: str,
    filename: str,
    destination: Path,
    expected_sha256: str,
    expected_size: int,
) -> bool:
    candidates = (
        cache_dir / model_id / filename,
        cache_dir / expected_sha256 / filename,
        cache_dir / filename,
    )
    for candidate in candidates:
        if _verified(candidate, expected_sha256, expected_size):
            destination.parent.mkdir(parents=True, exist_ok=True)
            temporary = destination.with_suffix(destination.suffix + ".cache-copy.part")
            shutil.copyfile(candidate, temporary)
            os.replace(temporary, destination)
            return True
    return False


def download_with_resume(
    url: str,
    destination: Path,
    expected_sha256: str,
    expected_size: int,
    *,
    timeout: float,
    retries: int,
    chunk_size: int = 1024 * 1024,
) -> dict[str, Any]:
    destination.parent.mkdir(parents=True, exist_ok=True)
    if _verified(destination, expected_sha256, expected_size):
        return {"cache_hit": True, "resumed_from_bytes": expected_size}
    if destination.exists():
        destination.unlink()
    partial = destination.with_suffix(destination.suffix + ".part")
    if partial.exists() and partial.stat().st_size > expected_size:
        partial.unlink()
    if partial.exists() and partial.stat().st_size == expected_size:
        if file_sha256(partial) == expected_sha256:
            os.replace(partial, destination)
            return {"cache_hit": False, "resumed_from_bytes": expected_size}
        partial.unlink()

    last_error: Exception | None = None
    initial_offset = partial.stat().st_size if partial.exists() else 0
    for attempt in range(retries + 1):
        offset = partial.stat().st_size if partial.exists() else 0
        headers = {"User-Agent": "TZcup-pretrained-model-fetch/1"}
        if offset:
            headers["Range"] = f"bytes={offset}-"
        request = urllib.request.Request(url, headers=headers)
        try:
            with urllib.request.urlopen(request, timeout=timeout) as response:
                status = getattr(response, "status", response.getcode())
                if offset and status == 206:
                    mode = "ab"
                elif status == 200:
                    mode = "wb"
                    offset = 0
                else:
                    raise FetchBlocked(
                        f"unexpected HTTP status {status} while resuming at {offset}"
                    )
                with partial.open(mode) as stream:
                    while True:
                        chunk = response.read(chunk_size)
                        if not chunk:
                            break
                        stream.write(chunk)
            if partial.stat().st_size != expected_size:
                raise FetchBlocked(
                    f"downloaded size {partial.stat().st_size} does not match "
                    f"expected {expected_size}"
                )
            actual_sha = file_sha256(partial)
            if actual_sha != expected_sha256:
                raise FetchBlocked(
                    f"downloaded SHA-256 {actual_sha} does not match expected "
                    f"{expected_sha256}"
                )
            os.replace(partial, destination)
            return {"cache_hit": False, "resumed_from_bytes": initial_offset}
        except (OSError, urllib.error.URLError, FetchBlocked) as error:
            last_error = error
            if isinstance(error, FetchBlocked) and (
                "SHA-256" in str(error) or "unexpected HTTP" in str(error)
            ):
                break
            if attempt < retries:
                time.sleep(min(2**attempt, 8))
    raise FetchBlocked(f"download failed after {retries + 1} attempt(s): {last_error}")


def fetch_one(
    model_id: str,
    entry: dict[str, Any],
    artifact_root: Path,
    *,
    offline: bool,
    cache_dir: Path | None,
    timeout: float,
    retries: int,
    asserted_sha256: str | None = None,
) -> dict[str, Any]:
    model_id = _safe_model_id(model_id)
    source = validate_fetchable(model_id, entry)
    expected_sha = str(source["expected_sha256"])
    if asserted_sha256 is not None and asserted_sha256.lower() != expected_sha:
        raise FetchBlocked(
            f"{model_id}: --expected-sha256 differs from the pinned registry"
        )
    filename = _safe_filename(str(source["filename"]))
    destination = artifact_root / model_id / filename
    expected_size = int(source["expected_size_bytes"])
    method = "artifact_cache"
    destination_verified = _verified(destination, expected_sha, expected_size)
    details: dict[str, Any] = {
        "cache_hit": destination_verified,
        "resumed_from_bytes": expected_size if destination_verified else 0,
    }
    if not destination_verified:
        copied = bool(
            cache_dir
            and copy_from_offline_cache(
                cache_dir,
                model_id,
                filename,
                destination,
                expected_sha,
                expected_size,
            )
        )
        if copied:
            method = "offline_cache"
            details["cache_hit"] = True
        elif offline:
            raise FetchBlocked(
                f"{model_id}: verified artifact not found in offline cache"
            )
        else:
            method = "download"
            details = download_with_resume(
                huggingface_url(source),
                destination,
                expected_sha,
                expected_size,
                timeout=timeout,
                retries=retries,
            )
    return {
        "model_id": model_id,
        "candidate": entry.get("candidate"),
        "source_uri": source.get("uri"),
        "repo_id": source["repo_id"],
        "revision": source["revision"],
        "filename": filename,
        "sha256": file_sha256(destination),
        "size_bytes": destination.stat().st_size,
        "artifact": str(destination),
        "method": method,
        **details,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, default=DEFAULT_REGISTRY)
    parser.add_argument("--artifact-root", type=Path, default=DEFAULT_ARTIFACT_ROOT)
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument("--model-id", action="append")
    selection.add_argument("--all", action="store_true")
    parser.add_argument("--expected-sha256")
    parser.add_argument("--cache-dir", type=Path)
    parser.add_argument("--offline", action="store_true")
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--retries", type=int, default=3)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.timeout <= 0 or args.retries < 0:
        print("ERROR: timeout must be positive and retries non-negative", file=sys.stderr)
        return 2
    registry = load_registry(args.registry.resolve())
    models = registry["models"]
    selected = list(models) if args.all else list(args.model_id)
    if args.expected_sha256 and len(selected) != 1:
        print("ERROR: --expected-sha256 requires exactly one --model-id", file=sys.stderr)
        return 2
    unknown = sorted(set(selected) - set(models))
    if unknown:
        print(f"ERROR: unknown model id(s): {', '.join(unknown)}", file=sys.stderr)
        return 2
    artifact_root = args.artifact_root.resolve()
    artifact_root.mkdir(parents=True, exist_ok=True)
    cache_dir = args.cache_dir.resolve() if args.cache_dir else None
    records: list[dict[str, Any]] = []
    failures: list[dict[str, str]] = []
    for model_id in selected:
        try:
            records.append(
                fetch_one(
                    model_id,
                    models[model_id],
                    artifact_root,
                    offline=args.offline,
                    cache_dir=cache_dir,
                    timeout=args.timeout,
                    retries=args.retries,
                    asserted_sha256=args.expected_sha256,
                )
            )
        except FetchBlocked as error:
            failures.append({"model_id": model_id, "error": str(error)})
            print(f"BLOCKED: {error}", file=sys.stderr)
    lock = {
        "schema_version": 1,
        "registry": str(args.registry.resolve()),
        "artifact_root": str(artifact_root),
        "offline": bool(args.offline),
        "models": records,
        "failures": failures,
        "complete": not failures,
    }
    (artifact_root / LOCK_FILENAME).write_text(
        yaml.safe_dump(lock, sort_keys=False), encoding="utf-8"
    )
    print(yaml.safe_dump(lock, sort_keys=False), end="")
    return 0 if not failures else 2


if __name__ == "__main__":
    raise SystemExit(main())
