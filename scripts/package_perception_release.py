#!/usr/bin/env python3
"""Build a fail-closed, auditable product-perception release bundle."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
from datetime import datetime, timezone
import zipfile


ROOT = Path(__file__).resolve().parents[1]
MODEL_ROLES = ("detector", "classifier", "leaf_segmenter", "puddle_segmenter")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _copy(source: Path, target: Path) -> None:
    if not source.is_file():
        raise FileNotFoundError(source)
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)


def _registry(pipeline: Path, artifact_root: Path, required_provider: str):
    package_root = ROOT / "starter_ws/src/sanitation_perception"
    import sys

    sys.path.insert(0, str(package_root))
    from sanitation_perception.model_registry import ProductModelRegistry

    return ProductModelRegistry.load(
        pipeline,
        artifact_root,
        required_provider=required_provider,
        required_claim="formal",
    )


def _git_commit() -> str:
    return subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, check=True, text=True,
        capture_output=True,
    ).stdout.strip()


def _write_json(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def package(
    pipeline: Path,
    artifact_root: Path,
    output_dir: Path,
    *,
    required_provider: str,
    commit: str,
) -> tuple[Path, Path]:
    registry = _registry(pipeline, artifact_root, required_provider)
    release_id = f"{registry.pipeline_id}-{commit[:12]}"
    output_dir.mkdir(parents=True, exist_ok=True)
    archive = output_dir / f"TZcup_perception_product_{commit}.zip"
    with tempfile.TemporaryDirectory(prefix="tzcup-perception-release-") as temporary:
        stage = Path(temporary) / "TZcup_perception_product"
        manifests = stage / "manifests"
        models = stage / "models"
        configs = stage / "configs"
        launch = stage / "launch"
        licenses = stage / "licenses"
        rollback = stage / "rollback"
        for directory in (manifests, models, configs, launch, licenses, rollback):
            directory.mkdir(parents=True)
        _copy(pipeline, manifests / "perception_pipeline_manifest.yaml")
        for role in MODEL_ROLES:
            model = registry.models[role]
            _copy(model.manifest_path, manifests / model.manifest_path.name)
            _copy(model.artifact_path, models / model.artifact_path.name)
        config_root = ROOT / "starter_ws/src/sanitation_perception/config"
        for name in ("preprocess_spec.yaml", "postprocess_spec.yaml", "garbage_registry.yaml"):
            _copy(config_root / name, configs / name)
        for path in sorted((ROOT / "starter_ws/src/sanitation_perception/launch").glob("*.launch.py")):
            _copy(path, launch / path.name)
        for name in ("LICENSE.md", "MODEL_AND_ASSET_LICENSES.md", "THIRD_PARTY_SELECTION.md"):
            _copy(ROOT / name, licenses / name)
        _copy(ROOT / "docker/Dockerfile.perception-product", stage / "Dockerfile.perception-product")
        _copy(ROOT / "docker/compose.perception-product.yaml", stage / "compose.perception-product.yaml")
        _copy(ROOT / "scripts/healthcheck_perception.sh", stage / "healthcheck_perception.sh")
        _copy(ROOT / "scripts/perception_entrypoint.sh", stage / "perception_entrypoint.sh")
        _write_json(
            stage / "PERCEPTION_MODEL_REGISTRY.json",
            {"schema_version": 1, "release_id": release_id, **registry.model_info()},
        )
        _write_json(
            stage / "PERCEPTION_RELEASE_MANIFEST.json",
            {
                "schema_version": 1,
                "release_id": release_id,
                "source_commit": commit,
                "required_provider": required_provider,
                "created_at": datetime.now(timezone.utc).isoformat(),
                "claim": "formal_models_packaged; live readiness remains evidence-gated",
            },
        )
        (stage / "environment.lock").write_text(
            "ros_distro=jazzy\nonnxruntime_gpu=1.20.2\npython=3.12\nprovider="
            f"{required_provider}\n", encoding="utf-8",
        )
        (stage / "README.md").write_text(
            "# TZcup perception product release\n\n"
            "This bundle is immutable. Mount it read-only at `/opt/tzcup/release`. "
            "A successful package validates hashes and formal claims; it does not by "
            "itself prove live, performance, J6-board, or field acceptance.\n",
            encoding="utf-8",
        )
        (rollback / "README.md").write_text(
            "Rollback changes the atomic active-release pointer to the retained previous "
            "immutable bundle. Never overwrite files under a live release directory.\n",
            encoding="utf-8",
        )
        _write_json(
            stage / "SBOM.spdx.json",
            {
                "spdxVersion": "SPDX-2.3", "dataLicense": "CC0-1.0",
                "SPDXID": "SPDXRef-DOCUMENT", "name": release_id,
                "documentNamespace": f"https://github.com/zhexuexiaotudou/TZcup/perception/{release_id}",
                "creationInfo": {"created": datetime.now(timezone.utc).isoformat(),
                                 "creators": ["Tool: scripts/package_perception_release.py"]},
                "packages": [{"SPDXID": "SPDXRef-Package-TZcupPerception",
                              "name": "TZcup-perception-product", "versionInfo": commit,
                              "downloadLocation": "NOASSERTION", "filesAnalyzed": False,
                              "licenseConcluded": "Apache-2.0", "licenseDeclared": "Apache-2.0",
                              "copyrightText": "NOASSERTION"}],
            },
        )
        checksum_paths = [path for path in sorted(stage.rglob("*")) if path.is_file()]
        (stage / "SHA256SUMS").write_text(
            "".join(f"{sha256(path)}  {path.relative_to(stage).as_posix()}\n" for path in checksum_paths),
            encoding="ascii",
        )
        with zipfile.ZipFile(archive, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as target:
            for path in sorted(stage.rglob("*")):
                if path.is_file():
                    target.write(path, path.relative_to(stage.parent).as_posix())
    digest_path = archive.with_suffix(".zip.sha256")
    digest_path.write_text(f"{sha256(archive)}  {archive.name}\n", encoding="ascii")
    return archive, digest_path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pipeline", type=Path, required=True)
    parser.add_argument("--artifact-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--required-provider", default="CUDAExecutionProvider")
    parser.add_argument("--commit", default=None)
    args = parser.parse_args()
    archive, digest = package(
        args.pipeline.resolve(), args.artifact_root.resolve(), args.output_dir.resolve(),
        required_provider=args.required_provider, commit=args.commit or _git_commit(),
    )
    print(json.dumps({"archive": str(archive), "sha256_file": str(digest)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
