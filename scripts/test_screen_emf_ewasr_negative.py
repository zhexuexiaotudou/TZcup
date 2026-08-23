import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import pytest
import screen_emf_ewasr_negative as screening


class _Tensor:
    def __init__(self, name, shape):
        self.name = name
        self.shape = shape


class _Session:
    def __init__(self, outputs):
        self.outputs = iter(outputs)

    def get_providers(self):
        return ["CPUExecutionProvider"]

    def get_inputs(self):
        return [_Tensor("image", [1, 3, 384, 512])]

    def get_outputs(self):
        return [
            _Tensor("prediction", [1, 3, 96, 128]),
            _Tensor("intermediate", [1, 256, 24, 32]),
        ]

    def run(self, output_names, inputs):
        assert output_names == ["prediction"]
        assert set(inputs) == {"image"}
        assert inputs["image"].shape == (1, 3, 384, 512)
        return [next(self.outputs)]


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_record(root: Path, name: str, semantic_label: int = 0) -> dict:
    rgb = root / f"{name}.png"
    semantic = root / f"{name}.npy"
    image = np.full((24, 32, 3), 127, dtype=np.uint8)
    image[0, 0, 0] = sum(name.encode("utf-8")) % 256
    assert cv2.imwrite(str(rgb), image)
    np.save(semantic, np.full((24, 32), semantic_label, dtype=np.uint8))
    return {
        "record_id": name,
        "rgb_path": rgb.name,
        "rgb_sha256": _sha(rgb),
        "semantic_path": semantic.name,
        "semantic_sha256": _sha(semantic),
    }


def _write_manifest(root: Path, records: list[dict], **updates) -> Path:
    payload = {
        "schema_version": 1,
        "dataset_id": "fixed_train_negative",
        "split": "train",
        "negative_only": True,
        "semantic_label_contract": {"leaf_pile": 4, "puddle": 5},
        "records": records,
    }
    payload.update(updates)
    path = root / "manifest.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def _write_area_dataset_manifest(
    root: Path,
    *,
    protocol_id: str = "EMFJ6V3",
    development_only: bool = True,
    sealed_access_allowed: bool = False,
    selected_split: str = "TRAIN",
) -> tuple[Path, Path]:
    source_root = root / "training_capture"
    source_root.mkdir(parents=True)
    legacy_records = [
        _write_record(source_root, "area_frame_0"),
        _write_record(source_root, "area_frame_1"),
    ]
    frames = []
    for index, record in enumerate(legacy_records):
        frames.append(
            {
                "root_id": "TRAIN_004",
                "split": "TRAIN",
                "world_id": "outdoor_world",
                "scene_id": "scene_negative",
                "frame_index": index,
                "paths": {
                    "rgb": record["rgb_path"],
                    "semantic": record["semantic_path"],
                },
                "sha256": {
                    "rgb": record["rgb_sha256"],
                    "semantic": record["semantic_sha256"],
                },
                "semantic_pixel_counts": {
                    "0": 24 * 32,
                    "1": 0,
                    "2": 0,
                    "3": 0,
                    "4": 0,
                    "5": 0,
                },
            }
        )
    payload = {
        "schema_version": screening.AREA_DATASET_SCHEMA,
        "protocol_id": protocol_id,
        "dataset_role": "area_model_screening_development_only",
        "development_only": development_only,
        "sealed_access_allowed": sealed_access_allowed,
        "source_roots": [
            {
                "root_id": "TRAIN_004",
                "split": selected_split,
                "path": str(source_root),
            },
            {
                "root_id": "HOLDOUT_000",
                "split": "HOLDOUT",
                "path": str(root / "unused_holdout"),
            },
        ],
        "frames": frames,
    }
    payload["manifest_sha256"] = screening._canonical_manifest_sha256(payload)
    manifest = root / "area_dataset_manifest.json"
    manifest.write_text(json.dumps(payload, sort_keys=True), encoding="utf-8")
    return manifest, source_root


def _rewrite_area_manifest(manifest: Path, mutate) -> dict:
    payload = json.loads(manifest.read_text(encoding="utf-8"))
    mutate(payload)
    payload["manifest_sha256"] = screening._canonical_manifest_sha256(payload)
    manifest.write_text(json.dumps(payload, sort_keys=True), encoding="utf-8")
    return payload


def _allow_area_manifest(monkeypatch, manifest: Path) -> tuple[str, str]:
    payload = json.loads(manifest.read_text(encoding="utf-8"))
    pair = (_sha(manifest), payload["manifest_sha256"])
    monkeypatch.setattr(
        screening, "AREA_MANIFEST_SHA256_ALLOWLIST", frozenset({pair})
    )
    return pair


def _simulate_symlink_when_unavailable(monkeypatch, link: Path, target: Path) -> None:
    try:
        link.symlink_to(target, target_is_directory=True)
    except OSError:
        original_is_symlink = Path.is_symlink
        link_absolute = link.absolute()

        def simulated_is_symlink(path):
            return path.absolute() == link_absolute or original_is_symlink(path)

        monkeypatch.setattr(Path, "is_symlink", simulated_is_symlink)


def test_preprocess_uses_rgb_imagenet_contract():
    rgb = np.zeros((10, 20, 3), dtype=np.uint8)
    rgb[:, :, 0] = 255
    tensor = screening.preprocess_rgb(rgb)
    assert tensor.shape == (1, 3, 384, 512)
    assert tensor.dtype == np.float32
    assert tensor.flags.c_contiguous
    assert tensor[0, 0, 0, 0] == pytest.approx((1.0 - 0.485) / 0.229)
    assert tensor[0, 1, 0, 0] == pytest.approx((0.0 - 0.456) / 0.224)


def test_evaluate_preserves_source_classes_and_reports_water_activation(
    tmp_path, monkeypatch
):
    records = [_write_record(tmp_path, "frame_0"), _write_record(tmp_path, "frame_1")]
    manifest = _write_manifest(tmp_path, records)
    model = tmp_path / "ewasr.onnx"
    model.write_bytes(b"fixed ewasr test model")
    monkeypatch.setattr(screening, "EXPECTED_MODEL_SHA256", _sha(model))

    first = np.zeros((1, 3, 96, 128), dtype=np.float32)
    first[:, 0] = 1.0
    first[0, 1, 1:3, 1:3] = 2.0
    first[0, 1, 10, 10] = 2.0
    second = np.zeros((1, 3, 96, 128), dtype=np.float32)
    second[:, 2] = 1.0
    session = _Session([first, second])

    report = screening.evaluate(
        model,
        manifest,
        session_factory=lambda *_args, **_kwargs: session,
    )

    assert report["model"]["class_order"] == [
        "obstacles_or_environment",
        "water",
        "sky",
    ]
    assert report["model"]["decode"] == "raw_logits_argmax"
    assert report["water_activation"] == {
        "measurement_grid_hw": [96, 128],
        "activated_frame_count": 1,
        "activated_frame_rate": 0.5,
        "water_pixel_count": 5,
        "total_pixel_count": 2 * 96 * 128,
        "water_pixel_fraction": 5 / (2 * 96 * 128),
        "water_component_count": 2,
    }
    assert report["target_class_mapping"] is None
    assert report["target_area_metrics_computed"] is False
    assert report["a4_area_pass_computed"] is False


def test_area_dataset_manifest_selects_explicit_train_root_and_reports_source_sha(
    tmp_path, monkeypatch
):
    manifest, _source_root = _write_area_dataset_manifest(tmp_path)
    file_sha, canonical_sha = _allow_area_manifest(monkeypatch, manifest)
    model = tmp_path / "ewasr.onnx"
    model.write_bytes(b"fixed ewasr test model")
    monkeypatch.setattr(screening, "EXPECTED_MODEL_SHA256", _sha(model))
    outputs = []
    for _index in range(2):
        logits = np.zeros((1, 3, 96, 128), dtype=np.float32)
        logits[:, 0] = 1.0
        outputs.append(logits)
    session = _Session(outputs)

    report = screening.evaluate(
        model,
        manifest,
        source_root_id="TRAIN_004",
        session_factory=lambda *_args, **_kwargs: session,
    )

    assert report["source_root_id"] == "TRAIN_004"
    assert report["source_area_manifest_sha256"] == _sha(manifest)
    assert report["source_area_manifest_sha256"] == file_sha
    assert report["source_area_manifest_canonical_sha256"] == canonical_sha
    assert report["dataset"]["source_root_id"] == "TRAIN_004"
    assert report["dataset"]["source_area_manifest_sha256"] == _sha(manifest)
    assert report["dataset"]["source_area_manifest_canonical_sha256"] == canonical_sha
    assert report["dataset"]["split"] == "TRAIN"
    assert report["dataset"]["frame_count"] == 2


def test_final_area_manifest_sha_pair_is_locked():
    assert (
        "8c9f4c06bcf2a59a3ce15bc53c716c6411945b92870eaee18789bf4ddc291720",
        "056a3b599e8b2b3aa5de141a0e6234ec6b1dbe1ed561c999b7e43123a827828e",
    ) in screening.AREA_MANIFEST_SHA256_ALLOWLIST


def test_area_dataset_manifest_requires_source_root_and_unique_train_selection(
    tmp_path, monkeypatch
):
    manifest, _source_root = _write_area_dataset_manifest(tmp_path)
    _allow_area_manifest(monkeypatch, manifest)
    with pytest.raises(ValueError, match="source_root_id is required"):
        screening.load_negative_manifest(manifest)

    _rewrite_area_manifest(
        manifest,
        lambda payload: payload["source_roots"].append(
            dict(payload["source_roots"][0])
        ),
    )
    _allow_area_manifest(monkeypatch, manifest)
    with pytest.raises(ValueError, match="root ids must be unique"):
        screening.load_negative_manifest(manifest, source_root_id="TRAIN_004")

    manifest, _source_root = _write_area_dataset_manifest(
        tmp_path / "holdout_case", selected_split="HOLDOUT"
    )
    _allow_area_manifest(monkeypatch, manifest)
    with pytest.raises(ValueError, match="split=TRAIN"):
        screening.load_negative_manifest(manifest, source_root_id="TRAIN_004")


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("protocol_id", "OTHER", "protocol_id"),
        ("development_only", False, "development_only"),
        ("sealed_access_allowed", True, "forbid sealed access"),
    ],
)
def test_area_dataset_manifest_protocol_boundary_fails_closed(
    tmp_path, monkeypatch, field, value, message
):
    updates = {field: value}
    manifest, _source_root = _write_area_dataset_manifest(tmp_path, **updates)
    _allow_area_manifest(monkeypatch, manifest)
    with pytest.raises(ValueError, match=message):
        screening.load_negative_manifest(manifest, source_root_id="TRAIN_004")


def test_area_dataset_manifest_checks_declared_and_actual_negative_semantics(
    tmp_path, monkeypatch
):
    manifest, source_root = _write_area_dataset_manifest(tmp_path)
    _rewrite_area_manifest(
        manifest,
        lambda payload: payload["frames"][0]["semantic_pixel_counts"].update(
            {"4": 1}
        ),
    )
    _allow_area_manifest(monkeypatch, manifest)
    with pytest.raises(ValueError, match="declares semantic ID 4 or 5"):
        screening.load_negative_manifest(manifest, source_root_id="TRAIN_004")

    manifest, source_root = _write_area_dataset_manifest(tmp_path / "actual_case")
    semantic = source_root / "area_frame_0.npy"
    np.save(semantic, np.full((24, 32), 5, dtype=np.uint8))

    def update_actual(payload):
        payload["frames"][0]["sha256"]["semantic"] = _sha(semantic)

    _rewrite_area_manifest(manifest, update_actual)
    _allow_area_manifest(monkeypatch, manifest)
    with pytest.raises(ValueError, match="negative-only record contains"):
        screening.load_negative_manifest(manifest, source_root_id="TRAIN_004")


def test_area_dataset_manifest_rejects_unallowlisted_sha_pair(tmp_path):
    manifest, _source_root = _write_area_dataset_manifest(tmp_path)
    with pytest.raises(ValueError, match="pair is not allowlisted"):
        screening.load_negative_manifest(manifest, source_root_id="TRAIN_004")


def test_area_dataset_manifest_rejects_unknown_declared_and_actual_semantic_ids(
    tmp_path, monkeypatch
):
    manifest, source_root = _write_area_dataset_manifest(tmp_path)
    _rewrite_area_manifest(
        manifest,
        lambda payload: payload["frames"][0]["semantic_pixel_counts"].update(
            {"6": 1}
        ),
    )
    _allow_area_manifest(monkeypatch, manifest)
    with pytest.raises(ValueError, match="declares unknown semantic IDs"):
        screening.load_negative_manifest(manifest, source_root_id="TRAIN_004")

    manifest, source_root = _write_area_dataset_manifest(tmp_path / "actual_unknown")
    semantic = source_root / "area_frame_0.npy"
    np.save(semantic, np.full((24, 32), 6, dtype=np.uint8))

    def update_actual(payload):
        payload["frames"][0]["sha256"]["semantic"] = _sha(semantic)

    _rewrite_area_manifest(manifest, update_actual)
    _allow_area_manifest(monkeypatch, manifest)
    with pytest.raises(ValueError, match="contains unknown semantic IDs"):
        screening.load_negative_manifest(manifest, source_root_id="TRAIN_004")


def test_area_dataset_manifest_rejects_path_escape(tmp_path, monkeypatch):
    manifest, source_root = _write_area_dataset_manifest(tmp_path)
    outside = tmp_path / "outside.png"
    outside.write_bytes((source_root / "area_frame_0.png").read_bytes())

    def escape(payload):
        payload["frames"][0]["paths"]["rgb"] = "../outside.png"
        payload["frames"][0]["sha256"]["rgb"] = _sha(outside)

    _rewrite_area_manifest(manifest, escape)
    _allow_area_manifest(monkeypatch, manifest)
    with pytest.raises(ValueError, match="escapes its selected source root"):
        screening.load_negative_manifest(manifest, source_root_id="TRAIN_004")


def test_area_dataset_manifest_rejects_source_root_ancestor_symlink(
    tmp_path, monkeypatch
):
    real_parent = tmp_path / "real_parent"
    manifest, _source_root = _write_area_dataset_manifest(real_parent)
    alias = tmp_path / "aliased_parent"
    _simulate_symlink_when_unavailable(monkeypatch, alias, real_parent)

    _rewrite_area_manifest(
        manifest,
        lambda payload: payload["source_roots"][0].update(
            {"path": str(alias / "training_capture")}
        ),
    )
    _allow_area_manifest(monkeypatch, manifest)
    with pytest.raises(ValueError, match="symlinked input is not allowed"):
        screening.load_negative_manifest(manifest, source_root_id="TRAIN_004")


def test_evaluate_rejects_manifest_ancestor_symlink_before_resolve(
    tmp_path, monkeypatch
):
    real_parent = tmp_path / "real_parent"
    manifest, _source_root = _write_area_dataset_manifest(real_parent)
    _allow_area_manifest(monkeypatch, manifest)
    alias = tmp_path / "aliased_parent"
    _simulate_symlink_when_unavailable(monkeypatch, alias, real_parent)
    model = tmp_path / "ewasr.onnx"
    model.write_bytes(b"fixed ewasr test model")
    monkeypatch.setattr(screening, "EXPECTED_MODEL_SHA256", _sha(model))

    with pytest.raises(ValueError, match="symlinked input is not allowed"):
        screening.evaluate(
            model,
            alias / manifest.name,
            source_root_id="TRAIN_004",
            session_factory=lambda *_args, **_kwargs: None,
        )


@pytest.mark.parametrize(
    "forbidden",
    ["G5", "g5-v2", "G5_V2", "VAL_NEW", "dev-val", "SEALED_FINAL"],
)
def test_forbidden_dataset_families_fail_closed(forbidden):
    with pytest.raises(ValueError, match="forbidden source"):
        screening.validate_nonsealed_value(f"root/{forbidden}/train")


@pytest.mark.parametrize("semantic_label", [4, 5])
def test_manifest_rejects_target_positive_frames(tmp_path, semantic_label):
    record = _write_record(tmp_path, "positive", semantic_label)
    manifest = _write_manifest(tmp_path, [record])
    with pytest.raises(ValueError, match="negative-only record contains"):
        screening.load_negative_manifest(manifest)


def test_manifest_requires_locked_hashes(tmp_path):
    record = _write_record(tmp_path, "negative")
    record["rgb_sha256"] = "0" * 64
    manifest = _write_manifest(tmp_path, [record])
    with pytest.raises(ValueError, match="RGB SHA-256 mismatch"):
        screening.load_negative_manifest(manifest)


def test_manifest_rejects_duplicate_rgb_hashes(tmp_path):
    first = _write_record(tmp_path, "negative")
    second = dict(first, record_id="duplicate")
    manifest = _write_manifest(tmp_path, [first, second])
    with pytest.raises(ValueError, match="duplicate RGB SHA-256"):
        screening.load_negative_manifest(manifest)
