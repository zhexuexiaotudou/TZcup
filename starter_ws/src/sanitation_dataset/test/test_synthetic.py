import json

import numpy as np

from sanitation_dataset.synthetic import CLASS_ORDER, generate_scene, split_for_seed, write_dataset


def test_scene_has_all_classes_and_negative_is_not_labeled():
    scene = generate_scene(7)
    assert set(np.unique(scene.labels)) == set(range(len(CLASS_ORDER)))
    assert len(scene.objects) == 5
    assert [obj["class_id"] for obj in scene.objects] == list(CLASS_ORDER[1:])
    assert np.all(scene.labels[40:62, 3:15] == 0)


def test_scene_seed_split_is_leakage_safe(tmp_path):
    manifest = write_dataset(tmp_path, list(range(20)))
    split_sets = [set(manifest["split_scene_seeds"][name]) for name in ("train", "val", "test")]
    assert not split_sets[0] & split_sets[1]
    assert not split_sets[0] & split_sets[2]
    assert not split_sets[1] & split_sets[2]
    assert manifest["scene_count"] == 20
    assert manifest["duplicate_image_count"] == 0
    assert manifest["adjacent_frames_cross_split"] is False
    coco = json.loads((tmp_path / "annotations" / "coco.json").read_text(encoding="utf-8"))
    assert len(coco["images"]) == 20
    assert len(coco["annotations"]) == 100
