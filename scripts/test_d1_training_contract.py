from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "scripts/train_ddrv4_d1_rtmdet.py").read_text(encoding="utf-8")


def test_d1_a_b_share_one_training_protocol_and_g7_only():
    assert '"D1-A"' in SOURCE and '"D1-B"' in SOURCE
    assert "require_ddrv4_selection_inputs([G7_DATASET_ID])" in SOURCE
    assert 'cfg.train_dataloader.dataset.ann_file = str(args.prepared / "fit.json")' in SOURCE
    assert 'cfg.val_dataloader.dataset.ann_file = str(args.prepared / "holdout.json")' in SOURCE
    assert "val.json" not in SOURCE


def test_d1_training_disables_mosaic_mixup_and_style_augmentation():
    assert 'safe_train_pipeline' in SOURCE
    assert 'Mosaic' not in SOURCE and 'MixUp' not in SOURCE and 'PhotoMetricDistortion' not in SOURCE
    assert '"arbitrary_stylization": False' in SOURCE
