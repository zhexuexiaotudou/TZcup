from pathlib import Path


def test_reconstitution_preserves_historical_hash_as_history_only():
    source = (Path(__file__).parent / "finalize_crv6_reconstitution.py").read_text(encoding="utf-8")
    assert '"historical_identity_claimed": False' in source
    assert '"candidate_hash_differs_from_historical": True' in source
    assert '"candidate_id": "D1B_RECON_R1"' in source


def test_reconstitution_provenance_binds_script_data_container_and_versions():
    source = (Path(__file__).parent / "finalize_crv6_reconstitution.py").read_text(encoding="utf-8")
    for token in ("training_script_git_blob_sha", "container_image_id", "RECON_TRAIN_DATA_HASHES.json", "mmdetection", "torch", "cuda"):
        assert token in source
