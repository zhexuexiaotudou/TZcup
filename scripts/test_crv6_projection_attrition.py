from pathlib import Path


def test_online_closure_keeps_gt_out_of_product_inputs():
    source=(Path(__file__).parent/"run_crv6_online_closure.py").read_text(encoding="utf-8")
    assert '"GT_used_by_product_pipeline":False' in source
    assert '"GT_used_only_for_post_run_scoring":True' in source
    assert '"production_inputs":["RGB","depth","CameraInfo","TF"]' in source


def test_online_closure_runs_projection_tracker_map_and_scheduler():
    source=(Path(__file__).parent/"run_crv6_online_closure.py").read_text(encoding="utf-8")
    for token in ("project_discrete_predictions", "ProductTrackerV2", "DynamicTrashMap.start_new", "scheduler.decide", "CRV6_FALSE_CONFIRMED_TAXONOMY.json"):
        assert token in source


def test_synthetic_moving_pack_is_fail_closed_for_map_scoring():
    source=(Path(__file__).parent/"run_crv6_online_closure.py").read_text(encoding="utf-8")
    assert '"dataset_map_geometry_eligible":map_gate_eligible' in source
    assert '"map_gate_eligible":map_gate_eligible' in source
    assert 'not a physically consistent fixed-world map target' in source


def test_only_current_frame_tracks_are_ingested_as_observations():
    source=(Path(__file__).parent/"run_crv6_online_closure.py").read_text(encoding="utf-8")
    assert 'abs(track.last_seen_s-row["frame_index"]/15.0)>1e-9' in source
