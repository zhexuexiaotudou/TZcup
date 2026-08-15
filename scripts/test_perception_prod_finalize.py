from perception_prod_finalize import build_status, release_manifest


def test_final_status_is_fail_closed_after_three_routes():
    payload = build_status("a" * 40)
    assert payload["model_route_status"]["routes_exhausted"] is True
    assert payload["model_route_status"]["MODEL_BLOCKED_INTERNAL"] is True
    assert payload["sealed_data"]["G5_SEALED_FINAL_read"] is False
    assert payload["sealed_data"]["freeze_performed"] is False
    assert all(value is False for value in payload["statuses"].values())
    assert payload["product_deployment_complete"] is False


def test_blocked_release_manifest_is_not_a_deploy_claim():
    payload = release_manifest("b" * 40)
    assert payload["release_ready"] is False
    assert payload["selected_model"] is None
    assert payload["container_digest"] is None
    assert payload["production_verification_run"] is False
