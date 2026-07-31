from human_visualization_gate import audit


def test_human_visualization_software_contract_is_complete_but_does_not_fake_live_ready():
    report = audit()
    assert report["software_contract_pass"] is True
    assert report["human_visualization_ready"] is False
    assert report["blockers"] == ["live_runtime_not_checked"]
