from __future__ import annotations

import copy
import json
from pathlib import Path

import pytest
import yaml

from validate_pre_urdf_readiness import (
    ContractError,
    load_contract,
    validate_budget_csvs,
    validate_contract,
)


ROOT = Path(__file__).resolve().parents[1]


def test_committed_contract_and_report_match() -> None:
    result = validate_contract(load_contract())
    validate_budget_csvs(result)
    report = json.loads(
        (ROOT / "reports" / "engineering" / "pre_urdf_readiness.json").read_text(encoding="utf-8")
    )
    assert result == report
    assert result["status"] == "READY_FOR_URDF_IMPLEMENTATION_WITH_LAYOUT_GATES"
    assert result["formal_urdf_created"] is False
    assert result["preliminary_wastewater_usable_l"] > 0


def test_repository_locks_match_contract_and_global_registry() -> None:
    contract = load_contract()
    contract_repos = {
        item["id"]: {"url": item["url"], "commit": item["commit"]}
        for item in contract["source_repositories"]
    }
    vcs = yaml.safe_load(
        (ROOT / "repos" / "high_fidelity_vehicle.repos").read_text(encoding="utf-8")
    )["repositories"]
    locked = json.loads((ROOT / "repos" / "locked_revisions.json").read_text(encoding="utf-8"))[
        "repositories"
    ]
    assert set(vcs) == set(contract_repos)
    for repo_id, expected in contract_repos.items():
        assert vcs[repo_id]["url"] == expected["url"]
        assert vcs[repo_id]["version"] == expected["commit"]
        assert locked[repo_id]["url"] == expected["url"]
        assert locked[repo_id]["commit"] == expected["commit"]


def test_rejects_unpinned_repository() -> None:
    contract = copy.deepcopy(load_contract())
    contract["source_repositories"][0]["commit"] = "jazzy"
    with pytest.raises(ContractError, match="not pinned"):
        validate_contract(contract)


def test_rejects_missing_required_component_role() -> None:
    contract = copy.deepcopy(load_contract())
    contract["component_selections"] = [
        item for item in contract["component_selections"] if item["role"] != "rear_right_fisheye"
    ]
    with pytest.raises(ContractError, match="missing component roles"):
        validate_contract(contract)


def test_rejects_duplicate_sensor_topic() -> None:
    contract = copy.deepcopy(load_contract())
    contract["sensor_contracts"][1]["topic"] = contract["sensor_contracts"][0]["topic"]
    with pytest.raises(ContractError, match="sensor topics must be unique"):
        validate_contract(contract)


def test_rejects_dry_bin_below_competition_requirement() -> None:
    contract = copy.deepcopy(load_contract())
    contract["mass_capacity_budget"]["dry_bin"]["usable_volume_l"] = 39.9
    with pytest.raises(ContractError, match="at least 40 L"):
        validate_contract(contract)


def test_rejects_payload_without_water_capacity() -> None:
    contract = copy.deepcopy(load_contract())
    contract["mass_capacity_budget"]["engineering_allowances"]["split_bin_structure"] = 30.0
    with pytest.raises(ContractError, match="leaves no wastewater capacity"):
        validate_contract(contract)


def test_rejects_sensor_rail_overload() -> None:
    contract = copy.deepcopy(load_contract())
    contract["power_budget"]["loads"][-1]["rail"] = "sensor_12v"
    contract["power_budget"]["loads"][-1]["peak_w"] = 100.0
    with pytest.raises(ContractError, match="12 V sensor rail peak budget exceeded"):
        validate_contract(contract)


def test_rejects_dry_throughput_below_competition_target() -> None:
    contract = copy.deepcopy(load_contract())
    contract["throughput_budget"]["dry_cleaning"]["minimum_route_efficiency"] = 0.5
    with pytest.raises(ContractError, match="throughput does not meet"):
        validate_contract(contract)
