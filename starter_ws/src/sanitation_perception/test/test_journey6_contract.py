import pytest

from sanitation_perception.journey6_contract import Journey6Target


def test_journey6_target_stays_auto_until_official_inventory():
    target = Journey6Target()
    assert target.target_sku == "auto"
    assert target.target_march == "auto"
    with pytest.raises(ValueError, match="resolved Journey 6 SKU"):
        target.resolve(
            {
                "target_family": "journey6",
                "target_sku": "auto",
                "target_march": "nash-e",
                "fact_source": "official_j6_sdk",
            }
        )


def test_journey6_target_resolves_from_board_facts_only():
    target = Journey6Target().resolve(
        {
            "target_family": "journey6",
            "target_sku": "journey6e",
            "target_march": "nash-e",
            "fact_source": "board_inventory",
        }
    )
    assert target.profile == "journey6_nash_e"
    with pytest.raises(ValueError, match="board or official"):
        Journey6Target().resolve(
            {
                "target_family": "journey6",
                "target_sku": "journey6e",
                "target_march": "nash-e",
                "fact_source": "model_name_guess",
            }
        )


def test_journey6_target_rejects_s100_and_profile_mismatch():
    with pytest.raises(ValueError, match="RDK/J5"):
        Journey6Target(target_family="rdk_s100").validate()
    with pytest.raises(ValueError, match="do not match"):
        Journey6Target(target_march="nash-e", profile="journey6_nash_m").validate()
