import evaluate_trcrv10_action_verifier as evaluator


def row(encounter, truth_kind, truth_class, predicted, decision, small=False):
    return {"encounter_id": encounter, "truth_kind": truth_kind, "truth_class": truth_class,
            "predicted_class": predicted, "decision": decision, "small_at_first_proposal": small}


def test_perfect_gate_passes() -> None:
    rows = [
        row("a", "target", "metal_can", "metal_can", "ACCEPT", True),
        row("b", "target", "paper_litter", "paper_litter", "ACCEPT"),
        row("n", "negative", None, None, "VETO"),
    ]
    assert evaluator.metrics(rows)["pass"]


def test_one_false_clean_always_fails() -> None:
    rows = [
        row("a", "target", "metal_can", "metal_can", "ACCEPT", True),
        row("n", "negative", None, "paper_litter", "ACCEPT"),
    ]
    result = evaluator.metrics(rows)
    assert result["metrics"]["false_CLEAN_NOW"] == 1
    assert not result["gates"]["false_clean_zero"]
    assert not result["pass"]
