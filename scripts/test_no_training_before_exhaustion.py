from emfj6v3_contract import training_allowed


def test_training_is_blocked_before_screening_and_nontraining_adjustment():
    base = {
        "EMF_EXISTING_MODEL_SCREENING_COMPLETE": False,
        "EMF_NONTRAINING_ADJUSTMENT_COMPLETE": False,
        "EMF_TRANSFER_LEARNING_REQUIRED": False,
        "sealed_access_allowed": False,
    }
    assert training_allowed(base) is False
    for field in (
        "EMF_EXISTING_MODEL_SCREENING_COMPLETE",
        "EMF_NONTRAINING_ADJUSTMENT_COMPLETE",
        "EMF_TRANSFER_LEARNING_REQUIRED",
    ):
        state = dict(base)
        state[field] = True
        assert training_allowed(state) is False


def test_training_requires_all_three_positive_gates_and_sealed_denial():
    ready = {
        "EMF_EXISTING_MODEL_SCREENING_COMPLETE": True,
        "EMF_NONTRAINING_ADJUSTMENT_COMPLETE": True,
        "EMF_TRANSFER_LEARNING_REQUIRED": True,
        "sealed_access_allowed": False,
    }
    assert training_allowed(ready) is True
    assert training_allowed({**ready, "sealed_access_allowed": True}) is False
