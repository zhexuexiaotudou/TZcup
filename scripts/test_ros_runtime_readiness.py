from ros_runtime_readiness import (
    REQUIRED_SERVICES,
    REQUIRED_TOPICS,
    readiness_decision,
)


def test_readiness_requires_complete_graph_lifecycle_and_dashboard():
    assert readiness_decision(
        set(REQUIRED_TOPICS), set(REQUIRED_SERVICES), 3, 3, True
    )
    assert not readiness_decision(
        set(REQUIRED_TOPICS) - {"/scan"}, set(REQUIRED_SERVICES), 3, 3, True
    )
    assert not readiness_decision(
        set(REQUIRED_TOPICS), set(REQUIRED_SERVICES), 2, 3, True
    )
    assert not readiness_decision(
        set(REQUIRED_TOPICS), set(REQUIRED_SERVICES), 3, 3, False
    )
