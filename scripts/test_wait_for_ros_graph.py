from pathlib import Path

from wait_for_ros_graph import graph_contract_status


ROOT = Path(__file__).resolve().parents[1]


def test_graph_contract_requires_exact_topic_types_and_fully_qualified_nodes():
    status = graph_contract_status(
        {
            "/map": ["nav_msgs/msg/OccupancyGrid"],
            "/depth": ["sensor_msgs/msg/Image"],
        },
        [("pc_open_vocab_product_adapter", "/")],
        [
            ("/map", "nav_msgs/msg/OccupancyGrid"),
            ("/depth", "sensor_msgs/msg/Image"),
        ],
        ["/pc_open_vocab_product_adapter"],
    )
    assert status["ready"] is True
    assert status["missing_topics"] == []
    assert status["missing_nodes"] == []

    wrong_type = graph_contract_status(
        {"/map": ["std_msgs/msg/String"]},
        [],
        [("/map", "nav_msgs/msg/OccupancyGrid")],
        ["/pc_open_vocab_product_adapter"],
    )
    assert wrong_type["ready"] is False
    assert wrong_type["missing_topics"] == [
        {"name": "/map", "type": "nav_msgs/msg/OccupancyGrid"}
    ]
    assert wrong_type["missing_nodes"] == ["/pc_open_vocab_product_adapter"]


def test_perception_runner_uses_bounded_single_process_graph_probes():
    source = (ROOT / "scripts/run_formal_random_scene_perception.sh").read_text(
        encoding="utf-8"
    )
    assert "wait_for_ros_graph.py" in source
    assert "campus_graph_readiness.json" in source
    assert "product_graph_readiness.json" in source
    assert "ros2 topic list" not in source
    assert "ros2 node list" not in source
