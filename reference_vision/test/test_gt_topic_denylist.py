import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_production_subscription_topics_do_not_include_ground_truth_prefixes():
    production_files = (
        ROOT / "starter_ws/src/sanitation_perception/sanitation_perception/product_pipeline_node.py",
        ROOT / "starter_ws/src/sanitation_spot_cleaning/sanitation_spot_cleaning/node.py",
    )
    topics = []
    for path in production_files:
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
                continue
            if node.func.attr not in {"create_subscription", "Subscriber"}:
                continue
            for argument in node.args:
                if isinstance(argument, ast.Constant) and isinstance(argument.value, str) and argument.value.startswith("/"):
                    topics.append(argument.value)
    assert topics
    assert not any(topic.startswith(("/ground_truth/", "/gazebo/ground_truth/")) for topic in topics)
