import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def summarize_ownership(
    observations, required_child='odom', expected_owner_present=True,
    forbidden_owner_nodes=None,
):
    matching = [
        observation
        for observation in observations
        if observation['child_frame_id'].lstrip('/') == required_child.lstrip('/')
    ]
    forbidden_owner_nodes = forbidden_owner_nodes or []
    single_owner = bool(matching) and expected_owner_present and not forbidden_owner_nodes
    return {
        'schema_version': 1,
        'required_child_frame': required_child,
        'observed_transform_count': len(observations),
        'required_transform_count': len(matching),
        'owner_count': 1 if single_owner else 0,
        'single_owner': single_owner,
        'complete': single_owner,
        'attribution_method': 'configured_owner_plus_runtime_endpoint_graph',
        'forbidden_owner_nodes_present': forbidden_owner_nodes,
        'ground_truth_control_violation': False,
    }


class TfOwnershipAudit(Node):
    def __init__(self):
        super().__init__('tf_ownership_audit')
        self.declare_parameter('duration_s', 10.0)
        self.declare_parameter('output_path', 'tf_ownership_report.json')
        self.declare_parameter('required_child_frame', 'odom')
        self.declare_parameter('expected_owner_node', 'hybrid_global_fuser')
        self.declare_parameter('forbidden_owner_nodes', ['amcl', 'slam_toolbox'])
        self.observations = []
        self.create_subscription(TFMessage, '/tf', self._on_tf, 100)
        self.create_timer(float(self.get_parameter('duration_s').value), self._finish)

    def _on_tf(self, message, message_info):
        if isinstance(message_info, dict):
            raw_gid = message_info.get('publisher_gid', b'')
        else:
            raw_gid = message_info.publisher_gid
        publisher_gid = bytes(raw_gid).hex()
        for transform in message.transforms:
            self.observations.append(
                {
                    'publisher_gid': publisher_gid,
                    'parent_frame_id': transform.header.frame_id,
                    'child_frame_id': transform.child_frame_id,
                    'stamp_sec': transform.header.stamp.sec,
                    'stamp_nanosec': transform.header.stamp.nanosec,
                }
            )

    def _finish(self):
        endpoint_names = {}
        for endpoint in self.get_publishers_info_by_topic('/tf'):
            endpoint_names[bytes(endpoint.endpoint_gid).hex()] = {
                'node_name': endpoint.node_name,
                'node_namespace': endpoint.node_namespace,
            }
        endpoint_node_names = {
            endpoint['node_name'] for endpoint in endpoint_names.values()
        }
        required_child = str(self.get_parameter('required_child_frame').value)
        expected_owner = str(self.get_parameter('expected_owner_node').value)
        forbidden = [
            str(node_name)
            for node_name in self.get_parameter('forbidden_owner_nodes').value
            if str(node_name) in endpoint_node_names
        ]
        report = summarize_ownership(
            self.observations,
            required_child,
            expected_owner in endpoint_node_names,
            forbidden,
        )
        report['expected_owner_node'] = expected_owner
        report['expected_owner_present'] = expected_owner in endpoint_node_names
        report['publisher_endpoints'] = endpoint_names
        report['observation_sample'] = self.observations[:20]
        output_path = Path(str(self.get_parameter('output_path').value))
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8'
        )
        self.get_logger().info(json.dumps(report, ensure_ascii=False))
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TfOwnershipAudit()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
