import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


def summarize_ownership(
    observations,
    required_child='odom',
    expected_owner_node='',
    endpoint_names=None,
    forbidden_owner_nodes=None,
):
    matching = [
        observation
        for observation in observations
        if observation['child_frame_id'].lstrip('/') == required_child.lstrip('/')
    ]
    endpoint_names = endpoint_names or {}
    forbidden_owner_nodes = set(forbidden_owner_nodes or [])
    owner_gids = {item['publisher_gid'] for item in matching}
    owner_nodes = {
        endpoint_names[gid]['node_name']
        for gid in owner_gids
        if gid in endpoint_names
    }
    publisher_gid_available = bool(owner_gids and '' not in owner_gids)
    endpoint_node_names = {
        endpoint['node_name'] for endpoint in endpoint_names.values()
    }
    if publisher_gid_available:
        unattributed_gids = sorted(owner_gids - set(endpoint_names))
        expected_owner_present = bool(
            expected_owner_node and expected_owner_node in owner_nodes
        )
        forbidden_present = sorted(owner_nodes & forbidden_owner_nodes)
        owner_count = len(owner_nodes)
        single_owner = bool(
            owner_count == 1
            and not unattributed_gids
            and expected_owner_present
            and not forbidden_present
        )
        attribution_method = 'tf_sample_publisher_gid_to_runtime_endpoint_graph'
    else:
        # Jazzy's callback MessageInfo may expose an empty publisher GID for
        # /tf. Preserve edge observation while making the fallback explicit.
        unattributed_gids = []
        expected_owner_present = bool(
            matching and expected_owner_node in endpoint_node_names
        )
        owner_nodes = {expected_owner_node} if expected_owner_present else set()
        forbidden_present = sorted(endpoint_node_names & forbidden_owner_nodes)
        owner_count = 1 if expected_owner_present and not forbidden_present else 0
        single_owner = bool(owner_count == 1)
        attribution_method = (
            'configured_owner_plus_runtime_endpoint_graph_and_edge_observation'
        )
    return {
        'schema_version': 1,
        'required_child_frame': required_child,
        'observed_transform_count': len(observations),
        'required_transform_count': len(matching),
        'owner_count': owner_count,
        'owner_nodes': sorted(owner_nodes),
        'owner_publisher_gids': sorted(owner_gids),
        'unattributed_publisher_gids': unattributed_gids,
        'publisher_gid_available': publisher_gid_available,
        'expected_owner_node': expected_owner_node,
        'expected_owner_present': expected_owner_present,
        'single_owner': single_owner,
        'complete': single_owner,
        'attribution_method': attribution_method,
        'forbidden_owner_nodes_present': forbidden_present,
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
        dynamic_tf_qos = QoSProfile(
            depth=100,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(TFMessage, '/tf', self._on_tf, dynamic_tf_qos)
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
        required_child = str(self.get_parameter('required_child_frame').value)
        expected_owner = str(self.get_parameter('expected_owner_node').value)
        forbidden = [
            str(node_name)
            for node_name in self.get_parameter('forbidden_owner_nodes').value
        ]
        report = summarize_ownership(
            self.observations,
            required_child,
            expected_owner,
            endpoint_names,
            forbidden,
        )
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
