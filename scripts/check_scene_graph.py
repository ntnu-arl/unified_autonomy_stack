#!/usr/bin/env python3
"""Check Hydra's native binary stream and the task agent's direct-receiver status."""

import argparse
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from spark_dsg import SemanticNodeAttributes
from std_msgs.msg import String

from agentic_uas.scene_graph import SceneGraphReceiver, SceneGraphReceiverConfig


def main() -> None:
    """Verify native object attributes and matching live agent node/edge counts.

    :return: None; raise AssertionError if the integration does not pass in time.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=120.)
    parser.add_argument('--endpoint', default='tcp://127.0.0.1:8002')
    args = parser.parse_args()
    rclpy.init()
    node = Node('scene_graph_integration_check')
    receiver = SceneGraphReceiver(SceneGraphReceiverConfig(endpoint=args.endpoint))
    statuses = []
    node.create_subscription(
        String, '/agentic_uas/status', lambda msg: statuses.append(json.loads(msg.data)),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            receiver.poll()
            rclpy.spin_once(node, timeout_sec=.1)
            graph = receiver.graph
            if graph is None or not statuses:
                continue
            nodes = list(graph.nodes)
            objects = [n for n in nodes if isinstance(n.attributes, SemanticNodeAttributes)]
            status = statuses[-1]
            if (not objects or status.get('scene_graph_nodes') != graph.num_nodes()
                    or status.get('scene_graph_edges') != graph.num_edges()
                    or not status.get('scene_graph_sequence')):
                continue
            ids = {n.id.value for n in nodes}
            assert len(ids) == graph.num_nodes()
            positions = np.asarray([n.attributes.position for n in nodes])
            assert positions.shape == (len(ids), 3) and np.isfinite(positions).all()
            assert all(e.source in ids and e.target in ids for e in graph.edges)
            assert all(n.attributes.bounding_box.is_valid() for n in objects)
            assert not graph.has_mesh()
            print(json.dumps({
                'result': 'PASS', 'nodes': graph.num_nodes(), 'objects': len(objects),
                'edges': graph.num_edges(),
                'label_ids': sorted({int(n.attributes.semantic_label) for n in objects}),
                'agent_state': status['state'],
                'agent_received_updates': status['scene_graph_sequence'],
            }, indent=2))
            return
        raise AssertionError('No native object graph matching the agent status')
    finally:
        receiver.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
