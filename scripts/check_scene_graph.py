#!/usr/bin/env python3
"""Check received object graphs and the task agent's placeholder cache in Humble."""

import argparse
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


def main() -> None:
    """Wait for live object snapshots and matching agent status.

    :return: None; raise AssertionError if integration does not work within the timeout.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=120.0)
    args = parser.parse_args()
    rclpy.init()
    node = Node("scene_graph_integration_check")
    snapshots, statuses = [], []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, "/agentic_uas/scene_graph",
                             lambda msg: snapshots.append(json.loads(msg.data)), qos)
    node.create_subscription(String, "/agentic_uas/status",
                             lambda msg: statuses.append(json.loads(msg.data)), qos)
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not snapshots or not statuses:
                continue
            graph, status = snapshots[-1], statuses[-1]
            objects = [item for item in graph["nodes"] if "label_id" in item]
            if (not objects or status.get("scene_graph_sequence") != graph["sequence"]
                    or status.get("scene_graph_nodes") != len(graph["nodes"])):
                continue
            assert graph["schema"] == "agentic_uas.scene_graph" and graph["version"] == 1
            assert graph["frame_id"] == "map" and graph["stamp_ns"] > 0
            ids = {item["id"] for item in graph["nodes"]}
            assert len(ids) == len(graph["nodes"])
            assert all(isinstance(item["id"], str) for item in graph["nodes"])
            positions = np.array([item["position"] for item in graph["nodes"]])
            assert positions.shape == (len(ids), 3) and np.isfinite(positions).all()
            assert all(edge["source"] in ids and edge["target"] in ids for edge in graph["edges"])
            assert all("bounding_box" in item for item in objects)
            print(json.dumps({
                "result": "PASS", "session": graph["session"], "sequence": graph["sequence"],
                "nodes": len(ids), "objects": len(objects), "edges": len(graph["edges"]),
                "labels": sorted({item["label"] for item in objects}),
                "agent_state": status["state"],
                "agent_scene_graph_sequence": status["scene_graph_sequence"],
            }, indent=2))
            return
        raise AssertionError(f"No matching object graph/agent status: "
                             f"{len(snapshots)} snapshots, {len(statuses)} statuses")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
