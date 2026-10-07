#!/usr/bin/env python3
"""Verify real target-reach heading execution in a running Unipilot simulation."""

import argparse
import json
import math
import time
from pathlib import Path
from typing import Callable

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time


def yaw(q) -> float:
    """Extract planar heading from a ROS quaternion.

    :param q: Quaternion message.
    :return: Heading in radians.
    """
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def angle_error(a: float, b: float) -> float:
    """Compute shortest unsigned heading error.

    :param a: First heading in radians.
    :param b: Second heading in radians.
    :return: Absolute wrapped difference in radians.
    """
    return abs(math.atan2(math.sin(a-b), math.cos(a-b)))


class TargetReachCheck(Node):
    """Exercise goals through the existing ROS bridge and measure odometry."""

    def __init__(self, timeout: float) -> None:
        super().__init__('target_reach_yaw_check')
        self.timeout = timeout
        self.odom = None
        self.path = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(Odometry, '/rmf_unipilot/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(NavPath, '/gbplanner_path', self.on_path, 10)
        self.publisher = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        self.stop = self.create_client(Trigger, '/planner_control_interface/std_srvs/stop')
        self.start = self.create_client(Trigger, '/planner_control_interface/std_srvs/automatic_planning')
        self.mode = self.create_client(SetBool, '/gbplanner/switch_operation_mode')

    def on_odom(self, msg: Odometry) -> None:
        """Retain measured robot state.

        :param msg: Latest odometry.
        :return: None.
        """
        self.odom = msg

    def on_path(self, msg: NavPath) -> None:
        """Retain the executable planner path.

        :param msg: Bridged planner path.
        :return: None.
        """
        if msg.poses:
            self.path = msg

    def wait(self, predicate: Callable[[], bool], description: str) -> None:
        """Spin until evidence arrives or raise a bounded timeout.

        :param predicate: Required completion condition.
        :param description: Condition to identify in a failure.
        :return: None.
        """
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.05)
            if predicate():
                return
        raise TimeoutError(description)

    def call(self, client, request) -> None:
        """Call a service and check its application result.

        :param client: ROS client.
        :param request: Request message.
        :return: None.
        """
        print(f"Calling {client.srv_name}", flush=True)
        self.wait(client.service_is_ready, f'service {client.srv_name} unavailable')
        ready_at = time.monotonic() + .5
        self.wait(lambda: time.monotonic() >= ready_at, 'service discovery delay')
        future = client.call_async(request)
        self.wait(future.done, f'service {client.srv_name} timed out')
        result = future.result()
        print(f"Service result: {result}", flush=True)
        if not result.success:
            raise RuntimeError(f'{client.srv_name}: {result.message}')

    def position(self) -> tuple[float, float, float]:
        """Read robot position in the planner world frame.

        :return: Global xyz position.
        """
        transform = self.tf.lookup_transform('map', self.odom.header.frame_id, Time())
        # This check uses the Unipilot identity map->odom profile.
        t, q = transform.transform.translation, transform.transform.rotation
        if max(abs(t.x), abs(t.y), abs(t.z), abs(q.x), abs(q.y), abs(q.z)) > 1e-6:
            raise ValueError('Check requires the Unipilot identity map->odom transform')
        p = self.odom.pose.pose.position
        return p.x, p.y, p.z

    def goal(self, xyz: tuple[float, float, float], heading: float, label: str) -> dict:
        """Send stop/goal/start and require actual position and yaw convergence.

        :param xyz: Target in map coordinates.
        :param heading: Desired global yaw in radians.
        :param label: Test case label.
        :return: Measured execution evidence.
        """
        self.call(self.stop, Trigger.Request())
        self.call(self.mode, SetBool.Request(data=True))
        self.wait(lambda: self.publisher.get_subscription_count() > 0, 'goal bridge unavailable')
        ready_at = time.monotonic() + 1.0
        self.wait(lambda: time.monotonic() >= ready_at, 'DDS goal discovery delay')
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = xyz
        msg.pose.orientation.z = math.sin(heading/2)
        msg.pose.orientation.w = math.cos(heading/2)
        self.path = None
        self.publisher.publish(msg)
        until = time.monotonic() + .5
        self.wait(lambda: time.monotonic() >= until, 'goal delivery delay')
        self.call(self.start, Trigger.Request())
        started = time.monotonic()
        self.wait(lambda: self.path is not None, f'{label}: no executable path')
        def terminal_path() -> bool:
            if self.path is None:
                return False
            terminal = self.path.poses[-1].pose
            p = terminal.position
            return (math.dist((p.x, p.y, p.z), xyz) <= 2.0 and
                    angle_error(yaw(terminal.orientation), heading) <= .01)
        self.wait(terminal_path, f'{label}: no goal-reaching path with requested terminal yaw')
        self.wait(lambda: math.dist(self.position(), xyz) < 2.0 and
                  angle_error(yaw(self.odom.pose.pose.orientation), heading) < .2,
                  f'{label}: robot failed position/yaw convergence')
        evidence = {'case': label, 'target': xyz, 'target_yaw': heading,
                    'position': self.position(), 'measured_yaw': yaw(self.odom.pose.pose.orientation),
                    'yaw_error': angle_error(yaw(self.odom.pose.pose.orientation), heading),
                    'path_points': len(self.path.poses), 'wall_seconds': time.monotonic()-started}
        print(json.dumps(evidence), flush=True)
        return evidence


def main() -> None:
    """Run stationary, wrap-boundary and translated heading tests.

    :return: None; raise on failed execution.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=120)
    parser.add_argument('--takeoff', action='store_true', help='Initialize simulation flight before checking goals')
    parser.add_argument('--translation', type=float, default=4, help='Final target offset along global x in metres; 0 skips')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = TargetReachCheck(args.timeout)
    evidence = []
    try:
        print("Waiting for odometry", flush=True)
        node.wait(lambda: node.odom is not None, 'odometry unavailable')
        node.wait(lambda: node.tf.can_transform('map', node.odom.header.frame_id, Time()),
                  'map->odometry transform unavailable')
        if args.takeoff:
            constraints = node.create_client(SetBool, '/sdf_nmpc/set_flag')
            node.call(constraints, SetBool.Request(data=True))
            takeoff = node.create_client(Trigger, '/sdf_nmpc/takeoff')
            node.call(takeoff, Trigger.Request())
            node.wait(lambda: node.position()[2] > 1.7 and abs(node.odom.twist.twist.linear.z) < .1,
                      'takeoff not settled')
        origin = node.position()
        evidence.append(node.goal(origin, math.pi/2, 'in_place'))
        evidence.append(node.goal(origin, 3.05, 'positive_wrap'))
        evidence.append(node.goal(origin, -3.05, 'negative_wrap'))
        if args.translation:
            target = (origin[0]+args.translation, origin[1], origin[2])
            evidence.append(node.goal(target, 0., 'translated'))
        if args.output:
            args.output.write_text(json.dumps(evidence, indent=2))
    finally:
        if rclpy.ok() and node.stop.service_is_ready():
            node.call(node.stop, Trigger.Request())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
