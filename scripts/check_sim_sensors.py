#!/usr/bin/env python3
"""Capture and validate real RGB, metric depth, LiDAR, calibration and camera TF."""

import argparse
import json
import time
from collections import OrderedDict
from pathlib import Path

import numpy as np
import rclpy
from cv_bridge import CvBridge
from PIL import Image as PilImage
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


def stamp_ns(message) -> int:
    """Return the capture timestamp in nanoseconds.

    :param message: Message with a header.
    :return: Integer ROS timestamp.
    """
    stamp = message.header.stamp
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class SensorCheck(Node):
    """Collect bounded sensor evidence without altering simulation state."""

    def __init__(self, output: Path, frames: int, expected_rate: float) -> None:
        super().__init__("simulation_sensor_check")
        self.set_parameters([Parameter("use_sim_time", value=True)])
        self.output = output
        self.frames = frames
        self.expected_rate = expected_rate
        self.bridge = CvBridge()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.images = {"rgb": OrderedDict(), "depth": OrderedDict()}
        self.stamps = {name: [] for name in ("rgb", "depth", "lidar")}
        self.counts = dict.fromkeys(self.stamps, 0)
        self.infos = {}
        self.cloud = None
        self.clock_count = 0
        self.pair = None
        self.camera_transform = None
        self.create_subscription(Image, "/rmf_unipilot/cam/rgb", lambda msg: self.image("rgb", msg), qos_profile_sensor_data)
        self.create_subscription(Image, "/rmf_unipilot/cam/depth", lambda msg: self.image("depth", msg), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/rmf_unipilot/cam/rgb/camera_info", lambda msg: self.infos.update(rgb=msg), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/rmf_unipilot/cam/camera_info", lambda msg: self.infos.update(depth=msg), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/rmf_unipilot/lidar/points", self.lidar, qos_profile_sensor_data)
        self.create_subscription(Clock, "/clock", self.clock, qos_profile_sensor_data)

    def image(self, name: str, message: Image) -> None:
        """Keep recent images for exact timestamp pairing.

        :param name: RGB or depth stream.
        :param message: Sensor image.
        """
        stamp = stamp_ns(message)
        self.counts[name] += 1
        self.stamps[name].append(stamp)
        self.images[name][stamp] = message
        if len(self.images[name]) > 100:
            self.images[name].popitem(last=False)

    def lidar(self, message: PointCloud2) -> None:
        """Retain a point cloud and count received scans.

        :param message: Sensor point cloud.
        """
        self.counts["lidar"] += 1
        self.stamps["lidar"].append(stamp_ns(message))
        self.cloud = message

    def clock(self, message: Clock) -> None:
        """Count simulation clock updates.

        :param message: Simulation clock.
        """
        self.clock_count += 1

    def ready(self) -> bool:
        """Find an exactly paired RGB-D image with timestamped camera TF.

        :return: Whether all requested streams have sufficient evidence.
        """
        if min(self.counts.values()) < self.frames or len(self.infos) != 2:
            return False
        common = self.images["rgb"].keys() & self.images["depth"].keys()
        for stamp in sorted(common, reverse=True):
            rgb = self.images["rgb"][stamp]
            try:
                transform = self.tf.lookup_transform("map", rgb.header.frame_id, Time.from_msg(rgb.header.stamp))
            except TransformException:
                continue
            self.pair = (rgb, self.images["depth"][stamp])
            self.camera_transform = transform
            return True
        return False

    def save(self, timed_out: bool) -> bool:
        """Save captures and assess actual sensor content.

        :param timed_out: Whether the acquisition deadline elapsed.
        :return: Whether all content checks passed.
        """
        self.output.mkdir(parents=True, exist_ok=True)
        report = {"passed": False, "timed_out": timed_out, "required_frames": self.frames, "expected_camera_rate_hz": self.expected_rate, "clock_messages": self.clock_count, "streams": {}, "checks": {}}
        for name, stamps in self.stamps.items():
            times = np.unique(stamps)
            rate = float((len(times) - 1) * 1e9 / (times[-1] - times[0])) if len(times) > 1 else None
            report["streams"][name] = {"messages": self.counts[name], "unique_stamps": len(times), "rate_sim_hz": rate}
        checks = report["checks"]
        checks["sufficient_frames"] = min(self.counts.values()) >= self.frames
        checks["simulation_clock"] = self.clock_count > 0
        checks["exact_rgb_depth_pair_and_camera_tf"] = self.pair is not None
        if self.pair:
            rgb_msg, depth_msg = self.pair
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "rgb8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float32)
            if depth_msg.encoding == "16UC1":
                depth *= 0.001
            checks["metric_depth_encoding"] = depth_msg.encoding in ("32FC1", "16UC1")
            checks["rgb_depth_dimensions_match"] = rgb.shape[:2] == depth.shape
            checks["rgb_nonblank"] = float(np.std(rgb)) > 2 and float(np.mean(rgb)) > 1
            checks["rgb_spatial_texture"] = float(np.std(rgb.astype(float).mean(axis=2))) > 2
            valid = np.isfinite(depth) & (depth > 0.05) & (depth < 100)
            checks["depth_scene_returns"] = int(valid.sum()) > max(100, int(depth.size * 0.01))
            checks["depth_nonconstant"] = bool(valid.any() and float(np.ptp(depth[valid])) > 0.05)
            for name, msg, array in (("rgb", rgb_msg, rgb), ("depth", depth_msg, depth)):
                info = self.infos[name]
                checks[f"{name}_calibration"] = bool(info.width == msg.width and info.height == msg.height and info.k[0] > 0 and info.k[4] > 0 and np.isfinite(info.k).all())
                report["streams"][name].update(shape=list(array.shape), encoding=msg.encoding, frame=msg.header.frame_id, saved_stamp_ns=stamp_ns(msg), intrinsics=list(info.k), distortion=list(info.d))
            report["streams"]["rgb"].update(mean=float(rgb.mean()), std=float(rgb.std()))
            report["streams"]["depth"].update(valid_fraction=float(valid.mean()), range_m=[float(depth[valid].min()), float(depth[valid].max())] if valid.any() else None)
            PilImage.fromarray(rgb).save(self.output / "rgb.png")
            np.save(self.output / "depth_m.npy", depth)
            preview = np.zeros(depth.shape, dtype=np.uint8)
            if valid.any():
                scale = max(float(np.percentile(depth[valid], 95)), 0.1)
                preview[valid] = np.clip(depth[valid] / scale * 255, 1, 255).astype(np.uint8)
            PilImage.fromarray(preview).save(self.output / "depth_preview.png")
            t = self.camera_transform.transform
            report["camera_tf"] = {"parent": "map", "child": rgb_msg.header.frame_id, "translation": [t.translation.x, t.translation.y, t.translation.z], "quaternion_xyzw": [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w], "requested_stamp_ns": stamp_ns(rgb_msg)}
        if self.cloud is not None:
            points = point_cloud2.read_points(self.cloud, field_names=("x", "y", "z"), skip_nans=False)
            if not isinstance(points, np.ndarray):
                points = np.asarray(list(points), dtype=np.float32)
            if points.dtype.names:
                xyz = np.column_stack([points[name].ravel() for name in ("x", "y", "z")]).astype(np.float32)
            else:
                xyz = np.asarray(points, dtype=np.float32).reshape(-1, 3)
            radii = np.linalg.norm(xyz, axis=1)
            valid = np.isfinite(xyz).all(axis=1) & (radii > 0.05) & (radii < 100)
            checks["lidar_scene_returns"] = int(valid.sum()) >= 100
            checks["lidar_spatial_extent"] = bool(valid.any() and np.max(np.ptp(xyz[valid], axis=0)) > 0.2)
            report["streams"]["lidar"].update(frame=self.cloud.header.frame_id, points=len(xyz), valid_points=int(valid.sum()), range_m=[float(radii[valid].min()), float(radii[valid].max())] if valid.any() else None)
            np.save(self.output / "lidar_xyz.npy", xyz)
        report["passed"] = bool(checks and all(checks.values()))
        (self.output / "sensor_report.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2), flush=True)
        return report["passed"]


def main() -> None:
    """Capture sensor evidence from a running simulation and return failure on invalid data."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=120)
    parser.add_argument("--frames", type=int, default=5)
    parser.add_argument("--expected-rate", type=float, default=20, help="Recorded reference rate; renderer slowdown does not fail content validation.")
    args = parser.parse_args()
    if args.frames < 5 or args.timeout <= 0:
        parser.error("--frames must be at least 5 and --timeout must be positive")
    rclpy.init()
    node = SensorCheck(args.output, args.frames, args.expected_rate)
    deadline = time.monotonic() + args.timeout
    ready = False
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.ready():
                ready = True
                break
        passed = node.save(timed_out=not ready)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
