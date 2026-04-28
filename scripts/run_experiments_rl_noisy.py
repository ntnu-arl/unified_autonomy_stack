#!/usr/bin/env python3

import subprocess
import time
import csv
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist, Pose
from nav_msgs.msg import Odometry, Path
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import PointCloud2, Imu
import numpy as np

# Global Settings
ENABLE_ROSBAGS = False

# Sync ROS_DOMAIN_ID from .env if present
if os.path.exists('.env'):
    with open('.env') as f:
        for line in f:
            if line.startswith('DOMAIN_ID='):
                domain = line.split('=')[1].strip()
                os.environ['ROS_DOMAIN_ID'] = domain
                print(f"Global Sync to ROS_DOMAIN_ID: {domain}")

class ExperimentLogger(Node):
    def __init__(self, experiment_id):
        super().__init__('experiment_logger')
        self.experiment_id = experiment_id
        self.start_time_ns = self.get_clock().now().nanoseconds
        
        # Metrics
        self.total_distance = 0.0
        self.last_pose = None
        self.collided = False
        self.accel_history = []
        self.finished = False
        self.reason = "timeout"
        self.start_pose = None
        self.stall_start_time = None
        self.last_stall_check_pose = None
        self.last_stall_check_time = None

        # Strict stall tracking (collision-level: <5cm/s for 5s, or attitude >45deg)
        self.strict_vel_stall_start = None

        # Latest point cloud for obstacle proximity check
        self.latest_pc = None

        # IMU spike detection
        self.last_imu_accel = None

        # Velocity history for windowed checks
        self.vel_history = []

        # Full speed history for avg/max reporting (post path-publish)
        self.speed_history = []
        
        # Target info
        self.start_pos = np.array([0.0, 0.0, 3.0])
        self.target_pos = np.array([35.0, 0.0, 3.0])
        self.goal_threshold = 1.0 # meters
        
        # QoS for sensors
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # Subscriptions
        self.create_subscription(Odometry, '/rmf_unipilot/odom', self.odom_callback, sensor_qos)
        self.create_subscription(Contacts, '/rmf_unipilot/contacts', self.contact_callback, sensor_qos)
        self.create_subscription(Twist, '/rmf_unipilot/cmd/acc', self.accel_callback, 10)
        self.create_subscription(PointCloud2, '/cbf_pc_selector/output_pc', self.pc_callback, sensor_qos)
        self.create_subscription(Imu, '/rmf_unipilot/imu', self.imu_callback, sensor_qos)
        
        # Publishers
        self.path_pub = self.create_publisher(Path, '/gbplanner_path', 10)
        self.is_published = False
        self.path_published_time_ns = None  # set from odom stamp at publish time

        # Track time via odom header stamps (reliable even if /clock is intermittent)
        self.first_odom_time_ns = None
        self.last_odom_time_ns = None
        
    def odom_callback(self, msg):
        curr_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        curr_vel = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
        vel_mag_odom = np.linalg.norm(curr_vel)

        # Extract roll and pitch from quaternion
        q = msg.pose.pose.orientation
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        sinp = np.clip(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0)
        pitch = np.arcsin(sinp)

        # Track time using the odom header stamp — reliable regardless of /clock state
        odom_time_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if self.first_odom_time_ns is None:
            self.first_odom_time_ns = odom_time_ns
        self.last_odom_time_ns = odom_time_ns
        
        # Calculate velocity based on position change (more reliable if twist info is missing/noisy)
        dist_moved = 0.0
        if self.last_pose is not None:
            dist_moved = np.linalg.norm(curr_pos - self.last_pose)
            if self.is_published:
                self.total_distance += dist_moved
            
        # Use the maximum of Odom velocity and calculated velocity
        # (calculated velocity might be 0 if the robot is jittering perfectly around a point)
        vel_mag = max(vel_mag_odom, dist_moved / 0.1) # Approx 10Hz if using spin_once 0.1
            
        # --- FALLBACK COLLISION DETECTION ---
        # All gating uses odom header stamps — immune to /clock returning 0
        odom_time_s = odom_time_ns / 1e9
        # Discard stale messages that were queued before the path was published
        if self.is_published and self.path_published_time_ns is not None and odom_time_ns < self.path_published_time_ns:
            self.last_pose = curr_pos
            if self.start_pose is None:
                self.start_pose = curr_pos
            return
        if self.is_published and self.path_published_time_ns is not None:
            elapsed_since_publish = (odom_time_ns - self.path_published_time_ns) / 1e9
            self.speed_history.append(vel_mag)
            if elapsed_since_publish > 3.0:
                # 1. Sudden deceleration (Physics impact)
                if hasattr(self, 'last_vel_mag') and self.last_vel_mag > 0.5 and vel_mag < (self.last_vel_mag * 0.3):
                    print(f"!!! COLLISION: Sudden Velocity Drop ({self.last_vel_mag:.2f} -> {vel_mag:.2f})")
                    self.collided = True
                    self.finished = True
                    self.reason = "collision_fallback_drop"

                # 2. Strict collision checks: attitude >45deg, <5cm/s or <5cm position change for 5s
                if not self.finished:
                    if abs(roll) > 30.0 * np.pi / 180 or abs(pitch) > 30.0 * np.pi / 180:
                        print(f"!!! COLLISION: Attitude limit exceeded (roll={np.degrees(roll):.1f}deg, pitch={np.degrees(pitch):.1f}deg) in Exp {self.experiment_id}")
                        self.collided = True
                        self.finished = True
                        self.reason = "collision_attitude"

                if not self.finished:
                    if vel_mag < 0.05:
                        if self.strict_vel_stall_start is None:
                            self.strict_vel_stall_start = odom_time_s
                        elif odom_time_s - self.strict_vel_stall_start > 5.0:
                            dist_from_start = np.linalg.norm(curr_pos - self.start_pose) if self.start_pose is not None else 0.0
                            if dist_from_start > 0.1:
                                min_pc_dist = self._pc_min_distance(self.latest_pc)
                                if min_pc_dist < 0.3:
                                    print(f"!!! COLLISION: Strict Stall (Vel < 5cm/s for 5s, nearest obstacle {min_pc_dist:.2f}m) in Exp {self.experiment_id}")
                                    self.collided = True
                                    self.finished = True
                                    self.reason = "collision_strict_vel"
                                else:
                                    print(f"--- STALLED: Low velocity but no nearby obstacle (nearest {min_pc_dist:.2f}m) in Exp {self.experiment_id}")
                                    self.finished = True
                                    self.reason = "stalled_strict_vel"
                    else:
                        self.strict_vel_stall_start = None

                # 3. Loose stall checks (robot not making progress — logged as stalled, not collision)
                if not self.finished:
                    if vel_mag < 0.2:
                        if self.stall_start_time is None:
                            self.stall_start_time = odom_time_s
                        elif odom_time_s - self.stall_start_time > 10.0:
                            dist_from_start = np.linalg.norm(curr_pos - self.start_pose) if self.start_pose is not None else 0.0
                            if dist_from_start > 0.1:
                                print(f"--- STALLED: Robot Stalled (Vel < 0.2m/s for 10s) in Exp {self.experiment_id}")
                                self.finished = True
                                self.reason = "stalled_vel"
                    else:
                        self.stall_start_time = None

                if not self.finished:
                    if self.last_stall_check_time is None:
                        self.last_stall_check_time = odom_time_s
                        self.last_stall_check_pose = curr_pos
                    elif odom_time_s - self.last_stall_check_time > 10.0:
                        dist_prog = np.linalg.norm(curr_pos - self.last_stall_check_pose)
                        if dist_prog < 0.2:
                            print(f"--- STALLED: No Progress (< 20cm/10s) in Exp {self.experiment_id}")
                            self.finished = True
                            self.reason = "stalled_pos"
                        self.last_stall_check_time = odom_time_s
                        self.last_stall_check_pose = curr_pos

        self.last_pose = curr_pos
        if self.start_pose is None:
            self.start_pose = curr_pos
        
        # Check success
        dist_to_target = np.linalg.norm(curr_pos - self.target_pos)
        if dist_to_target < self.goal_threshold:
            self.finished = True
            self.reason = "success"
            
    def contact_callback(self, msg):
        if not self.is_published or self.path_published_time_ns is None:
            return
        if self.last_odom_time_ns is None:
            return
        if (self.last_odom_time_ns - self.path_published_time_ns) / 1e9 < 3.0:
            return
        if len(msg.contacts) > 0:
            if not self.collided:
                print(f"!!! COLLISION DETECTED (Contact Sensor) in Exp {self.experiment_id} !!!")
            self.collided = True
            self.finished = True
            self.reason = "collision_contact"
            
    def accel_callback(self, msg):
        self.accel_history.append({
            't': (self.get_clock().now().nanoseconds - self.start_time_ns) / 1e9,
            'ax': msg.linear.x,
            'ay': msg.linear.y,
            'az': msg.linear.z
        })

    def imu_callback(self, msg):
        if not self.is_published or self.path_published_time_ns is None or self.last_odom_time_ns is None:
            return
        if (self.last_odom_time_ns - self.path_published_time_ns) / 1e9 < 3.0:
            return
        if self.finished:
            return
        curr_accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        if self.last_imu_accel is not None:
            delta = np.abs(curr_accel - self.last_imu_accel)
            if np.any(delta > 25.0):
                print(f"!!! COLLISION: IMU Acceleration Spike (delta={delta}) in Exp {self.experiment_id}")
                self.collided = True
                self.finished = True
                self.reason = "collision_imu_spike"
        self.last_imu_accel = curr_accel

    def pc_callback(self, msg):
        self.latest_pc = msg
        if not self.is_published or self.path_published_time_ns is None or self.last_odom_time_ns is None:
            return
        if (self.last_odom_time_ns - self.path_published_time_ns) / 1e9 < 3.0:
            return
        if self.finished:
            return
        min_dist = self._pc_min_distance(msg)
        if min_dist < 0.3:
            print(f"!!! COLLISION: Obstacle within {min_dist:.2f}m (PC proximity) in Exp {self.experiment_id}")
            self.collided = True
            self.finished = True
            self.reason = "collision_pc_proximity"

    def _pc_min_distance(self, pc_msg):
        if pc_msg is None:
            return float('inf')
        n = pc_msg.width * pc_msg.height
        if n == 0:
            return float('inf')
        step = pc_msg.point_step
        off = {f.name: f.offset for f in pc_msg.fields if f.name in ('x', 'y', 'z')}
        if len(off) < 3:
            return float('inf')
        buf = np.frombuffer(bytes(pc_msg.data), dtype=np.uint8).reshape(n, step)
        xs = buf[:, off['x']:off['x']+4].copy().view(np.float32).flatten()
        ys = buf[:, off['y']:off['y']+4].copy().view(np.float32).flatten()
        zs = buf[:, off['z']:off['z']+4].copy().view(np.float32).flatten()
        dists = np.sqrt(xs**2 + ys**2 + zs**2)
        valid = np.isfinite(dists)
        return float(np.min(dists[valid])) if valid.any() else float('inf')

    def publish_path(self):
        # Publish Path from Start to Target
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"
        
        # Start Pose (Yaw = PI)
        p1 = PoseStamped()
        p1.header = path.header
        p1.pose.position.x = self.start_pos[0]
        p1.pose.position.y = self.start_pos[1]
        p1.pose.position.z = self.start_pos[2]
        p1.pose.orientation.x = 0.0
        p1.pose.orientation.y = 0.0
        p1.pose.orientation.z = 1.0
        p1.pose.orientation.w = 0.0
        
        # Target Pose (Yaw = PI)
        p2 = PoseStamped()
        p2.header = path.header
        p2.pose.position.x = self.target_pos[0]
        p2.pose.position.y = self.target_pos[1]
        p2.pose.position.z = self.target_pos[2]
        p2.pose.orientation.x = 0.0
        p2.pose.orientation.y = 0.0
        p2.pose.orientation.z = 1.0
        p2.pose.orientation.w = 0.0
        
        path.poses = [p1, p2]
        self.path_pub.publish(path)
        self.is_published = True
        self.path_published_time_ns = self.last_odom_time_ns  # odom stamp, immune to /clock = 0
        self.get_logger().info(f"Path published for Experiment {self.experiment_id}")

def run_experiment(exp_id):
    # 1. Start Docker
    print(f"\n--- Starting Experiment {exp_id} ---")
    subprocess.run(["make", "launch", "DOCKER_COMPOSE_FILE=docker-compose.uav_rl_cbf_eval.yml", "DOCKER_ARGS=-d"])
    
    # 2. Start Rosbag Recording
    bag_process = None
    if ENABLE_ROSBAGS:
        os.makedirs('bags', exist_ok=True)
        bag_path = f"bags/exp_{exp_id}"
        # Record relevant topics
        topics = [
            '/rmf_unipilot/odom',
            '/rmf_unipilot/cmd/acc',
            '/rmf_unipilot/cmd/vel',
            '/rmf_unipilot/lidar/points_downsampled',
            '/tf',
            '/tf_static'
        ]
        bag_process = subprocess.Popen(["ros2", "bag", "record", "-o", bag_path] + topics)
    
    # Initialize ROS
    rclpy.init()
    # Use sim time so clock matches Gazebo
    node = ExperimentLogger(exp_id)
    node.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    
    # 2. Wait for simulation to be ready (Odom appearing)
    print("Waiting for simulation to stabilize...")
    start_wait = time.time()
    while rclpy.ok() and node.last_pose is None:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - start_wait >20:
            print("Timeout waiting for simulation!")
            break
            
    if node.last_pose is None:
        print("!!! Simulation failed to stabilize. Killing and retrying... !!!")
        node.destroy_node()
        rclpy.shutdown()
        subprocess.run(["make", "stop", "DOCKER_COMPOSE_FILE=docker-compose.uav_rl_cbf_eval.yml"])
        return False # Signal failure to start
            
    # Wait for other nodes (RL) to be fully ready
    print("Sim stabilized. Waiting 5s for RL inference node to be ready...")
    time.sleep(1)

    # Wait for RL to take control and robot to start moving
    print("Waiting 10s for RL to take control...")
    time.sleep(1)

    # Flush the queued odom messages so last_odom_time_ns is current before we
    # record it as path_published_time_ns — prevents stale fall-velocity messages
    # from slipping past the post-publish gate.
    print("Flushing odom queue...")
    flush_end = time.time() + 1.0
    while time.time() < flush_end:
        rclpy.spin_once(node, timeout_sec=0.05)

    # 3. Publish Path
    node.publish_path()

    # 4. Monitor run
    timeout = 80.0 # seconds (Sim Time)

    while rclpy.ok() and not node.finished:
        rclpy.spin_once(node, timeout_sec=0.1)

        # Timeout measured from path publish using odom stamps
        if node.path_published_time_ns is not None and node.last_odom_time_ns is not None:
            elapsed = (node.last_odom_time_ns - node.path_published_time_ns) / 1e9
            if elapsed > timeout:
                node.finished = True
                node.reason = "timeout"
            

    # Capture duration from path publish to end, using odom stamps for reliability
    if node.path_published_time_ns is not None and node.last_odom_time_ns is not None:
        sim_duration = (node.last_odom_time_ns - node.path_published_time_ns) / 1e9
    else:
        sim_duration = (node.get_clock().now().nanoseconds - node.start_time_ns) / 1e9

    # Pause simulation to allow visual inspection
    print(f"Experiment {exp_id} finished ({node.reason}). Pausing simulation...")
    # This finds the container for the uav_sim service and runs the pause command
    pause_cmd = (
        "docker exec $(docker ps -q -f name=ros2_launch_uav_sim) "
        "gz service -s /world/map/control "
        "--reqtype gz.msgs.WorldControl "
        "--reptype gz.msgs.Boolean "
        "--timeout 1500 "
        "--req 'pause: true'"
    )
    subprocess.run(pause_cmd, shell=True)
    
    # Wait a few seconds so the user can actually see the paused state before cleanup
    time.sleep(3)

    # 5. Save Results
    results = {
        'id': exp_id,
        'status': node.reason,
        'distance': round(node.total_distance, 2),
        'duration': round(sim_duration, 2),
        'avg_speed': round(float(np.mean(node.speed_history)), 3) if node.speed_history else 0,
        'max_speed': round(float(np.max(node.speed_history)), 3) if node.speed_history else 0,
        'avg_accel_x': np.mean([a['ax'] for a in node.accel_history]) if node.accel_history else 0
    }

    # Save detailed accel log
    os.makedirs('logs/rl/accel', exist_ok=True)
    with open(f'logs/rl/accel/exp_{exp_id}.csv', 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['t', 'ax', 'ay', 'az'])
        writer.writeheader()
        writer.writerows(node.accel_history)

    # Append to summary
    os.makedirs('logs/rl', exist_ok=True)
    summary_file = 'logs/rl/experiment_summary.csv'
    is_new = not os.path.exists(summary_file)
    with open(summary_file, 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=results.keys())
        if is_new: writer.writeheader()
        writer.writerow(results)

    # Append to human-readable log
    with open('logs/rl/experiment_summary.txt', 'a') as f:
        f.write(f"\n--- Experiment {exp_id} Results ---\n")
        for key, value in results.items():
            f.write(f"{key}: {value}\n")

    print(f"\n--- Experiment {exp_id} Results ---")
    for key, value in results.items():
        print(f"{key}: {value}")

    # 6. Cleanup
    node.destroy_node()
    rclpy.shutdown()

    # Stop Rosbag
    if bag_process:
        bag_process.terminate()
        try:
            bag_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            bag_process.kill()

    subprocess.run(["make", "stop", "DOCKER_COMPOSE_FILE=docker-compose.uav_rl_cbf_eval.yml"])
    time.sleep(2) # Cooldown
    return True

if __name__ == "__main__":
    rviz_process = None
    try:
        print("Starting RViz 2...")
        # Start rviz2 with use_sim_time:=True once at the beginning
        # It inherits ROS_DOMAIN_ID from the environment
        rviz_process = subprocess.Popen(
            ["ros2", "run", "rviz2", "rviz2", "--ros-args", "-p", "use_sim_time:=True"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

        num_samples = 50
        total_exp_run = 0
        while total_exp_run < num_samples:
            success = run_experiment(total_exp_run)
            if success:
                total_exp_run += 1
            else:
                print(f"Retrying experiment {total_exp_run}...")
        print("\nAll experiments complete! Summary saved to experiment_summary.csv")
    except KeyboardInterrupt:
        print("\n!!! Experiment interrupted by user !!!")
    finally:
        print("Ensuring simulation is stopped...")
        subprocess.run(["make", "stop", "DOCKER_COMPOSE_FILE=docker-compose.uav_rl_cbf_eval.yml"])
        if rviz_process:
            print("Stopping RViz 2...")
            rviz_process.terminate()
            try:
                rviz_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                rviz_process.kill()