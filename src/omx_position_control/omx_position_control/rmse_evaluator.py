"""

Evaluates the trajectory tracking performance of a robotic manipulator by comparing commanded end-effector poses with the actual poses reported by the robot kinematics.

The node time-synchronizes target (PoseStamped) and feedback (KinematicsPose) messages and computes two metrics: 
Absolute Trajectory Error (ATE), which measures instantaneous pose error, and 
Relative Pose Error (RPE), which evaluates motion tracking accuracy over a fixed time interval.

Both cumulative and sliding-window statistics are published in real time, and a reset service is provided to clear all stored metrics for repeated trials.

"""

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time

from geometry_msgs.msg import Pose, PoseStamped
from open_manipulator_msgs.msg import KinematicsPose  # --- MODIFIED ---
from my_service_package.msg import RmseMetrics
from my_service_package.srv import ResetMetrics

import numpy as np
from scipy.spatial.transform import Rotation
from collections import deque
import message_filters
import threading
import time
from typing import List, Tuple, Optional


class RmseEvaluatorNode(Node):
    """
    Comprehensive trajectory evaluation node for robotic manipulators.

    This node subscribes to commanded (target) and actual (feedback) pose topics,
    synchronizes the messages, and calculates key performance metrics:
    1.  Absolute Trajectory Error (ATE): The direct error between the commanded
        and actual pose at each timestamp.
    2.  Relative Pose Error (RPE): The error in the robot's motion between
        two points in time, comparing the commanded motion to the actual motion.

    It computes and publishes detailed statistics (RMSE, mean, max, etc.) for both
    ATE and RPE, providing deep insight into the tracking performance of the robot.
    """

    def __init__(self):
        super().__init__('rmse_evaluator_node')

        # --- ROS Parameters for Configuration ---
        # Topic names, synchronization tolerance, and evaluation settings
        self.declare_parameter('target_topic', '/target_pose')
        self.declare_parameter('actual_topic', '/kinematics_pose')  # --- MODIFIED ---
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('sync_slop_s', 0.1)
        self.declare_parameter('rpe_delta_s', 1.0)  # Time delta for Relative Pose Error
        self.declare_parameter('history_buffer_size', 200)  # Must be > publish_rate * rpe_delta_s # --- MODIFIED ---
        self.declare_parameter('window_duration_s', 5.0)  # Duration for sliding window stats

        # --- Get Parameters ---
        self.target_topic_name = self.get_parameter('target_topic').get_parameter_value().string_value
        self.actual_topic_name = self.get_parameter('actual_topic').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate_hz').get_parameter_value().double_value
        self.sync_slop = self.get_parameter('sync_slop_s').get_parameter_value().double_value
        self.rpe_delta = self.get_parameter('rpe_delta_s').get_parameter_value().double_value
        self.buffer_size = self.get_parameter('history_buffer_size').get_parameter_value().integer_value
        self.window_duration = self.get_parameter('window_duration_s').get_parameter_value().double_value

        self.get_logger().info("--- Trajectory Evaluator Node Initializing ---")
        self.get_logger().info(f"  - Target Topic: {self.target_topic_name}")
        self.get_logger().info(f"  - Actual Topic: {self.actual_topic_name}")
        self.get_logger().info(f"  - RPE Delta: {self.rpe_delta}s")

        # --- Data Storage & Threading ---
        # Lock protects shared data accessed by callbacks and timers
        self.data_lock = threading.Lock()

        # Tracks callback execution time for performance diagnostics
        self.processing_times = deque(maxlen=100)
        
        # Data history for RPE and windowed calculations
        # Stores tuples of (timestamp, T_cmd, T_act, pos_err_components_mm)
        self.pose_history = deque(maxlen=self.buffer_size)

        # Cumulative error storage for ATE and RPE
        self.ate_pos_errors_mm = []
        self.ate_orient_errors_deg = []
        self.rpe_pos_errors_mm = []
        self.rpe_orient_errors_deg = []

        # --- Publisher & Services ---
        # Publishes aggregated performance metrics
        self.metrics_publisher = self.create_publisher(RmseMetrics, '/rmse_metrics', 1)

        # Timer periodically publishes summary statistics
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate, self.publish_metrics_callback
        )

        # Service to reset all collected metrics
        self.reset_service = self.create_service(
            ResetMetrics, 'reset_rmse_metrics', self.reset_metrics_callback
        )

        # --- Subscribers and Time Synchronizer ---
        # BEST_EFFORT is sufficient for high-rate telemetry data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Target (commanded) pose subscriber
        self.target_sub = message_filters.Subscriber(
            self, PoseStamped, self.target_topic_name, qos_profile=qos_profile
        )

        # Actual pose subscriber (from manipulator kinematics)
        self.actual_sub = message_filters.Subscriber(
            self, KinematicsPose, self.actual_topic_name, qos_profile=qos_profile
        )  # --- MODIFIED ---

        # Approximate time synchronization to tolerate small timestamp jitter
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.target_sub, self.actual_sub],
            queue_size=30,
            slop=self.sync_slop
        )  # --- MODIFIED ---

        self.ts.registerCallback(self.synchronized_callback)
        self.get_logger().info("Node initialized. Waiting for synchronized messages...")

    def synchronized_callback(
        self, target_msg: PoseStamped, kinematics_msg: KinematicsPose
    ):  # --- MODIFIED ---
        """
        Core callback that processes a synchronized pair of target and actual poses.
        """
        start_time = time.perf_counter()

        # Convert ROS pose messages into homogeneous transformation matrices
        T_cmd = self.pose_msg_to_matrix(target_msg.pose)
        T_act = self.pose_msg_to_matrix(kinematics_msg.pose)  # --- MODIFIED ---
        
        # Guard against invalid or malformed pose data
        if T_cmd is None or T_act is None:
            self.get_logger().warn(
                "Received invalid pose data. Skipping.", throttle_duration_sec=5
            )
            return

        current_timestamp = Time.from_msg(target_msg.header.stamp)

        # Compute Absolute Trajectory Error
        ate_pos_err_mm, ate_orient_err_deg, pos_err_components_mm = \
            self.calculate_ate(T_cmd, T_act)

        rpe_pos_err_mm, rpe_orient_err_deg = None, None
        
        # Retrieve a previous pose for RPE computation
        prev_pose_data = self.find_previous_pose(
            current_timestamp, self.rpe_delta
        )
        if prev_pose_data:
            # CORRECTED: Unpack all 4 values from the history tuple, ignoring the last one.
            _prev_ts, T_cmd_prev, T_act_prev, _ = prev_pose_data
            rpe_pos_err_mm, rpe_orient_err_deg = self.calculate_rpe(
                T_cmd, T_act, T_cmd_prev, T_act_prev
            )

        # Store results in shared buffers
        with self.data_lock:
            self.ate_pos_errors_mm.append(ate_pos_err_mm)
            self.ate_orient_errors_deg.append(ate_orient_err_deg)

            if rpe_pos_err_mm is not None:
                self.rpe_pos_errors_mm.append(rpe_pos_err_mm)
                self.rpe_orient_errors_deg.append(rpe_orient_err_deg)
            
            # History needed for RPE and per-axis error reporting
            self.pose_history.append(
                (current_timestamp, T_cmd, T_act, pos_err_components_mm)
            )

        # Measure callback processing latency
        self.processing_times.append(
            (time.perf_counter() - start_time) * 1000
        )

    def publish_metrics_callback(self):
        # Periodically calculates summary statistics and publishes them.
        
        with self.data_lock:
            # Do nothing until at least one sample is available
            if not self.ate_pos_errors_mm:
                return

            msg = RmseMetrics()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'

            # Determine window size in samples
            window_size = int(self.publish_rate * self.window_duration)
            ate_pos_window = self.ate_pos_errors_mm[-window_size:]
            ate_orient_window = self.ate_orient_errors_deg[-window_size:]

            msg.total_samples = len(self.ate_pos_errors_mm)
            msg.window_samples = len(ate_pos_window)

            # --- Position ATE statistics ---
            stats_ate_pos = self.calculate_statistics(self.ate_pos_errors_mm)
            stats_ate_pos_window = self.calculate_statistics(ate_pos_window)

            msg.cumulative_pos_rmse_mm = stats_ate_pos.get('rmse', 0.0)
            msg.window_pos_rmse_mm = stats_ate_pos_window.get('rmse', 0.0)
            msg.instantaneous_pos_error_mm = self.ate_pos_errors_mm[-1]
            
            # Per-axis position error (most recent sample)
            if self.pose_history:
                _, _, _, last_pos_err_components = self.pose_history[-1]
                msg.pos_error_x_mm = last_pos_err_components[0]
                msg.pos_error_y_mm = last_pos_err_components[1]
                msg.pos_error_z_mm = last_pos_err_components[2]

            msg.pos_max_error_mm = stats_ate_pos.get('max', 0.0)
            msg.pos_p95_error_mm = stats_ate_pos.get('p95', 0.0)
            
            # --- Orientation ATE statistics ---
            stats_ate_orient = self.calculate_statistics(
                self.ate_orient_errors_deg
            )
            stats_ate_orient_window = self.calculate_statistics(
                ate_orient_window
            )

            msg.cumulative_orient_rmse_deg = stats_ate_orient.get('rmse', 0.0)
            msg.window_orient_rmse_deg = stats_ate_orient_window.get('rmse', 0.0)
            msg.instantaneous_orient_error_deg = self.ate_orient_errors_deg[-1]
            msg.orient_max_error_deg = stats_ate_orient.get('max', 0.0)

            # Average callback processing time
            msg.avg_processing_time_ms = (
                np.mean(self.processing_times)
                if self.processing_times else 0.0
            )
            
            # Estimate incoming data rate
            if len(self.pose_history) > 1:
                time_diff = (
                    self.pose_history[-1][0].nanoseconds -
                    self.pose_history[0][0].nanoseconds
                ) / 1e9
                msg.data_rate_hz = (
                    (len(self.pose_history) - 1) / time_diff
                    if time_diff > 0 else 0.0
                )
            else:
                msg.data_rate_hz = 0.0

            # Placeholder for future data-quality heuristics
            msg.data_quality_good = True

        self.metrics_publisher.publish(msg)

    def reset_metrics_callback(self, request, response):
        # Service callback to clear all stored data and reset calculations.
        with self.data_lock:
            self.pose_history.clear()
            self.ate_pos_errors_mm.clear()
            self.ate_orient_errors_deg.clear()
            self.rpe_pos_errors_mm.clear()
            self.rpe_orient_errors_deg.clear()
            self.processing_times.clear()
        
        self.get_logger().info(
            "Trajectory metrics have been reset via service call."
        )
        response.success = True
        response.message = "RMSE metrics reset successfully."
        return response

    @staticmethod
    def calculate_ate(
        T_cmd: np.ndarray, T_act: np.ndarray
    ) -> Tuple[float, float, Tuple[float, float, float]]:
        # Calculates the Absolute Trajectory Error between two poses.
        T_err = np.linalg.inv(T_act) @ T_cmd

        # Translational error
        pos_error_vec_m = T_err[:3, 3]
        pos_error_m = np.linalg.norm(pos_error_vec_m)

        # Rotational error
        rot_err = Rotation.from_matrix(T_err[:3, :3])
        angle_rad = np.linalg.norm(rot_err.as_rotvec())

        # Per-axis position error (mm)
        pos_error_components_mm = (
            pos_error_vec_m[0] * 1000.0,
            pos_error_vec_m[1] * 1000.0,
            pos_error_vec_m[2] * 1000.0
        )

        return pos_error_m * 1000.0, np.rad2deg(angle_rad), pos_error_components_mm

    @staticmethod
    def calculate_rpe(
        T_cmd_curr: np.ndarray, T_act_curr: np.ndarray,
        T_cmd_prev: np.ndarray, T_act_prev: np.ndarray
    ) -> Tuple[float, float]:
        # Calculates the Relative Pose Error.
        delta_cmd = np.linalg.inv(T_cmd_prev) @ T_cmd_curr
        delta_act = np.linalg.inv(T_act_prev) @ T_act_curr

        T_err_rpe = np.linalg.inv(delta_act) @ delta_cmd

        pos_error_m = np.linalg.norm(T_err_rpe[:3, 3])
        rot_err = Rotation.from_matrix(T_err_rpe[:3, :3])
        angle_rad = np.linalg.norm(rot_err.as_rotvec())

        return pos_error_m * 1000.0, np.rad2deg(angle_rad)

    def find_previous_pose(
        self, current_stamp: Time, delta_s: float
    ) -> Optional[tuple]:
        # Searches the history for the pose closest to a specific time delta in the past.
        target_time_s = current_stamp.nanoseconds / 1e9 - delta_s
        with self.data_lock:
            if not self.pose_history:
                return None

            # Select the pose whose timestamp best matches the desired offset
            best_match = min(
                self.pose_history,
                key=lambda p: abs(
                    p[0].nanoseconds / 1e9 - target_time_s
                )
            )
            return best_match

    @staticmethod
    def pose_msg_to_matrix(pose: Pose) -> Optional[np.ndarray]:
        # Converts a geometry_msgs/Pose message to a 4x4 homogeneous matrix.
        p = pose.position
        q = pose.orientation
        try:
            quat_array = np.array([q.x, q.y, q.z, q.w])
            norm = np.linalg.norm(quat_array)
            if np.isclose(norm, 0):
                return None
            rot = Rotation.from_quat(quat_array / norm)
        except ValueError:
            return None

        T = np.eye(4)
        T[:3, :3] = rot.as_matrix()
        T[:3, 3] = [p.x, p.y, p.z]
        return T

    @staticmethod
    def calculate_statistics(data: List[float]) -> dict:
        # Calculates a dictionary of summary statistics for a list of numbers.
        if not data:
            return {}
        arr = np.array(data)
        stats = {
            'mean': np.mean(arr),
            'rmse': np.sqrt(np.mean(arr**2)),
            'median': np.median(arr),
            'std': np.std(arr),
            'max': np.max(arr),
            'p95': np.percentile(arr, 95),
        }
        return stats


def main(args=None):
    rclpy.init(args=args)
    node = RmseEvaluatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

