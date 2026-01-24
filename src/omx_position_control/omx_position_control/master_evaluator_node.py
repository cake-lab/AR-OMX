"""

Evaluates end-to-end teleoperation performance for an OpenManipulator system. 
Synchronizes teleoperation metrics with the robot kinematic pose, computes IK solver delay and end-to-end latency, 
and logs all relevant data to a CSV file for offline analysis.

"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv, os, atexit
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Pose
from open_manipulator_msgs.msg import KinematicsPose
from my_service_package.msg import TeleopMetric
import message_filters


def _shutdown_safely(node: Node):
    try:
        node.destroy_node()
    except Exception:
        pass
    try:
        rclpy.shutdown()
    except Exception:
        pass


class MasterEvaluatorNode(Node):
    """
    Subscribes:
      - /teleop_metrics (TeleopMetric: carries camera t0 + IK delay)
      - /kinematics_pose (OpenManipulator KinematicsPose)

    Logs per synchronized sample:
      capture_timestamp (s since epoch),
      target pose (xyz qxqyqzqw),
      kinematic pose (xyz qxqyqzqw),
      ik_solver_delay_ms,
      e2e_latency_ms = (now - capture_timestamp) measured here.
    """

    def __init__(self):
        super().__init__('master_evaluator_node')

        # -------- Parameters --------
        self.declare_parameter('teleop_metrics_topic', '/teleop_metrics')
        self.declare_parameter('kinematics_pose_topic', '/kinematics_pose')
        self.declare_parameter('evaluation_duration_s', 60.0)
        self.declare_parameter('sync_slop_s', 0.15)
        self.declare_parameter('qos_depth', 50)

        self.teleop_topic = self.get_parameter('teleop_metrics_topic').value
        self.kin_topic = self.get_parameter('kinematics_pose_topic').value
        self.eval_duration = float(self.get_parameter('evaluation_duration_s').value)
        self.sync_slop = float(self.get_parameter('sync_slop_s').value)
        self.qos_depth = int(self.get_parameter('qos_depth').value)

        # -------- Output path: ~/eval --------
        try:
            self.output_dir = str(Path.home() / 'eval')  # e.g., /home/harshrocks/eval
            os.makedirs(self.output_dir, exist_ok=True)
        except Exception as e:
            # Fallback to ~/.ros if home/eval fails for any reason
            self.get_logger().error(f"Failed to create {Path.home() / 'eval'}: {e}. Falling back to ~/.ros/omx_evals")
            self.output_dir = str(Path.home() / '.ros' / 'omx_evals')
            os.makedirs(self.output_dir, exist_ok=True)

        ts = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.output_path = os.path.join(self.output_dir, f"evaluation_run_{ts}.csv")

        # -------- QoS --------
        qos = QoSProfile(depth=self.qos_depth)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.history = HistoryPolicy.KEEP_LAST

        # -------- Subscribers + Synchronizer --------
        self.teleop_sub = message_filters.Subscriber(self, TeleopMetric, self.teleop_topic, qos_profile=qos)
        self.kin_sub = message_filters.Subscriber(self, KinematicsPose, self.kin_topic, qos_profile=qos)

        # Allow headerless in case /kinematics_pose lacks std_msgs/Header
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.teleop_sub, self.kin_sub],
            queue_size=self.qos_depth,
            slop=self.sync_slop,
            allow_headerless=True
        )
        self.sync.registerCallback(self._synced_cb)

        # -------- Accumulators --------
        self.rows = []
        self.ik_delay_sum_ms = 0.0
        self.ik_delay_count = 0
        self.e2e_sum_ms = 0.0
        self.e2e_count = 0
        self._last_log_ns = 0

        # Timed finish
        self._done = False
        self._finish_timer = self.create_timer(self.eval_duration, self._on_evaluation_done)

        # Always try to save on interpreter shutdown too
        atexit.register(self._save_if_needed)

        self.get_logger().info(
            "MasterEvaluatorNode started.\n"
            f"  teleop: {self.teleop_topic}\n"
            f"  kin:    {self.kin_topic}\n"
            f"  duration: {self.eval_duration:.1f}s  slop: {self.sync_slop:.3f}s\n"
            f"  writing to: {self.output_path}"
        )

    @staticmethod
    def _pose_to_list(p: Pose):
        return [float(p.position.x), float(p.position.y), float(p.position.z),
                float(p.orientation.x), float(p.orientation.y), float(p.orientation.z), float(p.orientation.w)]

    def _synced_cb(self, teleop_msg: TeleopMetric, kin_msg: KinematicsPose):
        # Capture timestamp from camera (via TeleopMetric.header.stamp)
        cap = teleop_msg.header.stamp
        capture_ts = float(cap.sec) + float(cap.nanosec) * 1e-9

        # E2E latency (ms): now - capture
        now_ns = self.get_clock().now().nanoseconds
        cap_ns = cap.sec * 1_000_000_000 + cap.nanosec
        e2e_ms = (now_ns - cap_ns) / 1e6
        if e2e_ms == e2e_ms:  # NaN-safe
            self.e2e_sum_ms += e2e_ms
            self.e2e_count += 1

        tgt_list = self._pose_to_list(teleop_msg.target_pose)
        kin_list = self._pose_to_list(kin_msg.pose)

        # IK delay
        ik_ms = float(teleop_msg.ik_solver_delay_ms)
        if ik_ms == ik_ms:
            self.ik_delay_sum_ms += ik_ms
            self.ik_delay_count += 1

        # Append row
        row = [capture_ts] + tgt_list + kin_list + [ik_ms, e2e_ms]
        self.rows.append(row)

        # Throttled status
        if now_ns - self._last_log_ns > 1_000_000_000:
            avg_ik = (self.ik_delay_sum_ms / max(1, self.ik_delay_count))
            avg_e2e = (self.e2e_sum_ms / max(1, self.e2e_count))
            self.get_logger().info(f"E2E ~ {avg_e2e:.1f} ms | Avg IK ~ {avg_ik:.1f} ms")
            self._last_log_ns = now_ns

    def _on_evaluation_done(self):
        if self._done:
            return
        self._done = True
        self._save_and_exit()

    # ---------- Save helpers ----------
    def _csv_header(self):
        return [
            'capture_timestamp',
            'target_pos_x','target_pos_y','target_pos_z','target_orient_x','target_orient_y','target_orient_z','target_orient_w',
            'kinematic_pos_x','kinematic_pos_y','kinematic_pos_z','kinematic_orient_x','kinematic_orient_y','kinematic_orient_z','kinematic_orient_w',
            'ik_solver_delay_ms',
            'e2e_latency_ms'
        ]

    def _write_csv(self):
        try:
            with open(self.output_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(self._csv_header())
                if self.rows:
                    writer.writerows(self.rows)
            self.get_logger().info(f"Saved {len(self.rows)} rows to {self.output_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to write CSV to {self.output_path}: {e}")

    def _save_if_needed(self):
        if not self._done:
            self._done = True
            self._write_csv()

    def _save_and_exit(self):
        self._write_csv()
        self.get_logger().info("Evaluation complete. Shutting down.")
        _shutdown_safely(self)


def main(args=None):
    rclpy.init(args=args)
    node = MasterEvaluatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C received — saving and shutting down.")
        node._save_and_exit()
    except Exception as e:
        node.get_logger().error(f"Unhandled exception: {e}")
        node._save_and_exit()

if __name__ == '__main__':
    main()
