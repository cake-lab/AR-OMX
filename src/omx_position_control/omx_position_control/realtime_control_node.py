"""

Performs real-time control for a teleoperated OpenManipulator system.
Receives time-stamped target poses from perception, computes inverse kinematics via a service call and
sends joint commands to the robot, and publishes teleoperation performance metrics. 
Logged metrics include IK solver delay and end-to-end latency measured at the control command issuance point, enabling
system-level latency evaluation.

"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

# IK service provided by your inverse_kinematics_node.py
from tutorial_interfaces.srv import CalculateIK

# OpenManipulator joint service
from open_manipulator_msgs.srv import SetJointPosition

# Custom metrics message (works with/without the new e2e field)
from my_service_package.msg import TeleopMetric


def _shutdown_safely(node: Node):
    try:
        node.destroy_node()
    except Exception:
        pass
    try:
        rclpy.shutdown()
    except Exception:
        pass


def stamp_to_ns(stamp) -> int:
    # Convert builtin_interfaces/Time to integer nanoseconds.
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class RealtimeControlNode(Node):
    """
    Subscribes to /target_pose stamped with the original camera capture timestamp, calls IK, sends joint commands, and publishes TeleopMetric with:
      - header (propagated t0)
      - target_pose
      - ik_solver_delay_ms
      - e2e_latency_ms_at_control (if field exists in TeleopMetric.msg)
    """

    def __init__(self):
        super().__init__("realtime_control_node")

        # ---------- Parameters ----------
        self.declare_parameter("target_pose_topic", "/target_pose")
        self.declare_parameter("teleop_metrics_topic", "/teleop_metrics")
        self.declare_parameter("ik_service_name", "calculate_ik")
        self.declare_parameter("joint_service_name", "goal_joint_space_path")
        self.declare_parameter("robot_frame_id", "base_link")
        self.declare_parameter("joint_names", ["joint1", "joint2", "joint3", "joint4"])
        self.declare_parameter("path_time", 0.10)  # seconds

        self.target_pose_topic = self.get_parameter("target_pose_topic").value
        self.teleop_metrics_topic = self.get_parameter("teleop_metrics_topic").value
        self.ik_service_name = self.get_parameter("ik_service_name").value
        self.joint_service_name = self.get_parameter("joint_service_name").value
        self.robot_frame_id = self.get_parameter("robot_frame_id").value
        self.joint_names = list(self.get_parameter("joint_names").value)
        self.path_time = float(self.get_parameter("path_time").value)

        # ---------- QoS ----------
        sub_qos = QoSProfile(depth=10)
        sub_qos.reliability = ReliabilityPolicy.RELIABLE
        sub_qos.history = HistoryPolicy.KEEP_LAST

        pub_qos = QoSProfile(depth=10)
        pub_qos.reliability = ReliabilityPolicy.RELIABLE
        pub_qos.history = HistoryPolicy.KEEP_LAST

        # Allow concurrent callbacks (IK + joint service futures)
        self.cb_group = ReentrantCallbackGroup()

        # ---------- Pub/Sub & Services ----------
        self.metrics_pub = self.create_publisher(
            TeleopMetric, self.teleop_metrics_topic, pub_qos
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, self.target_pose_topic, self._pose_cb, sub_qos
        )

        self.ik_client = self.create_client(
            CalculateIK, self.ik_service_name, callback_group=self.cb_group
        )
        self.joint_client = self.create_client(
            SetJointPosition, self.joint_service_name, callback_group=self.cb_group
        )

        # ---------- State ----------
        self._latest_pose: PoseStamped | None = None
        self._ik_inflight = False
        self._last_log_ns = 0

        self.get_logger().info(
            f"RealtimeControlNode ready\n"
            f"  Sub:  {self.target_pose_topic}\n"
            f"  IK:   {self.ik_service_name}\n"
            f"  Joint:{self.joint_service_name}"
        )
        self._wait_for_services()

    # ---------------- Helpers ----------------
    def _wait_for_services(self):
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                "IK service not available yet. Will wait on first call."
            )
        if not self.joint_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                "Joint service not available yet. Will wait on first call."
            )

    def _pose_cb(self, msg: PoseStamped):
        # keep only the newest target
        self._latest_pose = msg
        if not self._ik_inflight:
            self._start_ik_call()

    def _start_ik_call(self):
        if self._latest_pose is None or self._ik_inflight:
            return

        pose_msg = self._latest_pose
        self._latest_pose = None
        self._ik_inflight = True

        # Build IK request
        req = CalculateIK.Request()
        req.x = float(pose_msg.pose.position.x)
        req.y = float(pose_msg.pose.position.y)
        req.z = float(pose_msg.pose.position.z)
        req.qx = float(pose_msg.pose.orientation.x)
        req.qy = float(pose_msg.pose.orientation.y)
        req.qz = float(pose_msg.pose.orientation.z)
        req.qw = float(pose_msg.pose.orientation.w)

        t_req = self.get_clock().now()

        if not self.ik_client.service_is_ready():
            self.get_logger().debug("Waiting for IK service...")
            self.ik_client.wait_for_service()

        fut = self.ik_client.call_async(req)
        fut.add_done_callback(
            lambda f, hdr=pose_msg.header, p=pose_msg.pose, t0=t_req: self._on_ik_done(
                f, hdr, p, t0
            )
        )

    def _on_ik_done(self, future, src_header: Header, target_pose, t0):
        # Compute IK delay
        try:
            ik_res = future.result()
        except Exception as e:
            self.get_logger().error(f"IK service call failed: {e}")
            self._ik_inflight = False
            self._publish_metric(src_header, target_pose, float("nan"), None)
            self._start_ik_call()
            return

        t1 = self.get_clock().now()
        ik_delay_ms = (t1.nanoseconds - t0.nanoseconds) / 1e6

        # Send joint command and capture the command-issue timestamp (for true E2E at control)
        try:
            t_cmd_ns = self._send_joint_positions(ik_res)
        except Exception as e:
            self.get_logger().error(f"Failed to send joint positions: {e}")
            t_cmd_ns = None

        # Publish metrics (includes e2e_latency_ms_at_control when available)
        self._publish_metric(src_header, target_pose, ik_delay_ms, t_cmd_ns)

        self._ik_inflight = False
        # If a newer pose arrived while we were solving IK, process it now
        self._start_ik_call()

    def _send_joint_positions(self, ik_res) -> int | None:
        """Build and send a SetJointPosition request. Returns the ns timestamp at command issue."""
        req = SetJointPosition.Request()
        req.planning_group = ""
        req.joint_position.joint_name = list(self.joint_names)
        # theta1..theta4 from IK (fallback 0.0 if absent)
        req.joint_position.position = [
            float(getattr(ik_res, f"theta{i}", 0.0)) for i in range(1, 5)
        ]
        req.path_time = float(self.path_time)

        if not self.joint_client.service_is_ready():
            self.get_logger().debug("Waiting for joint service...")
            self.joint_client.wait_for_service()

        # Timestamp at the control point (camera→joints reference)
        t_cmd_ns = self.get_clock().now().nanoseconds

        # Fire-and-forget; we don't block
        self.joint_client.call_async(req)
        return t_cmd_ns

    def _publish_metric(
        self, header: Header, pose, ik_delay_ms: float, t_cmd_ns: int | None
    ):
        m = TeleopMetric()
        m.header = header  # original camera t0
        m.target_pose = pose
        m.ik_solver_delay_ms = float(ik_delay_ms)

        # If TeleopMetric.msg has the new field, populate it;
        # otherwise this no-ops and keeps backward compatibility.
        if t_cmd_ns is not None:
            try:
                t0_ns = stamp_to_ns(header.stamp)
                e2e_at_control_ms = (t_cmd_ns - t0_ns) / 1e6
                setattr(m, "e2e_latency_ms_at_control", float(e2e_at_control_ms))
            except Exception:
                pass

        self.metrics_pub.publish(m)

        # Throttled console (1 Hz)
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_log_ns > 1_000_000_000:
            log = f"IK delay: {ik_delay_ms:.1f} ms"
            if t_cmd_ns is not None:
                try:
                    e2e_ms = (t_cmd_ns - stamp_to_ns(header.stamp)) / 1e6
                    log += f" | E2E@control: {e2e_ms:.1f} ms"
                except Exception:
                    pass
            self.get_logger().info(log)
            self._last_log_ns = now_ns


def main(args=None):
    rclpy.init(args=args)
    node = RealtimeControlNode()
    try:
        # Multi-threaded so the IK and joint futures don't block callbacks
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        _shutdown_safely(node)


if __name__ == "__main__":
    main()
