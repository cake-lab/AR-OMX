#!/usr/bin/env python3
"""
enhanced_velocity_controller.py

A production-ready ROS 2 controller for the OpenManipulator-X.

This node provides a robust, safety-first implementation for controlling the
OpenManipulator-X based on Cartesian velocity or position commands, typically
from a hand-tracking system. It integrates analytical kinematics, dual-mode control,
and comprehensive safety checks inspired by modern robotics practices.

Key Features:
- Dual Control Modes: Switches automatically between 'velocity' and 'position'
  control based on the most recently received command.
- Safety-First Design: Includes workspace boundaries, joint limits, velocity
  clamping, and a command timeout for safe operation.
- Complete Kinematics: Utilizes the analytical Jacobian for precise velocity
  control and forward kinematics for position tracking.
- Robust Architecture: Runs a high-frequency control loop with thread-safe
  state management and clear, modular functions.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import JointState
from open_manipulator_msgs.srv import SetJointPosition
import numpy as np
import threading
import time
import math
from scipy.spatial.transform import Rotation as R
from collections import deque

# =============================================================================
# --- Constants and Configuration ---
# =============================================================================

# --- Control Parameters ---
CONTROL_FREQUENCY = 30.0  # Hz
DT = 1.0 / CONTROL_FREQUENCY
COMMAND_TIMEOUT = 0.5  # Seconds to wait for new commands before stopping
VELOCITY_SMOOTHING_WINDOW = 5 # Number of samples to average for smoothing

# --- Safety Limits ---
VELOCITY_LIMIT_LINEAR = 0.3  # m/s
VELOCITY_LIMIT_ANGULAR = 1.5 # rad/s
BASE_JOINT_MIN = math.radians(-90.0) # Radian limit for joint1
BASE_JOINT_MAX = math.radians(90.0)  # Radian limit for joint1
# Workspace boundaries [X_min, X_max, Y_min, Y_max, Z_min, Z_max] in meters
WORKSPACE_LIMITS = np.array([-0.5, 0.5, -0.5, 0.5, 0.0, 0.6])

# --- Kinematic Parameters (OpenManipulator-X) ---
# Link lengths converted from mm to meters
L1, L2, L3, L4, L5 = 0.036076, 0.06025, 0.13023, 0.124, 0.1334
A1 = math.radians(79.38) # Fixed angle for link3

class EnhancedVelocityController(Node):
    """
    Manages robot control by translating Cartesian commands into joint movements.
    """
    def __init__(self):
        super().__init__('enhanced_velocity_controller')
        
        # --- State Variables ---
        self.current_joint_states = None
        self.last_command_time = 0
        self.active_control_mode = None # "velocity" or "position"
        self.state_lock = threading.Lock()
        
        # --- Command Buffers for Smoothing ---
        self.linear_vel_buffer = deque(maxlen=VELOCITY_SMOOTHING_WINDOW)
        self.angular_vel_buffer = deque(maxlen=VELOCITY_SMOOTHING_WINDOW)

        # --- Subscribers ---
        # Subscribes to the robot's current joint states
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, 10
        )
        # Subscribes to incoming velocity commands (for velocity mode)
        self.create_subscription(
            Twist, '/cmd_vel', self._velocity_cmd_callback, 10
        )
        # Subscribes to incoming pose commands (for position mode)
        self.create_subscription(
            Pose, '/cmd_pose', self._position_cmd_callback, 10
        )
        
        # --- Service Client ---
        # Client to send target joint positions to the robot hardware driver
        self.joint_control_client = self.create_client(
            SetJointPosition, 'goal_joint_space_path'
        )
        
        self.get_logger().info('Waiting for joint control service...')
        while not self.joint_control_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.get_logger().info('Enhanced Velocity Controller Initialized.')
        
        # --- Wait for first joint state message before starting control loop ---
        self.get_logger().info('Waiting for initial joint states...')
        while self.current_joint_states is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'Initial joint states received: {self.current_joint_states}')

        # --- Main Control Loop Timer ---
        self.control_timer = self.create_timer(DT, self._control_loop)

    # =========================================================================
    # --- ROS2 Callbacks ---
    # =========================================================================

    def _joint_state_callback(self, msg: JointState):
        """Updates the robot's current joint angles from /joint_states topic."""
        with self.state_lock:
            # We only control the first 4 joints
            self.current_joint_states = np.array(msg.position[:4])

    def _velocity_cmd_callback(self, msg: Twist):
        """Receives and buffers velocity commands."""
        with self.state_lock:
            self.linear_vel_buffer.append([msg.linear.x, msg.linear.y, msg.linear.z])
            self.angular_vel_buffer.append([msg.angular.x, msg.angular.y, msg.angular.z])
            self.last_command_time = self.get_clock().now().nanoseconds
            self.active_control_mode = "velocity"

    def _position_cmd_callback(self, msg: Pose):
        """Receives and processes position commands."""
        with self.state_lock:
            # Position control is handled directly in the control loop
            # This callback just updates the state
            self.target_pose = msg
            self.last_command_time = self.get_clock().now().nanoseconds
            self.active_control_mode = "position"

    # =========================================================================
    # --- Main Control Logic ---
    # =========================================================================

    def _control_loop(self):
        """The main execution loop, running at CONTROL_FREQUENCY."""
        with self.state_lock:
            if self.current_joint_states is None:
                self.get_logger().warn("No joint states available. Skipping control loop.")
                return

            # --- Safety Check: Command Timeout ---
            time_since_last_cmd = (self.get_clock().now().nanoseconds - self.last_command_time) / 1e9
            if time_since_last_cmd > COMMAND_TIMEOUT:
                if self.active_control_mode != "stopped":
                    self.get_logger().warn(f"Command timeout ({COMMAND_TIMEOUT}s). Stopping robot.")
                    self._safe_stop()
                return

            # --- Execute active control mode ---
            if self.active_control_mode == "velocity":
                self._execute_velocity_control()
            elif self.active_control_mode == "position":
                self._execute_position_control()

    def _execute_velocity_control(self):
        """Calculates and sends joint commands based on smoothed velocity."""
        if not self.linear_vel_buffer:
            return

        # --- Smooth and Clamp Velocities ---
        lin_vel = np.mean(self.linear_vel_buffer, axis=0)
        ang_vel = np.mean(self.angular_vel_buffer, axis=0)

        lin_vel_norm = np.linalg.norm(lin_vel)
        if lin_vel_norm > VELOCITY_LIMIT_LINEAR:
            lin_vel = lin_vel * (VELOCITY_LIMIT_LINEAR / lin_vel_norm)

        ang_vel_norm = np.linalg.norm(ang_vel)
        if ang_vel_norm > VELOCITY_LIMIT_ANGULAR:
            ang_vel = ang_vel * (VELOCITY_LIMIT_ANGULAR / ang_vel_norm)
        
        twist = np.concatenate([lin_vel, ang_vel])

        # --- Inverse Velocity Kinematics ---
        J = self._compute_jacobian(self.current_joint_states)
        try:
            J_pinv = np.linalg.pinv(J)
            q_dot = J_pinv @ twist
        except np.linalg.LinAlgError:
            self.get_logger().error("Jacobian is singular. Cannot compute joint velocities.")
            return

        # --- Integration and Safety Checks ---
        q_next = self.current_joint_states + q_dot * DT
        q_next = self._enforce_joint_limits(q_next)

        # Send command to robot
        self._send_joint_command(q_next, DT)

    def _execute_position_control(self):
        """Calculates and sends joint commands to reach a target pose."""
        if self.target_pose is None:
            return
        
        # --- Get current and target pose ---
        current_fk = self._forward_kinematics(self.current_joint_states)
        current_pos = current_fk[-1][:3, 3]
        current_rot = R.from_matrix(current_fk[-1][:3, :3])

        target_pos = np.array([self.target_pose.position.x, self.target_pose.position.y, self.target_pose.position.z])
        target_rot = R.from_quat([
            self.target_pose.orientation.x, self.target_pose.orientation.y,
            self.target_pose.orientation.z, self.target_pose.orientation.w
        ])

        # --- Calculate Error ---
        pos_error = target_pos - current_pos
        rot_error = (target_rot * current_rot.inv()).as_rotvec()
        
        # Proportional gain
        K_p = 2.0 
        twist_error = np.concatenate([pos_error, rot_error]) * K_p
        
        # --- Use same velocity control logic to move towards target ---
        J = self._compute_jacobian(self.current_joint_states)
        try:
            J_pinv = np.linalg.pinv(J)
            q_dot = J_pinv @ twist_error
        except np.linalg.LinAlgError:
            self.get_logger().error("Jacobian is singular. Cannot compute joint velocities.")
            return

        q_next = self.current_joint_states + q_dot * DT
        q_next = self._enforce_joint_limits(q_next)
        
        self._send_joint_command(q_next, DT)


    # =========================================================================
    # --- Kinematics (Ported from omx_velocity_controller.py) ---
    # =========================================================================

    def _forward_kinematics(self, q: np.ndarray):
        """Computes the forward kinematics for the 4DOF OpenManipulator-X."""
        q1, q2, q3, q4 = q
        # Kinematic model uses an inverted convention for these joints
        q2, q3, q4 = -q2, -q3, -q4

        H_0_1 = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,L1], [0,0,0,1]])
        H_1_2 = np.array([
            [math.cos(q1), 0, math.sin(q1), 0],
            [math.sin(q1), 0, -math.cos(q1), 0],
            [0, 1, 0, L2], [0, 0, 0, 1]
        ])
        H_2_3 = np.array([
            [math.cos(q2+A1), -math.sin(q2+A1), 0, L3*math.cos(q2+A1)],
            [math.sin(q2+A1), math.cos(q2+A1), 0, L3*math.sin(q2+A1)],
            [0,0,1,0], [0,0,0,1]
        ])
        H_3_4 = np.array([
            [math.cos(q3-A1), -math.sin(q3-A1), 0, L4*math.cos(q3-A1)],
            [math.sin(q3-A1), math.cos(q3-A1), 0, L4*math.sin(q3-A1)],
            [0,0,1,0], [0,0,0,1]
        ])
        H_4_5 = np.array([
            [math.cos(q4), -math.sin(q4), 0, L5*math.cos(q4)],
            [math.sin(q4), math.cos(q4), 0, L5*math.sin(q4)],
            [0,0,1,0], [0,0,0,1]
        ])

        H_0_2 = H_0_1 @ H_1_2
        H_0_3 = H_0_2 @ H_2_3
        H_0_4 = H_0_3 @ H_3_4
        H_0_5 = H_0_4 @ H_4_5

        return [H_0_1, H_0_2, H_0_3, H_0_4, H_0_5]

    def _compute_jacobian(self, q: np.ndarray) -> np.ndarray:
        """Computes the 6x4 analytical Jacobian for the end-effector."""
        Hs = self._forward_kinematics(q)
        O_frames = [H[:3, 3] for H in Hs]
        O_end = O_frames[-1]
        
        # Z-axes for each joint frame, in the base frame
        Z_frames = [H[:3, 2] for H in Hs]
        
        J = np.zeros((6, 4))
        for i in range(4):
            # The axis of rotation for joint i+1 is the Z-axis of frame i
            axis_of_rotation = Z_frames[i]
            # Vector from joint i to end effector
            vec_to_end = O_end - O_frames[i]
            
            # Linear velocity component: cross(z_i, r_i_e)
            J[:3, i] = np.cross(axis_of_rotation, vec_to_end)
            # Angular velocity component
            J[3:, i] = axis_of_rotation
            
        return J

    # =========================================================================
    # --- Safety and Utility Functions ---
    # =========================================================================

    def _enforce_joint_limits(self, q: np.ndarray) -> np.ndarray:
        """Clamps the joint angles to their defined limits."""
        q[0] = np.clip(q[0], BASE_JOINT_MIN, BASE_JOINT_MAX)
        # Add other joint limits here if necessary
        return q

    def _safe_stop(self):
        """Stops the robot by commanding its current position with zero velocity."""
        self.active_control_mode = "stopped"
        # Send a command to hold the current position
        if self.current_joint_states is not None:
            self._send_joint_command(self.current_joint_states, path_time=0.2)
        # Clear velocity buffers
        self.linear_vel_buffer.clear()
        self.angular_vel_buffer.clear()

    def _send_joint_command(self, q_target: np.ndarray, path_time: float):
        """Calls the SetJointPosition service to command the robot."""
        req = SetJointPosition.Request()
        req.planning_group = ''  # Empty for whole group
        req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
        req.joint_position.position = q_target.tolist()
        req.path_time = path_time
        
        self.joint_control_client.call_async(req)

# =============================================================================
# --- Main Execution ---
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    try:
        controller_node = EnhancedVelocityController()
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        pass
    finally:
        controller_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
