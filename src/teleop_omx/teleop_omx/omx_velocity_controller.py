#!/usr/bin/env python3
"""
omx_velocity_controller.py

A ROS 2 node that loads a CSV of (timestamp, vx, vy, vz), interpolates to get
desired Cartesian velocity at each control tick, computes joint velocities via
pseudo-inverse Jacobian, integrates to get q_next, and commands the
OpenManipulator-X via SetJointPosition every DT.

Critical fixes applied:
 1. Removed open-loop state update: self.current_q is no longer overwritten by
    commanded q_next; it is updated only by the subscriber callback (_joint_state_cb).
 2. Robust Phase 1 startup: move to initial joint angles over 2 seconds and wait
    2.5s to ensure settled starting pose before high-frequency tracking.
"""

import sys
import time
import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from open_manipulator_msgs.srv import SetJointPosition

# -----------------------------------------------------------------------------
# Tunable constants
# -----------------------------------------------------------------------------
CONTROL_FREQUENCY_HZ = 100             # control loop frequency
DT = 1.0 / CONTROL_FREQUENCY_HZ        # time step

# Robot link lengths (from forward.py)
L1, L2, L3, L4, L5 = 36.076, 60.25, 130.23, 124.0, 133.4  # mm
# Fixed constant angle (deg → rad)
A1 = math.radians(79.38)

class OMVelocityController(Node):
    def __init__(self, csv_path: str):
        super().__init__('omx_velocity_controller')

        # -- state holders --
        self.current_q = None        # updated only by _joint_state_cb

        # Load trajectory CSV
        self._load_csv(csv_path)

        # Subscribe to joint_states for feedback
        self.create_subscription(
            JointState, '/joint_states',
            self._joint_state_cb, 10
        )

        # Client for commanding joint positions
        self.client = self.create_client(
            SetJointPosition,
            'goal_joint_space_path'
        )
        self.get_logger().info('Waiting for SetJointPosition service...')
        while rclpy.ok() and not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('  service unavailable, retrying...')
        self.get_logger().info('SetJointPosition service ready')

        # Await first joint_state message
        while rclpy.ok() and self.current_q is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'Initial joint_state: {self.current_q}')

    def _load_csv(self, path: str):
        """Load timestamped vx,vy,vz and build zero-based time vector."""
        try:
            data = np.loadtxt(path, delimiter=',', skiprows=1)
            stamps = data[:, 0]
            self.vx = data[:, 1]
            self.vy = data[:, 2]
            self.vz = data[:, 3]
            t0 = stamps[0]
            self.times = stamps - t0
            self.duration = self.times[-1]
            self.get_logger().info(
                f'Loaded CSV "{path}", duration={self.duration:.3f}s, '
                f'{len(self.times)} samples'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to load CSV "{path}": {e}')
            sys.exit(1)

    def _joint_state_cb(self, msg: JointState):
        """Update current_q from real sensor feedback only."""
        q = np.array(msg.position[0:4], dtype=float)
        self.current_q = q

    def _forward_kinematics(self, q: np.ndarray):
        """Compute homogeneous transforms H_0_i for i=1..5 given q."""
        q1, q2, q3, q4 = q
        # forward.py convention: flip signs on q2,q3,q4
        q2, q3, q4 = -q2, -q3, -q4

        H_0_1 = np.array([
            [1, 0,          0,         0],
            [0, 1,          0,         0],
            [0, 0,          1,        L1],
            [0, 0,          0,         1],
        ])
        H_1_2 = np.array([
            [ math.cos(q1), 0,  math.sin(q1), 0],
            [ math.sin(q1), 0, -math.cos(q1), 0],
            [ 0,            1,  0,            L2],
            [ 0,            0,  0,            1 ],
        ])
        H_2_3 = np.array([
            [ math.cos(q2+A1), -math.sin(q2+A1), 0, L3*math.cos(q2+A1)],
            [ math.sin(q2+A1),  math.cos(q2+A1), 0, L3*math.sin(q2+A1)],
            [ 0,                0,               1, 0               ],
            [ 0,                0,               0, 1               ],
        ])
        H_3_4 = np.array([
            [ math.cos(q3-A1), -math.sin(q3-A1), 0, L4*math.cos(q3-A1)],
            [ math.sin(q3-A1),  math.cos(q3-A1), 0, L4*math.sin(q3-A1)],
            [ 0,                0,               1, 0               ],
            [ 0,                0,               0, 1               ],
        ])
        H_4_5 = np.array([
            [ math.cos(q4), -math.sin(q4), 0, L5*math.cos(q4)],
            [ math.sin(q4),  math.cos(q4), 0, L5*math.sin(q4)],
            [ 0,             0,            1, 0            ],
            [ 0,             0,            0, 1            ],
        ])

        H_0_2 = H_0_1 @ H_1_2
        H_0_3 = H_0_2 @ H_2_3
        H_0_4 = H_0_3 @ H_3_4
        H_0_5 = H_0_4 @ H_4_5

        return [H_0_1, H_0_2, H_0_3, H_0_4, H_0_5]

    def _compute_jacobian(self, q: np.ndarray) -> np.ndarray:
        """Build 6×4 Jacobian from forward kinematics frames."""
        Hs = self._forward_kinematics(q)
        Rs = [H[0:3, 0:3] for H in Hs[:-1]]
        Os = [H[0:3, 3].reshape(3,1) for H in Hs]
        O_end = Os[-1]
        k = np.array([[0],[0],[1]], dtype=float)

        v_cols, w_cols = [], []
        for i in range(len(Rs)):
            z_i = (Rs[i] @ k).flatten()
            o_i = Os[i].flatten()
            v_i = np.cross(z_i, (O_end.flatten() - o_i))
            v_cols.append(v_i.reshape(3,1))
            w_cols.append(z_i.reshape(3,1))
        V = np.hstack(v_cols)
        W = np.hstack(w_cols)
        J = np.vstack((V, W))
        return J

    def _send_joint_positions(self, q_target: np.ndarray, path_time: float):
        """Call SetJointPosition with q_target (4-vector)."""
        req = SetJointPosition.Request()
        req.planning_group = ''
        req.joint_position.joint_name = ['joint1','joint2','joint3','joint4']
        req.joint_position.position = list(q_target)
        req.path_time = path_time
        self.client.call_async(req)

    def run(self):
        """
        Phase 1: Move to a known starting configuration.
        Phase 2: High-frequency real-time velocity tracking.
        """
        # --- Phase 1: Move to start pose using actual feedback ---
        self.get_logger().info(
            'Phase 1: Moving to initial joint configuration to begin trajectory.'
        )
        q_start_of_trajectory = self.current_q.copy()
        # send a longer-duration move to ensure robot is at known start
        self._send_joint_positions(q_start_of_trajectory, path_time=2.0)
        time.sleep(2.5)  # wait longer than path_time to ensure settle
        self.get_logger().info('Ready to track.')

        # --- Phase 2: high-frequency real-time velocity tracking ---
        self.get_logger().info('Phase 2: starting real-time loop')
        t_start = time.monotonic()
        while rclpy.ok():
            t = time.monotonic() - t_start
            if t > self.duration:
                break

            # handle incoming joint_states to update self.current_q
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.current_q is None:
                continue

            # interpolate desired Cartesian velocities
            vx_d = np.interp(t, self.times, self.vx)
            vy_d = np.interp(t, self.times, self.vy)
            vz_d = np.interp(t, self.times, self.vz)

            # compute velocity IK
            J = self._compute_jacobian(self.current_q)
            twist = np.array([vx_d, vy_d, vz_d, 0, 0, 0], dtype=float)
            q_dot = np.linalg.pinv(J) @ twist.reshape(6,1)
            q_next = (self.current_q + (q_dot.flatten() * DT))

            # command next joint position (no internal state overwrite)
            self._send_joint_positions(q_next, DT)
            # CRITICAL FIX: do NOT assign self.current_q = q_next here;
            # rely on subscriber feedback to update current_q.

            # maintain loop timing
            elapsed = (time.monotonic() - (t_start + t))
            to_sleep = DT - elapsed
            if to_sleep > 0:
                time.sleep(to_sleep)

        self.get_logger().info('Trajectory complete; shutting down.')


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 2:
        print('Usage: omx_velocity_controller.py <path_to_vel_log.csv>')
        sys.exit(1)

    node = OMVelocityController(sys.argv[1])
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
