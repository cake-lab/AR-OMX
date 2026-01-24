"""

Computes the forward kinematics of the OpenManipulator-X using incoming joint states and publishes the resulting  end-effector pose 
as a PoseStamped message on the `/actual_pose` topic. 
The computation is performed using homogeneous transformation matrices and converted to a quaternion representation for orientation.

"""

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
from scipy.spatial.transform import Rotation as R

class ForwardKinematicsNode(Node):
    # Calculates forward kinematics from joint states and publishes the end-effector pose as a PoseStamped message to the /actual_pose topic.
 
    def __init__(self):
        super().__init__('forward_kinematics_node')

        # Link lengths in METERS (converted from mm)
        self.L1 = 0.036076
        self.L2 = 0.06025
        self.L3 = 0.13023
        self.L4 = 0.124
        self.L5 = 0.1334
        self.get_logger().info(
            f'FK Node started with link lengths (m): '
            f'[{self.L1}, {self.L2}, {self.L3}, {self.L4}, {self.L5}]'
        )

        # Constant angle from your model
        self.A1 = math.radians(79.38)

        # Subscription to joint states from the robot/simulation
        self.create_subscription(JointState, '/joint_states', self.forward_callback, 10)

        # Publisher for the actual end-effector pose
        self.pose_publisher = self.create_publisher(PoseStamped, '/actual_pose', 10)
        
        self.get_logger().info("Publishing end-effector pose to /actual_pose")

    def forward_callback(self, msg: JointState):
        # Calculates FK from joint states and publishes the end-effector pose.
        
        if len(msg.position) < 4:
            self.get_logger().warn("JointState message has < 4 positions. Skipping.", throttle_duration_sec=5)
            return

        # Assign joint angles using your convention
        q1 = msg.position[0]
        q2 = -msg.position[1]
        q3 = -msg.position[2]
        q4 = -msg.position[3]

        # Homogeneous transformation matrices (using instance variables for lengths)
        H_0_1 = np.array([[1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, self.L1],
                          [0, 0, 0, 1]])

        H_1_2 = np.array([[math.cos(q1), 0, math.sin(q1), 0],
                          [math.sin(q1), 0, -math.cos(q1), 0],
                          [0, 1, 0, self.L2],
                          [0, 0, 0, 1]])

        H_2_3 = np.array([[math.cos(q2 + self.A1), -math.sin(q2 + self.A1), 0, self.L3 * math.cos(q2 + self.A1)],
                          [math.sin(q2 + self.A1), math.cos(q2 + self.A1), 0, self.L3 * math.sin(q2 + self.A1)],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])

        H_3_4 = np.array([[math.cos(q3 - self.A1), -math.sin(q3 - self.A1), 0, self.L4 * math.cos(q3 - self.A1)],
                          [math.sin(q3 - self.A1), math.cos(q3 - self.A1), 0, self.L4 * math.sin(q3 - self.A1)],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])

        H_4_5 = np.array([[math.cos(q4), -math.sin(q4), 0, self.L5 * math.cos(q4)],
                          [math.sin(q4), math.cos(q4), 0, self.L5 * math.sin(q4)],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])
        
        # Calculate the final transformation matrix
        H_0_5 = H_0_1 @ H_1_2 @ H_2_3 @ H_3_4 @ H_4_5

        # Extract position vector and rotation matrix
        position = H_0_5[0:3, 3]
        rotation_matrix = H_0_5[0:3, 0:3]

        # Convert rotation matrix to quaternion
        quat = R.from_matrix(rotation_matrix).as_quat() # [x, y, z, w]

        # Create and publish the PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link'  # Or your robot's base frame

        pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z = position
        pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = quat

        self.pose_publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
