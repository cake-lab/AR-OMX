import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import numpy as np

class ForwardKinematicsNode(Node):
    """
    A dedicated node that subscribes to the robot's /joint_states,
    calculates the forward kinematics using DH parameters, and publishes
    the resulting end-effector pose to the /actual_pose topic.
    """
    def __init__(self):
        super().__init__('forward_kinematics_node')
        self.publisher_ = self.create_publisher(Pose, '/actual_pose', 10)
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        self.get_logger().info('Forward Kinematics Node is running.')

    def joint_state_callback(self, msg):
        """Callback to calculate and publish FK when a new joint state is received."""
        # Ensure the message has enough positions, assuming order is joint1-4
        if len(msg.position) < 4:
            return
            
        Q1, Q2, Q3, Q4 = msg.position[:4]
        
        # DH parameters
        alphaa = [-np.pi/2, 0, 0, np.pi/2]
        a = [0, 0.13023, 0.124, 0.1334]
        d = [0.06025, 0, 0, 0]
        Q = [Q1, Q2 - np.pi/2 + np.radians(10.6197), Q3 + np.pi/2  - np.radians(10.6197), Q4]

        # Create transformation matrices
        T = np.identity(4)
        for i in range(4):
            Ti = np.array([
                [np.cos(Q[i]), -np.sin(Q[i])*np.cos(alphaa[i]),  np.sin(Q[i])*np.sin(alphaa[i]), a[i]*np.cos(Q[i])],
                [np.sin(Q[i]),  np.cos(Q[i])*np.cos(alphaa[i]), -np.cos(Q[i])*np.sin(alphaa[i]), a[i]*np.sin(Q[i])],
                [0,             np.sin(alphaa[i]),               np.cos(alphaa[i]),              d[i]],
                [0,             0,                               0,                              1]
            ])
            T = np.matmul(T, Ti)

        self.publish_pose(T)

    def publish_pose(self, T_matrix):
        """Extracts pose from a transformation matrix and publishes it."""
        pose_msg = Pose()
        pose_msg.position.x = T_matrix[0, 3]
        pose_msg.position.y = T_matrix[1, 3]
        pose_msg.position.z = T_matrix[2, 3]
        
        # Calculate orientation (quaternion) from rotation matrix
        rot_matrix = T_matrix[:3, :3]
        tr = np.trace(rot_matrix)
        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2
            qw = 0.25 * S
            qx = (rot_matrix[2, 1] - rot_matrix[1, 2]) / S
            qy = (rot_matrix[0, 2] - rot_matrix[2, 0]) / S
            qz = (rot_matrix[1, 0] - rot_matrix[0, 1]) / S
        elif (rot_matrix[0, 0] > rot_matrix[1, 1]) and (rot_matrix[0, 0] > rot_matrix[2, 2]):
            S = np.sqrt(1.0 + rot_matrix[0, 0] - rot_matrix[1, 1] - rot_matrix[2, 2]) * 2
            qw = (rot_matrix[2, 1] - rot_matrix[1, 2]) / S
            qx = 0.25 * S
            qy = (rot_matrix[0, 1] + rot_matrix[1, 0]) / S
            qz = (rot_matrix[0, 2] + rot_matrix[2, 0]) / S
        elif rot_matrix[1, 1] > rot_matrix[2, 2]:
            S = np.sqrt(1.0 + rot_matrix[1, 1] - rot_matrix[0, 0] - rot_matrix[2, 2]) * 2
            qw = (rot_matrix[0, 2] - rot_matrix[2, 0]) / S
            qx = (rot_matrix[0, 1] + rot_matrix[1, 0]) / S
            qy = 0.25 * S
            qz = (rot_matrix[1, 2] + rot_matrix[2, 1]) / S
        else:
            S = np.sqrt(1.0 + rot_matrix[2, 2] - rot_matrix[0, 0] - rot_matrix[1, 1]) * 2
            qw = (rot_matrix[1, 0] - rot_matrix[0, 1]) / S
            qx = (rot_matrix[0, 2] + rot_matrix[2, 0]) / S
            qy = (rot_matrix[1, 2] + rot_matrix[2, 1]) / S
            qz = 0.25 * S
            
        pose_msg.orientation.x = qx
        pose_msg.orientation.y = qy
        pose_msg.orientation.z = qz
        pose_msg.orientation.w = qw
        
        self.publisher_.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    fk_node = ForwardKinematicsNode()
    rclpy.spin(fk_node)
    fk_node.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
