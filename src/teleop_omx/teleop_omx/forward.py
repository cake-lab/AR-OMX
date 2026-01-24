import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
import math
import numpy as np
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R
from my_service_package.msg import RotPos
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Forward(Node):
    def __init__(self):
        super().__init__('Forward')

        # Link lengths
        self.L1 = 36.076
        self.L2 = 60.25
        self.L3 = 130.23
        self.L4 = 124
        self.L5 = 133.4
        self.get_logger().info(f'Lengths are [{self.L1}, {self.L2}, {self.L3}, {self.L4}, {self.L5}]')

        # Subscription to joint states
        self.create_subscription(JointState, '/joint_states', self.forward_callback, 10)

        # Publisher for forward kinematics results
        self.publisher_ = self.create_publisher(RotPos, 'Fwd_op', 10)

        # Initialize matplotlib figures
        self.fig = plt.figure(figsize=(10, 8))
        self.ax3d = self.fig.add_subplot(221, projection='3d')
        self.ax_xy = self.fig.add_subplot(222)
        self.ax_yz = self.fig.add_subplot(223)
        self.ax_xz = self.fig.add_subplot(224)

        # Data for plots
        self.positions = []

    def forward_callback(self, msg):
        """Calculate forward kinematics and publish rotation and position."""
        # Lengths of the links
        l1 = self.L1
        l2 = self.L2
        l3 = self.L3
        l4 = self.L4
        l5 = self.L5

        # The joint angles
        q1 = msg.position[0]
        q2 = -msg.position[1]
        q3 = -msg.position[2]
        q4 = -msg.position[3]

        # The constant angle at start
        A1 = math.radians(79.38)

        # Homogeneous transformation matrices
        H_0_1 = np.array([[1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, l1],
                          [0, 0, 0, 1]])

        H_1_2 = np.array([[math.cos(q1), 0, math.sin(q1), 0],
                          [math.sin(q1), 0, -math.cos(q1), 0],
                          [0, 1, 0, l2],
                          [0, 0, 0, 1]])

        H_2_3 = np.array([[math.cos(q2 + A1), -math.sin(q2 + A1), 0, l3 * math.cos(q2 + A1)],
                          [math.sin(q2 + A1), math.cos(q2 + A1), 0, l3 * math.sin(q2 + A1)],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])

        H_3_4 = np.array([[math.cos(q3 - A1), -math.sin(q3 - A1), 0, l4 * math.cos(q3 - A1)],
                          [math.sin(q3 - A1), math.cos(q3 - A1), 0, l4 * math.sin(q3 - A1)],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])

        H_4_5 = np.array([[math.cos(q4), -math.sin(q4), 0, l5 * math.cos(q4)],
                          [math.sin(q4), math.cos(q4), 0, l5 * math.sin(q4)],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])

        # Combine transformations
        H_0_2 = np.dot(H_0_1, H_1_2)
        H_0_3 = np.dot(H_0_2, H_2_3)
        H_0_4 = np.dot(H_0_3, H_3_4)
        H_0_5 = np.dot(H_0_4, H_4_5)

        # Extract rotation and position
        R_0_5 = H_0_5[0:3, 0:3]
        O_5 = H_0_5[0:3, 3]

        for i in range(0, H_0_5.shape[0]):
            for j in range(0, H_0_5.shape[1]):
                H_0_5[i][j] = 0 if abs(H_0_5[i][j]) < 10e-2 else H_0_5[i][j]

        # self.get_logger().info(f'The end effector pose is \n{H_0_5}')
        # self.get_logger().info(f'The end effector pose is \n{self.R_0_5}')
        
        # Convert rotation matrix to quaternion
        rotation = R.from_matrix(R_0_5)
        quaternion = rotation.as_quat()  # Returns [qx, qy, qz, qw]
        
        # self.get_logger().info(f'The end effector pose is Position = {self.O_5}, Quaternions = {quaternion}')

        # Append position to list
        self.positions.append(O_5)

        # Plot the results
        self.plot_positions()

        
        # Build rotation and position arrays
        rot = np.array([H_0_1[0:3, 0:3], H_0_2[0:3, 0:3], H_0_3[0:3, 0:3], H_0_4[0:3, 0:3]])
        pos = np.array([H_0_1[0:3, 3], H_0_2[0:3, 3], H_0_3[0:3, 3], H_0_4[0:3, 3], O_5])

        # Publish results
        msg = RotPos()
        msg.r = list(rot.flatten())
        msg.o = list(pos.flatten())
        self.publisher_.publish(msg)

        self.get_logger().info(f'Published End Effector Co-ordinates: {O_5}')

    def plot_positions(self):
        """Plot the 3D and 2D projections of the end-effector positions."""
        # Clear previous plots
        self.ax3d.clear()
        self.ax_xy.clear()
        self.ax_yz.clear()
        self.ax_xz.clear()

        # Extract coordinates
        x = [pos[0] for pos in self.positions]
        y = [pos[1] for pos in self.positions]
        z = [pos[2] for pos in self.positions]

        # Plot 3D trajectory
        self.ax3d.plot(x, y, z, marker='o')
        self.ax3d.set_title('3D Trajectory')
        self.ax3d.set_xlabel('X')
        self.ax3d.set_ylabel('Y')
        self.ax3d.set_zlabel('Z')

        # Plot 2D projections
        self.ax_xy.plot(x, y, marker='o')
        self.ax_xy.set_title('XY Projection')
        self.ax_xy.set_xlabel('X')
        self.ax_xy.set_ylabel('Y')

        self.ax_yz.plot(y, z, marker='o')
        self.ax_yz.set_title('YZ Projection')
        self.ax_yz.set_xlabel('Y')
        self.ax_yz.set_ylabel('Z')

        self.ax_xz.plot(x, z, marker='o')
        self.ax_xz.set_title('XZ Projection')
        self.ax_xz.set_xlabel('X')
        self.ax_xz.set_ylabel('Z')

        # Show the plot
        plt.pause(0.01)

def main():
    rclpy.init()
    node = Forward()
    try:
        plt.ion()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.show()


if __name__ == '__main__':
    main()
