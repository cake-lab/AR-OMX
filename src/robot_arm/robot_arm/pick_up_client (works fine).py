import rclpy
from rclpy.node import Node
from tutorial_interfaces.srv import CalculateIK
from open_manipulator_msgs.srv import SetJointPosition
from geometry_msgs.msg import Pose # Import Pose for the subscriber
import sys
import csv
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

class HandTrackingClient(Node):
    def __init__(self):
        super().__init__('hand_tracking_client')
        
        # Service clients
        self.ik_client = self.create_client(CalculateIK, 'calculate_ik')
        self.joint_position_client = self.create_client(SetJointPosition, 'goal_joint_space_path')

        # Subscriber to the /actual_pose topic from our new FK node
        self.actual_pose_sub = self.create_subscription(
            Pose,
            '/actual_pose',
            self.actual_pose_callback,
            10)
        
        # Data logging lists
        self.commanded_trajectory = []
        self.actual_trajectory = []
        self.is_recording = False

        self.get_logger().info('Waiting for services...')
        if not self.ik_client.wait_for_service(timeout_sec=5.0) or \
           not self.joint_position_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('One or more services not available. Exiting.')
            sys.exit(0)
        self.get_logger().info('All services are available.')

    def actual_pose_callback(self, msg):
        """Callback function for the /actual_pose subscriber."""
        if self.is_recording:
            self.actual_trajectory.append({'x': msg.position.x, 'y': msg.position.y, 'z': msg.position.z})

    def execute_smooth_trajectory(self, file_path):
        self.get_logger().info(f"Reading and processing trajectory from: {file_path}")
        
        try:
            with open(file_path, mode='r', newline='') as csvfile:
                points_data = list(csv.reader(csvfile))
                header = points_data.pop(0)
        except FileNotFoundError:
            self.get_logger().error(f"Error: The file '{file_path}' was not found.")
            return

        # 1. Pre-calculate all IK solutions and log commanded trajectory
        self.get_logger().info("Pre-calculating all IK solutions...")
        joint_commands = []
        previous_timestamp = None
        for row in points_data:
            if len(row) != 8: continue
            try:
                timestamp, x, y, z, qx, qy, qz, qw = [float(val) for val in row]
                self.commanded_trajectory.append({'x': x, 'y': y, 'z': z})
            except ValueError: continue

            ik_future = self.ik_client.call_async(CalculateIK.Request(x=x, y=y, z=z, qx=qx, qy=qy, qz=qz, qw=qw))
            rclpy.spin_until_future_complete(self, ik_future)
            ik_response = ik_future.result()

            if ik_response:
                delta_t = 2.0 if previous_timestamp is None else timestamp - previous_timestamp
                joint_commands.append({"thetas": [ik_response.theta1, ik_response.theta2, ik_response.theta3, ik_response.theta4], "path_time": max(delta_t, 0.1)})
                previous_timestamp = timestamp
        
        if not joint_commands: return

        # 2. MODIFIED: Move to the starting position FIRST without recording
        self.get_logger().info("Moving to the starting position...")
        first_cmd = joint_commands.pop(0) # Get and remove the first command
        req = SetJointPosition.Request()
        req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
        req.joint_position.position = first_cmd["thetas"]
        req.path_time = 2.5  # Use a fixed, reasonable time to get to the start
        self.joint_position_client.call_async(req)
        time.sleep(3.0) # Wait for the robot to arrive at the start position

        # 3. MODIFIED: Start recording and then send the rest of the commands
        self.get_logger().info("Starting trajectory recording and sending command stream...")
        self.is_recording = True
        
        for cmd in joint_commands:
            req = SetJointPosition.Request()
            req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
            req.joint_position.position = cmd["thetas"]
            req.path_time = cmd["path_time"]
            self.joint_position_client.call_async(req)
            time.sleep(cmd["path_time"] * 0.5)
            rclpy.spin_once(self, timeout_sec=0.01)

        time.sleep(2.0)
        self.is_recording = False
        self.get_logger().info("--- Finished trajectory. Preparing plot. ---")
        
        # 4. Plot the results
        self.plot_trajectories()

    def plot_trajectories(self):
        """Uses matplotlib to create a 3D plot of the commanded vs actual trajectories."""
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')

        # Commanded Trajectory data
        if self.commanded_trajectory:
            cx = [p['x'] for p in self.commanded_trajectory]
            cy = [p['y'] for p in self.commanded_trajectory]
            cz = [p['z'] for p in self.commanded_trajectory]
            ax.plot(cx, cy, cz, label='Commanded Trajectory', color='b', linestyle='-')
            # MODIFIED: Plot start (green) and end (red) points
            ax.scatter(cx[0], cy[0], cz[0], color='g', s=100, label='Start Point', zorder=5)
            ax.scatter(cx[-1], cy[-1], cz[-1], color='r', s=100, label='End Point', zorder=5)

        # Actual Trajectory data
        if len(self.actual_trajectory) > 1:
            unique_points = sorted(list(set(tuple(p.items()) for p in self.actual_trajectory)))
            ax_pts = [{k: v for k, v in p} for p in unique_points]
            ax_x = [p['x'] for p in ax_pts]
            ax_y = [p['y'] for p in ax_pts]
            ax_z = [p['z'] for p in ax_pts]
            ax.plot(ax_x, ax_y, ax_z, label='Actual Trajectory', color='m', linestyle='--')

        ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
        ax.set_title('Commanded vs. Actual Robot Trajectory'); ax.legend(); ax.grid(True)
        
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    hand_tracking_client = HandTrackingClient()
    
    if len(sys.argv) < 2:
        print("Usage: python hand_tracking_client.py <path_to_csv_file>")
        sys.exit(1)
        
    csv_file_path = sys.argv[1]

    try:
        hand_tracking_client.execute_smooth_trajectory(csv_file_path)
    except KeyboardInterrupt:
        hand_tracking_client.get_logger().info('Program interrupted by user.')
    finally:
        hand_tracking_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
