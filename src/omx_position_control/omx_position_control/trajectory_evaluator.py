"""

Records and visualizes end-effector trajectories by time-synchronizing a commanded target pose with the robot’s actual kinematics pose.
Subscribes to `/target_pose` (PoseStamped) and `/kinematics_pose` (KinematicsPose), logs synchronized 3D position data, and stores it in memory during runtime. 
Upon shutdown, the collected data is saved to a CSV file and displayed as a comparative 3D trajectory plot.

Intended for offline analysis and visual inspection of trajectory tracking accuracy in robotic manipulation experiments.

"""

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from open_manipulator_msgs.msg import KinematicsPose
import message_filters
import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import threading

class TrajectoryPlotterNode(Node):
    # A ROS 2 node that synchronizes two distinct pose topics, logs their data, and generates a comparative 3D plot upon termination.
    def __init__(self):
        super().__init__('trajectory_plotter_node')
        self.get_logger().info('Trajectory Plotter Node started.')
        # --- MODIFIED: Updated log message to reflect the change ---
        self.get_logger().info('Synchronizing topics: /target_pose, /kinematics_pose')
        self.get_logger().info('Waiting for messages... (This may take a moment)')

        # --- Data Storage ---
        self.trajectory_data = []
        self.data_lock = threading.Lock()
        self.callback_count = 0

        # --- Subscribers ---
        target_sub = message_filters.Subscriber(self, PoseStamped, '/target_pose')
        # --- MODIFIED: Subscriber for /actual_pose has been removed ---
        # actual_sub = message_filters.Subscriber(self, PoseStamped, '/actual_pose')
        kinematics_sub = message_filters.Subscriber(self, KinematicsPose, '/kinematics_pose')

        # --- Time Synchronizer ---
        # --- MODIFIED: Synchronizer now only handles two topics ---
        self.time_synchronizer = message_filters.ApproximateTimeSynchronizer(
            [target_sub, kinematics_sub],
            queue_size=30,
            slop=0.2
        )
        self.time_synchronizer.registerCallback(self.sync_callback)

    def sync_callback(self, target_msg: PoseStamped, kinematic_msg: KinematicsPose):
        # This callback is executed only when a synchronized set of messages is received from the active topics.
        
        timestamp = target_msg.header.stamp.sec + target_msg.header.stamp.nanosec * 1e-9
        target_pos = target_msg.pose.position
        kinematic_pos = kinematic_msg.pose.position

        with self.data_lock:
            # --- MODIFIED: Data structure now only includes target and kinematics ---
            self.trajectory_data.append([
                timestamp,
                target_pos.x, target_pos.y, target_pos.z,
                kinematic_pos.x, kinematic_pos.y, kinematic_pos.z
            ])
            self.callback_count += 1

        if self.callback_count == 1:
            self.get_logger().info("First synchronized message received! Now logging data...")
        elif self.callback_count % 50 == 0:
            self.get_logger().info(f"Successfully synchronized and logged {self.callback_count} data points.")

    def save_and_plot(self):
        # Handles the final data processing steps: saving to CSV and plotting.
        # Called once upon node shutdown.
       
        with self.data_lock:
            data = list(self.trajectory_data)

        if not data:
            self.get_logger().warn("No synchronized data was collected. Cannot save or plot.")
            self.get_logger().warn("Please ensure both topics (/target_pose, /kinematics_pose) are publishing data.")
            return

        self.save_to_csv(data)
        self.plot_trajectories(data)

    def save_to_csv(self, data, filename="trajectory_log_2-way.csv"):
        # Saves the collected trajectory data to a CSV file.
        self.get_logger().info(f"Saving {len(data)} data points to {filename}...")
        # --- MODIFIED: Header now only includes the two active topics ---
        header = [
            'timestamp',
            'target_x', 'target_y', 'target_z',
            'kinematic_x', 'kinematic_y', 'kinematic_z'
        ]
        try:
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(header)
                writer.writerows(data)
            self.get_logger().info("Data saved successfully.")
        except IOError as e:
            self.get_logger().error(f"Failed to save CSV file: {e}")

    def plot_trajectories(self, data):
        # Generates and displays a 3D plot of the two trajectories.
        self.get_logger().info("Generating 3D trajectory plot...")
        data_array = np.array(data)

        fig = plt.figure(figsize=(12, 9))
        ax = fig.add_subplot(111, projection='3d')

        ax.plot(data_array[:, 1], data_array[:, 2], data_array[:, 3], 'o-', color='blue', label='/target_pose (Commanded)', markersize=2)
        # --- MODIFIED: Plot for /actual_pose has been removed ---
        # ax.plot(data_array[:, 4], data_array[:, 5], data_array[:, 6], '^-', color='red', label='/actual_pose (FK)', markersize=2)
        
        # --- MODIFIED: Indexing for kinematics data is now 4, 5, 6 ---
        ax.plot(data_array[:, 4], data_array[:, 5], data_array[:, 6], 'x-', color='green', label='/kinematics_pose (Controller)', markersize=2)

        ax.set_title('Trajectory Comparison')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.legend()
        ax.grid(True)
        
        plt.tight_layout()
        
        self.get_logger().info("Displaying plot. Close the plot window to exit completely.")
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt detected, shutting down.')
    finally:
        node.get_logger().info('Performing final data saving and plotting...')
        
        node.save_and_plot()
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

