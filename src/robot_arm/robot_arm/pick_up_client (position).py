import rclpy
from rclpy.node import Node
from tutorial_interfaces.srv import CalculateIK
from open_manipulator_msgs.srv import SetJointPosition
import sys
import csv
import time

class HandTrackingClient(Node):
    """
    A ROS2 node to control the OMX robot based on hand tracking data from a CSV file.
    """
    def __init__(self):
        """
        Initializes the node, creates service clients, and waits for services to become available.
        """
        super().__init__('hand_tracking_client')
        
        # Client for the inverse kinematics service
        self.ik_client = self.create_client(CalculateIK, 'calculate_ik')
        
        # Client for setting the robot's joint positions
        self.joint_position_client = self.create_client(SetJointPosition, 'goal_joint_space_path')
        
        # Client for controlling the gripper
        self.tool_control_client = self.create_client(SetJointPosition, 'goal_tool_control')

        # Wait for all services to be ready
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Inverse Kinematics service not available. Exiting.')
            sys.exit(0)
        if not self.joint_position_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Joint Position service not available. Exiting.')
            sys.exit(0)
        if not self.tool_control_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Tool Control service not available. Exiting.')
            sys.exit(0)
            
        self.get_logger().info('All services are available. Ready to process movements.')

    def send_ik_request(self, x, y, z, qx, qy, qz, qw):
        """
        Sends a request to the inverse kinematics service to calculate joint angles.

        Args:
            x, y, z (float): The target cartesian coordinates.
            qx, qy, qz, qw (float): The target orientation as a quaternion.
        
        Returns:
            The service response containing the calculated joint angles (theta1, theta2, theta3, theta4).
        """
        request = CalculateIK.Request()
        # The values are now guaranteed to be floats from the calling function.
        request.x = x
        request.y = y
        request.z = z
        request.qx = qx
        request.qy = qy
        request.qz = qz
        request.qw = qw

        # Asynchronously call the service
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            # This f-string now works because x, y, z are floats.
            self.get_logger().info(f'Successfully calculated IK for point ({x:.3f}, {y:.3f}, {z:.3f})')
            return future.result()
        else:
            self.get_logger().error(f'Failed to call IK service for point ({x:.3f}, {y:.3f}, {z:.3f})')
            return None

    def move_robot_to_position(self, theta1, theta2, theta3, theta4):
        """
        Commands the robot to move to the specified joint angles.

        Args:
            theta1, theta2, theta3, theta4 (float): The target joint angles in radians.
        """
        request = SetJointPosition.Request()
        request.planning_group = '' # Use default planning group
        request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
        request.joint_position.position = [theta1, theta2, theta3, theta4]
        request.path_time = 2.0  # Time in seconds to reach the target position

        future = self.joint_position_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Successfully sent joint position command.')
        else:
            self.get_logger().error('Failed to call joint position service.')

    def process_csv_movements(self, file_path):
        """
        Reads a CSV file and sends each point to the robot.

        Args:
            file_path (str): The full path to the CSV file.
        """
        self.get_logger().info(f"Reading hand movements from: {file_path}")
        try:
            with open(file_path, mode='r', newline='') as csvfile:
                csv_reader = csv.reader(csvfile)
                # Skip header row
                header = next(csv_reader)
                self.get_logger().info(f"CSV Header: {header}")

                for i, row in enumerate(csv_reader):
                    if len(row) != 7:
                        self.get_logger().warning(f"Skipping malformed row {i+1}: {row}")
                        continue
                    
                    self.get_logger().info(f"--- Processing point {i+1} ---")
                    
                    # *** FIX: Convert all values from the row to float. ***
                    try:
                        x, y, z, qx, qy, qz, qw = [float(val) for val in row]
                    except ValueError as e:
                        self.get_logger().error(f"Could not convert row {i+1} to floats. Skipping. Error: {e}. Row: {row}")
                        continue
                    
                    # 1. Calculate Inverse Kinematics
                    ik_response = self.send_ik_request(x, y, z, qx, qy, qz, qw)
                    
                    if ik_response:
                        # 2. Move robot to the calculated position
                        self.move_robot_to_position(
                            ik_response.theta1,
                            ik_response.theta2,
                            ik_response.theta3,
                            ik_response.theta4
                        )
                        # Wait for the robot to physically move
                        time.sleep(2.5) # Adjust this delay as needed for smoother motion
                
                self.get_logger().info("--- Finished processing all points from CSV ---")

        except FileNotFoundError:
            self.get_logger().error(f"Error: The file '{file_path}' was not found.")
        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("Usage: python hand_tracking_client.py <path_to_csv_file>")
        sys.exit(1)
        
    csv_file_path = sys.argv[1]

    hand_tracking_client = HandTrackingClient()
    
    try:
        # Process the movements from the CSV
        hand_tracking_client.process_csv_movements(csv_file_path)
    except KeyboardInterrupt:
        hand_tracking_client.get_logger().info('Program interrupted by user.')
    finally:
        # Cleanup
        hand_tracking_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
