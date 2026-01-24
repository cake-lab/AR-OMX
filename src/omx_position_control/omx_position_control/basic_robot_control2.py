"""

Controls the OpenManipulator-X by sending joint-space motion commands through the `goal_joint_space_path` service. 
Waits for the service to become available, sends asynchronous motion requests to predefined joint configurations, 
and waits for execution feedback before proceeding.

"""

import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import SetJointPosition
import sys
import time

# This class controls the OpenManipulator-X by sending service requests to move its joints.
class BasicRobotControl2(Node):
    # This node sends service requests to the 'goal_joint_space_path' service to control the OpenManipulator-X robot.

    def __init__(self):
        super().__init__('basic_robot_control')
        # Create a service client for the 'goal_joint_space_path' service.
        self.client = self.create_client(SetJointPosition, 'goal_joint_space_path')
        self.future = None # To hold the future object from the service call.

    # This method waits for the service to become available.
    def wait_for_service(self):
        self.get_logger().info('Waiting for the goal_joint_space_path service...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(0)
            self.get_logger().info('Service not available, waiting again...')
        self.get_logger().info('Service is available!')

    # This method sends a single request to move the robot to a specified pose.
    def send_pose_request(self, joint_positions, path_time):
        """
        Sends a service request to move the robot to a specific joint configuration.
        Args:
            joint_positions (list): A list of joint positions [joint1, joint2, joint3, joint4, gripper].
            path_time (float): The time in seconds for the movement.
        """
        request = SetJointPosition.Request()
        request.planning_group = ''
        request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        request.joint_position.position = joint_positions
        request.path_time = path_time

        self.get_logger().info(f'Sending request to move to position: {joint_positions} in {path_time} seconds.')
        self.future = self.client.call_async(request)
        
    def wait_for_response(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future.done():
                try:
                    response = self.future.result()
                    if response.is_planned:
                        self.get_logger().info('Movement successful!')
                    else:
                        self.get_logger().error(f'Movement failed! Result: {response.result}')
                except Exception as e:
                    self.get_logger().error('Service call failed %r' % (e,))
                break

def main(args=None):
    rclpy.init(args=args)
    
    robot_control_node = BasicRobotControl2()
    
    # Wait for the service to be available before trying to send requests.
    robot_control_node.wait_for_service()
    
    # Define the poses you want to move to.
    pose1 = [0.0, 0.0, 0.0, 0.0, 0.0]  # First pose (home position)
    pose2 = [0.0, 0.30, 0.0, 0.8, 0.0] # Second pose
    
    # 1. Move to the first pose.
    robot_control_node.send_pose_request(pose1, 5.0)
    robot_control_node.wait_for_response()
    
    robot_control_node.get_logger().info('Waiting for 2 seconds before moving to the next pose...')
    time.sleep(5)
    
    # 2. Move to the second pose.
    robot_control_node.send_pose_request(pose2, 5.0)
    robot_control_node.wait_for_response()

    robot_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
