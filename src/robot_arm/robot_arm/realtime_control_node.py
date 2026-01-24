#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped
from tutorial_interfaces.srv import CalculateIK
from open_manipulator_msgs.srv import SetJointPosition
import sys

class RealtimeControlNode(Node):
    """
    Subscribes to a target pose, calls an IK service to get joint angles,
    and sends the joint commands to the robot controller.
    """
    def __init__(self):
        super().__init__('realtime_control_node')
        
        # Use a ReentrantCallbackGroup to allow service calls within a subscription callback
        self.callback_group = ReentrantCallbackGroup()
        
        # Subscriber to the target pose from the websocket bridge
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/target_pose',
            self.pose_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Service client for Inverse Kinematics
        self.ik_client = self.create_client(
            CalculateIK, 
            'calculate_ik',
            callback_group=self.callback_group
        )
        
        # Service client to command the robot's joints
        self.joint_position_client = self.create_client(
            SetJointPosition, 
            'goal_joint_space_path',
            callback_group=self.callback_group
        )
        
        self.get_logger().info("Waiting for services...")
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("IK service 'calculate_ik' not available. Exiting.")
            sys.exit(1)
        if not self.joint_position_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Joint position service 'goal_joint_space_path' not available. Exiting.")
            sys.exit(1)
            
        self.get_logger().info("All services are available. Realtime control node is ready.")
        self.is_request_pending = False

    async def pose_callback(self, msg: PoseStamped):
        """Callback function for receiving target poses."""
        if self.is_request_pending:
            self.get_logger().warn("Skipping new pose, previous request still pending.", throttle_duration_sec=1.0)
            return

        self.is_request_pending = True
        
        try:
            # 1. Call the IK service
            ik_request = CalculateIK.Request()
            ik_request.x = msg.pose.position.x
            ik_request.y = msg.pose.position.y
            ik_request.z = msg.pose.position.z
            ik_request.qx = msg.pose.orientation.x
            ik_request.qy = msg.pose.orientation.y
            ik_request.qz = msg.pose.orientation.z
            ik_request.qw = msg.pose.orientation.w
            
            self.get_logger().debug(f"Requesting IK for pose: {ik_request}")
            ik_future = self.ik_client.call_async(ik_request)
            ik_response = await ik_future

            if ik_response is None:
                self.get_logger().error("Failed to call IK service.")
                return

            # 2. If IK is successful, call the joint position service
            # Note: Your IK service seems to always succeed. In a real scenario,
            # you might check for a success flag in the response.
            thetas = [
                ik_response.theta1, 
                ik_response.theta2, 
                ik_response.theta3, 
                ik_response.theta4
            ]
            
            joint_request = SetJointPosition.Request()
            joint_request.planning_group = '' # Often empty for this service
            joint_request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
            joint_request.joint_position.position = thetas
            joint_request.path_time = 1.0  # Time to reach the target. Adjust for smoother/faster motion.

            self.get_logger().debug(f"Sending joint command: {thetas}")
            joint_future = self.joint_position_client.call_async(joint_request)
            # We can 'await' here to ensure commands are sent sequentially,
            # or not await to send them as fast as possible.
            # Awaiting is generally safer to prevent overwhelming the controller.
            await joint_future

        except Exception as e:
            self.get_logger().error(f"An error occurred in the pose callback: {e}")
        finally:
            self.is_request_pending = False

def main(args=None):
    rclpy.init(args=args)
    node = RealtimeControlNode()
    try:
        # Use a MultiThreadedExecutor to handle callbacks in parallel
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
