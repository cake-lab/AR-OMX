import rclpy
from rclpy.node import Node
from tutorial_interfaces.srv import CalculateIK
from open_manipulator_msgs.srv import SetJointPosition
import sys
import numpy as np
import time

class RobotPickUpControl(Node):
    def __init__(self):
        super().__init__('robot_pick_up_control')
        self.client1 = self.create_client(CalculateIK, 'calculate_ik')                  # Client that calls Inverse Kinematics service
        self.client = self.create_client(SetJointPosition, 'goal_joint_space_path')     # Client that requests joint positions to be set
        self.client2 = self.create_client(SetJointPosition, 'goal_tool_control')        # Client that requests gripper positions to be set
        self.end_task = False
        while not self.client1.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                sys.exit(0)
            self.get_logger().info('Service not available, waiting again...')
        self.send_request()

    def send_request(self):
        # Define the start and final position of the object as well as end effector orientation to reach those points
        request = CalculateIK.Request()
        gripper_position = 0.010
        obj_x = 0.18441602564428683;
        obj_y = -0.08349713749496296;
        obj_z = 0.010;

        obj_qw = 0.7572386117302964;
        obj_qz = 0.03836523061506765;
        obj_qy = 0.6511753604836197;
        obj_qx = -0.03299157291876424;
        
        final_x = 0.17210048916362783
        final_y = 0.10207200294492504
        final_z = 0.010

        interim_z = obj_z + 0.05

        # Move above object

        request.x = obj_x
        request.y = obj_y
        request.z = interim_z
        request.qx = obj_qx
        request.qy = obj_qy
        request.qz = obj_qz
        request.qw = obj_qw

        self.future = self.client1.call_async(request)      # Request inverse kinematics for given pose
        rclpy.spin_until_future_complete(self, self.future) # Wait for the service to generate response
        response = self.future.result()                     # Store response in variable
        
        self.move_to_position(response.theta1, response.theta2, response.theta3, response.theta4)
        self.move_gripper(gripper_position)
        time.sleep(5)       # Wait for robot to achieve position
        
        # Move to object
        request = CalculateIK.Request()
        
        request.x = obj_x
        request.y = obj_y
        request.z = obj_z
        request.qx = obj_qx
        request.qy = obj_qy
        request.qz = obj_qz
        request.qw = obj_qw

        self.future = self.client1.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        response = self.future.result()
        self.move_to_position(response.theta1, response.theta2, response.theta3, response.theta4)
        time.sleep(5)

        # Close gripper
        closed_gripper_position = -0.0025
        self.move_gripper(closed_gripper_position)
        time.sleep(1)

        # Lift object
        request = CalculateIK.Request()

        request.x = obj_x
        request.y = obj_y
        request.z = interim_z
        request.qx = obj_qx
        request.qy = obj_qy
        request.qz = obj_qz
        request.qw = obj_qw

        self.future = self.client1.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        response = self.future.result()
        self.move_to_position(response.theta1, response.theta2, response.theta3, response.theta4)
        time.sleep(5)

        # Move to new position
        request = CalculateIK.Request()

        request.x = final_x
        request.y = final_y
        request.z = interim_z
        request.qx = obj_qx
        request.qy = obj_qy
        request.qz = obj_qz
        request.qw = obj_qw

        self.future = self.client1.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        response = self.future.result()
        self.move_to_position(response.theta1, response.theta2, response.theta3, response.theta4)
        time.sleep(5)

        # Place Object
        request = CalculateIK.Request()

        request.x = final_x
        request.y = final_y
        request.z = final_z
        request.qx = obj_qx
        request.qy = obj_qy
        request.qz = obj_qz
        request.qw = obj_qw

        self.future = self.client1.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
        response = self.future.result()
        self.move_to_position(response.theta1, response.theta2, response.theta3, response.theta4)
        time.sleep(5)

        # Open gripper

        self.move_gripper(gripper_position)
        time.sleep(5)

        self.end_task = True


        

    def move_to_position(self, q1, q2, q3, q4):
        request = SetJointPosition.Request()
        request.planning_group = ''
        request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
        request.joint_position.position = [q1, q2, q3, q4]
        request.path_time = 5.0
        self.future = self.client.call_async(request)

    def move_gripper(self, gripper_position):
        request = SetJointPosition.Request()
        request.joint_position.joint_name = ['gripper']
        request.joint_position.position = [gripper_position]
        request.path_time = 5.0
        self.future = self.client2.call_async(request)
    
        
def main(args=None):
    rclpy.init(args=args)

    robot_pick_up_control = RobotPickUpControl()

    while rclpy.ok():
        rclpy.spin_once(robot_pick_up_control)
        if robot_pick_up_control.end_task == True:
            try:
                response = robot_pick_up_control.future.result()
            except Exception as e:
                robot_pick_up_control.get_logger().error('Service call failed %r' % (e,))
            break

    robot_pick_up_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
