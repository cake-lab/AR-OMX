import rclpy
from rclpy.node import Node
from tutorial_interfaces.srv import CalculateIK
from tutorial_interfaces.srv import CalculateEfVel
from tutorial_interfaces.srv import CalculateJointVel
from open_manipulator_msgs.srv import SetJointPosition
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from open_manipulator_msgs.msg import KinematicsPose
import sys
import numpy as np
import time

class IncrementalRobotControl(Node):
    def __init__(self):
        super().__init__('incremental_robot_control')
        #subscriber not calling the function it is supposed to for some reason even though it works in velocity_kinematics.py
        self.joint_state_subscriber = self.create_subscription(JointState, 'joint_states', self.get_current_angles, 10)
        self.client = self.create_client(SetJointPosition, 'goal_joint_space_path')
        self.client1 = self.create_client(CalculateJointVel, 'calculate_joint_velocities')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        while not self.client1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.send_request()
        
    def get_current_angles(self, msg):
        self.Q1_ref_old, self.Q2_ref_old, self.Q3_ref_old, self.Q4_ref_old, _ = msg.position

    def send_request(self):
    	self.end_task = False
    	request = CalculateJointVel.Request()
    	request.v_x = 0.0
    	request.v_y = 0.0
    	request.v_z = 0.0
    	request.omega_x = 0.0
    	request.omega_y = 0.0
    	request.omega_z = 0.0
    	self.future = self.client1.call_async(request)
    	rclpy.spin_until_future_complete(self, self.future)
    	response = self.future.result()
    	q1_ref_old = response.q1
    	q2_ref_old = response.q2
    	q3_ref_old = response.q3
    	q4_ref_old = response.q4
    	total_time = 8.0
    	self.sampling_time = 0.02
    	time = 0.0
    	for i in range(int(total_time/self.sampling_time)):
    	    request = CalculateJointVel.Request()
    	    request.v_x = 0.0
    	    request.v_y = 0.0
    	    request.v_z = 0.1
    	    request.omega_x = 0.0
    	    request.omega_y = 0.0
    	    request.omega_z = 0.0
    	    
    	    self.future = self.client1.call_async(request)
    	    rclpy.spin_until_future_complete(self, self.future)
    	    response = self.future.result()
    	    
    	    Q1_ref = q1_ref_old + response.theta1_dot*self.sampling_time
    	    q1_ref_old = Q1_ref
    	    Q2_ref = q2_ref_old + response.theta2_dot*self.sampling_time
    	    q2_ref_old = Q2_ref
    	    Q3_ref = q3_ref_old + response.theta3_dot*self.sampling_time
    	    q3_ref_old = Q3_ref
    	    Q4_ref = q4_ref_old + response.theta4_dot*self.sampling_time
    	    q4_ref_old = Q4_ref
    	    
    	    self.move_to_position(Q1_ref, Q2_ref, Q3_ref, Q4_ref, self.sampling_time)
    	    time += self.sampling_time
    	    self.end_task = True
        

    def move_to_position(self, q1, q2, q3, q4, sampling_time):
        request = SetJointPosition.Request()
        request.planning_group = ''
        request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
        request.joint_position.position = [q1, q2, q3, q4]
        request.path_time = sampling_time
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)
    
        
def main(args=None):
    rclpy.init(args=args)

    incremental_robot_control = IncrementalRobotControl()

    while rclpy.ok():
        rclpy.spin_once(incremental_robot_control)
        if incremental_robot_control.end_task:
            try:
                response = incremental_robot_control.future.result()
            except Exception as e:
                incremental_robot_control.get_logger().error('Service call failed %r' % (e,))
            break

    incremental_robot_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
