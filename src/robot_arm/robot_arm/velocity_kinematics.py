# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from tutorial_interfaces.srv import CalculateEfVel
from tutorial_interfaces.srv import CalculateJointVel
import numpy as np
import math as m
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class VelocityKinematicsNode(Node):
    def __init__(self):
        super().__init__('velocity_kinematics_node')

        self.joint_state_subscriber = self.create_subscription(JointState, '/joint_states', self.get_joint_angles, 10)
        self.srv1 = self.create_service(CalculateEfVel, 'calculate_end_effector_velocities', self.calc_ef_vel)
        self.srv2 = self.create_service(CalculateJointVel, 'calculate_joint_velocities', self.calc_joint_vel)


    def get_joint_angles(self, msg):

        self.Q1, self.Q2, self.Q3, self.Q4, _ = msg.position


    def calc_ef_vel(self, request, response):

        theta1_dot = request.theta1_dot
        theta2_dot = request.theta2_dot
        theta3_dot = request.theta3_dot
        theta4_dot = request.theta4_dot

        J, Q1, Q2, Q3, Q4 = self.calc_jacobian()

        joint_velocities = np.array([[theta1_dot], [theta2_dot], [theta3_dot], [theta4_dot]])
        ef_velocities = np.matmul(J, joint_velocities)

        response.v_x = ef_velocities[0][0]
        response.v_y = ef_velocities[1][0]
        response.v_z = ef_velocities[2][0]
        response.omega_x = ef_velocities[3][0]
        response.omega_y = ef_velocities[4][0]
        response.omega_z = ef_velocities[5][0]

        return response


    def calc_joint_vel(self, request, response):

        vx = request.v_x
        vy = request.v_y
        vz = request.v_z
        omega_x = request.omega_x
        omega_y = request.omega_y
        omega_z = request.omega_z

        J, Q1, Q2, Q3, Q4 = self.calc_jacobian()
        J_inverse = np.linalg.pinv(J)

        ef_velocities = np.array([vx, vy, vz, omega_x, omega_y, omega_z]).reshape(6, 1)
        joint_velocities = J_inverse @ ef_velocities

        response.theta1_dot = joint_velocities[0][0]
        response.theta2_dot = joint_velocities[1][0]
        response.theta3_dot = joint_velocities[2][0]
        response.theta4_dot = joint_velocities[3][0]
        response.q1 = Q1
        response.q2 = Q2
        response.q3 = Q3
        response.q4 = Q4

        return response


    def calc_jacobian(self):

        # DH parameters
        
        alphaa = [-np.pi/2, 0, 0, np.pi/2]
        a = [0, 0.13023, 0.124, 0.1334]
        d = [0.06025, 0, 0, 0]
        
        Q1, Q2, Q3, Q4 = self.Q1, self.Q2, self.Q3, self.Q4
        
        
        Q = [Q1, Q2 - np.pi/2 + np.radians(10.6197), Q3 + np.pi/2  - np.radians(10.6197), Q4]
        
        

        T01 = np.array([[np.cos(Q[0]), -np.sin(Q[0])*np.cos(alphaa[0]), np.sin(alphaa[0])*np.sin(Q[0]), a[0]*np.cos(Q[0])],
              [np.sin(Q[0]), np.cos(Q[0])*np.cos(alphaa[0]), -np.sin(alphaa[0])*np.cos(Q[0]), a[0]*np.sin(Q[0])],
              [0, np.sin(alphaa[0]), np.cos(alphaa[0]), d[0]],
              [0, 0, 0, 1]])

        T12 = np.array([[np.cos(Q[1]), -np.sin(Q[1])*np.cos(alphaa[1]), np.sin(alphaa[1])*np.sin(Q[1]), a[1]*np.cos(Q[1])],
                      [np.sin(Q[1]), np.cos(Q[1])*np.cos(alphaa[1]), -np.sin(alphaa[1])*np.cos(Q[1]), a[1]*np.sin(Q[1])],
                      [0, np.sin(alphaa[1]), np.cos(alphaa[1]), d[1]],
                      [0, 0, 0, 1]])

        T23 = np.array([[np.cos(Q[2]), -np.sin(Q[2])*np.cos(alphaa[2]), np.sin(alphaa[2])*np.sin(Q[2]), a[2]*np.cos(Q[2])],
                      [np.sin(Q[2]), np.cos(Q[2])*np.cos(alphaa[2]), -np.sin(alphaa[2])*np.cos(Q[2]), a[2]*np.sin(Q[2])],
                      [0, np.sin(alphaa[2]), np.cos(alphaa[2]), d[2]],
                      [0, 0, 0, 1]])

        T34 = np.array([[np.cos(Q[3]), -np.sin(Q[3])*np.cos(alphaa[3]), np.sin(alphaa[3])*np.sin(Q[3]), a[3]*np.cos(Q[3])],
                      [np.sin(Q[3]), np.cos(Q[3])*np.cos(alphaa[3]), -np.sin(alphaa[3])*np.cos(Q[3]), a[3]*np.sin(Q[3])],
                      [0, np.sin(alphaa[3]), np.cos(alphaa[3]), d[3]],
                      [0, 0, 0, 1]])

        T02 = np.matmul(T01,T12)
        T03 = np.matmul(T02,T23)
        T04 = np.matmul(T03,T34)

        z0 = np.array([0, 0, 1])
        O0 = np.array([0, 0, 0])
        z1 = T01[:3, 2]
        O1 = T01[:3, 3]
        z2 = T02[:3, 2]
        O2 = T02[:3, 3]
        z3 = T03[:3, 2]
        O3 = T03[:3, 3]
        z4 = T04[:3, 2]
        O4 = T04[:3, 3]


        Jv1 = np.cross(z0, (O4 - O0))
        Jw1 = z0
        Jv2 = np.cross(z1, (O4 - O1))
        Jw2 = z1
        Jv3 = np.cross(z2, (O4 - O2))
        Jw3 = z2
        Jv4 = np.cross(z3, (O4 - O3))
        Jw4 = z3

        J = np.vstack([np.column_stack([Jv1, Jv2, Jv3, Jv4]), np.column_stack([Jw1, Jw2, Jw3, Jw4])])
        
        return J, Q1, Q2, Q3, Q4
        
        


       

def main():
    rclpy.init()

    velocity_kinematics_node = VelocityKinematicsNode()

    rclpy.spin(velocity_kinematics_node)

    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

