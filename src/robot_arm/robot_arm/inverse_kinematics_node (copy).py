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

from tutorial_interfaces.srv import CalculateIK
import numpy as np
import math
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')
        self.srv = self.create_service(CalculateIK, 'calculate_ik', self.calc_inverse_kinematics)



    def calc_inverse_kinematics(self, request, response):
        x = request.x
        y = request.y
        z = request.z
        qx = request.qx
        qy = request.qy
        qz = request.qz
        qw = request.qw
        
        theta1, theta2, theta3, theta4 = self.inverse_kinematics(x,y,z,qx,qy,qz,qw)
        
        response.theta1 = theta1
        response.theta2 = theta2
        response.theta3 = theta3
        response.theta4 = theta4

        self.get_logger().info(f'Incoming request\nx:{x} y:{y} z:{z} qx:{qx} qy:{qy} qz:{qz} qw:{qw}\n')
        self.get_logger().info(f'Outgoing response\ntheta1:{theta1} theta2:{theta2} theta3:{theta3} theta4:{theta4}')
        
        return response

    
    def inverse_kinematics(self, x,y,z,qx,qy,qz,qw):

        # Calculate the Pitch
        
        t2 = +2.0 * (qw * qy - qz * qx)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = -math.asin(t2)

        L1 = 0.06025
        L2 = 0.13023
        L3 = 0.124
        L4 = 0.1334

        r = math.sqrt(x**2 + y**2)
        r1 = r - L4*math.cos(pitch)
        z1 = z - L1 - L4*math.sin(pitch)

        cos_phi3 = (r1**2 + z1**2 - L2**2 - L3**2) / (2 * L2 * L3)
        sin_phi3 = -math.sqrt(1 - cos_phi3**2)


        phi3 = math.atan2(sin_phi3, cos_phi3)
        phi2 = math.atan2(z1, r1) - math.atan2(L3*sin_phi3, L2 + L3*cos_phi3)
        phi4 = pitch - phi2 - phi3


        theta1 = math.atan2(y, x)
        theta2 = -phi2 + math.pi/2 - math.atan2(24, 128)
        theta3 = -phi3 - math.pi/2 + math.atan2(24, 128)
        theta4 = -phi4

        return (theta1, theta2, theta3, theta4)


       

def main():
    rclpy.init()

    inverse_kinematics_node = InverseKinematicsNode()

    rclpy.spin(inverse_kinematics_node)

    rclpy.shutdown()
    
if __name__ == '__main__':
    main()

