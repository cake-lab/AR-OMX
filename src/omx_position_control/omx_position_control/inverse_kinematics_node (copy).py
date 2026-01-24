import rclpy
from rclpy.node import Node
from tutorial_interfaces.srv import CalculateIK
import numpy as np
import math

class InverseKinematicsNode(Node):
    """
    A dedicated node that provides an Inverse Kinematics (IK) service.
    It takes a Cartesian pose (x, y, z, quaternion), returns the corresponding
    joint angles, and enforces joint limits for safety.
    """
    def __init__(self):
        super().__init__('inverse_kinematics_node')
        self.srv = self.create_service(CalculateIK, 'calculate_ik', self.calc_inverse_kinematics)

        # Robot arm link lengths
        self.L1 = 0.06025
        self.L2 = 0.13023
        self.L3 = 0.124
        self.L4 = 0.1334

        # --- JOINT LIMITS ADDED ---
        # Define joint limits in radians. Adjust these to your robot's specifications.
        # Joint 1 (Base)
        self.joint1_min = np.deg2rad(-120)
        self.joint1_max = np.deg2rad(120)
        
        # Joint 2 (Shoulder)
        self.joint2_min = np.deg2rad(-90)
        self.joint2_max = np.deg2rad(90)
        
        # Joint 3 (Elbow)
        self.joint3_min = np.deg2rad(-80)
        self.joint3_max = np.deg2rad(80)

        # Joint 4 (Wrist)
        self.joint4_min = np.deg2rad(-100)
        self.joint4_max = np.deg2rad(100)

        self.get_logger().info('Inverse Kinematics Service with Joint Limits is ready.')

    def calc_inverse_kinematics(self, request, response):
        """Service callback to perform the IK calculation and enforce limits."""
        x, y, z = request.x, request.y, request.z
        qx, qy, qz, qw = request.qx, request.qy, request.qz, request.qw
        
        # Calculate the raw joint angles from the geometric model
        theta1, theta2, theta3, theta4 = self.inverse_kinematics(x,y,z,qx,qy,qz,qw)
        
        # --- CLAMPING LOGIC ADDED ---
        # Clamp the calculated angles to the defined joint limits
        clamped_theta1 = np.clip(theta1, self.joint1_min, self.joint1_max)
        clamped_theta2 = np.clip(theta2, self.joint2_min, self.joint2_max)
        clamped_theta3 = np.clip(theta3, self.joint3_min, self.joint3_max)
        clamped_theta4 = np.clip(theta4, self.joint4_min, self.joint4_max)

        # Log a warning if any joint angle was clamped
        if not np.isclose(theta1, clamped_theta1):
            self.get_logger().warn(f"Joint 1 angle ({np.rad2deg(theta1):.2f} deg) out of range. Clamped to {np.rad2deg(clamped_theta1):.2f} deg.", throttle_duration_sec=1)
        if not np.isclose(theta2, clamped_theta2):
            self.get_logger().warn(f"Joint 2 angle ({np.rad2deg(theta2):.2f} deg) out of range. Clamped to {np.rad2deg(clamped_theta2):.2f} deg.", throttle_duration_sec=1)
        if not np.isclose(theta3, clamped_theta3):
            self.get_logger().warn(f"Joint 3 angle ({np.rad2deg(theta3):.2f} deg) out of range. Clamped to {np.rad2deg(clamped_theta3):.2f} deg.", throttle_duration_sec=1)
        if not np.isclose(theta4, clamped_theta4):
            self.get_logger().warn(f"Joint 4 angle ({np.rad2deg(theta4):.2f} deg) out of range. Clamped to {np.rad2deg(clamped_theta4):.2f} deg.", throttle_duration_sec=1)

        # Assign the safe, clamped values to the service response
        response.theta1 = clamped_theta1
        response.theta2 = clamped_theta2
        response.theta3 = clamped_theta3
        response.theta4 = clamped_theta4
        
        self.get_logger().debug(f'IK Request processed. Angles: ({clamped_theta1:.2f}, {clamped_theta2:.2f}, {clamped_theta3:.2f}, {clamped_theta4:.2f})')
        return response

    def inverse_kinematics(self, x,y,z,qx,qy,qz,qw):
        """The core IK geometric calculation."""
        # This part of the logic remains unchanged
        t2 = +2.0 * (qw * qy - qz * qx)
        t2 = max(-1.0, min(1.0, t2)) # Clamp the value
        pitch = -math.asin(t2)

        r = math.sqrt(x**2 + y**2)
        r1 = r - self.L4 * math.cos(pitch)
        z1 = z - self.L1 - self.L4 * math.sin(pitch)

        cos_phi3_numerator = r1**2 + z1**2 - self.L2**2 - self.L3**2
        cos_phi3_denominator = 2 * self.L2 * self.L3
        
        # Avoid division by zero and handle floating point inaccuracies
        if cos_phi3_denominator == 0: cos_phi3 = 0.0
        else: cos_phi3 = cos_phi3_numerator / cos_phi3_denominator
        
        cos_phi3 = max(-1.0, min(1.0, cos_phi3)) # Clamp value
        sin_phi3 = -math.sqrt(1 - cos_phi3**2)

        phi3 = math.atan2(sin_phi3, cos_phi3)
        phi2 = math.atan2(z1, r1) - math.atan2(self.L3 * sin_phi3, self.L2 + self.L3 * cos_phi3)
        phi4 = pitch - phi2 - phi3

        theta1 = math.atan2(y, x)
        theta2 = -phi2 + math.pi/2 - math.atan2(24, 128)
        theta3 = -phi3 - math.pi/2 + math.atan2(24, 128)
        theta4 = -phi4

        return (theta1, theta2, theta3, theta4)

def main():
    rclpy.init()
    ik_node = InverseKinematicsNode()
    rclpy.spin(ik_node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
