#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from my_service_package.msg import RotPos
from std_msgs.msg import Float32MultiArray
import numpy as np
from scipy.linalg import pinv

class EnhancedVelKinematics(Node):
    def __init__(self):
        super().__init__('enhanced_vel_kinematics')
        
        # State variables
        self.J = None
        self.latest_twist = None
        
        # Subscribers
        self.create_subscription(RotPos, 'Fwd_op', self.jacobian_callback, 10)
        self.create_subscription(Twist, '/cmd_twist', self.twist_callback, 10)
        
        # Publishers
        self.joint_vel_publisher = self.create_publisher(Float32MultiArray, 'JV_info', 10)
        
        # Timer for computation
        self.create_timer(0.03, self.compute_joint_velocities)  # ~30Hz
        
        self.get_logger().info('Enhanced Velocity Kinematics Node initialized')

    def jacobian_callback(self, msg: RotPos):
        """Compute Jacobian from forward kinematics data"""
        try:
            # Your existing Jacobian computation code
            Rot = np.array(msg.r)
            Pos = np.array(msg.o)
            
            # Reshape rotation and position data
            R = np.zeros((4, 3, 3))
            O = np.zeros((5, 3, 1))
            
            # Fill rotation matrices
            idx = 0
            for i in range(4):
                for j in range(3):
                    for k in range(3):
                        R[i, j, k] = Rot[idx]
                        idx += 1
            
            # Fill position vectors
            idx = 0
            for i in range(5):
                for j in range(3):
                    O[i, j, 0] = Pos[idx]
                    idx += 1
            
            # Compute Jacobian
            k = np.array([[0], [0], [1]])
            v = [np.cross((R[i] @ k).flatten(), (O[4] - O[i]).flatten()).reshape(3, 1) for i in range(4)]
            w = [(R[i] @ k).reshape(3, 1) for i in range(4)]
            
            V = np.hstack(v)
            W = np.hstack(w)
            self.J = np.vstack((V, W))
            
        except Exception as e:
            self.get_logger().error(f'Error computing Jacobian: {e}')

    def twist_callback(self, msg: Twist):
        """Store latest twist command"""
        self.latest_twist = msg

    def compute_joint_velocities(self):
        """Compute and publish joint velocities"""
        if self.J is None or self.latest_twist is None:
            return
            
        try:
            # Convert Twist message to numpy array
            twist_vec = np.array([
                self.latest_twist.linear.x,
                self.latest_twist.linear.y, 
                self.latest_twist.linear.z,
                self.latest_twist.angular.x,
                self.latest_twist.angular.y,
                self.latest_twist.angular.z
            ]).reshape((6, 1))
            
            # Compute joint velocities using pseudo-inverse
            J_pinv = pinv(self.J)
            joint_vels = (J_pinv @ twist_vec).flatten().tolist()
            
            # Publish joint velocities
            msg = Float32MultiArray(data=joint_vels)
            self.joint_vel_publisher.publish(msg)
            
            # Log at reduced frequency
            if hasattr(self, '_log_counter'):
                self._log_counter += 1
            else:
                self._log_counter = 0
                
            if self._log_counter % 30 == 0:  # Log every second at 30Hz
                self.get_logger().info(f'Joint velocities: {joint_vels}')
            
        except Exception as e:
            self.get_logger().error(f"Error computing joint velocities: {e}")

def main():
    rclpy.init()
    node = EnhancedVelKinematics()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
