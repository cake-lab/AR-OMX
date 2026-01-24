#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Quaternion
from std_msgs.msg import Float32MultiArray
import websocket
import json
import threading
import time
import numpy as np
from threading import Lock

class WebSocketBridgeNode(Node):
    def __init__(self):
        super().__init__('websocket_bridge_node')
        
        # Parameters
        self.declare_parameter('websocket_url', 'ws://localhost:8765')
        self.declare_parameter('control_frequency', 30.0)  # Hz
        self.declare_parameter('position_topic', '/target_position')
        self.declare_parameter('velocity_topic', '/cmd_twist')
        self.declare_parameter('angular_velocity_topic', '/angular_velocity')
        
        # Get parameters
        self.websocket_url = self.get_parameter('websocket_url').get_parameter_value().string_value
        self.control_freq = self.get_parameter('control_frequency').get_parameter_value().double_value
        
        # Publishers
        self.twist_publisher = self.create_publisher(
            Twist, 
            self.get_parameter('velocity_topic').get_parameter_value().string_value, 
            10
        )
        
        self.position_publisher = self.create_publisher(
            Point,
            self.get_parameter('position_topic').get_parameter_value().string_value,
            10
        )
        
        self.angular_vel_publisher = self.create_publisher(
            Float32MultiArray,
            self.get_parameter('angular_velocity_topic').get_parameter_value().string_value,
            10
        )
        
        # Data storage with thread safety
        self.data_lock = Lock()
        self.latest_data = None
        self.last_data_time = time.time()
        self.connection_active = False
        
        # WebSocket setup
        self.ws = None
        self.ws_thread = None
        
        # Control timer
        self.create_timer(1.0 / self.control_freq, self.control_callback)
        
        # Initialize WebSocket connection
        self.connect_websocket()
        
        self.get_logger().info('WebSocket Bridge Node initialized')

    def connect_websocket(self):
        """Initialize WebSocket connection in a separate thread"""
        try:
            self.ws = websocket.WebSocketApp(
                self.websocket_url,
                on_message=self.on_message,
                on_error=self.on_error,
                on_close=self.on_close,
                on_open=self.on_open
            )
            
            # Run WebSocket in separate thread
            self.ws_thread = threading.Thread(target=self.ws.run_forever, daemon=True)
            self.ws_thread.start()
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect to WebSocket: {e}')

    def on_open(self, ws):
        """Called when WebSocket connection is opened"""
        self.connection_active = True
        self.get_logger().info(f'WebSocket connected to {self.websocket_url}')

    def on_message(self, ws, message):
        """Handle incoming WebSocket messages"""
        try:
            data = json.loads(message)
            
            with self.data_lock:
                self.latest_data = data
                self.last_data_time = time.time()
                
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Failed to decode WebSocket message: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing WebSocket message: {e}')

    def on_error(self, ws, error):
        """Handle WebSocket errors"""
        self.get_logger().error(f'WebSocket error: {error}')
        self.connection_active = False

    def on_close(self, ws, close_status_code, close_msg):
        """Handle WebSocket connection close"""
        self.connection_active = False
        self.get_logger().warn('WebSocket connection closed')
        
        # Attempt to reconnect after a delay
        time.sleep(2.0)
        self.connect_websocket()

    def control_callback(self):
        """Main control loop - processes latest data and publishes ROS messages"""
        with self.data_lock:
            if self.latest_data is None:
                return
                
            # Check data freshness (timeout after 0.5 seconds)
            if time.time() - self.last_data_time > 0.5:
                self.get_logger().warn('WebSocket data timeout - stopping robot')
                self.publish_zero_velocity()
                return
                
            current_data = self.latest_data.copy()
        
        # Process data based on mode
        try:
            self.process_websocket_data(current_data)
        except Exception as e:
            self.get_logger().error(f'Error processing control data: {e}')

    def process_websocket_data(self, data):
        """Process WebSocket data and publish appropriate ROS messages"""
        mode = data.get('mode', 'position')
        timestamp = data.get('timestamp', time.time())
        
        # Process position data
        if 'position' in data and data['position'] is not None:
            pos_data = data['position']
            
            # Publish position
            position_msg = Point()
            position_msg.x = float(pos_data.get('x', 0.0))
            position_msg.y = float(pos_data.get('y', 0.0))
            position_msg.z = float(pos_data.get('z', 0.0))
            self.position_publisher.publish(position_msg)
            
        # Process velocity data
        if 'velocity' in data and data['velocity'] is not None:
            vel_data = data['velocity']
            
            # Publish Twist message for linear velocities
            twist_msg = Twist()
            twist_msg.linear.x = float(vel_data.get('vx', 0.0))
            twist_msg.linear.y = float(vel_data.get('vy', 0.0))
            twist_msg.linear.z = float(vel_data.get('vz', 0.0))
            
            # Add angular velocities if available
            if 'wx' in vel_data:
                twist_msg.angular.x = float(vel_data.get('wx', 0.0))
                twist_msg.angular.y = float(vel_data.get('wy', 0.0))
                twist_msg.angular.z = float(vel_data.get('wz', 0.0))
            
            self.twist_publisher.publish(twist_msg)
            
            # Publish separate angular velocity array for your kinematics node
            if all(key in vel_data for key in ['wx', 'wy', 'wz']):
                angular_vel_msg = Float32MultiArray()
                angular_vel_msg.data = [
                    float(vel_data['wx']),
                    float(vel_data['wy']), 
                    float(vel_data['wz'])
                ]
                self.angular_vel_publisher.publish(angular_vel_msg)

    def publish_zero_velocity(self):
        """Publish zero velocity for safety"""
        twist_msg = Twist()
        # All fields default to 0.0
        self.twist_publisher.publish(twist_msg)

    def destroy_node(self):
        """Clean shutdown"""
        if self.ws:
            self.ws.close()
        if self.ws_thread and self.ws_thread.is_alive():
            self.ws_thread.join(timeout=1.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = WebSocketBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
