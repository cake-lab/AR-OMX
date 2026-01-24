#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import websocket
import json
import threading
import time
from threading import Lock

class WebSocketBridgeNode(Node):
    """
    Connects to a WebSocket server, receives JSON data containing position and 
    orientation, and publishes it as a ROS2 PoseStamped message.
    """
    def __init__(self):
        super().__init__('websocket_bridge_node')
        
        # Declare parameters
        self.declare_parameter('websocket_url', 'ws://localhost:8765')
        self.declare_parameter('control_frequency', 30.0)
        self.declare_parameter('pose_topic', '/target_pose')
        self.declare_parameter('robot_frame_id', 'base_link')

        # Get parameters
        self.websocket_url = self.get_parameter('websocket_url').get_parameter_value().string_value
        self.control_freq = self.get_parameter('control_frequency').get_parameter_value().double_value
        self.robot_frame_id = self.get_parameter('robot_frame_id').get_parameter_value().string_value
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        
        # Publisher for the target pose
        self.pose_publisher = self.create_publisher(PoseStamped, pose_topic, 10)
        
        # Data storage with thread safety
        self.data_lock = Lock()
        self.latest_data = None
        self.last_data_time = 0.0
        
        # WebSocket setup
        self.ws_thread = threading.Thread(target=self._ws_thread, daemon=True)
        self.ws_thread.start()
        
        # Control timer
        self.timer = self.create_timer(1.0 / self.control_freq, self.control_callback)
        
        self.get_logger().info(f"WebSocket Bridge initialized. Connecting to {self.websocket_url}")

    def _ws_thread(self):
        """Manages the WebSocket connection lifecycle."""
        while rclpy.ok():
            self.get_logger().info(f"Attempting to connect to {self.websocket_url}...")
            try:
                ws = websocket.WebSocketApp(
                    self.websocket_url,
                    on_message=self.on_message,
                    on_open=self.on_open,
                    on_error=self.on_error,
                    on_close=self.on_close
                )
                ws.run_forever()
            except Exception as e:
                self.get_logger().error(f"WebSocket connection error: {e}")
            self.get_logger().warn("WebSocket connection closed. Reconnecting in 5 seconds...")
            time.sleep(5)

    def on_open(self, ws):
        self.get_logger().info("WebSocket connection established.")

    def on_message(self, ws, message):
        """Handle incoming WebSocket messages."""
        try:
            data = json.loads(message)
            # We only care about position data for the pose
            if data and data.get('mode') in ['position', 'both'] and data.get('position') is not None:
                with self.data_lock:
                    self.latest_data = data['position']
                    self.last_data_time = self.get_clock().now().nanoseconds / 1e9
        except json.JSONDecodeError:
            self.get_logger().warn(f"Failed to decode JSON message: {message}")
        except Exception as e:
            self.get_logger().error(f"Error processing message: {e}")

    def on_error(self, ws, error):
        self.get_logger().error(f"WebSocket error: {error}")

    def on_close(self, ws, close_status_code, close_msg):
        self.get_logger().warn("WebSocket connection closed.")

    def control_callback(self):
        """Processes the latest data and publishes the ROS message."""
        with self.data_lock:
            if self.latest_data is None:
                return
            
            # Check for data timeout (e.g., 1 second)
            if (self.get_clock().now().nanoseconds / 1e9 - self.last_data_time) > 1.0:
                self.get_logger().warn("WebSocket data timeout. No new pose is being published.")
                self.latest_data = None # Clear data to prevent re-publishing old data
                return
                
            current_data = self.latest_data

        # Create and publish the PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.robot_frame_id
        
        pose_msg.pose.position.x = float(current_data.get('x', 0.0))
        pose_msg.pose.position.y = float(current_data.get('y', 0.0))
        pose_msg.pose.position.z = float(current_data.get('z', 0.0))
        
        pose_msg.pose.orientation.x = float(current_data.get('qx', 0.0))
        pose_msg.pose.orientation.y = float(current_data.get('qy', 0.0))
        pose_msg.pose.orientation.z = float(current_data.get('qz', 0.0))
        pose_msg.pose.orientation.w = float(current_data.get('qw', 1.0))
        
        self.pose_publisher.publish(pose_msg)

    def destroy_node(self):
        self.timer.cancel()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WebSocketBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
