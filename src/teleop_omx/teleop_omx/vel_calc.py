import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from open_manipulator_msgs.srv import SetJointPosition
from std_msgs.msg import Float32MultiArray
import numpy as np


class Velupdate(Node):

    def __init__(self):
        super().__init__('Velupdate')

        # Initialize attributes
        self.current_joint_states = None
        self.joint_state_received = False
        self.latest_joint_velocities = None
        self.joint_vel_received = False
        self.theta = [0.0, 0.0, 0.0, 0.0]
        self.time = 0.5

        # Subscriptions
        self.vel_subscription = self.create_subscription(Float32MultiArray, 'JV_info', self.update_joint_velocities, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Service client
        self.joint_client = self.create_client(SetJointPosition, 'goal_joint_space_path')
        self.wait_for_service_with_timeout(self.joint_client, 'goal_joint_space_path', 10.0)

    def wait_for_service_with_timeout(self, client, service_name, timeout_sec=10.0):
        start_time = self.get_clock().now()
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Waiting for {service_name} service...')
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout_sec:
                self.get_logger().error(f'Failed to find {service_name} service after {timeout_sec} seconds')
                return False
        self.get_logger().info(f'{service_name} service is ready')
        return True

    def joint_state_callback(self, msg):
        """Callback to receive joint states."""
        if len(msg.position) != 5:
            self.get_logger().error(f"Invalid joint state size: {len(msg.position)}. Expected 5.")
            return

        self.current_joint_states = np.array(msg.position[0:4])
        self.joint_state_received = True
        self.get_logger().info(f"Joint states updated: {self.current_joint_states}")

    def update_joint_velocities(self, msg):
        """Update the latest joint velocities from JV_info topic."""
        if len(msg.data) != 4:
            self.get_logger().error(f"Invalid joint velocity data size: {len(msg.data)}. Expected 4.")
            return

        self.latest_joint_velocities = np.array(msg.data)
        

        self.joint_vel_received = True
        self.get_logger().info(f"Updated latest joint velocities: {self.latest_joint_velocities}")

    def send_request(self, joint_positions):
        """Send a joint position request to SetJointPosition."""
        joint_positions = [joint_positions[0], joint_positions[1], joint_positions[2], joint_positions[3]]
        try:
            request = SetJointPosition.Request()
            request.planning_group = ''
            request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
            request.joint_position.position = joint_positions
            request.path_time = self.time

            self.future = self.joint_client.call_async(request)
            self.get_logger().info("Sent joint position request")
        except Exception as e:
            self.get_logger().error(f"Failed to send joint position request: {e}")

    def wait_until_position_reached(self):
        """Wait until the position is reached after sending a joint position request."""
        timeout_sec = 0.5
        start_time = self.get_clock().now()
        self.get_logger().info("Inside wait function.")

        while rclpy.ok():
            self.get_logger().info("Inside rclpy.")
            rclpy.spin_once(self, timeout_sec=0.1)
            elapsed_time = (self.get_clock().now() - start_time).nanoseconds / 1e9
            self.get_logger().info(f"Elapsed time: {elapsed_time:.2f} seconds")

            if elapsed_time > timeout_sec:
                self.get_logger().error("Timeout waiting for robot to reach position")
                break

            if self.joint_state_received and self.current_joint_states is not None:
                differences = [abs(target - current) for target, current in zip(self.theta, self.current_joint_states)]
                self.get_logger().info(f"Differences: {differences}")
                if all(diff < 0.05 for diff in differences):
                    self.get_logger().info("Target joint angles reached.")
                    break
        
        self.get_logger().info("After while.")

    def update_pos(self):
        """Update the robot position based on the latest joint velocities, running for 100 iterations."""
        self.get_logger().info("Starting position updates for iterations.")
        count = 0
        old_joint_state = None
        vel = 0.00
        flag = True

        while count < 10000 and rclpy.ok():

            if self.current_joint_states is not None and flag:
                old_joint_state = self.current_joint_states
                flag = False

            self.get_logger().info(f"Iteration {count + 1}")

            # Process incoming messages to ensure callbacks are invoked
            rclpy.spin_once(self, timeout_sec=0.1)

            # Ensure joint velocities are received
            if not self.joint_vel_received or self.latest_joint_velocities is None:
                self.get_logger().warn("No joint velocities received yet.")
                continue

            # Ensure joint states are received
            if self.joint_state_received and self.current_joint_states is not None and self.current_joint_states.size > 0:
                self.get_logger().info(f"Current joint states: {self.current_joint_states}")
                self.get_logger().info(f"latest_joint_velocities: {self.latest_joint_velocities}")

                for i in range(len(self.latest_joint_velocities)):
                    
                    # if abs(self.latest_joint_velocities[i]) < 1e-3:  # Skip if velocity is effectively zero
                    #     #self.latest_joint_velocities[i] = 0
                    #     self.theta[i] = old_joint_state[i]
                    #     self.get_logger().info(f"Joint {i} velocity is near zero; skipping update.")
                    #     continue
                        


                    # if i == 1 and self.theta[i] >= 0:
                    #     e = -self.theta[i]*0.1
                    # elif i == 1 and self.theta[i] < 0:
                    #     e = -self.theta[i]*0.007
                    # elif i == 2 and self.theta[i] >= 0.1:
                    #     e = -self.theta[i]*0.15
                    # elif i == 2 and self.theta[i] < 0.1 and self.theta[i] >= -0.2:
                    #     e = -0.12
                    # elif i == 3 and self.theta[i] < 0.0:
                    #     e = self.theta[i]*0.05
                    # elif i == 3 and self.theta[i] < 0.1 and self.theta[i] > 0.0:
                    #     e = -0.001
                    # elif i == 3 and self.theta[i] >= 0.1:
                    #     e = -self.theta[i]*0.05
                    # else:
                    #     e = 0

                    vel = self.latest_joint_velocities[i]

                    
                    self.theta[i] = old_joint_state[i] + vel*self.time #+ e
                    old_joint_state[i] = self.theta[i]
                    #self.theta[i] = round(self.theta[i], 3)
                    self.get_logger().info(f"Updated Theta {i}: {self.theta[i]}")
                
                # differences = [abs(target - current) for target, current in zip(self.theta, old_theta)]
                # self.get_logger().info(f"Difference = {differences}")
                # if any(diff > 0.0001 for diff in differences):
                self.send_request(self.theta)
                self.get_logger().info("Calling wait_until_position_reached()...")
                self.wait_until_position_reached()
                
                count += 1

            else:
                self.get_logger().warn("Joint states are not received or invalid.")

            

        self.get_logger().info("Completed iterations of position updates.")


def main():
    rclpy.init()
    node = Velupdate()
    try:
        node.update_pos()  # Run the 100 iterations directly
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
