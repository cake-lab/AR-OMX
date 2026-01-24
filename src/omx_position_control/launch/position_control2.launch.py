from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    ExecuteProcess,
    LogInfo
)
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launches the real-time robot control system using the AR-compatible bridge (websocket_bridge_node2).
    - WebSocket Bridge 2: Connects to AR/Mobile app (Server Mode).
    - IK Service: Solves for joint angles.
    - Realtime Controller: Orchestrates the control loop.
    - Shutdown Hook: Returns the robot to home on exit.
    """
    
    # This action will be triggered when the launch system receives a shutdown signal (e.g., Ctrl+C).
    # It runs your existing script to move the robot to a predefined home position.
    homing_action = ExecuteProcess(
        cmd=['ros2', 'run', 'omx_position_control', 'basic_robot_control'],
        output='screen'
    )

    return LaunchDescription([
        # --- Arguments ---
        DeclareLaunchArgument(
            'server_port',
            default_value='8765',
            description='Port for the WebSocket server (websocket_bridge_node2)'
        ),
        DeclareLaunchArgument(
            'robot_frame_id',
            default_value='base_link',
            description="The name of the robot's base frame."
        ),

        # --- Nodes ---
        
        # 1. WebSocket Bridge Node 2 (AR/Server Version)
        # Replaces the original client-based bridge. This node ACTS AS A SERVER.
        Node(
            package='omx_position_control',
            executable='websocket_bridge_node2',
            name='websocket_bridge2',
            output='screen',
            parameters=[{
                'server_port': LaunchConfiguration('server_port'),
                'robot_frame_id': LaunchConfiguration('robot_frame_id'),
                # Topics must match what realtime_control_node expects
                'pose_topic': '/target_pose', 
                'gripper_topic': '/gripper',
                'feedback_topic': '/teleop_metrics'
            }]
        ),
        
        # 2. Inverse Kinematics Node
        # Provides the service to calculate joint angles from a Cartesian pose.
        Node(
            package='omx_position_control',
            executable='inverse_kinematics_node',
            name='inverse_kinematics_service',
            output='screen'
        ),
        
        # 3. Realtime Control Node
        # Subscribes to the target pose, calls the IK service, and commands the robot.
        # Ensure 'target_pose_topic' matches the output of the bridge above.
        Node(
            package='omx_position_control',
            executable='realtime_control_node',
            name='realtime_controller',
            output='screen',
            parameters=[{
                'target_pose_topic': '/target_pose',
                'teleop_metrics_topic': '/teleop_metrics'
            }]
        ),

        # --- Shutdown Hook ---
        # Register an event handler to run the homing action when a shutdown is initiated.
        RegisterEventHandler(
            event_handler=OnShutdown(
                on_shutdown=[
                    LogInfo(msg="Shutdown signal received. Returning robot to home position..."),
                    homing_action
                ],
            )
        ),
    ])
