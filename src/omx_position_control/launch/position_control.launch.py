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
    Launches the complete real-time robot control system.
    - WebSocket Bridge: Connects to the CV script.
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
            'websocket_url',
            default_value='ws://localhost:8765',
            description='WebSocket server URL for the CV script'
        ),
        DeclareLaunchArgument(
            'robot_frame_id',
            default_value='base_link',
            description="The name of the robot's base frame."
        ),

        # --- Nodes ---
        # 1. WebSocket Bridge Node
        # Receives hand pose from the CV script and publishes it as a PoseStamped.
        Node(
            package='omx_position_control',
            executable='websocket_bridge_node',
            name='websocket_bridge',
            output='screen',
            parameters=[{
                'websocket_url': LaunchConfiguration('websocket_url'),
                'robot_frame_id': LaunchConfiguration('robot_frame_id')
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
        Node(
            package='omx_position_control',
            executable='realtime_control_node',
            name='realtime_controller',
            output='screen'
        ),

        # --- Shutdown Hook ---
        # Register an event handler to run the homing action when a shutdown is initiated.
        # This is triggered by pressing Ctrl+C in the terminal where the launch file is running.
        RegisterEventHandler(
            event_handler=OnShutdown(
                on_shutdown=[
                    LogInfo(msg="Shutdown signal received. Returning robot to home position..."),
                    homing_action
                ],
            )
        ),
    ])
