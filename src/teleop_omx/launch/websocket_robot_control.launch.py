from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'websocket_url',
            default_value='ws://localhost:8765',
            description='WebSocket server URL'
        ),
        
        DeclareLaunchArgument(
            'control_frequency',
            default_value='30.0',
            description='Control loop frequency in Hz'
        ),
        
        # WebSocket Bridge Node
        Node(
            package='teleop_omx',
            executable='websocket_bridge_node',
            name='websocket_bridge',
            parameters=[{
                'websocket_url': LaunchConfiguration('websocket_url'),
                'control_frequency': LaunchConfiguration('control_frequency'),
                'velocity_topic': '/cmd_twist',
                'position_topic': '/target_position',
                'angular_velocity_topic': '/angular_velocity'
            }],
            output='screen'
        ),
        
        # Enhanced Velocity Controller
        Node(
            package='teleop_omx',
            executable='enhanced_velocity_controller',
            name='enhanced_velocity_controller',
            parameters=[{
                'control_frequency': LaunchConfiguration('control_frequency'),
                'velocity_smoothing_window': 5,
                'position_control_gain': 0.5,
                'velocity_limit': 3.0
            }],
            output='screen'
        ),
        
        # Enhanced Velocity Kinematics
        Node(
            package='teleop_omx',
            executable='enhanced_vel_kinematics',
            name='enhanced_vel_kinematics',
            output='screen'
        ),
        
        # Forward Kinematics (your existing node)
        Node(
            package='teleop_omx',
            executable='forward',
            name='forward_kinematics',
            output='screen'
        ),
    ])
