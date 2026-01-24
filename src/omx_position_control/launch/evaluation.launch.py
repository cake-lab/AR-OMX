from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launches the RMSE evaluator and the forward kinematics node needed for it.
    """
    # Declare the launch arguments
    window_size_arg = DeclareLaunchArgument(
        'window_size',
        default_value='90',
        description='Number of samples for the sliding window RMSE calculation.'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Rate (in Hz) to publish RMSE metrics.'
    )

    # Node that calculates actual pose from joint states
    forward_kinematics_node = Node(
        package='omx_position_control',
        executable='forward_kinematics',
        name='forward_kinematics_node',
        output='screen'
    )

    # The RMSE evaluator node
    rmse_evaluator_node = Node(
        package='omx_position_control',
        executable='rmse_evaluator',
        name='rmse_evaluator_node',
        output='screen',
        parameters=[{
        'publish_rate_hz': 1.0,
        'rpe_delta_s': 1.0, # Calculate RPE over 1.0 second intervals
        'sync_slop_s': 0.1 # Tighter sync tolerance
    }]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(window_size_arg)
    ld.add_action(publish_rate_arg)
    ld.add_action(forward_kinematics_node)
    ld.add_action(rmse_evaluator_node)

    return ld
