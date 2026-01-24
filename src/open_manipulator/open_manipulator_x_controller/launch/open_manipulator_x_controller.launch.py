import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the path to the package's share directory
    pkg_share = get_package_share_directory('open_manipulator_x_controller')

    # Define the path to the parameter file
    default_yaml_path = os.path.join(
        pkg_share,
        'config',
        'open_manipulator_x_controller.yaml'
    )

    return LaunchDescription([
        # Declare the 'use_sim_time' launch argument
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # Declare the 'usb_port' launch argument
        DeclareLaunchArgument(
            'usb_port',
            default_value='/dev/ttyACM0',
            description='USB port name for dynamixel U2D2.'),

        # Declare the 'baud_rate' launch argument
        DeclareLaunchArgument(
            'baud_rate',
            default_value='1000000',
            description='Baud rate for dynamixel U2D2.'),

        # Declare the path to the YAML file
        DeclareLaunchArgument(
            'param_dir',
            default_value=default_yaml_path,
            description='Full path to the parameter file to use'),

        # Define the Node
        Node(
            package='open_manipulator_x_controller',
            # This is the line that was fixed. 'node_executable' is now 'executable'
            executable='open_manipulator_x_controller',
            name='open_manipulator_x_controller',
            output='screen',
            parameters=[
                LaunchConfiguration('param_dir'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'usb_port': LaunchConfiguration('usb_port'),
                    'baud_rate': LaunchConfiguration('baud_rate')
                }
            ]
        )
    ])
