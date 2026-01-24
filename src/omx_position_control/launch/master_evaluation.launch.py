from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Starts the evaluation stack (without the vision script):
      - websocket_bridge_node
      - inverse_kinematics_node
      - realtime_control_node
      - master_evaluator_node (CSV logger; timed exit)
    """
    eval_dur_arg = DeclareLaunchArgument(
        "evaluation_duration_s",
        default_value="60.0",
        description="How long to log, in seconds, before saving CSV and exiting.",
    )

    home_pose = Node(
        package="omx_position_control",
        executable="basic_robot_control",
        name="basic_robot_control",
        output="screen",
    )

    # Params (tweak as needed)
    evaluation_duration_s = LaunchConfiguration("evaluation_duration_s")

    websocket_bridge = Node(
        package="omx_position_control",
        executable="websocket_bridge_node",
        name="websocket_bridge_node",
        output="screen",
        parameters=[
            {
                "websocket_url": "ws://localhost:8765",
                "pose_topic": "/target_pose",
                "robot_frame_id": "base_link",
                "control_frequency": 30.0,
            }
        ],
    )

    ik_node = Node(
        package="omx_position_control",
        executable="inverse_kinematics_node",
        name="inverse_kinematics_node",
        output="screen",
    )

    realtime_ctrl = Node(
        package="omx_position_control",
        executable="realtime_control_node",
        name="realtime_control_node",
        output="screen",
        parameters=[
            {
                "target_pose_topic": "/target_pose",
                "teleop_metrics_topic": "/teleop_metrics",
                "ik_service_name": "calculate_ik",
                "joint_service_name": "goal_joint_space_path",
                "path_time": 0.30,
                "joint_names": ["joint1", "joint2", "joint3", "joint4"],
            }
        ],
    )

    master_eval = Node(
        package="omx_position_control",
        executable="master_evaluator_node",
        name="master_evaluator_node",
        output="screen",
        parameters=[
            {
                "teleop_metrics_topic": "/teleop_metrics",
                "kinematics_pose_topic": "/kinematics_pose",
                "evaluation_duration_s": evaluation_duration_s,
                "sync_slop_s": 0.15,
                "qos_depth": 50,
            }
        ],
    )

    ld = LaunchDescription()
    ld.add_action(home_pose)
    ld.add_action(eval_dur_arg)
    ld.add_action(websocket_bridge)
    ld.add_action(ik_node)
    ld.add_action(realtime_ctrl)
    ld.add_action(master_eval)
    return ld
