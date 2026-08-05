"""Launch file for the MarsDog Action Executor node.

Usage::

    ros2 launch marsdog_action_executor action_executor.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("marsdog_action_executor"),
                        "config",
                        "attention_tracking.yaml",
                    ]
                ),
                description=(
                    "ROS 2 parameter file loaded after launch defaults; values in "
                    "this file take precedence"
                ),
            ),
            DeclareLaunchArgument(
                "agv_enabled",
                default_value="false",
                description="Enable ACT_* to geometry_msgs/Twist AGV output",
            ),
            DeclareLaunchArgument(
                "agv_cmd_vel_topic",
                default_value="/cmd_vel",
                description="AGV velocity command topic",
            ),
            DeclareLaunchArgument(
                "agv_publish_rate_hz",
                default_value="10.0",
                description="Twist command publication rate",
            ),
            DeclareLaunchArgument(
                "recharge_result_energy_value",
                default_value="100.0",
                description=(
                    "Actual battery percentage returned after a successful "
                    "recharge action; replace with BMS data when available"
                ),
            ),
            DeclareLaunchArgument(
                "wake_orientation_enabled",
                default_value="true",
                description="Orient respond_owner_call using Nav2 Spin",
            ),
            DeclareLaunchArgument(
                "wake_spin_action_name",
                default_value="/spin",
                description="Nav2 Spin action name",
            ),
            DeclareLaunchArgument(
                "wake_spin_server_timeout_sec",
                default_value="5.0",
                description="Seconds to wait for the Nav2 Spin server",
            ),
            DeclareLaunchArgument(
                "wake_spin_result_timeout_sec",
                default_value="15.0",
                description="Maximum wait for a wake orientation result",
            ),
            DeclareLaunchArgument(
                "wake_spin_time_allowance_sec",
                default_value="12.0",
                description="Time allowance sent to each Nav2 Spin goal",
            ),
            DeclareLaunchArgument(
                "wake_angle_zero_offset_deg",
                default_value="0.0",
                description="Raw wake angle measured straight ahead",
            ),
            DeclareLaunchArgument(
                "wake_angle_direction_sign",
                default_value="1.0",
                description="Use -1 when hardware angle direction is reversed",
            ),
            DeclareLaunchArgument(
                "wake_angle_deadband_deg",
                default_value="5.0",
                description="Do not rotate inside this forward deadband",
            ),
            DeclareLaunchArgument(
                "wake_angle_frame_id",
                default_value="base_link",
                description="Required coordinate frame for wake_angle_deg",
            ),
            DeclareLaunchArgument(
                "attention_tracking_enabled",
                default_value="true",
                description="Center the active visual target during voice sessions",
            ),
            DeclareLaunchArgument(
                "attention_tracking_gain",
                default_value="0.8",
                description="Proportional gain from normalized image error to yaw rate",
            ),
            DeclareLaunchArgument(
                "attention_tracking_deadband",
                default_value="0.08",
                description="Normalized horizontal centering deadband",
            ),
            DeclareLaunchArgument(
                "attention_tracking_activation_deadband",
                default_value="0.14",
                description="Error required to resume after entering the center zone",
            ),
            DeclareLaunchArgument(
                "attention_tracking_smoothing_alpha",
                default_value="0.15",
                description="EMA weight for each new visual center sample",
            ),
            DeclareLaunchArgument(
                "attention_tracking_max_angular_z",
                default_value="0.25",
                description="Maximum session-tracking yaw rate in rad/s",
            ),
            DeclareLaunchArgument(
                "attention_tracking_max_angular_accel",
                default_value="0.25",
                description="Maximum yaw-rate change per second",
            ),
            DeclareLaunchArgument(
                "attention_tracking_visual_timeout_sec",
                default_value="0.8",
                description="Stop when visual target data is older than this",
            ),
            DeclareLaunchArgument(
                "attention_tracking_direction_sign",
                default_value="-1.0",
                description="Flip to 1.0 if camera horizontal direction is reversed",
            ),
            DeclareLaunchArgument(
                "follow_target_height",
                default_value="0.68",
                description="Normalized body-box height at the desired following distance",
            ),
            DeclareLaunchArgument(
                "follow_height_deadband",
                default_value="0.05",
                description="Distance-size error at which forward motion stops",
            ),
            DeclareLaunchArgument(
                "follow_activation_deadband",
                default_value="0.10",
                description="Distance-size error required to resume forward motion",
            ),
            DeclareLaunchArgument(
                "follow_linear_gain",
                default_value="0.55",
                description="Proportional gain from body-size error to forward speed",
            ),
            DeclareLaunchArgument(
                "follow_max_linear_x",
                default_value="0.22",
                description="Maximum visual-follow forward speed in m/s",
            ),
            DeclareLaunchArgument(
                "follow_max_linear_accel",
                default_value="0.20",
                description="Maximum visual-follow speed change per second",
            ),
            DeclareLaunchArgument(
                "follow_max_heading_error",
                default_value="0.30",
                description="Stop driving forward beyond this normalized horizontal error",
            ),
            DeclareLaunchArgument(
                "navigation_enabled",
                default_value="false",
                description="Enable Nav2 semantic-waypoint behavior routing",
            ),
            DeclareLaunchArgument(
                "navigation_action_name",
                default_value="/navigate_to_pose",
                description="Nav2 NavigateToPose action name",
            ),
            DeclareLaunchArgument(
                "navigation_frame_id",
                default_value="map",
                description="Frame used by semantic waypoint poses",
            ),
            DeclareLaunchArgument(
                "navigation_server_timeout_sec",
                default_value="10.0",
                description="Seconds to wait for the Nav2 action server",
            ),
            DeclareLaunchArgument(
                "navigation_result_timeout_sec",
                default_value="300.0",
                description="Maximum navigation wait per behavior",
            ),
            Node(
                package="marsdog_action_executor",
                executable="action_executor_node",
                name="action_executor_node",
                output="screen",
                parameters=[
                    {
                        "agv_enabled": ParameterValue(
                            LaunchConfiguration("agv_enabled"),
                            value_type=bool,
                        ),
                        "agv_cmd_vel_topic": ParameterValue(
                            LaunchConfiguration("agv_cmd_vel_topic"),
                            value_type=str,
                        ),
                        "agv_publish_rate_hz": ParameterValue(
                            LaunchConfiguration("agv_publish_rate_hz"),
                            value_type=float,
                        ),
                        "recharge_result_energy_value": ParameterValue(
                            LaunchConfiguration(
                                "recharge_result_energy_value"
                            ),
                            value_type=float,
                        ),
                        "wake_orientation_enabled": ParameterValue(
                            LaunchConfiguration(
                                "wake_orientation_enabled"
                            ),
                            value_type=bool,
                        ),
                        "wake_spin_action_name": ParameterValue(
                            LaunchConfiguration("wake_spin_action_name"),
                            value_type=str,
                        ),
                        "wake_spin_server_timeout_sec": ParameterValue(
                            LaunchConfiguration(
                                "wake_spin_server_timeout_sec"
                            ),
                            value_type=float,
                        ),
                        "wake_spin_result_timeout_sec": ParameterValue(
                            LaunchConfiguration(
                                "wake_spin_result_timeout_sec"
                            ),
                            value_type=float,
                        ),
                        "wake_spin_time_allowance_sec": ParameterValue(
                            LaunchConfiguration(
                                "wake_spin_time_allowance_sec"
                            ),
                            value_type=float,
                        ),
                        "wake_angle_zero_offset_deg": ParameterValue(
                            LaunchConfiguration(
                                "wake_angle_zero_offset_deg"
                            ),
                            value_type=float,
                        ),
                        "wake_angle_direction_sign": ParameterValue(
                            LaunchConfiguration(
                                "wake_angle_direction_sign"
                            ),
                            value_type=float,
                        ),
                        "wake_angle_deadband_deg": ParameterValue(
                            LaunchConfiguration(
                                "wake_angle_deadband_deg"
                            ),
                            value_type=float,
                        ),
                        "wake_angle_frame_id": ParameterValue(
                            LaunchConfiguration("wake_angle_frame_id"),
                            value_type=str,
                        ),
                        "attention_tracking_enabled": ParameterValue(
                            LaunchConfiguration("attention_tracking_enabled"),
                            value_type=bool,
                        ),
                        "attention_tracking_gain": ParameterValue(
                            LaunchConfiguration("attention_tracking_gain"),
                            value_type=float,
                        ),
                        "attention_tracking_deadband": ParameterValue(
                            LaunchConfiguration("attention_tracking_deadband"),
                            value_type=float,
                        ),
                        "attention_tracking_activation_deadband": ParameterValue(
                            LaunchConfiguration(
                                "attention_tracking_activation_deadband"
                            ),
                            value_type=float,
                        ),
                        "attention_tracking_smoothing_alpha": ParameterValue(
                            LaunchConfiguration(
                                "attention_tracking_smoothing_alpha"
                            ),
                            value_type=float,
                        ),
                        "attention_tracking_max_angular_z": ParameterValue(
                            LaunchConfiguration(
                                "attention_tracking_max_angular_z"
                            ),
                            value_type=float,
                        ),
                        "attention_tracking_max_angular_accel": ParameterValue(
                            LaunchConfiguration(
                                "attention_tracking_max_angular_accel"
                            ),
                            value_type=float,
                        ),
                        "attention_tracking_visual_timeout_sec": ParameterValue(
                            LaunchConfiguration(
                                "attention_tracking_visual_timeout_sec"
                            ),
                            value_type=float,
                        ),
                        "attention_tracking_direction_sign": ParameterValue(
                            LaunchConfiguration(
                                "attention_tracking_direction_sign"
                            ),
                            value_type=float,
                        ),
                        "follow_target_height": ParameterValue(
                            LaunchConfiguration("follow_target_height"),
                            value_type=float,
                        ),
                        "follow_height_deadband": ParameterValue(
                            LaunchConfiguration("follow_height_deadband"),
                            value_type=float,
                        ),
                        "follow_activation_deadband": ParameterValue(
                            LaunchConfiguration("follow_activation_deadband"),
                            value_type=float,
                        ),
                        "follow_linear_gain": ParameterValue(
                            LaunchConfiguration("follow_linear_gain"),
                            value_type=float,
                        ),
                        "follow_max_linear_x": ParameterValue(
                            LaunchConfiguration("follow_max_linear_x"),
                            value_type=float,
                        ),
                        "follow_max_linear_accel": ParameterValue(
                            LaunchConfiguration("follow_max_linear_accel"),
                            value_type=float,
                        ),
                        "follow_max_heading_error": ParameterValue(
                            LaunchConfiguration("follow_max_heading_error"),
                            value_type=float,
                        ),
                        "navigation_enabled": ParameterValue(
                            LaunchConfiguration("navigation_enabled"),
                            value_type=bool,
                        ),
                        "navigation_action_name": ParameterValue(
                            LaunchConfiguration("navigation_action_name"),
                            value_type=str,
                        ),
                        "navigation_frame_id": ParameterValue(
                            LaunchConfiguration("navigation_frame_id"),
                            value_type=str,
                        ),
                        "navigation_server_timeout_sec": ParameterValue(
                            LaunchConfiguration(
                                "navigation_server_timeout_sec"
                            ),
                            value_type=float,
                        ),
                        "navigation_result_timeout_sec": ParameterValue(
                            LaunchConfiguration(
                                "navigation_result_timeout_sec"
                            ),
                            value_type=float,
                        ),
                    },
                    LaunchConfiguration("params_file"),
                ],
            ),
        ]
    )
