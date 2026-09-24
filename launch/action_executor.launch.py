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
                "chassis_type",
                default_value="go2",
                description="Chassis backend: 'go2' or 'lite3'",
            ),
            DeclareLaunchArgument(
                "go2_enabled",
                default_value="false",
                description="Enable Unitree Go2 SportMode output",
            ),
            DeclareLaunchArgument(
                "go2_request_topic",
                default_value="/api/sport/request",
                description="Unitree Go2 SportMode request topic",
            ),
            DeclareLaunchArgument(
                "go2_publish_rate_hz",
                default_value="10.0",
                description="Go2 Move command refresh rate",
            ),
            DeclareLaunchArgument(
                "go2_max_linear_x",
                default_value="0.30",
                description="Go2 forward/backward velocity software clamp",
            ),
            DeclareLaunchArgument(
                "go2_max_linear_y",
                default_value="0.20",
                description="Go2 lateral velocity software clamp",
            ),
            DeclareLaunchArgument(
                "go2_max_angular_z",
                default_value="1.20",
                description="Go2 yaw velocity software clamp",
            ),
            DeclareLaunchArgument(
                "lite3_enabled",
                default_value="false",
                description="Enable Lite3 /cmd_vel + /simple_cmd output",
            ),
            DeclareLaunchArgument(
                "lite3_simple_cmd_topic",
                default_value="/simple_cmd",
                description="Lite3 MotionSimpleCMD topic",
            ),
            DeclareLaunchArgument(
                "lite3_status_topic",
                default_value="/robot_status",
                description="Lite3 RobotStatus feedback topic",
            ),
            DeclareLaunchArgument(
                "lite3_cmd_vel_topic",
                default_value="/cmd_vel",
                description="Lite3 direct velocity topic; bringup bridge must be off",
            ),
            DeclareLaunchArgument(
                "lite3_allow_proxies",
                default_value="true",
                description="Allow safe Lite3 HOLD/rear-guarded behavior proxies",
            ),
            DeclareLaunchArgument(
                "lite3_allow_unverified",
                default_value="false",
                description="Allow Lite3 commands pending hardware acceptance",
            ),
            DeclareLaunchArgument(
                "lite3_publish_rate_hz",
                default_value="20.0",
                description="Lite3 velocity and pose-axis refresh rate",
            ),
            # Must match ``limits`` in config/lite3_actions.yaml: this value is
            # passed as a parameter and wins over the config-derived default, so
            # a drift here silently clamps the 上厕所 orbit to a wider radius.
            DeclareLaunchArgument(
                "lite3_max_linear_x",
                default_value="0.24",
                description="Lite3 forward/backward velocity clamp",
            ),
            DeclareLaunchArgument(
                "lite3_max_linear_y",
                default_value="0.15",
                description="Lite3 lateral velocity clamp",
            ),
            DeclareLaunchArgument(
                "lite3_max_angular_z",
                default_value="1.20",
                description="Lite3 yaw velocity clamp",
            ),
            DeclareLaunchArgument(
                "uwb_follow_enabled",
                default_value="true",
                description=(
                    "Enable the chassis-specific UWB owner-follow adapter"
                ),
            ),
            DeclareLaunchArgument(
                "go2_uwb_request_topic",
                default_value="/api/uwbswitch/request",
                description="Go2 built-in UTrack request topic",
            ),
            DeclareLaunchArgument(
                "go2_uwb_response_topic",
                default_value="/api/uwbswitch/response",
                description="Go2 built-in UTrack response topic",
            ),
            DeclareLaunchArgument(
                "go2_uwb_state_topic",
                default_value="/uwbstate",
                description="Go2 built-in UTrack telemetry topic",
            ),
            DeclareLaunchArgument(
                "go2_uwb_response_timeout_sec",
                default_value="2.0",
                description="Timeout for a correlated Go2 UTrack response",
            ),
            DeclareLaunchArgument(
                "uwb_follow_setup_script",
                default_value="/home/cat/robot_ws/go2_follow/install/setup.bash",
                description="Go2 external UWB workspace setup",
            ),
            DeclareLaunchArgument(
                "uwb_follow_serial_device",
                default_value=(
                    "/dev/serial/by-id/"
                    "usb-FTDI_FT232R_USB_UART_AP2315SD-if00-port0"
                ),
                description="Stable serial device used by libAoa_robot_example",
            ),
            DeclareLaunchArgument(
                "uwb_follow_cmd_vel_topic",
                default_value="/cmd_vel",
                description="Final UWB local planner velocity topic",
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
                default_value="90.0",
                description="Raw wake angle measured straight ahead",
            ),
            DeclareLaunchArgument(
                "wake_angle_direction_sign",
                default_value="-1.0",
                description="Use -1 when hardware angle direction is reversed",
            ),
            DeclareLaunchArgument(
                "wake_angle_deadband_deg",
                default_value="5.0",
                description="Do not rotate inside this forward deadband",
            ),
            DeclareLaunchArgument(
                "wake_angle_frame_id",
                default_value="microphone_array",
                description=(
                    "Required raw wake-angle frame; Action calibrates it into "
                    "base_link-relative yaw"
                ),
            ),
            DeclareLaunchArgument(
                "wake_linear_array_back_search_enabled",
                default_value="false",
                description=(
                    "Check the mirrored rear heading after a turn with no "
                    "fresh visual human; needs /perception/visual_event"
                ),
            ),
            DeclareLaunchArgument(
                "wake_visual_confirm_timeout_sec",
                default_value="1.5",
                description="Seconds to wait for a fresh human after each Spin",
            ),
            DeclareLaunchArgument(
                "wake_visual_min_confidence",
                default_value="0.60",
                description="Minimum active-target confidence for wake confirmation",
            ),
            DeclareLaunchArgument(
                "wake_visual_max_age_ms",
                default_value="800.0",
                description="Maximum active-target observation age for wake confirmation",
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
                description="Legacy visual-follow parameter; ignored by UWB follow",
            ),
            DeclareLaunchArgument(
                "follow_height_deadband",
                default_value="0.05",
                description="Legacy visual-follow parameter; ignored by UWB follow",
            ),
            DeclareLaunchArgument(
                "follow_activation_deadband",
                default_value="0.10",
                description="Legacy visual-follow parameter; ignored by UWB follow",
            ),
            DeclareLaunchArgument(
                "follow_linear_gain",
                default_value="1.0",
                description="Legacy visual-follow parameter; ignored by UWB follow",
            ),
            DeclareLaunchArgument(
                "follow_max_linear_x",
                default_value="0.25",
                description="Legacy visual-follow parameter; ignored by UWB follow",
            ),
            DeclareLaunchArgument(
                "follow_max_linear_accel",
                default_value="0.50",
                description="Legacy visual-follow parameter; ignored by UWB follow",
            ),
            DeclareLaunchArgument(
                "follow_max_heading_error",
                default_value="0.30",
                description="Legacy visual-follow parameter; ignored by UWB follow",
            ),
            DeclareLaunchArgument(
                "target_approach_enabled",
                default_value="true",
                description="Enable one-shot closed-loop approach to an immutable visual target",
            ),
            DeclareLaunchArgument(
                "target_approach_allow_bbox_distance_fallback",
                default_value="false",
                description="DEMO ONLY: permit bbox height when metric distance is unavailable",
            ),
            DeclareLaunchArgument(
                "target_approach_stop_distance_m",
                default_value="1.20",
                description="Metric stopping distance for approach_voice_caller",
            ),
            DeclareLaunchArgument(
                "target_approach_minimum_safe_distance_m",
                default_value="0.80",
                description=(
                    "Hard lower safety distance; a Goal may only make this "
                    "larger"
                ),
            ),
            DeclareLaunchArgument(
                "target_approach_timeout_sec",
                default_value="20.0",
                description="Maximum closed-loop target approach duration",
            ),
            DeclareLaunchArgument(
                "navigation_enabled",
                default_value="false",
                description="Enable waypoint_nav fixed points and random Nav2 routing",
            ),
            DeclareLaunchArgument(
                "navigation_action_name",
                default_value="/navigate_to_pose",
                description="Legacy direct Nav2 fallback; production random routes use A-E waypoint_nav",
            ),
            DeclareLaunchArgument(
                "navigation_frame_id",
                default_value="map",
                description="Frame used by the legacy direct Nav2 fallback",
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
            DeclareLaunchArgument(
                "waypoint_nav_service_name",
                default_value="/waypoint_nav/task",
                description="Shared named-waypoint VoiceTask service",
            ),
            DeclareLaunchArgument(
                "waypoint_nav_status_topic",
                default_value="/waypoint_nav/status",
                description="Shared named-waypoint JSON status topic",
            ),
            DeclareLaunchArgument(
                "waypoint_nav_protocol_version",
                default_value="1.0",
                description="waypoint_nav JSON protocol version",
            ),
            DeclareLaunchArgument(
                "waypoint_nav_client_id",
                default_value="marsdog_action_executor",
                description="Stable caller identity for waypoint_nav",
            ),
            DeclareLaunchArgument(
                "waypoint_nav_service_timeout_sec",
                default_value="5.0",
                description="Timeout for one waypoint_nav service call",
            ),
            DeclareLaunchArgument(
                "waypoint_nav_query_timeout_sec",
                default_value="2.0",
                description="Timeout for durable waypoint task query",
            ),
            DeclareLaunchArgument(
                "waypoint_nav_cancel_confirmation_timeout_sec",
                default_value="5.0",
                description="Delay before querying for a missing cancel terminal",
            ),
            DeclareLaunchArgument(
                "waypoint_nav_terminal_retention_sec",
                default_value="86400.0",
                description="Required waypoint terminal retention period",
            ),
            DeclareLaunchArgument(
                "navigation_preempt_lock_wait_sec",
                default_value="8.0",
                description="Wait budget for the canceled task to release execution lock",
            ),
            Node(
                package="marsdog_action_executor",
                executable="action_executor_node",
                name="action_executor_node",
                output="screen",
                parameters=[
                    {
                        "chassis_type": ParameterValue(
                            LaunchConfiguration("chassis_type"),
                            value_type=str,
                        ),
                        "go2_enabled": ParameterValue(
                            LaunchConfiguration("go2_enabled"),
                            value_type=bool,
                        ),
                        "go2_request_topic": ParameterValue(
                            LaunchConfiguration("go2_request_topic"),
                            value_type=str,
                        ),
                        "go2_publish_rate_hz": ParameterValue(
                            LaunchConfiguration("go2_publish_rate_hz"),
                            value_type=float,
                        ),
                        "go2_max_linear_x": ParameterValue(
                            LaunchConfiguration("go2_max_linear_x"),
                            value_type=float,
                        ),
                        "go2_max_linear_y": ParameterValue(
                            LaunchConfiguration("go2_max_linear_y"),
                            value_type=float,
                        ),
                        "go2_max_angular_z": ParameterValue(
                            LaunchConfiguration("go2_max_angular_z"),
                            value_type=float,
                        ),
                        "lite3_enabled": ParameterValue(
                            LaunchConfiguration("lite3_enabled"),
                            value_type=bool,
                        ),
                        "lite3_simple_cmd_topic": ParameterValue(
                            LaunchConfiguration("lite3_simple_cmd_topic"),
                            value_type=str,
                        ),
                        "lite3_status_topic": ParameterValue(
                            LaunchConfiguration("lite3_status_topic"),
                            value_type=str,
                        ),
                        "lite3_cmd_vel_topic": ParameterValue(
                            LaunchConfiguration("lite3_cmd_vel_topic"),
                            value_type=str,
                        ),
                        "lite3_allow_proxies": ParameterValue(
                            LaunchConfiguration("lite3_allow_proxies"),
                            value_type=bool,
                        ),
                        "lite3_allow_unverified": ParameterValue(
                            LaunchConfiguration("lite3_allow_unverified"),
                            value_type=bool,
                        ),
                        "lite3_publish_rate_hz": ParameterValue(
                            LaunchConfiguration("lite3_publish_rate_hz"),
                            value_type=float,
                        ),
                        "lite3_max_linear_x": ParameterValue(
                            LaunchConfiguration("lite3_max_linear_x"),
                            value_type=float,
                        ),
                        "lite3_max_linear_y": ParameterValue(
                            LaunchConfiguration("lite3_max_linear_y"),
                            value_type=float,
                        ),
                        "lite3_max_angular_z": ParameterValue(
                            LaunchConfiguration("lite3_max_angular_z"),
                            value_type=float,
                        ),
                        "uwb_follow_enabled": ParameterValue(
                            LaunchConfiguration("uwb_follow_enabled"),
                            value_type=bool,
                        ),
                        "go2_uwb_request_topic": ParameterValue(
                            LaunchConfiguration("go2_uwb_request_topic"),
                            value_type=str,
                        ),
                        "go2_uwb_response_topic": ParameterValue(
                            LaunchConfiguration("go2_uwb_response_topic"),
                            value_type=str,
                        ),
                        "go2_uwb_state_topic": ParameterValue(
                            LaunchConfiguration("go2_uwb_state_topic"),
                            value_type=str,
                        ),
                        "go2_uwb_response_timeout_sec": ParameterValue(
                            LaunchConfiguration(
                                "go2_uwb_response_timeout_sec"
                            ),
                            value_type=float,
                        ),
                        "uwb_follow_setup_script": ParameterValue(
                            LaunchConfiguration("uwb_follow_setup_script"),
                            value_type=str,
                        ),
                        "uwb_follow_serial_device": ParameterValue(
                            LaunchConfiguration("uwb_follow_serial_device"),
                            value_type=str,
                        ),
                        "uwb_follow_cmd_vel_topic": ParameterValue(
                            LaunchConfiguration("uwb_follow_cmd_vel_topic"),
                            value_type=str,
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
                        "wake_linear_array_back_search_enabled": ParameterValue(
                            LaunchConfiguration(
                                "wake_linear_array_back_search_enabled"
                            ),
                            value_type=bool,
                        ),
                        "wake_visual_confirm_timeout_sec": ParameterValue(
                            LaunchConfiguration(
                                "wake_visual_confirm_timeout_sec"
                            ),
                            value_type=float,
                        ),
                        "wake_visual_min_confidence": ParameterValue(
                            LaunchConfiguration(
                                "wake_visual_min_confidence"
                            ),
                            value_type=float,
                        ),
                        "wake_visual_max_age_ms": ParameterValue(
                            LaunchConfiguration("wake_visual_max_age_ms"),
                            value_type=float,
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
                        "target_approach_enabled": ParameterValue(
                            LaunchConfiguration("target_approach_enabled"),
                            value_type=bool,
                        ),
                        "target_approach_allow_bbox_distance_fallback": ParameterValue(
                            LaunchConfiguration(
                                "target_approach_allow_bbox_distance_fallback"
                            ),
                            value_type=bool,
                        ),
                        "target_approach_stop_distance_m": ParameterValue(
                            LaunchConfiguration(
                                "target_approach_stop_distance_m"
                            ),
                            value_type=float,
                        ),
                        "target_approach_minimum_safe_distance_m": ParameterValue(
                            LaunchConfiguration(
                                "target_approach_minimum_safe_distance_m"
                            ),
                            value_type=float,
                        ),
                        "target_approach_timeout_sec": ParameterValue(
                            LaunchConfiguration(
                                "target_approach_timeout_sec"
                            ),
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
                        "waypoint_nav_service_name": ParameterValue(
                            LaunchConfiguration("waypoint_nav_service_name"),
                            value_type=str,
                        ),
                        "waypoint_nav_status_topic": ParameterValue(
                            LaunchConfiguration("waypoint_nav_status_topic"),
                            value_type=str,
                        ),
                        "waypoint_nav_protocol_version": ParameterValue(
                            LaunchConfiguration("waypoint_nav_protocol_version"),
                            value_type=str,
                        ),
                        "waypoint_nav_client_id": ParameterValue(
                            LaunchConfiguration("waypoint_nav_client_id"),
                            value_type=str,
                        ),
                        "waypoint_nav_service_timeout_sec": ParameterValue(
                            LaunchConfiguration(
                                "waypoint_nav_service_timeout_sec"
                            ),
                            value_type=float,
                        ),
                        "waypoint_nav_query_timeout_sec": ParameterValue(
                            LaunchConfiguration(
                                "waypoint_nav_query_timeout_sec"
                            ),
                            value_type=float,
                        ),
                        "waypoint_nav_cancel_confirmation_timeout_sec": ParameterValue(
                            LaunchConfiguration(
                                "waypoint_nav_cancel_confirmation_timeout_sec"
                            ),
                            value_type=float,
                        ),
                        "waypoint_nav_terminal_retention_sec": ParameterValue(
                            LaunchConfiguration(
                                "waypoint_nav_terminal_retention_sec"
                            ),
                            value_type=float,
                        ),
                        "navigation_preempt_lock_wait_sec": ParameterValue(
                            LaunchConfiguration(
                                "navigation_preempt_lock_wait_sec"
                            ),
                            value_type=float,
                        ),
                    },
                    LaunchConfiguration("params_file"),
                ],
            ),
        ]
    )
