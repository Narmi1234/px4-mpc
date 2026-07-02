#!/usr/bin/env python3
"""Launch standard VTOL NMPC, reference generator, and optional commander."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    altitude = LaunchConfiguration("altitude")
    forward_speed = LaunchConfiguration("forward_speed")
    profile = LaunchConfiguration("profile")
    transition_delay = LaunchConfiguration("transition_delay")
    speed_ramp_time = LaunchConfiguration("speed_ramp_time")
    lookahead_time = LaunchConfiguration("lookahead_time")
    min_lookahead = LaunchConfiguration("min_lookahead")
    control_mode = LaunchConfiguration("control_mode")
    manual_lift = LaunchConfiguration("manual_lift")
    manual_pusher = LaunchConfiguration("manual_pusher")
    manual_roll_rate = LaunchConfiguration("manual_roll_rate")
    manual_pitch_rate = LaunchConfiguration("manual_pitch_rate")
    manual_yaw_rate = LaunchConfiguration("manual_yaw_rate")
    control_dt = LaunchConfiguration("control_dt")
    horizon_steps = LaunchConfiguration("horizon_steps")
    max_ipopt_iter = LaunchConfiguration("max_ipopt_iter")
    rate_setpoint_limit = LaunchConfiguration("rate_setpoint_limit")
    nmpc_max_body_rate = LaunchConfiguration("nmpc_max_body_rate")
    max_safe_speed = LaunchConfiguration("max_safe_speed")
    speed_safety_min_altitude = LaunchConfiguration("speed_safety_min_altitude")
    max_safe_tilt_deg = LaunchConfiguration("max_safe_tilt_deg")
    max_odometry_position_norm = LaunchConfiguration(
        "max_odometry_position_norm"
    )
    fallback_hover_thrust = LaunchConfiguration("fallback_hover_thrust")
    fallback_altitude_gain = LaunchConfiguration("fallback_altitude_gain")
    fallback_vertical_velocity_gain = LaunchConfiguration(
        "fallback_vertical_velocity_gain"
    )
    fallback_attitude_gain = LaunchConfiguration("fallback_attitude_gain")
    fallback_min_thrust = LaunchConfiguration("fallback_min_thrust")
    fallback_max_thrust = LaunchConfiguration("fallback_max_thrust")
    auto_start = LaunchConfiguration("auto_start")

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("altitude", default_value="20.0"),
            DeclareLaunchArgument("forward_speed", default_value="16.0"),
            DeclareLaunchArgument("profile", default_value="transition"),
            DeclareLaunchArgument("transition_delay", default_value="5.0"),
            DeclareLaunchArgument("speed_ramp_time", default_value="8.0"),
            DeclareLaunchArgument("lookahead_time", default_value="2.7"),
            DeclareLaunchArgument("min_lookahead", default_value="10.0"),
            DeclareLaunchArgument("control_mode", default_value="nmpc"),
            DeclareLaunchArgument("manual_lift", default_value="0.0"),
            DeclareLaunchArgument("manual_pusher", default_value="0.0"),
            DeclareLaunchArgument("manual_roll_rate", default_value="0.0"),
            DeclareLaunchArgument("manual_pitch_rate", default_value="0.0"),
            DeclareLaunchArgument("manual_yaw_rate", default_value="0.0"),
            DeclareLaunchArgument("control_dt", default_value="0.05"),
            DeclareLaunchArgument("horizon_steps", default_value="8"),
            DeclareLaunchArgument("max_ipopt_iter", default_value="80"),
            DeclareLaunchArgument("rate_setpoint_limit", default_value="0.7"),
            DeclareLaunchArgument("nmpc_max_body_rate", default_value="2.0"),
            DeclareLaunchArgument("max_safe_speed", default_value="15.0"),
            DeclareLaunchArgument(
                "speed_safety_min_altitude",
                default_value="1.0",
            ),
            DeclareLaunchArgument("max_safe_tilt_deg", default_value="95.0"),
            DeclareLaunchArgument(
                "max_odometry_position_norm",
                default_value="1000.0",
            ),
            DeclareLaunchArgument("fallback_hover_thrust", default_value="0.56"),
            DeclareLaunchArgument("fallback_altitude_gain", default_value="0.03"),
            DeclareLaunchArgument(
                "fallback_vertical_velocity_gain",
                default_value="0.12",
            ),
            DeclareLaunchArgument("fallback_attitude_gain", default_value="1.8"),
            DeclareLaunchArgument("fallback_min_thrust", default_value="0.15"),
            DeclareLaunchArgument("fallback_max_thrust", default_value="0.75"),
            DeclareLaunchArgument("auto_start", default_value="false"),
            Node(
                package="px4_mpc",
                executable="mpc_standard_vtol",
                name="mpc_standard_vtol",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "namespace": namespace,
                        "control_mode": control_mode,
                        "manual_lift": ParameterValue(
                            manual_lift,
                            value_type=float,
                        ),
                        "manual_pusher": ParameterValue(
                            manual_pusher,
                            value_type=float,
                        ),
                        "manual_roll_rate": ParameterValue(
                            manual_roll_rate,
                            value_type=float,
                        ),
                        "manual_pitch_rate": ParameterValue(
                            manual_pitch_rate,
                            value_type=float,
                        ),
                        "manual_yaw_rate": ParameterValue(
                            manual_yaw_rate,
                            value_type=float,
                        ),
                        "control_dt": ParameterValue(
                            control_dt,
                            value_type=float,
                        ),
                        "horizon_steps": ParameterValue(
                            horizon_steps,
                            value_type=int,
                        ),
                        "max_ipopt_iter": ParameterValue(
                            max_ipopt_iter,
                            value_type=int,
                        ),
                        "rate_setpoint_limit": ParameterValue(
                            rate_setpoint_limit,
                            value_type=float,
                        ),
                        "nmpc_max_body_rate": ParameterValue(
                            nmpc_max_body_rate,
                            value_type=float,
                        ),
                        "max_safe_speed": ParameterValue(
                            max_safe_speed,
                            value_type=float,
                        ),
                        "speed_safety_min_altitude": ParameterValue(
                            speed_safety_min_altitude,
                            value_type=float,
                        ),
                        "max_safe_tilt_deg": ParameterValue(
                            max_safe_tilt_deg,
                            value_type=float,
                        ),
                        "max_odometry_position_norm": ParameterValue(
                            max_odometry_position_norm,
                            value_type=float,
                        ),
                        "fallback_hover_thrust": ParameterValue(
                            fallback_hover_thrust,
                            value_type=float,
                        ),
                        "fallback_altitude_gain": ParameterValue(
                            fallback_altitude_gain,
                            value_type=float,
                        ),
                        "fallback_vertical_velocity_gain": ParameterValue(
                            fallback_vertical_velocity_gain,
                            value_type=float,
                        ),
                        "fallback_attitude_gain": ParameterValue(
                            fallback_attitude_gain,
                            value_type=float,
                        ),
                        "fallback_min_thrust": ParameterValue(
                            fallback_min_thrust,
                            value_type=float,
                        ),
                        "fallback_max_thrust": ParameterValue(
                            fallback_max_thrust,
                            value_type=float,
                        ),
                        "reference_altitude": ParameterValue(
                            altitude,
                            value_type=float,
                        ),
                    }
                ],
            ),
            Node(
                package="px4_mpc",
                executable="standard_vtol_reference",
                name="standard_vtol_reference",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "namespace": namespace,
                        "profile": profile,
                        "altitude": ParameterValue(
                            altitude,
                            value_type=float,
                        ),
                        "forward_speed": ParameterValue(
                            forward_speed,
                            value_type=float,
                        ),
                        "transition_delay": ParameterValue(
                            transition_delay,
                            value_type=float,
                        ),
                        "speed_ramp_time": ParameterValue(
                            speed_ramp_time,
                            value_type=float,
                        ),
                        "lookahead_time": ParameterValue(
                            lookahead_time,
                            value_type=float,
                        ),
                        "min_lookahead": ParameterValue(
                            min_lookahead,
                            value_type=float,
                        ),
                        "max_odometry_position_norm": ParameterValue(
                            max_odometry_position_norm,
                            value_type=float,
                        ),
                    }
                ],
            ),
            Node(
                package="px4_mpc",
                executable="standard_vtol_commander",
                name="standard_vtol_commander",
                output="screen",
                emulate_tty=True,
                condition=IfCondition(auto_start),
                parameters=[{"namespace": namespace}],
            ),
        ]
    )
