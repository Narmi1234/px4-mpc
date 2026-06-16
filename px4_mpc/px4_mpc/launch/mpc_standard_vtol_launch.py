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
    control_dt = LaunchConfiguration("control_dt")
    horizon_steps = LaunchConfiguration("horizon_steps")
    max_ipopt_iter = LaunchConfiguration("max_ipopt_iter")
    auto_start = LaunchConfiguration("auto_start")

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("altitude", default_value="20.0"),
            DeclareLaunchArgument("forward_speed", default_value="16.0"),
            DeclareLaunchArgument("profile", default_value="transition"),
            DeclareLaunchArgument("control_dt", default_value="0.15"),
            DeclareLaunchArgument("horizon_steps", default_value="8"),
            DeclareLaunchArgument("max_ipopt_iter", default_value="80"),
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
