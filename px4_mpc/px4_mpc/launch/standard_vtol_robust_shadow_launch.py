"""Launch the read-only 16-state Standard VTOL robust NMPC shadow."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("horizon_steps", default_value="25"),
            DeclareLaunchArgument("horizon_seconds", default_value="2.0"),
            Node(
                package="px4_mpc",
                executable="standard_vtol_robust_shadow",
                name="standard_vtol_robust_shadow",
                output="screen",
                parameters=[
                    {
                        "horizon_steps": LaunchConfiguration("horizon_steps"),
                        "horizon_seconds": LaunchConfiguration("horizon_seconds"),
                    }
                ],
            ),
        ]
    )
