"""Launch the Standard VTOL NMPC in shadow or guarded hover mode."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    allow_output = LaunchConfiguration("allow_offboard_output")
    max_offboard_seconds = LaunchConfiguration("hover_offboard_max_seconds")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "allow_offboard_output",
                default_value="false",
                description="Allow explicit hover-only Offboard service",
            ),
            DeclareLaunchArgument(
                "hover_offboard_max_seconds",
                default_value="5.0",
                description="Automatic Position-mode fallback timeout",
            ),
            Node(
                package="px4_mpc",
                executable="standard_vtol_nmpc",
                name="standard_vtol_nmpc",
                output="screen",
                parameters=[
                    {
                        "allow_offboard_output": allow_output,
                        "hover_offboard_max_seconds": max_offboard_seconds,
                    }
                ],
            ),
        ]
    )
