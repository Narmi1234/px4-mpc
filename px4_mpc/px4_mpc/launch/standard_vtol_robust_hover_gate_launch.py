"""Launch the guarded five-second robust-NMPC hover output gate."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="px4_mpc",
                executable="standard_vtol_robust_shadow",
                name="standard_vtol_robust_shadow",
                output="screen",
                parameters=[
                    {
                        "horizon_steps": 20,
                        "horizon_seconds": 2.0,
                        "allow_hover_output": True,
                        "hover_test_seconds": 5.0,
                    }
                ],
            )
        ]
    )
