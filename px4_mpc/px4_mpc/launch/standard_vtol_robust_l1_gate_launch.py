"""Launch guarded 5 m/s, lambda >= 0.8 live allocation gate."""

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
                        "allow_l1_output": True,
                        "l1_target_speed": 5.0,
                        "l1_acceleration": 0.4,
                        "l1_hold_seconds": 3.0,
                        "l1_min_lambda": 0.8,
                        "l1_pusher_max": 0.25,
                    }
                ],
            )
        ]
    )
