"""Launch guarded 9 m/s, lambda >= 0.5 live allocation gate."""

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
                        "allow_l2_output": True,
                        "l2_target_speed": 9.0,
                        "l2_acceleration": 0.4,
                        "l2_hold_seconds": 3.0,
                        "l2_min_lambda": 0.5,
                        "l2_pusher_max": 0.35,
                    }
                ],
            )
        ]
    )
