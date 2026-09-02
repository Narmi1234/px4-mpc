"""Launch guarded L3c 12 m/s, lambda >= 0.20 lift-transfer gate."""

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
                        "active_state_stale_abort_seconds": 0.45,
                        "max_airspeed_age_seconds": 0.75,
                        "allocation_inactive_abort_seconds": 0.35,
                        "allow_l3_output": True,
                        "l3_target_speed": 12.0,
                        "l3_acceleration": 0.25,
                        "l3_brake_rate": 0.30,
                        "l3_recovery_seconds": 0.1,
                        "l3_brake_entry_lambda": 0.70,
                        "l3_hold_seconds": 4.0,
                        "l3_min_lambda": 0.20,
                        "l3_pusher_max": 0.45,
                        "l3_collective_min": 0.30,
                        "l3_pitch_rate_limit": 0.25,
                        "l3_vertical_correction_gain": 1.50,
                        "l3_vertical_speed_limit": 0.90,
                        "l3_vertical_speed_persistence_seconds": 0.20,
                        "l3_vertical_speed_emergency_limit": 1.20,
                    }
                ],
            )
        ]
    )
