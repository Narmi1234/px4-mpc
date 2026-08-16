"""Launch the Standard VTOL NMPC in shadow or guarded MC test modes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    allow_output = LaunchConfiguration("allow_offboard_output")
    max_offboard_seconds = LaunchConfiguration("hover_offboard_max_seconds")
    forward_test_seconds = LaunchConfiguration("mc_forward_test_max_seconds")
    forward_target_speed = LaunchConfiguration("mc_forward_target_speed")
    forward_acceleration = LaunchConfiguration("mc_forward_acceleration")
    forward_hold_seconds = LaunchConfiguration("mc_forward_hold_seconds")
    forward_start_delay = LaunchConfiguration("mc_forward_start_delay_seconds")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "allow_offboard_output",
                default_value="false",
                description="Allow explicit guarded MC Offboard services",
            ),
            DeclareLaunchArgument(
                "hover_offboard_max_seconds",
                default_value="5.0",
                description="Automatic Position-mode fallback timeout",
            ),
            DeclareLaunchArgument(
                "mc_forward_test_max_seconds",
                default_value="15.0",
                description="Automatic fallback timeout for the MC-forward gate",
            ),
            DeclareLaunchArgument(
                "mc_forward_target_speed",
                default_value="2.0",
                description="Peak MC-forward reference speed in m/s",
            ),
            DeclareLaunchArgument(
                "mc_forward_acceleration",
                default_value="1.0",
                description="MC-forward acceleration and braking magnitude",
            ),
            DeclareLaunchArgument(
                "mc_forward_hold_seconds",
                default_value="1.0",
                description="Time at peak MC-forward speed",
            ),
            DeclareLaunchArgument(
                "mc_forward_start_delay_seconds",
                default_value="2.0",
                description="Stationary NMPC warm-up before MC acceleration",
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
                        "mc_forward_test_max_seconds": forward_test_seconds,
                        "mc_forward_target_speed": forward_target_speed,
                        "mc_forward_acceleration": forward_acceleration,
                        "mc_forward_hold_seconds": forward_hold_seconds,
                        "mc_forward_start_delay_seconds": forward_start_delay,
                    }
                ],
            ),
        ]
    )
