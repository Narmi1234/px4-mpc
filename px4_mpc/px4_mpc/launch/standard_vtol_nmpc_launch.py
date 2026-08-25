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
    allow_external_pusher = LaunchConfiguration("allow_external_pusher_output")
    pusher_test_seconds = LaunchConfiguration("external_pusher_test_max_seconds")
    pusher_peak = LaunchConfiguration("external_pusher_peak")
    pusher_slew = LaunchConfiguration("external_pusher_slew")
    pusher_hold = LaunchConfiguration("external_pusher_hold_seconds")
    pusher_delay = LaunchConfiguration("external_pusher_start_delay_seconds")
    allow_pusher_forward = LaunchConfiguration("allow_pusher_forward_output")
    pusher_forward_seconds = LaunchConfiguration(
        "pusher_forward_test_max_seconds"
    )
    pusher_forward_speed = LaunchConfiguration("pusher_forward_target_speed")
    pusher_forward_acceleration = LaunchConfiguration(
        "pusher_forward_acceleration"
    )
    pusher_forward_hold = LaunchConfiguration("pusher_forward_hold_seconds")
    pusher_forward_delay = LaunchConfiguration(
        "pusher_forward_start_delay_seconds"
    )
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
            DeclareLaunchArgument(
                "allow_external_pusher_output",
                default_value="false",
                description="Allow the custom PX4 external-pusher pulse gate",
            ),
            DeclareLaunchArgument(
                "external_pusher_test_max_seconds",
                default_value="12.0",
                description="Automatic Position fallback for pusher pulse",
            ),
            DeclareLaunchArgument(
                "external_pusher_peak",
                default_value="0.05",
                description="First external pusher gate peak command",
            ),
            DeclareLaunchArgument(
                "external_pusher_slew",
                default_value="0.02",
                description="First external pusher gate command slew in 1/s",
            ),
            DeclareLaunchArgument(
                "external_pusher_hold_seconds",
                default_value="2.0",
                description="Time at peak external pusher command",
            ),
            DeclareLaunchArgument(
                "external_pusher_start_delay_seconds",
                default_value="2.0",
                description="Stationary delay before external pusher ramp",
            ),
            DeclareLaunchArgument(
                "allow_pusher_forward_output",
                default_value="false",
                description="Allow the guarded Gate A pusher-feedback test",
            ),
            DeclareLaunchArgument(
                "pusher_forward_test_max_seconds",
                default_value="26.5",
                description="PX4-time timeout for the Gate A profile",
            ),
            DeclareLaunchArgument(
                "pusher_forward_target_speed",
                default_value="3.0",
                description="Gate A forward-speed reference in m/s",
            ),
            DeclareLaunchArgument(
                "pusher_forward_acceleration",
                default_value="0.50",
                description="Gate A reference acceleration in m/s^2",
            ),
            DeclareLaunchArgument(
                "pusher_forward_hold_seconds",
                default_value="2.0",
                description="Gate A hold time at peak speed",
            ),
            DeclareLaunchArgument(
                "pusher_forward_start_delay_seconds",
                default_value="2.0",
                description="Gate A stationary warm-up before acceleration",
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
                        "allow_external_pusher_output": allow_external_pusher,
                        "external_pusher_test_max_seconds": pusher_test_seconds,
                        "external_pusher_peak": pusher_peak,
                        "external_pusher_slew": pusher_slew,
                        "external_pusher_hold_seconds": pusher_hold,
                        "external_pusher_start_delay_seconds": pusher_delay,
                        "allow_pusher_forward_output": allow_pusher_forward,
                        "pusher_forward_test_max_seconds": pusher_forward_seconds,
                        "pusher_forward_target_speed": pusher_forward_speed,
                        "pusher_forward_acceleration": pusher_forward_acceleration,
                        "pusher_forward_hold_seconds": pusher_forward_hold,
                        "pusher_forward_start_delay_seconds": pusher_forward_delay,
                    }
                ],
            ),
        ]
    )
