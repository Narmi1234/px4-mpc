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
    allow_pretransition = LaunchConfiguration("allow_pretransition_output")
    pretransition_seconds = LaunchConfiguration("pretransition_test_max_seconds")
    pretransition_speed = LaunchConfiguration("pretransition_target_speed")
    pretransition_acceleration = LaunchConfiguration(
        "pretransition_acceleration"
    )
    pretransition_hold = LaunchConfiguration("pretransition_hold_seconds")
    pretransition_delay = LaunchConfiguration(
        "pretransition_start_delay_seconds"
    )
    allow_lift_unloading = LaunchConfiguration("allow_lift_unloading_output")
    lift_unloading_seconds = LaunchConfiguration(
        "lift_unloading_test_max_seconds"
    )
    lift_unloading_speed = LaunchConfiguration("lift_unloading_target_speed")
    lift_unloading_acceleration = LaunchConfiguration(
        "lift_unloading_acceleration"
    )
    lift_unloading_hold = LaunchConfiguration("lift_unloading_hold_seconds")
    lift_unloading_delay = LaunchConfiguration(
        "lift_unloading_start_delay_seconds"
    )
    lift_unloading_maximum = LaunchConfiguration("lift_unloading_maximum")
    allow_transition_gate_d = LaunchConfiguration(
        "allow_transition_gate_d_output"
    )
    transition_gate_d_seconds = LaunchConfiguration(
        "transition_gate_d_max_seconds"
    )
    transition_gate_d_pusher_max = LaunchConfiguration(
        "transition_gate_d_pusher_max"
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
            DeclareLaunchArgument(
                "allow_pretransition_output",
                default_value="false",
                description="Allow the guarded Gate B1 5 m/s MC test",
            ),
            DeclareLaunchArgument(
                "pretransition_test_max_seconds",
                default_value="49.0",
                description="PX4-time timeout for the Gate B1 profile",
            ),
            DeclareLaunchArgument(
                "pretransition_target_speed",
                default_value="5.0",
                description="Gate B1 forward-speed reference in m/s",
            ),
            DeclareLaunchArgument(
                "pretransition_acceleration",
                default_value="0.40",
                description="Gate B1 reference acceleration in m/s^2",
            ),
            DeclareLaunchArgument(
                "pretransition_hold_seconds",
                default_value="3.0",
                description="Gate B1 hold time at peak speed",
            ),
            DeclareLaunchArgument(
                "pretransition_start_delay_seconds",
                default_value="2.0",
                description="Gate B1 stationary warm-up before acceleration",
            ),
            DeclareLaunchArgument(
                "allow_lift_unloading_output",
                default_value="false",
                description="Allow the guarded Gate B2 8 m/s MC test",
            ),
            DeclareLaunchArgument(
                "lift_unloading_test_max_seconds",
                default_value="60.0",
                description="PX4-time timeout for the Gate B2 profile",
            ),
            DeclareLaunchArgument(
                "lift_unloading_target_speed",
                default_value="8.0",
                description="Gate B2 forward-speed reference in m/s",
            ),
            DeclareLaunchArgument(
                "lift_unloading_acceleration",
                default_value="0.50",
                description="Gate B2 reference acceleration in m/s^2",
            ),
            DeclareLaunchArgument(
                "lift_unloading_hold_seconds",
                default_value="3.0",
                description="Gate B2 hold time at peak speed",
            ),
            DeclareLaunchArgument(
                "lift_unloading_start_delay_seconds",
                default_value="2.0",
                description="Gate B2 stationary warm-up before acceleration",
            ),
            DeclareLaunchArgument(
                "lift_unloading_maximum",
                default_value="0.020",
                description="Maximum ULog-bounded B2 collective unloading",
            ),
            DeclareLaunchArgument(
                "allow_transition_gate_d_output",
                default_value="false",
                description="Allow the first guarded NMPC front/back transition",
            ),
            DeclareLaunchArgument(
                "transition_gate_d_max_seconds",
                default_value="90.0",
                description="PX4-time deadline including Gate D recovery",
            ),
            DeclareLaunchArgument(
                "transition_gate_d_pusher_max",
                default_value="0.60",
                description="Gate D NMPC/external-pusher command ceiling",
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
                        "allow_pretransition_output": allow_pretransition,
                        "pretransition_test_max_seconds": pretransition_seconds,
                        "pretransition_target_speed": pretransition_speed,
                        "pretransition_acceleration": pretransition_acceleration,
                        "pretransition_hold_seconds": pretransition_hold,
                        "pretransition_start_delay_seconds": pretransition_delay,
                        "allow_lift_unloading_output": allow_lift_unloading,
                        "lift_unloading_test_max_seconds": lift_unloading_seconds,
                        "lift_unloading_target_speed": lift_unloading_speed,
                        "lift_unloading_acceleration": lift_unloading_acceleration,
                        "lift_unloading_hold_seconds": lift_unloading_hold,
                        "lift_unloading_start_delay_seconds": lift_unloading_delay,
                        "lift_unloading_maximum": lift_unloading_maximum,
                        "allow_transition_gate_d_output": allow_transition_gate_d,
                        "transition_gate_d_max_seconds": transition_gate_d_seconds,
                        "transition_gate_d_pusher_max": transition_gate_d_pusher_max,
                    }
                ],
            ),
        ]
    )
