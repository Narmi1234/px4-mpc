"""ROS 2 shadow and guarded multicopter-Offboard node for Standard VTOL NMPC."""

from __future__ import annotations

from collections import deque
import math

import numpy as np
from px4_msgs.msg import (
    AirspeedValidated,
    OffboardControlMode,
    TimesyncStatus,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLocalPosition,
    VehicleOdometry,
    VehicleRatesSetpoint,
    VehicleStatus,
    VtolVehicleStatus,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from px4_mpc.controllers.standard_vtol_nmpc import StandardVtolNmpc
from px4_mpc.controllers.standard_vtol_output import (
    govern_pusher_forward_lateral,
    govern_pusher_forward_envelope,
    limit_external_pusher_command,
    limit_mc_command,
    limit_pusher_forward_command,
    pretransition_lift_command,
    vertical_hover_lift,
)
from px4_mpc.models.external_pusher_profile import ExternalPusherProfile
from px4_mpc.models.frames import ned_to_enu, px4_quaternion_to_gazebo
from px4_mpc.models.mc_forward_profile import (
    mc_forward_reference_state,
    McForwardProfile,
    pusher_forward_feedforward,
    pusher_forward_reference_state,
    pusher_forward_speed_reference_state,
)
from px4_mpc.models.px4_timebase import Px4Timebase
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger


def _status_tracking_values(
    state: np.ndarray | None,
    current_reference: np.ndarray | None,
    hold_state: np.ndarray | None,
) -> tuple[list[float] | str, list[float] | str]:
    """Build status diagnostics safely before a hover reference is captured."""
    tracking_error: list[float] | str = "none"
    displacement: list[float] | str = "none"
    if state is not None and current_reference is not None:
        tracking_error = np.round(state[0:6] - current_reference[0:6], 4).tolist()
    if state is not None and hold_state is not None:
        displacement = np.round(state[0:2] - hold_state[0:2], 4).tolist()
    return tracking_error, displacement


def _state_age_limits(active: bool, test_mode: str) -> tuple[float, float]:
    """Return wall/PX4 odometry limits for one operating mode."""
    if not active:
        return 0.20, 0.20
    if test_mode in ("pretransition_5mps", "pretransition_8mps"):
        # The first B1 run recorded a 0.365 s DDS-only receive gap while PX4's
        # internal local-position log remained continuous at <=44 ms. Allow a
        # bounded bridge hiccup, but abort before 0.5 s of unobserved flight.
        return 0.45, 0.45
    return 0.30, 0.20


class StandardVtolNmpcNode(Node):
    """Solve continuously in shadow mode and run explicit guarded MC gates."""

    HANDOVER_FREEZE_SECONDS = 0.5

    def __init__(self) -> None:
        super().__init__("standard_vtol_nmpc")
        self.declare_parameter("allow_offboard_output", False)
        self.declare_parameter("hover_offboard_max_seconds", 5.0)
        self.declare_parameter("mc_forward_test_max_seconds", 15.0)
        self.declare_parameter("mc_forward_target_speed", 2.0)
        self.declare_parameter("mc_forward_acceleration", 1.0)
        self.declare_parameter("mc_forward_hold_seconds", 1.0)
        self.declare_parameter("mc_forward_start_delay_seconds", 2.0)
        self.declare_parameter("allow_external_pusher_output", False)
        self.declare_parameter("external_pusher_test_max_seconds", 12.0)
        self.declare_parameter("external_pusher_peak", 0.05)
        self.declare_parameter("external_pusher_slew", 0.02)
        self.declare_parameter("external_pusher_hold_seconds", 2.0)
        self.declare_parameter("external_pusher_start_delay_seconds", 2.0)
        self.declare_parameter("allow_pusher_forward_output", False)
        self.declare_parameter("pusher_forward_test_max_seconds", 26.5)
        self.declare_parameter("pusher_forward_target_speed", 3.0)
        self.declare_parameter("pusher_forward_acceleration", 0.50)
        self.declare_parameter("pusher_forward_hold_seconds", 2.0)
        self.declare_parameter("pusher_forward_start_delay_seconds", 2.0)
        self.declare_parameter("allow_pretransition_output", False)
        self.declare_parameter("pretransition_test_max_seconds", 49.0)
        self.declare_parameter("pretransition_target_speed", 5.0)
        self.declare_parameter("pretransition_acceleration", 0.40)
        self.declare_parameter("pretransition_hold_seconds", 3.0)
        self.declare_parameter("pretransition_start_delay_seconds", 2.0)
        self.declare_parameter("allow_lift_unloading_output", False)
        self.declare_parameter("lift_unloading_test_max_seconds", 60.0)
        self.declare_parameter("lift_unloading_target_speed", 8.0)
        self.declare_parameter("lift_unloading_acceleration", 0.50)
        self.declare_parameter("lift_unloading_hold_seconds", 3.0)
        self.declare_parameter("lift_unloading_start_delay_seconds", 2.0)
        self.declare_parameter("lift_unloading_maximum", 0.020)
        self.allow_output = bool(self.get_parameter("allow_offboard_output").value)
        self.max_offboard_seconds = float(
            self.get_parameter("hover_offboard_max_seconds").value
        )
        self.forward_test_max_seconds = float(
            self.get_parameter("mc_forward_test_max_seconds").value
        )
        self.forward_profile = McForwardProfile(
            target_speed=float(
                self.get_parameter("mc_forward_target_speed").value
            ),
            acceleration=float(
                self.get_parameter("mc_forward_acceleration").value
            ),
            hold_seconds=float(
                self.get_parameter("mc_forward_hold_seconds").value
            ),
            start_delay_seconds=float(
                self.get_parameter("mc_forward_start_delay_seconds").value
            ),
        )
        self.allow_external_pusher = bool(
            self.get_parameter("allow_external_pusher_output").value
        )
        self.external_pusher_test_max_seconds = float(
            self.get_parameter("external_pusher_test_max_seconds").value
        )
        self.external_pusher_profile = ExternalPusherProfile(
            peak_command=float(self.get_parameter("external_pusher_peak").value),
            slew_per_second=float(self.get_parameter("external_pusher_slew").value),
            hold_seconds=float(
                self.get_parameter("external_pusher_hold_seconds").value
            ),
            start_delay_seconds=float(
                self.get_parameter("external_pusher_start_delay_seconds").value
            ),
        )
        self.allow_pusher_forward = bool(
            self.get_parameter("allow_pusher_forward_output").value
        )
        self.pusher_forward_test_max_seconds = float(
            self.get_parameter("pusher_forward_test_max_seconds").value
        )
        self.pusher_forward_profile = McForwardProfile(
            target_speed=float(
                self.get_parameter("pusher_forward_target_speed").value
            ),
            acceleration=float(
                self.get_parameter("pusher_forward_acceleration").value
            ),
            hold_seconds=float(
                self.get_parameter("pusher_forward_hold_seconds").value
            ),
            start_delay_seconds=float(
                self.get_parameter("pusher_forward_start_delay_seconds").value
            ),
        )
        self.allow_pretransition = bool(
            self.get_parameter("allow_pretransition_output").value
        )
        self.pretransition_test_max_seconds = float(
            self.get_parameter("pretransition_test_max_seconds").value
        )
        self.pretransition_profile = McForwardProfile(
            target_speed=float(
                self.get_parameter("pretransition_target_speed").value
            ),
            acceleration=float(
                self.get_parameter("pretransition_acceleration").value
            ),
            hold_seconds=float(
                self.get_parameter("pretransition_hold_seconds").value
            ),
            start_delay_seconds=float(
                self.get_parameter("pretransition_start_delay_seconds").value
            ),
        )
        self.allow_lift_unloading = bool(
            self.get_parameter("allow_lift_unloading_output").value
        )
        self.lift_unloading_test_max_seconds = float(
            self.get_parameter("lift_unloading_test_max_seconds").value
        )
        self.lift_unloading_profile = McForwardProfile(
            target_speed=float(
                self.get_parameter("lift_unloading_target_speed").value
            ),
            acceleration=float(
                self.get_parameter("lift_unloading_acceleration").value
            ),
            hold_seconds=float(
                self.get_parameter("lift_unloading_hold_seconds").value
            ),
            start_delay_seconds=float(
                self.get_parameter("lift_unloading_start_delay_seconds").value
            ),
        )
        self.lift_unloading_maximum = float(
            self.get_parameter("lift_unloading_maximum").value
        )
        if self.allow_lift_unloading:
            self.nmpc_pusher_max = 0.20
        elif self.allow_pretransition:
            self.nmpc_pusher_max = 0.15
        elif self.allow_pusher_forward:
            self.nmpc_pusher_max = 0.10
        else:
            self.nmpc_pusher_max = 0.60
        if (
            self.allow_pusher_forward
            or self.allow_pretransition
            or self.allow_lift_unloading
        ):
            # Gate A must be optimized with the pusher envelope that the
            # output safety layer and custom PX4 branch can execute. Planning
            # with the generic 0.60 transition limit and clipping to 0.10
            # afterward invalidates NMPC's speed and braking prediction.
            build_name = (
                "standard_vtol_nmpc_gate_b2_pusher_020"
                if self.allow_lift_unloading
                else (
                    "standard_vtol_nmpc_gate_b1_pusher_015"
                    if self.allow_pretransition
                    else "standard_vtol_nmpc_gate_a_pusher_010"
                )
            )
            self.controller = StandardVtolNmpc(
                build_directory=f"build/{build_name}",
                control_lower_bounds=np.array(
                    [0.0, 0.0, -0.50, -0.50, -0.30]
                ),
                control_upper_bounds=np.array(
                    [0.65, self.nmpc_pusher_max, 0.50, 0.50, 0.30]
                ),
            )
        else:
            self.controller = StandardVtolNmpc()
        self.state: np.ndarray | None = None
        self.state_received_ns = 0
        self.state_px4_us = 0
        self.max_state_wall_age = 0.0
        self.max_state_px4_age = 0.0
        self.vertical_position_rate_enu = math.nan
        self.vertical_position_rate_received_ns = 0
        self.px4_timebase = Px4Timebase()
        self.last_control_px4_us = 0
        self.status: VehicleStatus | None = None
        self.vtol_status: VtolVehicleStatus | None = None
        self.airspeed: AirspeedValidated | None = None
        self.airspeed_received_ns = 0
        self.hold_state: np.ndarray | None = None
        self.current_reference: np.ndarray | None = None
        self.forward_direction = np.array([1.0, 0.0])
        self.test_mode = "idle"
        self.profile_phase = "none"
        self.last_command = np.array(
            [self.controller.model.plant.hover_command, 0.0, 0.0, 0.0, 0.0]
        )
        self.last_raw_control = self.last_command.copy()
        self.output_requested = False
        self.offboard_active = False
        self.ever_offboard = False
        self.last_offboard_duration = 0.0
        self.last_wall_offboard_duration = 0.0
        self.prestream_count = 0
        self.solver_failures = 0
        self.last_solve_time = math.nan
        self.solve_times_ms = deque(maxlen=5000)
        self.abort_reason = "none"
        self.last_command_ack = "none"
        self.max_forward_speed = 0.0
        self.max_cross_track_error = 0.0
        self.max_horizontal_tracking_error = 0.0
        self.max_altitude_error = 0.0
        self.max_tilt_degrees = 0.0
        self.max_commanded_pusher = 0.0
        self.max_calibrated_airspeed = 0.0
        self.max_abs_vertical_speed = 0.0
        self.last_lift_unloading = 0.0
        self.max_lift_unloading = 0.0

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self._odometry, qos
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._vehicle_local_position,
            qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self._vehicle_local_position,
            qos,
        )
        self.create_subscription(
            TimesyncStatus,
            "/fmu/out/timesync_status",
            self._timesync_status,
            qos,
        )
        # PX4 message versioning exposes VehicleStatus v4 with the `_v4`
        # suffix on current releases. Keep the unsuffixed subscription too so
        # the node remains usable with older bridges.
        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self._vehicle_status, qos
        )
        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v4",
            self._vehicle_status,
            qos,
        )
        self.create_subscription(
            VtolVehicleStatus,
            "/fmu/out/vtol_vehicle_status",
            self._vtol_vehicle_status,
            qos,
        )
        self.create_subscription(
            AirspeedValidated,
            "/fmu/out/airspeed_validated_v1",
            self._airspeed_validated,
            qos,
        )
        self.create_subscription(
            VehicleCommandAck,
            "/fmu/out/vehicle_command_ack_v1",
            self._vehicle_command_ack,
            qos,
        )
        self.offboard_publisher = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos
        )
        self.rates_publisher = self.create_publisher(
            VehicleRatesSetpoint, "/fmu/in/vehicle_rates_setpoint", qos
        )
        self.command_publisher = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos
        )
        self.diagnostic_publisher = self.create_publisher(
            Float64MultiArray, "/standard_vtol_nmpc/proposed_control", 10
        )
        self.create_service(
            Trigger,
            "/standard_vtol_nmpc/enable_hover_offboard",
            self._enable_hover,
        )
        self.create_service(
            Trigger,
            "/standard_vtol_nmpc/enable_mc_forward_test",
            self._enable_mc_forward,
        )
        self.create_service(
            Trigger,
            "/standard_vtol_nmpc/enable_external_pusher_test",
            self._enable_external_pusher,
        )
        self.create_service(
            Trigger,
            "/standard_vtol_nmpc/enable_pusher_forward_test",
            self._enable_pusher_forward,
        )
        self.create_service(
            Trigger,
            "/standard_vtol_nmpc/enable_pretransition_5mps_test",
            self._enable_pretransition,
        )
        self.create_service(
            Trigger,
            "/standard_vtol_nmpc/enable_lift_unloading_8mps_test",
            self._enable_lift_unloading,
        )
        self.create_service(
            Trigger,
            "/standard_vtol_nmpc/capture_hover_reference",
            self._capture_hover_reference,
        )
        self.create_service(
            Trigger, "/standard_vtol_nmpc/disable", self._disable
        )
        self.create_service(
            Trigger, "/standard_vtol_nmpc/status", self._status
        )
        self.create_timer(0.05, self._update)
        mode = "armed-capable guarded MC" if self.allow_output else "shadow-only"
        if self.allow_external_pusher:
            mode += " plus guarded external pusher"
        if self.allow_pusher_forward:
            mode += " plus guarded 3 m/s pusher feedback"
        if self.allow_pretransition:
            mode += " plus guarded 5 m/s MC pre-transition"
        if self.allow_lift_unloading:
            mode += " plus guarded 8 m/s MC lift-unloading"
        self.get_logger().info(f"Standard VTOL NMPC started in {mode} mode")

    def _odometry(self, message: VehicleOdometry) -> None:
        if message.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            self.abort_reason = "odometry_pose_frame_not_ned"
            return
        if message.velocity_frame != VehicleOdometry.VELOCITY_FRAME_NED:
            self.abort_reason = "odometry_velocity_frame_not_ned"
            return
        values = np.r_[message.position, message.velocity, message.q]
        if not np.all(np.isfinite(values)):
            self.abort_reason = "nonfinite_odometry"
            return
        self.state = np.r_[
            ned_to_enu(np.asarray(message.position)),
            ned_to_enu(np.asarray(message.velocity)),
            px4_quaternion_to_gazebo(np.asarray(message.q)),
        ]
        if self._vertical_position_rate_age() <= 0.20:
            self.state[5] = self.vertical_position_rate_enu
        now_ns = self.get_clock().now().nanoseconds
        self.px4_timebase.advance_from_wall(now_ns)
        self.state_received_ns = now_ns
        # Treat receipt of valid odometry as the state sample's plant-clock
        # anchor. DDS-translated absolute timestamps can jump when the uXRCE
        # offset changes, even though the stream itself remains healthy.
        self.state_px4_us = self.px4_timebase.latest_px4_us

    def _vehicle_local_position(self, message: VehicleLocalPosition) -> None:
        if message.z_valid and np.isfinite(message.z_deriv):
            # z_deriv is the derivative of the NED position used by the
            # altitude loop. Its sign remained consistent with position and
            # Gazebo ground truth in ULog, unlike one observed EKF vz bias.
            self.vertical_position_rate_enu = -float(message.z_deriv)
            self.vertical_position_rate_received_ns = (
                self.get_clock().now().nanoseconds
            )

    def _timesync_status(self, message: TimesyncStatus) -> None:
        now_ns = self.get_clock().now().nanoseconds
        self.px4_timebase.advance_from_wall(now_ns)
        previous_clock_us = self.px4_timebase.latest_px4_us
        self.px4_timebase.update_timesync(
            message.timestamp,
            message.estimated_offset,
            message.remote_timestamp,
            message.observed_offset,
            wall_ns=now_ns,
        )
        # Preserve the measured state age across an absolute clock correction.
        # Only elapsed plant time since the last odometry receipt may age it.
        if self.state_px4_us > 0 and previous_clock_us > 0:
            self.state_px4_us += (
                self.px4_timebase.latest_px4_us - previous_clock_us
            )

    def _vehicle_status(self, message: VehicleStatus) -> None:
        now_ns = self.get_clock().now().nanoseconds
        self.px4_timebase.advance_from_wall(now_ns)
        was_armed = (
            self.status is not None
            and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )
        self.status = message
        is_armed = message.arming_state == VehicleStatus.ARMING_STATE_ARMED
        was_offboard = self.offboard_active
        self.offboard_active = (
            message.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )
        if self.offboard_active and not was_offboard:
            # PX4 does not DDS-translate this nested event timestamp. Now that
            # the timebase itself is raw boot time, it is the authoritative
            # zero and is immune to an offset handover at mode entry.
            previous_clock_us = self.px4_timebase.latest_px4_us
            self.px4_timebase.start_offboard(
                message.nav_state_timestamp,
                now_ns,
            )
            if self.state_px4_us > 0 and previous_clock_us > 0:
                self.state_px4_us += (
                    self.px4_timebase.latest_px4_us - previous_clock_us
                )
        self.ever_offboard = self.ever_offboard or self.offboard_active
        if was_armed and not is_armed:
            self.output_requested = False
            self.prestream_count = 0
            self.hold_state = None
            self.current_reference = None
            self.test_mode = "idle"
            self.profile_phase = "none"
            self.last_command = np.array(
                [self.controller.model.plant.hover_command, 0.0, 0.0, 0.0, 0.0]
            )
            self.last_raw_control = self.last_command.copy()
            self.abort_reason = "reference_cleared_on_disarm"
            self.px4_timebase.reset()
            self.last_control_px4_us = 0

    def _vtol_vehicle_status(self, message: VtolVehicleStatus) -> None:
        self.vtol_status = message

    def _airspeed_validated(self, message: AirspeedValidated) -> None:
        self.airspeed = message
        self.airspeed_received_ns = self.get_clock().now().nanoseconds

    def _vehicle_command_ack(self, message: VehicleCommandAck) -> None:
        self.last_command_ack = f"command={message.command},result={message.result}"

    def _state_age(self) -> float:
        if not self.state_received_ns:
            return math.inf
        return (self.get_clock().now().nanoseconds - self.state_received_ns) * 1e-9

    def _state_px4_age(self) -> float:
        if self.state_px4_us <= 0 or self.px4_timebase.latest_px4_us <= 0:
            return math.inf
        return max(
            0.0,
            (self.px4_timebase.latest_px4_us - self.state_px4_us) * 1.0e-6,
        )

    def _state_is_stale(self, active: bool) -> bool:
        wall_limit, px4_limit = _state_age_limits(active, self.test_mode)
        return (
            self.state is None
            or self._state_age() > wall_limit
            or self._state_px4_age() > px4_limit
        )

    def _vertical_position_rate_age(self) -> float:
        if not self.vertical_position_rate_received_ns:
            return math.inf
        return (
            self.get_clock().now().nanoseconds
            - self.vertical_position_rate_received_ns
        ) * 1e-9

    def _airspeed_age(self) -> float:
        if not self.airspeed_received_ns:
            return math.inf
        return (
            self.get_clock().now().nanoseconds - self.airspeed_received_ns
        ) * 1e-9

    def _calibrated_airspeed(self) -> float:
        if self.airspeed is None:
            return math.nan
        return float(self.airspeed.calibrated_airspeed_m_s)

    def _airspeed_is_valid(self) -> bool:
        value = self._calibrated_airspeed()
        return (
            self._airspeed_is_available()
            and value >= 0.0
        )

    def _airspeed_is_available(self) -> bool:
        """Return whether a fresh finite airspeed stream and source exist.

        PX4's synthetic source can report a negative calibrated value around
        stationary hover. That sample is not a valid airspeed measurement, but
        it still proves that the stream needed by B1 is alive. Positive CAS is
        required later, once the aircraft has accelerated.
        """
        return (
            self.airspeed is not None
            and self._airspeed_age() <= 0.5
            and np.isfinite(self._calibrated_airspeed())
            and self.airspeed.airspeed_source
            != AirspeedValidated.SOURCE_DISABLED
        )

    def _control_dt(self) -> float:
        """Return the plant-time step used by command slew limiters."""
        latest = self.px4_timebase.latest_px4_us
        if latest <= 0:
            return self.controller.dt
        if self.last_control_px4_us <= 0:
            self.last_control_px4_us = latest
            return self.controller.dt
        elapsed = max(0.0, (latest - self.last_control_px4_us) * 1.0e-6)
        self.last_control_px4_us = latest
        return min(elapsed, 0.10)

    @staticmethod
    def _yaw_from_quaternion(quaternion: np.ndarray) -> float:
        qw, qx, qy, qz = quaternion
        return math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )

    @classmethod
    def _level_yaw_quaternion(cls, quaternion: np.ndarray) -> np.ndarray:
        """Keep measured yaw while setting the hover roll and pitch to zero."""
        yaw = cls._yaw_from_quaternion(quaternion)
        return np.array([math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)])

    def _ready_for_hover(self) -> tuple[bool, str]:
        if not self.px4_timebase.synchronized:
            return False, "px4_timesync_missing"
        if self._state_is_stale(active=False):
            return False, "odometry_stale"
        if self._vertical_position_rate_age() > 0.20:
            return False, "vertical_position_rate_stale"
        if self.status is None or self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            return False, "vehicle_not_armed"
        if self.status.failsafe:
            return False, "px4_failsafe"
        if self.vtol_status is None:
            return False, "vtol_status_missing"
        if self.vtol_status.vehicle_vtol_state != VtolVehicleStatus.VEHICLE_VTOL_STATE_MC:
            return False, "vehicle_not_in_mc_mode"
        if np.linalg.norm(self.state[3:5]) > 0.5:
            return False, "horizontal_speed_too_high_for_hover_handover"
        if abs(self.vertical_position_rate_enu) > 0.10:
            return False, "vertical_speed_too_high_for_hover_handover"
        return True, "ready"

    def _capture_reference(self) -> None:
        self.hold_state = self.state.copy()
        self.hold_state[3:6] = 0.0
        self.hold_state[6:10] = self._level_yaw_quaternion(
            self.hold_state[6:10]
        )
        self.current_reference = self.hold_state.copy()
        yaw = self._yaw_from_quaternion(self.hold_state[6:10])
        self.forward_direction = np.array([math.cos(yaw), math.sin(yaw)])

    def _start_output(self, mode: str) -> None:
        self._capture_reference()
        self.test_mode = mode
        self.profile_phase = "prestream"
        self.output_requested = True
        self.prestream_count = 0
        self.ever_offboard = False
        self.px4_timebase.reset()
        self.last_control_px4_us = self.px4_timebase.latest_px4_us
        self.last_offboard_duration = 0.0
        self.last_wall_offboard_duration = 0.0
        self.last_command = np.array(
            [self.controller.model.plant.hover_command, 0.0, 0.0, 0.0, 0.0]
        )
        self.last_raw_control = self.last_command.copy()
        self.abort_reason = "none"
        self.last_command_ack = "none"
        self.solve_times_ms.clear()
        self.max_forward_speed = 0.0
        self.max_cross_track_error = 0.0
        self.max_horizontal_tracking_error = 0.0
        self.max_altitude_error = 0.0
        self.max_tilt_degrees = 0.0
        self.max_commanded_pusher = 0.0
        self.max_calibrated_airspeed = 0.0
        self.max_abs_vertical_speed = 0.0
        self.max_state_wall_age = 0.0
        self.max_state_px4_age = 0.0
        self.last_lift_unloading = 0.0
        self.max_lift_unloading = 0.0

    def _enable_hover(self, _request, response):
        if not self.allow_output:
            response.success = False
            response.message = "launch with allow_offboard_output:=true first"
            return response
        ready, reason = self._ready_for_hover()
        if not ready:
            response.success = False
            response.message = reason
            return response
        self._start_output("hover")
        response.success = True
        response.message = "hover setpoint prestream started; PX4 Offboard follows after 1 s"
        return response

    def _enable_mc_forward(self, _request, response):
        if not self.allow_output:
            response.success = False
            response.message = "launch with allow_offboard_output:=true first"
            return response
        first_gate_configuration = (
            np.isclose(self.forward_profile.target_speed, 2.0)
            and np.isclose(self.forward_profile.acceleration, 1.0)
            and np.isclose(self.forward_profile.hold_seconds, 1.0)
            and np.isclose(self.forward_profile.start_delay_seconds, 2.0)
            and np.isclose(self.forward_test_max_seconds, 15.0)
        )
        if not first_gate_configuration:
            response.success = False
            response.message = "mc_forward_profile_not_first_gate_configuration"
            return response
        ready, reason = self._ready_for_hover()
        if not ready:
            response.success = False
            response.message = reason
            return response
        self._start_output("mc_forward")
        response.success = True
        response.message = (
            f"{self.forward_profile.target_speed:.1f} m/s MC-forward prestream "
            "started; pusher locked at zero, "
            "PX4 Offboard follows after 1 s"
        )
        return response

    def _enable_external_pusher(self, _request, response):
        if not self.allow_output or not self.allow_external_pusher:
            response.success = False
            response.message = (
                "launch with allow_offboard_output:=true and "
                "allow_external_pusher_output:=true first"
            )
            return response
        profile = self.external_pusher_profile
        first_gate_configuration = (
            np.isclose(profile.peak_command, 0.05)
            and np.isclose(profile.slew_per_second, 0.02)
            and np.isclose(profile.hold_seconds, 2.0)
            and np.isclose(profile.start_delay_seconds, 2.0)
            and np.isclose(self.external_pusher_test_max_seconds, 12.0)
        )
        if not first_gate_configuration:
            response.success = False
            response.message = "external_pusher_profile_not_first_gate_configuration"
            return response
        ready, reason = self._ready_for_hover()
        if not ready:
            response.success = False
            response.message = reason
            return response
        self._start_output("external_pusher")
        response.success = True
        response.message = (
            "external pusher 0->0.05->0 pulse prestream started; "
            "PX4 Offboard follows after 1 s"
        )
        return response

    def _enable_pusher_forward(self, _request, response):
        if (
            not self.allow_output
            or not self.allow_external_pusher
            or not self.allow_pusher_forward
        ):
            response.success = False
            response.message = (
                "launch with allow_offboard_output:=true, "
                "allow_external_pusher_output:=true and "
                "allow_pusher_forward_output:=true first"
            )
            return response
        profile = self.pusher_forward_profile
        first_gate_configuration = (
            np.isclose(profile.target_speed, 3.0)
            and np.isclose(profile.acceleration, 0.50)
            and np.isclose(profile.hold_seconds, 2.0)
            and np.isclose(profile.start_delay_seconds, 2.0)
            and np.isclose(self.pusher_forward_test_max_seconds, 26.5)
            and np.isclose(self.nmpc_pusher_max, 0.10)
        )
        if not first_gate_configuration:
            response.success = False
            response.message = "pusher_forward_profile_not_gate_a_configuration"
            return response
        ready, reason = self._ready_for_hover()
        if not ready:
            response.success = False
            response.message = reason
            return response
        if np.linalg.norm(self.state[3:5]) > 0.15:
            response.success = False
            response.message = "pusher_forward_handover_speed_too_high"
            return response
        self._start_output("pusher_forward")
        response.success = True
        response.message = (
            "3.0 m/s MC pusher-feedback prestream started; "
            "PX4 Offboard follows after 1 s"
        )
        return response

    def _enable_pretransition(self, _request, response):
        if (
            not self.allow_output
            or not self.allow_external_pusher
            or not self.allow_pretransition
        ):
            response.success = False
            response.message = (
                "launch with allow_offboard_output:=true, "
                "allow_external_pusher_output:=true and "
                "allow_pretransition_output:=true first"
            )
            return response
        profile = self.pretransition_profile
        b1_configuration = (
            np.isclose(profile.target_speed, 5.0)
            and np.isclose(profile.acceleration, 0.40)
            and np.isclose(profile.hold_seconds, 3.0)
            and np.isclose(profile.start_delay_seconds, 2.0)
            and np.isclose(self.pretransition_test_max_seconds, 49.0)
            and np.isclose(self.nmpc_pusher_max, 0.15)
        )
        if not b1_configuration:
            response.success = False
            response.message = "pretransition_profile_not_gate_b1_configuration"
            return response
        ready, reason = self._ready_for_hover()
        if not ready:
            response.success = False
            response.message = reason
            return response
        if not self._airspeed_is_available():
            response.success = False
            response.message = "airspeed_stream_unavailable_for_pretransition"
            return response
        if np.linalg.norm(self.state[3:5]) > 0.15:
            response.success = False
            response.message = "pretransition_handover_speed_too_high"
            return response
        self._start_output("pretransition_5mps")
        response.success = True
        response.message = (
            "5.0 m/s MC pre-transition prestream started; "
            "collective remains validated hover feedback, "
            "PX4 Offboard follows after 1 s"
        )
        return response

    def _enable_lift_unloading(self, _request, response):
        if (
            not self.allow_output
            or not self.allow_external_pusher
            or not self.allow_lift_unloading
        ):
            response.success = False
            response.message = (
                "launch with allow_offboard_output:=true, "
                "allow_external_pusher_output:=true and "
                "allow_lift_unloading_output:=true first"
            )
            return response
        profile = self.lift_unloading_profile
        b2_configuration = (
            np.isclose(profile.target_speed, 8.0)
            and np.isclose(profile.acceleration, 0.50)
            and np.isclose(profile.hold_seconds, 3.0)
            and np.isclose(profile.start_delay_seconds, 2.0)
            and np.isclose(self.lift_unloading_test_max_seconds, 60.0)
            and np.isclose(self.lift_unloading_maximum, 0.020)
            and np.isclose(self.nmpc_pusher_max, 0.20)
        )
        if not b2_configuration:
            response.success = False
            response.message = "lift_unloading_profile_not_gate_b2_configuration"
            return response
        ready, reason = self._ready_for_hover()
        if not ready:
            response.success = False
            response.message = reason
            return response
        if not self._airspeed_is_available():
            response.success = False
            response.message = "airspeed_stream_unavailable_for_lift_unloading"
            return response
        if np.linalg.norm(self.state[3:5]) > 0.15:
            response.success = False
            response.message = "lift_unloading_handover_speed_too_high"
            return response
        self._start_output("pretransition_8mps")
        response.success = True
        response.message = (
            "8.0 m/s MC lift-unloading prestream started; "
            "PX4 Offboard follows after 1 s"
        )
        return response

    def _capture_hover_reference(self, _request, response):
        ready, reason = self._ready_for_hover()
        if not ready:
            response.success = False
            response.message = reason
            return response
        self._capture_reference()
        self.test_mode = "shadow_hover"
        self.profile_phase = "stationary"
        self.output_requested = False
        self.prestream_count = 0
        self.abort_reason = "none"
        response.success = True
        response.message = "hover reference captured; output remains shadow-only"
        return response

    def _disable(self, _request, response):
        self._abort("operator_disabled")
        response.success = True
        response.message = "Position mode requested; NMPC output disabled"
        return response

    def _active_timeout(self) -> float:
        if self.test_mode == "pretransition_8mps":
            return self.lift_unloading_test_max_seconds
        if self.test_mode == "pretransition_5mps":
            return self.pretransition_test_max_seconds
        if self.test_mode == "pusher_forward":
            return self.pusher_forward_test_max_seconds
        if self.test_mode == "external_pusher":
            return self.external_pusher_test_max_seconds
        if self.test_mode == "mc_forward":
            return self.forward_test_max_seconds
        return self.max_offboard_seconds

    def _offboard_elapsed(self) -> float:
        return self.px4_timebase.px4_elapsed()

    def _wall_offboard_elapsed(self) -> float:
        return self.px4_timebase.wall_elapsed(
            self.get_clock().now().nanoseconds
        )

    def _profile_elapsed(self) -> float:
        return max(0.0, self._offboard_elapsed() - self.HANDOVER_FREEZE_SECONDS)

    def _status(self, _request, response):
        now_ns = self.get_clock().now().nanoseconds
        self.px4_timebase.advance_from_wall(now_ns)
        nav = -1 if self.status is None else int(self.status.nav_state)
        armed = False if self.status is None else (
            self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )
        tracking_error, displacement = _status_tracking_values(
            self.state, self.current_reference, self.hold_state
        )
        airspeed = self._calibrated_airspeed()
        airspeed_source = (
            -99 if self.airspeed is None else int(self.airspeed.airspeed_source)
        )
        realtime_factor = self.px4_timebase.realtime_factor(now_ns)
        solve_time_p99 = (
            float(np.percentile(self.solve_times_ms, 99))
            if self.solve_times_ms
            else math.nan
        )
        response.success = (
            self.solver_failures == 0
            and not self._state_is_stale(active=False)
            and self.px4_timebase.synchronized
        )
        response.message = (
            f"output_requested={self.output_requested}, offboard={self.offboard_active}, "
            f"armed={armed}, nav_state={nav}, state_age={self._state_age():.3f}s, "
            f"state_px4_age={self._state_px4_age():.3f}s, "
            f"vertical_rate=[value={self.vertical_position_rate_enu:.3f},"
            f"age={self._vertical_position_rate_age():.3f}s], "
            f"solve_time={1000.0 * self.last_solve_time:.2f}ms, "
            f"solver_failures={self.solver_failures}, abort_reason={self.abort_reason}"
            f", test_mode={self.test_mode}, profile_phase={self.profile_phase}"
            f", configured_timeout={self._active_timeout():.1f}s"
            f", forward_profile=[speed={self.forward_profile.target_speed:.1f},"
            f"accel={self.forward_profile.acceleration:.1f},"
            f"hold={self.forward_profile.hold_seconds:.1f}]"
            f", pusher_forward_profile=[speed={self.pusher_forward_profile.target_speed:.1f},"
            f"accel={self.pusher_forward_profile.acceleration:.2f},"
            f"hold={self.pusher_forward_profile.hold_seconds:.1f}]"
            f", pretransition_profile=[speed={self.pretransition_profile.target_speed:.1f},"
            f"accel={self.pretransition_profile.acceleration:.2f},"
            f"hold={self.pretransition_profile.hold_seconds:.1f}]"
            f", lift_unloading_profile=[speed={self.lift_unloading_profile.target_speed:.1f},"
            f"accel={self.lift_unloading_profile.acceleration:.2f},"
            f"hold={self.lift_unloading_profile.hold_seconds:.1f},"
            f"max_unload={self.lift_unloading_maximum:.3f}]"
            f", nmpc_pusher_max={self.nmpc_pusher_max:.3f}"
            f", last_offboard_duration={self.last_offboard_duration:.2f}s"
            f", last_wall_offboard_duration={self.last_wall_offboard_duration:.2f}s"
            f", px4_elapsed={self._offboard_elapsed():.2f}s"
            f", wall_elapsed={self._wall_offboard_elapsed():.2f}s"
            f", realtime_factor={realtime_factor:.3f}"
            f", px4_clock=[sync={self.px4_timebase.synchronized},"
            f"boot_us={self.px4_timebase.latest_px4_us}]"
            f", airspeed=[cas={airspeed:.3f},source={airspeed_source},"
            f"age={self._airspeed_age():.3f}s,"
            f"available={self._airspeed_is_available()},"
            f"valid={self._airspeed_is_valid()}]"
            f", solve_time_p99={solve_time_p99:.2f}ms"
            f", last_command_ack={self.last_command_ack}"
            f", control={np.round(self.last_command, 4).tolist()}"
            f", raw_control={np.round(self.last_raw_control, 4).tolist()}"
            f", tracking_error_pv={tracking_error}"
            f", displacement_xy={displacement}"
            f", maxima=[forward_speed={self.max_forward_speed:.3f},"
            f"cross_track={self.max_cross_track_error:.3f},"
            f"horizontal_tracking={self.max_horizontal_tracking_error:.3f},"
            f"altitude={self.max_altitude_error:.3f},"
            f"tilt_deg={self.max_tilt_degrees:.2f},"
            f"vertical_speed={self.max_abs_vertical_speed:.3f},"
            f"airspeed={self.max_calibrated_airspeed:.3f},"
            f"commanded_pusher={self.max_commanded_pusher:.3f},"
            f"lift_unloading={self.max_lift_unloading:.3f},"
            f"state_wall_gap={self.max_state_wall_age:.3f},"
            f"state_px4_gap={self.max_state_px4_age:.3f}]"
        )
        return response

    def _reference_at_profile_time(self, profile_time: float) -> np.ndarray:
        reference = self.state.copy() if self.hold_state is None else self.hold_state
        if self.hold_state is None:
            reference[3:6] = 0.0
            return reference
        reference = reference.copy()
        if self.test_mode == "mc_forward":
            sample = self.forward_profile.sample(profile_time)
            reference = mc_forward_reference_state(
                self.hold_state,
                self.forward_direction,
                sample,
                self.controller.model.plant.gravity,
            )
        elif self.test_mode in (
            "pusher_forward",
            "pretransition_5mps",
            "pretransition_8mps",
        ):
            profile = (
                self.lift_unloading_profile
                if self.test_mode == "pretransition_8mps"
                else (
                    self.pretransition_profile
                    if self.test_mode == "pretransition_5mps"
                    else self.pusher_forward_profile
                )
            )
            sample = profile.sample(profile_time)
            reference = pusher_forward_reference_state(
                self.hold_state,
                self.forward_direction,
                sample,
            )
        return reference

    def _references(self):
        base_time = self._profile_elapsed()
        if (
            self.test_mode in (
                "pusher_forward",
                "pretransition_5mps",
                "pretransition_8mps",
            )
            and self.hold_state is not None
        ):
            profile = (
                self.lift_unloading_profile
                if self.test_mode == "pretransition_8mps"
                else (
                    self.pretransition_profile
                    if self.test_mode == "pretransition_5mps"
                    else self.pusher_forward_profile
                )
            )
            base_sample = profile.sample(base_time)
            x_ref = np.vstack(
                [
                    pusher_forward_speed_reference_state(
                        self.hold_state,
                        self.state,
                        self.forward_direction,
                        base_sample,
                        profile.sample(
                            base_time + stage * self.controller.dt
                        ),
                    )
                    for stage in range(self.controller.N + 1)
                ]
            )
            self.current_reference = x_ref[0].copy()
        else:
            x_ref = np.vstack(
                [
                    self._reference_at_profile_time(
                        base_time + stage * self.controller.dt
                    )
                    for stage in range(self.controller.N + 1)
                ]
            )
            self.current_reference = self._reference_at_profile_time(base_time)
        if self.test_mode == "mc_forward":
            self.profile_phase = self.forward_profile.sample(base_time).phase
        elif self.test_mode == "pusher_forward":
            self.profile_phase = self.pusher_forward_profile.sample(
                base_time
            ).phase
        elif self.test_mode == "pretransition_5mps":
            self.profile_phase = self.pretransition_profile.sample(base_time).phase
        elif self.test_mode == "pretransition_8mps":
            self.profile_phase = self.lift_unloading_profile.sample(base_time).phase
        elif self.test_mode == "external_pusher":
            self.profile_phase = self.external_pusher_profile.sample(base_time).phase
        elif self.output_requested:
            self.profile_phase = "stationary_hover"
        u_ref = np.zeros((self.controller.N, 5))
        u_ref[:, 0] = self.controller.model.plant.hover_command
        if self.test_mode == "external_pusher":
            u_ref[:, 1] = [
                self.external_pusher_profile.sample(
                    base_time + stage * self.controller.dt
                ).command
                for stage in range(self.controller.N)
            ]
        elif self.test_mode in (
            "pusher_forward",
            "pretransition_5mps",
            "pretransition_8mps",
        ):
            profile = (
                self.lift_unloading_profile
                if self.test_mode == "pretransition_8mps"
                else (
                    self.pretransition_profile
                    if self.test_mode == "pretransition_5mps"
                    else self.pusher_forward_profile
                )
            )
            u_ref[:, 1] = [
                pusher_forward_feedforward(
                    self.controller.model.plant,
                    profile.sample(base_time + stage * self.controller.dt).speed,
                    profile.sample(base_time + stage * self.controller.dt).acceleration,
                    command_limit=self.nmpc_pusher_max,
                )
                for stage in range(self.controller.N)
            ]
        parameters = np.zeros((self.controller.N + 1, 4))
        return x_ref, u_ref, parameters

    def _vertical_hover_lift(self) -> float:
        """Return a slow, critically damped lift command about hover trim."""
        if self.hold_state is None:
            return self.controller.model.plant.hover_command
        altitude_error = self.state[2] - self.hold_state[2]
        vertical_speed = self.state[5]
        return vertical_hover_lift(
            self.controller.model.plant, altitude_error, vertical_speed
        )

    def _tilt_degrees(self) -> float:
        qw, qx, qy, qz = self.state[6:10]
        roll = math.atan2(
            2.0 * (qw * qx + qy * qz),
            1.0 - 2.0 * (qx * qx + qy * qy),
        )
        pitch = math.asin(np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0))
        return math.degrees(max(abs(roll), abs(pitch)))

    def _pitch_angle(self) -> float:
        """Return converted Gazebo/FLU pitch in radians."""
        qw, qx, qy, qz = self.state[6:10]
        return math.asin(np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0))

    def _roll_angle(self) -> float:
        """Return converted Gazebo/FLU roll in radians."""
        qw, qx, qy, qz = self.state[6:10]
        return math.atan2(
            2.0 * (qw * qx + qy * qz),
            1.0 - 2.0 * (qx * qx + qy * qy),
        )

    def _update_flight_metrics(self) -> None:
        if self.state is None or self.hold_state is None:
            return
        delta_xy = self.state[0:2] - self.hold_state[0:2]
        normal = np.array([-self.forward_direction[1], self.forward_direction[0]])
        forward_speed = float(np.dot(self.state[3:5], self.forward_direction))
        cross_track = abs(float(np.dot(delta_xy, normal)))
        tracking = (
            0.0
            if self.current_reference is None
            else float(np.linalg.norm(self.state[0:2] - self.current_reference[0:2]))
        )
        self.max_forward_speed = max(self.max_forward_speed, forward_speed)
        self.max_cross_track_error = max(self.max_cross_track_error, cross_track)
        self.max_horizontal_tracking_error = max(
            self.max_horizontal_tracking_error, tracking
        )
        self.max_altitude_error = max(
            self.max_altitude_error, abs(float(self.state[2] - self.hold_state[2]))
        )
        self.max_tilt_degrees = max(self.max_tilt_degrees, self._tilt_degrees())
        self.max_abs_vertical_speed = max(
            self.max_abs_vertical_speed, abs(float(self.state[5]))
        )
        airspeed = self._calibrated_airspeed()
        if np.isfinite(airspeed):
            self.max_calibrated_airspeed = max(
                self.max_calibrated_airspeed, airspeed
            )

    def _safety_reason(self) -> str | None:
        # Active-flight limits differ from the stricter handover conditions:
        # small velocities are required to enter Offboard, while modest
        # closed-loop corrections are allowed once Offboard is active.
        if self._state_is_stale(active=True):
            return "odometry_stale"
        if self.status is None or (
            self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED
        ):
            return "vehicle_not_armed"
        if self.status.failsafe:
            return "px4_failsafe"
        if self.vtol_status is None:
            return "vtol_status_missing"
        if (
            self.vtol_status.vehicle_vtol_state
            != VtolVehicleStatus.VEHICLE_VTOL_STATE_MC
        ):
            return "vehicle_not_in_mc_mode"
        if self.hold_state is None:
            return "hold_state_missing"
        if self.test_mode == "pusher_forward":
            altitude_limit = 0.30
        elif self.test_mode in ("pretransition_5mps", "pretransition_8mps"):
            altitude_limit = 0.40
        else:
            altitude_limit = 0.5
        if abs(self.state[2] - self.hold_state[2]) > altitude_limit:
            return "altitude_error"
        vertical_speed_limit = (
            1.0
            if self.test_mode in ("pretransition_5mps", "pretransition_8mps")
            else 0.75
        )
        if abs(self.state[5]) > vertical_speed_limit:
            return "vertical_speed_limit"
        if self.test_mode == "mc_forward":
            horizontal_speed_limit = 2.7
        elif self.test_mode == "pusher_forward":
            horizontal_speed_limit = 3.5
        elif self.test_mode == "pretransition_5mps":
            horizontal_speed_limit = 5.75
        elif self.test_mode == "pretransition_8mps":
            horizontal_speed_limit = 8.75
        elif self.test_mode == "external_pusher":
            horizontal_speed_limit = 1.5
        else:
            horizontal_speed_limit = 2.0
        if np.linalg.norm(self.state[3:5]) > horizontal_speed_limit:
            return "horizontal_speed_limit"
        delta_xy = self.state[0:2] - self.hold_state[0:2]
        if self.test_mode in (
            "mc_forward",
            "pusher_forward",
            "pretransition_5mps",
            "pretransition_8mps",
        ):
            profile = (
                self.lift_unloading_profile
                if self.test_mode == "pretransition_8mps"
                else (
                    self.pretransition_profile
                    if self.test_mode == "pretransition_5mps"
                    else (
                    self.pusher_forward_profile
                    if self.test_mode == "pusher_forward"
                    else self.forward_profile
                    )
                )
            )
            normal = np.array(
                [-self.forward_direction[1], self.forward_direction[0]]
            )
            along_track = float(np.dot(delta_xy, self.forward_direction))
            cross_track = abs(float(np.dot(delta_xy, normal)))
            if self.test_mode == "pretransition_8mps":
                end_margin = 8.0
            elif self.test_mode == "pretransition_5mps":
                end_margin = 5.0
            elif self.test_mode == "pusher_forward":
                end_margin = 3.0
            else:
                end_margin = 2.0
            if (
                along_track < -1.0
                or along_track > profile.final_distance + end_margin
            ):
                return "forward_geofence"
            cross_track_limit = (
                1.5
                if self.test_mode == "pretransition_8mps"
                else (
                    1.0
                    if self.test_mode in ("pusher_forward", "pretransition_5mps")
                    else 1.5
                )
            )
            if cross_track > cross_track_limit:
                return "cross_track_limit"
            if (
                    self.current_reference is not None
                    and np.linalg.norm(self.state[0:2] - self.current_reference[0:2])
                    > (
                        2.0
                        if self.test_mode in (
                            "pusher_forward",
                            "pretransition_5mps",
                            "pretransition_8mps",
                        )
                        else 1.5
                    )
                ):
                return "horizontal_tracking_error"
        elif self.test_mode == "external_pusher" and np.linalg.norm(delta_xy) > 3.0:
            return "external_pusher_geofence"
        elif np.linalg.norm(delta_xy) > 5.0:
            return "horizontal_geofence"
        tilt_limit = (
            12.0
            if self.test_mode == "pretransition_8mps"
            else (
                10.0
                if self.test_mode in ("pusher_forward", "pretransition_5mps")
                else 25.0
            )
        )
        if self._tilt_degrees() > tilt_limit:
            return "tilt_limit"
        if self.test_mode in ("pretransition_5mps", "pretransition_8mps"):
            profile = (
                self.lift_unloading_profile
                if self.test_mode == "pretransition_8mps"
                else self.pretransition_profile
            )
            reference_speed = profile.sample(self._profile_elapsed()).speed
            if reference_speed >= 2.0 and not self._airspeed_is_available():
                return "airspeed_stream_lost_during_pretransition"
        if self.solver_failures >= 3:
            return "three_solver_failures"
        if (
            self.offboard_active
            and self._active_timeout() > 0.0
            and self.px4_timebase.active
            and self._offboard_elapsed() >= self._active_timeout()
        ):
            if self.test_mode == "pretransition_8mps":
                if self.max_forward_speed < 7.25:
                    return "lift_unloading_speed_not_reached"
                if self.max_forward_speed > 8.75:
                    return "lift_unloading_speed_overshoot"
                if np.linalg.norm(self.state[3:5]) > 0.50:
                    return "lift_unloading_test_not_stopped"
                if self.max_calibrated_airspeed < 7.0:
                    return "lift_unloading_airspeed_not_reached"
                if self.max_commanded_pusher < 0.10:
                    return "lift_unloading_pusher_not_reached"
                if abs(self.last_command[1]) > 0.005:
                    return "lift_unloading_pusher_not_zero_at_end"
                if self.max_lift_unloading < 0.019:
                    return "lift_unloading_schedule_not_reached"
                if abs(self.last_lift_unloading) > 0.001:
                    return "lift_unloading_not_zero_at_end"
                if (
                    self.current_reference is None
                    or np.linalg.norm(
                        self.state[0:2] - self.current_reference[0:2]
                    )
                    > 1.5
                ):
                    return "lift_unloading_final_position_error"
                return "pretransition_8mps_test_timeout"
            if self.test_mode == "pretransition_5mps":
                if self.max_forward_speed < 4.25:
                    return "pretransition_speed_not_reached"
                if self.max_forward_speed > 5.75:
                    return "pretransition_speed_overshoot"
                if np.linalg.norm(self.state[3:5]) > 0.40:
                    return "pretransition_test_not_stopped"
                if self.max_calibrated_airspeed < 4.0:
                    return "pretransition_airspeed_not_reached"
                if self.max_commanded_pusher < 0.05:
                    return "pretransition_pusher_not_reached"
                if abs(self.last_command[1]) > 0.005:
                    return "pretransition_pusher_not_zero_at_end"
                if (
                    self.current_reference is None
                    or np.linalg.norm(
                        self.state[0:2] - self.current_reference[0:2]
                    )
                    > 1.0
                ):
                    return "pretransition_final_position_error"
                return "pretransition_5mps_test_timeout"
            if self.test_mode == "pusher_forward":
                if self.max_forward_speed < 2.5:
                    return "pusher_forward_speed_not_reached"
                if self.max_forward_speed > 3.5:
                    return "pusher_forward_speed_overshoot"
                if np.linalg.norm(self.state[3:5]) > 0.35:
                    return "pusher_forward_test_not_stopped"
                if self.max_commanded_pusher < 0.05:
                    return "pusher_forward_command_not_reached"
                if abs(self.last_command[1]) > 0.005:
                    return "pusher_forward_not_zero_at_end"
                if (
                    self.current_reference is None
                    or np.linalg.norm(
                        self.state[0:2] - self.current_reference[0:2]
                    )
                    > 1.0
                ):
                    return "pusher_forward_final_position_error"
                return "pusher_forward_test_timeout"
            if self.test_mode == "mc_forward":
                if self.max_forward_speed < 1.6:
                    return "forward_speed_not_reached"
                if np.linalg.norm(self.state[3:5]) > 0.35:
                    return "forward_test_not_stopped"
                if (
                    self.current_reference is None
                    or np.linalg.norm(
                        self.state[0:2] - self.current_reference[0:2]
                    )
                    > 0.75
                ):
                    return "forward_test_final_position_error"
                return "mc_forward_test_timeout"
            if self.test_mode == "external_pusher":
                if abs(self.last_command[1]) > 0.002:
                    return "external_pusher_not_zero_at_end"
                return "external_pusher_test_timeout"
            return "hover_test_timeout"
        return None

    def _publish_setpoint(self, command: np.ndarray) -> None:
        timestamp = self.get_clock().now().nanoseconds // 1000
        mode = OffboardControlMode()
        mode.timestamp = timestamp
        mode.body_rate = True
        self.offboard_publisher.publish(mode)
        message = VehicleRatesSetpoint()
        message.timestamp = timestamp
        message.roll = float(command[2])
        message.pitch = float(-command[3])
        message.yaw = float(-command[4])
        message.thrust_body = [float(command[1]), 0.0, float(-command[0])]
        self.rates_publisher.publish(message)

    def _request_nav_state(self, nav_state: int) -> None:
        message = VehicleCommand()
        message.timestamp = self.get_clock().now().nanoseconds // 1000
        if nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            # PX4 documented ROS 2 mode request: custom mode enabled,
            # PX4_CUSTOM_MAIN_MODE_OFFBOARD=6.
            message.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
            message.param1 = 1.0
            message.param2 = 6.0
        elif nav_state == VehicleStatus.NAVIGATION_STATE_POSCTL:
            # PX4_CUSTOM_MAIN_MODE_POSCTL=3.
            message.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
            message.param1 = 1.0
            message.param2 = 3.0
        else:
            message.command = VehicleCommand.VEHICLE_CMD_SET_NAV_STATE
            message.param1 = float(nav_state)
        message.target_system = 1
        message.target_component = 1
        message.source_system = 1
        message.source_component = 1
        message.from_external = True
        self.command_publisher.publish(message)

    def _abort(self, reason: str) -> None:
        was_requested = self.output_requested or self.offboard_active
        now_ns = self.get_clock().now().nanoseconds
        elapsed, wall_elapsed = self.px4_timebase.stop_offboard(now_ns)
        tracking_error = (
            self.state[0:6] - self.current_reference[0:6]
            if self.state is not None and self.current_reference is not None
            else np.full(6, math.nan)
        )
        self.output_requested = False
        self.prestream_count = 0
        self.last_offboard_duration = elapsed
        self.last_wall_offboard_duration = wall_elapsed
        self.abort_reason = reason
        if was_requested:
            self._request_nav_state(VehicleStatus.NAVIGATION_STATE_POSCTL)
            self.get_logger().error(
                f"NMPC stopped: reason={reason}, px4_elapsed={elapsed:.2f}s, "
                f"wall_elapsed={wall_elapsed:.2f}s, "
                f"mode={self.test_mode}, phase={self.profile_phase}, "
                f"tracking_error_pv={np.round(tracking_error, 3).tolist()}, "
                f"maxima=[forward_speed={self.max_forward_speed:.3f}, "
                f"cross_track={self.max_cross_track_error:.3f}, "
                f"horizontal_tracking={self.max_horizontal_tracking_error:.3f}, "
                f"altitude={self.max_altitude_error:.3f}, "
                f"tilt_deg={self.max_tilt_degrees:.2f}, "
                f"commanded_pusher={self.max_commanded_pusher:.3f}], "
                f"control={np.round(self.last_command, 4).tolist()}; "
                "Position mode requested"
            )

    def _update(self) -> None:
        self.px4_timebase.advance_from_wall(
            self.get_clock().now().nanoseconds
        )
        if self.output_requested or self.offboard_active:
            self.max_state_wall_age = max(
                self.max_state_wall_age, self._state_age()
            )
            self.max_state_px4_age = max(
                self.max_state_px4_age, self._state_px4_age()
            )
        if self._state_is_stale(active=self.output_requested or self.offboard_active):
            if self.output_requested or self.offboard_active:
                self._abort("odometry_stale")
            return
        x_ref, u_ref, parameters = self._references()
        try:
            solution = self.controller.solve(self.state, x_ref, u_ref, parameters)
        except Exception as error:  # acados errors must never kill the watchdog
            self.solver_failures += 1
            self.get_logger().error(f"NMPC solve exception: {error}")
            if self.output_requested and self.solver_failures >= 3:
                self._abort("three_solver_exceptions")
            return
        self.last_solve_time = solution.solve_time
        self.solve_times_ms.append(1000.0 * solution.solve_time)
        if solution.status != 0 or not np.all(np.isfinite(solution.control)):
            self.solver_failures += 1
        else:
            self.solver_failures = 0
            self.last_raw_control = solution.control.copy()
            requested_control = solution.control.copy()
            if self.test_mode == "pretransition_8mps":
                altitude_error = self.state[2] - self.hold_state[2]
                forward_speed_for_lift = float(
                    np.dot(self.state[3:5], self.forward_direction)
                )
                requested_control[0], self.last_lift_unloading = (
                    pretransition_lift_command(
                        self.controller.model.plant,
                        altitude_error,
                        self.state[5],
                        forward_speed_for_lift,
                        maximum_unloading=self.lift_unloading_maximum,
                    )
                )
                self.max_lift_unloading = max(
                    self.max_lift_unloading, self.last_lift_unloading
                )
            else:
                requested_control[0] = self._vertical_hover_lift()
                self.last_lift_unloading = 0.0
            control_dt = self._control_dt()
            if self.test_mode == "external_pusher":
                pusher_sample = self.external_pusher_profile.sample(
                    self._profile_elapsed()
                )
                self.last_command = limit_external_pusher_command(
                    self.last_command,
                    requested_control,
                    pusher_sample.command,
                    control_dt,
                )
                self.max_commanded_pusher = max(
                    self.max_commanded_pusher, self.last_command[1]
                )
            elif self.test_mode in (
                "pusher_forward",
                "pretransition_5mps",
                "pretransition_8mps",
            ):
                previous_command = self.last_command.copy()
                self.last_command = limit_pusher_forward_command(
                    self.last_command,
                    requested_control,
                    control_dt,
                    pusher_limit=self.nmpc_pusher_max,
                )
                normal = np.array(
                    [-self.forward_direction[1], self.forward_direction[0]]
                )
                self.last_command = govern_pusher_forward_lateral(
                    previous_command,
                    self.last_command,
                    float(
                        np.dot(self.state[0:2] - self.hold_state[0:2], normal)
                    ),
                    float(np.dot(self.state[3:5], normal)),
                    self._roll_angle(),
                    control_dt,
                )
                forward_speed = float(
                    np.dot(self.state[3:5], self.forward_direction)
                )
                reference_speed = float(
                    np.dot(
                        self.current_reference[3:5], self.forward_direction
                    )
                )
                target_speed = (
                    self.lift_unloading_profile.target_speed
                    if self.test_mode == "pretransition_8mps"
                    else (
                        self.pretransition_profile.target_speed
                        if self.test_mode == "pretransition_5mps"
                        else self.pusher_forward_profile.target_speed
                    )
                )
                self.last_command = govern_pusher_forward_envelope(
                    previous_command,
                    self.last_command,
                    forward_speed,
                    reference_speed,
                    target_speed,
                    self._pitch_angle(),
                    control_dt,
                )
                self.max_commanded_pusher = max(
                    self.max_commanded_pusher, self.last_command[1]
                )
            else:
                self.last_command = limit_mc_command(
                    self.last_command, requested_control, control_dt
                )
        diagnostic = Float64MultiArray()
        diagnostic.data = [
            *self.last_command.tolist(),
            float(solution.status),
            1000.0 * solution.solve_time,
            *self.last_raw_control.tolist(),
        ]
        self.diagnostic_publisher.publish(diagnostic)

        if not self.output_requested:
            return
        if self.ever_offboard and not self.offboard_active:
            self._abort("px4_left_offboard")
            return
        self._update_flight_metrics()
        reason = self._safety_reason()
        if reason is not None:
            self._abort(reason)
            return
        # Bumpless transfer: prestream and the first 0.5 s in Offboard use
        # the ULog-confirmed hover thrust with zero rates. Feedback then ramps
        # in through the normal slew limiter.
        handover_command = self.last_command
        freeze_handover = False
        if not self.offboard_active:
            freeze_handover = True
        elif self.px4_timebase.active:
            if self._offboard_elapsed() < self.HANDOVER_FREEZE_SECONDS:
                freeze_handover = True
        if freeze_handover:
            self.last_command = np.array(
                [self.controller.model.plant.hover_command, 0.0, 0.0, 0.0, 0.0]
            )
            handover_command = self.last_command
        # PX4 requires a stream of setpoints before accepting Offboard.
        self._publish_setpoint(handover_command)
        self.prestream_count += 1
        if (
            20 <= self.prestream_count <= 50
            and self.prestream_count % 10 == 0
            and not self.offboard_active
        ):
            self._request_nav_state(VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        elif self.prestream_count > 70 and not self.offboard_active:
            self._abort("px4_did_not_enter_offboard")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StandardVtolNmpcNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
