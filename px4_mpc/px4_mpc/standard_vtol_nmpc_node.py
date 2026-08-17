"""ROS 2 shadow and guarded multicopter-Offboard node for Standard VTOL NMPC."""

from __future__ import annotations

import math

import numpy as np
import rclpy
from px4_msgs.msg import (
    OffboardControlMode,
    VehicleCommand,
    VehicleCommandAck,
    VehicleOdometry,
    VehicleRatesSetpoint,
    VehicleStatus,
    VtolVehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from px4_mpc.controllers.standard_vtol_nmpc import StandardVtolNmpc
from px4_mpc.controllers.standard_vtol_output import (
    limit_external_pusher_command,
    limit_mc_command,
    vertical_hover_lift,
)
from px4_mpc.models.external_pusher_profile import ExternalPusherProfile
from px4_mpc.models.frames import ned_to_enu, px4_quaternion_to_gazebo
from px4_mpc.models.mc_forward_profile import (
    McForwardProfile,
    mc_forward_reference_state,
)


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
        self.controller = StandardVtolNmpc()
        self.state: np.ndarray | None = None
        self.state_received_ns = 0
        self.status: VehicleStatus | None = None
        self.vtol_status: VtolVehicleStatus | None = None
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
        self.offboard_entered_ns = 0
        self.last_offboard_duration = 0.0
        self.prestream_count = 0
        self.solver_failures = 0
        self.last_solve_time = math.nan
        self.abort_reason = "none"
        self.last_command_ack = "none"
        self.max_forward_speed = 0.0
        self.max_cross_track_error = 0.0
        self.max_horizontal_tracking_error = 0.0
        self.max_altitude_error = 0.0
        self.max_tilt_degrees = 0.0
        self.max_commanded_pusher = 0.0

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self._odometry, qos
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
        self.state_received_ns = self.get_clock().now().nanoseconds

    def _vehicle_status(self, message: VehicleStatus) -> None:
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
            self.offboard_entered_ns = self.get_clock().now().nanoseconds
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

    def _vtol_vehicle_status(self, message: VtolVehicleStatus) -> None:
        self.vtol_status = message

    def _vehicle_command_ack(self, message: VehicleCommandAck) -> None:
        self.last_command_ack = f"command={message.command},result={message.result}"

    def _state_age(self) -> float:
        if not self.state_received_ns:
            return math.inf
        return (self.get_clock().now().nanoseconds - self.state_received_ns) * 1e-9

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
        if self.state is None or self._state_age() > 0.20:
            return False, "odometry_stale"
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
        if abs(self.state[5]) > 0.20:
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
        self.offboard_entered_ns = 0
        self.last_offboard_duration = 0.0
        self.last_command = np.array(
            [self.controller.model.plant.hover_command, 0.0, 0.0, 0.0, 0.0]
        )
        self.last_raw_control = self.last_command.copy()
        self.abort_reason = "none"
        self.last_command_ack = "none"
        self.max_forward_speed = 0.0
        self.max_cross_track_error = 0.0
        self.max_horizontal_tracking_error = 0.0
        self.max_altitude_error = 0.0
        self.max_tilt_degrees = 0.0
        self.max_commanded_pusher = 0.0

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
        if self.test_mode == "external_pusher":
            return self.external_pusher_test_max_seconds
        if self.test_mode == "mc_forward":
            return self.forward_test_max_seconds
        return self.max_offboard_seconds

    def _offboard_elapsed(self) -> float:
        if self.offboard_entered_ns <= 0:
            return self.last_offboard_duration
        return (
            self.get_clock().now().nanoseconds - self.offboard_entered_ns
        ) * 1e-9

    def _profile_elapsed(self) -> float:
        return max(0.0, self._offboard_elapsed() - self.HANDOVER_FREEZE_SECONDS)

    def _status(self, _request, response):
        nav = -1 if self.status is None else int(self.status.nav_state)
        armed = False if self.status is None else (
            self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )
        tracking_error, displacement = _status_tracking_values(
            self.state, self.current_reference, self.hold_state
        )
        response.success = self.solver_failures == 0 and self._state_age() <= 0.20
        response.message = (
            f"output_requested={self.output_requested}, offboard={self.offboard_active}, "
            f"armed={armed}, nav_state={nav}, state_age={self._state_age():.3f}s, "
            f"solve_time={1000.0 * self.last_solve_time:.2f}ms, "
            f"solver_failures={self.solver_failures}, abort_reason={self.abort_reason}"
            f", test_mode={self.test_mode}, profile_phase={self.profile_phase}"
            f", configured_timeout={self._active_timeout():.1f}s"
            f", forward_profile=[speed={self.forward_profile.target_speed:.1f},"
            f"accel={self.forward_profile.acceleration:.1f},"
            f"hold={self.forward_profile.hold_seconds:.1f}]"
            f", last_offboard_duration={self.last_offboard_duration:.2f}s"
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
            f"commanded_pusher={self.max_commanded_pusher:.3f}]"
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
        return reference

    def _references(self):
        base_time = self._profile_elapsed()
        x_ref = np.vstack(
            [
                self._reference_at_profile_time(base_time + stage * self.controller.dt)
                for stage in range(self.controller.N + 1)
            ]
        )
        self.current_reference = self._reference_at_profile_time(base_time)
        if self.test_mode == "mc_forward":
            self.profile_phase = self.forward_profile.sample(base_time).phase
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

    def _safety_reason(self) -> str | None:
        # Active-flight limits differ from the stricter handover conditions:
        # small velocities are required to enter Offboard, while modest
        # closed-loop corrections are allowed once Offboard is active.
        if self.state is None or self._state_age() > 0.20:
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
        if abs(self.state[2] - self.hold_state[2]) > 0.5:
            return "altitude_error"
        if abs(self.state[5]) > 0.75:
            return "vertical_speed_limit"
        if self.test_mode == "mc_forward":
            horizontal_speed_limit = 2.7
        elif self.test_mode == "external_pusher":
            horizontal_speed_limit = 1.5
        else:
            horizontal_speed_limit = 2.0
        if np.linalg.norm(self.state[3:5]) > horizontal_speed_limit:
            return "horizontal_speed_limit"
        delta_xy = self.state[0:2] - self.hold_state[0:2]
        if self.test_mode == "mc_forward":
            normal = np.array(
                [-self.forward_direction[1], self.forward_direction[0]]
            )
            along_track = float(np.dot(delta_xy, self.forward_direction))
            cross_track = abs(float(np.dot(delta_xy, normal)))
            if along_track < -1.0 or along_track > self.forward_profile.final_distance + 2.0:
                return "forward_geofence"
            if cross_track > 1.5:
                return "cross_track_limit"
            if (
                self.current_reference is not None
                and np.linalg.norm(self.state[0:2] - self.current_reference[0:2])
                > 1.5
            ):
                return "horizontal_tracking_error"
        elif self.test_mode == "external_pusher" and np.linalg.norm(delta_xy) > 3.0:
            return "external_pusher_geofence"
        elif np.linalg.norm(delta_xy) > 5.0:
            return "horizontal_geofence"
        if self._tilt_degrees() > 25.0:
            return "tilt_limit"
        if self.solver_failures >= 3:
            return "three_solver_failures"
        if (
            self.offboard_active
            and self._active_timeout() > 0.0
            and self.offboard_entered_ns > 0
            and self._offboard_elapsed() >= self._active_timeout()
        ):
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
        elapsed = (
            (now_ns - self.offboard_entered_ns) * 1e-9
            if self.offboard_entered_ns > 0
            else 0.0
        )
        tracking_error = (
            self.state[0:6] - self.current_reference[0:6]
            if self.state is not None and self.current_reference is not None
            else np.full(6, math.nan)
        )
        self.output_requested = False
        self.prestream_count = 0
        self.last_offboard_duration = elapsed
        self.offboard_entered_ns = 0
        self.abort_reason = reason
        if was_requested:
            self._request_nav_state(VehicleStatus.NAVIGATION_STATE_POSCTL)
            self.get_logger().error(
                f"NMPC stopped: reason={reason}, offboard_elapsed={elapsed:.2f}s, "
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
        if self.state is None or self._state_age() > 0.20:
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
        if solution.status != 0 or not np.all(np.isfinite(solution.control)):
            self.solver_failures += 1
        else:
            self.solver_failures = 0
            self.last_raw_control = solution.control.copy()
            requested_control = solution.control.copy()
            requested_control[0] = self._vertical_hover_lift()
            if self.test_mode == "external_pusher":
                pusher_sample = self.external_pusher_profile.sample(
                    self._profile_elapsed()
                )
                self.last_command = limit_external_pusher_command(
                    self.last_command,
                    requested_control,
                    pusher_sample.command,
                )
                self.max_commanded_pusher = max(
                    self.max_commanded_pusher, self.last_command[1]
                )
            else:
                self.last_command = limit_mc_command(
                    self.last_command, requested_control
                )
        diagnostic = Float64MultiArray()
        diagnostic.data = [
            *self.last_command.tolist(),
            float(solution.status),
            1000.0 * solution.solve_time,
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
        elif self.offboard_entered_ns > 0:
            offboard_elapsed = (
                self.get_clock().now().nanoseconds - self.offboard_entered_ns
            ) * 1e-9
            if offboard_elapsed < self.HANDOVER_FREEZE_SECONDS:
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
