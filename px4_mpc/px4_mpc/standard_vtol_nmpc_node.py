"""ROS 2 shadow and guarded hover-Offboard node for Standard VTOL NMPC."""

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
from px4_mpc.models.frames import ned_to_enu, px4_quaternion_to_gazebo


class StandardVtolNmpcNode(Node):
    """Solve continuously in shadow mode and optionally hold hover Offboard."""

    def __init__(self) -> None:
        super().__init__("standard_vtol_nmpc")
        self.declare_parameter("allow_offboard_output", False)
        self.declare_parameter("hover_offboard_max_seconds", 5.0)
        self.allow_output = bool(self.get_parameter("allow_offboard_output").value)
        self.max_offboard_seconds = float(
            self.get_parameter("hover_offboard_max_seconds").value
        )
        self.controller = StandardVtolNmpc()
        self.state: np.ndarray | None = None
        self.state_received_ns = 0
        self.status: VehicleStatus | None = None
        self.vtol_status: VtolVehicleStatus | None = None
        self.hold_state: np.ndarray | None = None
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
            Trigger, "/standard_vtol_nmpc/enable_hover_offboard", self._enable
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
        mode = "armed-capable hover" if self.allow_output else "shadow-only"
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
    def _level_yaw_quaternion(quaternion: np.ndarray) -> np.ndarray:
        """Keep measured yaw while setting the hover roll and pitch to zero."""
        qw, qx, qy, qz = quaternion
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
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

    def _enable(self, _request, response):
        if not self.allow_output:
            response.success = False
            response.message = "launch with allow_offboard_output:=true first"
            return response
        ready, reason = self._ready_for_hover()
        if not ready:
            response.success = False
            response.message = reason
            return response
        self.hold_state = self.state.copy()
        self.hold_state[3:6] = 0.0
        self.hold_state[6:10] = self._level_yaw_quaternion(
            self.hold_state[6:10]
        )
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
        response.success = True
        response.message = "hover setpoint prestream started; PX4 Offboard follows after 1 s"
        return response

    def _capture_hover_reference(self, _request, response):
        ready, reason = self._ready_for_hover()
        if not ready:
            response.success = False
            response.message = reason
            return response
        self.hold_state = self.state.copy()
        self.hold_state[3:6] = 0.0
        self.hold_state[6:10] = self._level_yaw_quaternion(
            self.hold_state[6:10]
        )
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

    def _status(self, _request, response):
        nav = -1 if self.status is None else int(self.status.nav_state)
        armed = False if self.status is None else (
            self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )
        if self.state is None or self.hold_state is None:
            tracking_error = "none"
        else:
            tracking_error = np.round(
                self.state[0:6] - self.hold_state[0:6], 4
            ).tolist()
        response.success = self.solver_failures == 0 and self._state_age() <= 0.20
        response.message = (
            f"output_requested={self.output_requested}, offboard={self.offboard_active}, "
            f"armed={armed}, nav_state={nav}, state_age={self._state_age():.3f}s, "
            f"solve_time={1000.0 * self.last_solve_time:.2f}ms, "
            f"solver_failures={self.solver_failures}, abort_reason={self.abort_reason}"
            f", configured_timeout={self.max_offboard_seconds:.1f}s"
            f", last_offboard_duration={self.last_offboard_duration:.2f}s"
            f", last_command_ack={self.last_command_ack}"
            f", control={np.round(self.last_command, 4).tolist()}"
            f", raw_control={np.round(self.last_raw_control, 4).tolist()}"
            f", tracking_error_pv={tracking_error}"
        )
        return response

    def _hover_references(self):
        reference = self.state.copy() if self.hold_state is None else self.hold_state
        if self.hold_state is None:
            reference[3:6] = 0.0
        x_ref = np.repeat(reference[None, :], self.controller.N + 1, axis=0)
        u_ref = np.zeros((self.controller.N, 5))
        u_ref[:, 0] = self.controller.model.plant.hover_command
        parameters = np.zeros((self.controller.N + 1, 4))
        return x_ref, u_ref, parameters

    @staticmethod
    def _limit_hover_command(previous, requested, dt=0.05):
        """Apply conservative limits specific to the first hover handover."""
        requested = np.asarray(requested, dtype=float).copy()
        # The measured hover point is 0.5198. Keep the first hover controller
        # in a narrow envelope; +/-0.04 still provides ample vertical
        # authority without reproducing the observed climb/descent overshoot.
        requested[0] = np.clip(requested[0], 0.48, 0.56)
        requested[1] = 0.0  # Pusher is never active in hover-only mode.
        requested[2:5] = np.clip(
            0.5 * requested[2:5],
            [-0.20, -0.20, -0.15],
            [0.20, 0.20, 0.15],
        )
        slew_per_second = np.array([0.10, 0.0, 0.30, 0.30, 0.20])
        limited = previous + np.clip(
            requested - previous,
            -slew_per_second * dt,
            slew_per_second * dt,
        )
        limited[1] = 0.0
        return limited

    def _vertical_hover_lift(self) -> float:
        """Return a slow, critically damped lift command about hover trim."""
        if self.hold_state is None:
            return self.controller.model.plant.hover_command
        plant = self.controller.model.plant
        motor = plant.motors[0]
        hover = plant.hover_command
        hover_speed = motor.target_speed(hover)
        acceleration_per_command = (
            8.0
            * motor.motor_constant
            * hover_speed
            * (motor.maximum_speed - motor.minimum_speed)
            / plant.mass
        )
        altitude_error = self.state[2] - self.hold_state[2]
        vertical_speed = self.state[5]
        # Desired a_z = -wn^2 e_z - 2*zeta*wn*v_z, with wn=zeta=1.
        desired_acceleration = -altitude_error - 2.0 * vertical_speed
        return float(hover + desired_acceleration / acceleration_per_command)

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
        if np.linalg.norm(self.state[3:5]) > 2.0:
            return "horizontal_speed_limit"
        if np.linalg.norm(self.state[0:2] - self.hold_state[0:2]) > 5.0:
            return "horizontal_geofence"
        qw, qx, qy, qz = self.state[6:10]
        roll = math.atan2(
            2.0 * (qw * qx + qy * qz),
            1.0 - 2.0 * (qx * qx + qy * qy),
        )
        pitch = math.asin(np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0))
        if max(abs(roll), abs(pitch)) > math.radians(25.0):
            return "tilt_limit"
        if self.solver_failures >= 3:
            return "three_solver_failures"
        if (
            self.offboard_active
            and self.max_offboard_seconds > 0.0
            and self.offboard_entered_ns > 0
            and (self.get_clock().now().nanoseconds - self.offboard_entered_ns)
            * 1e-9
            >= self.max_offboard_seconds
        ):
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
            self.state[0:6] - self.hold_state[0:6]
            if self.state is not None and self.hold_state is not None
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
                f"tracking_error_pv={np.round(tracking_error, 3).tolist()}, "
                f"control={np.round(self.last_command, 4).tolist()}; "
                "Position mode requested"
            )

    def _update(self) -> None:
        if self.state is None or self._state_age() > 0.20:
            if self.output_requested or self.offboard_active:
                self._abort("odometry_stale")
            return
        x_ref, u_ref, parameters = self._hover_references()
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
            self.last_command = self._limit_hover_command(
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
            if offboard_elapsed < 0.5:
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
