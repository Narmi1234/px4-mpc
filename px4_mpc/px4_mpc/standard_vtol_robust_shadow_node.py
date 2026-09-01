"""ROS 2 shadow and guarded-hover node for the 16-state VTOL NMPC."""

from __future__ import annotations

from collections import deque
import math
from pathlib import Path

import numpy as np
from px4_mpc.controllers.standard_vtol_robust_nmpc import (
    StandardVtolRobustNmpc,
)
from px4_mpc.controllers.standard_vtol_output import (
    limit_mc_command,
    vertical_hover_lift,
)
from px4_mpc.models.frames import (
    frd_to_flu,
    ned_to_enu,
    px4_quaternion_to_gazebo,
)
from px4_msgs.msg import (
    AirspeedValidated,
    OffboardControlMode,
    VehicleCommand,
    VehicleOdometry,
    VehicleRatesSetpoint,
    VehicleStatus,
    VtolNmpcAllocationSetpoint,
    VtolNmpcAllocationStatus,
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
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger


class StandardVtolRobustShadow(Node):
    """Run the robust OCP read-only or in an explicit guarded hover gate."""

    def __init__(self) -> None:
        super().__init__("standard_vtol_robust_shadow")
        self.declare_parameter("horizon_steps", 20)
        self.declare_parameter("horizon_seconds", 2.0)
        self.declare_parameter("max_state_age_seconds", 0.20)
        self.declare_parameter("allow_hover_output", False)
        self.declare_parameter("allow_l1_output", False)
        self.declare_parameter("hover_test_seconds", 5.0)
        self.declare_parameter("l1_target_speed", 5.0)
        self.declare_parameter("l1_acceleration", 0.4)
        self.declare_parameter("l1_hold_seconds", 3.0)
        self.declare_parameter("l1_min_lambda", 0.8)
        self.declare_parameter("l1_pusher_max", 0.25)
        horizon_steps = int(self.get_parameter("horizon_steps").value)
        horizon_seconds = float(self.get_parameter("horizon_seconds").value)
        root = Path(__file__).resolve().parents[2]
        build_name = (
            f"standard_vtol_robust_nmpc_n{horizon_steps}_"
            f"tf{int(round(1000.0 * horizon_seconds))}ms"
        )
        self.controller = StandardVtolRobustNmpc(
            horizon_steps=horizon_steps,
            horizon_seconds=horizon_seconds,
            build_directory=root / "build" / build_name,
        )
        self.max_state_age = float(
            self.get_parameter("max_state_age_seconds").value
        )
        self.allow_hover_output = bool(
            self.get_parameter("allow_hover_output").value
        )
        self.allow_l1_output = bool(
            self.get_parameter("allow_l1_output").value
        )
        self.allow_output = self.allow_hover_output or self.allow_l1_output
        self.hover_test_seconds = float(
            self.get_parameter("hover_test_seconds").value
        )
        if self.hover_test_seconds <= 0.0 or self.hover_test_seconds > 5.0:
            raise ValueError("hover_test_seconds must be in (0, 5] seconds")
        self.l1_target_speed = float(
            self.get_parameter("l1_target_speed").value
        )
        self.l1_acceleration = float(
            self.get_parameter("l1_acceleration").value
        )
        self.l1_hold_seconds = float(
            self.get_parameter("l1_hold_seconds").value
        )
        self.l1_min_lambda = float(
            self.get_parameter("l1_min_lambda").value
        )
        self.l1_pusher_max = float(
            self.get_parameter("l1_pusher_max").value
        )
        if not (
            0.0 < self.l1_target_speed <= 5.0
            and 0.1 <= self.l1_acceleration <= 0.5
            and 1.0 <= self.l1_hold_seconds <= 5.0
            and 0.8 <= self.l1_min_lambda <= 1.0
            and 0.05 <= self.l1_pusher_max <= 0.25
        ):
            raise ValueError("L1 parameters exceed the guarded envelope")
        self.state: np.ndarray | None = None
        self.surface_state = np.zeros(3)
        self.reference: np.ndarray | None = None
        self.status_message: VehicleStatus | None = None
        self.vtol_status: VtolVehicleStatus | None = None
        self.airspeed: AirspeedValidated | None = None
        self.state_received_ns = 0
        self.last_update_ns = 0
        self.last_control = self.controller.model.hover_control()
        self.last_solver_status = -1
        self.last_solve_time_ms = math.nan
        self.solver_failures = 0
        self.solve_count = 0
        self.warmup_solve_count = 100
        self.solve_times_ms: deque[float] = deque(maxlen=2000)
        self.model_function = self.controller.model.function()
        self.output_requested = False
        self.ever_offboard = False
        self.prestream_count = 0
        self.offboard_start_ns = 0
        self.last_offboard_duration = 0.0
        self.abort_reason = "none"
        self.test_mode = "idle"
        self.published_control = self.controller.model.hover_control()
        self.allocation_status: VtolNmpcAllocationStatus | None = None
        self.allocation_ever_active = False
        self.allocation_ever_valid = False
        self.reference_forward = np.array([1.0, 0.0])
        self.reference_lateral = np.array([0.0, 1.0])
        self.max_forward_speed = 0.0
        self.max_cross_track = 0.0
        self.max_altitude_error = 0.0
        self.min_applied_lambda = 1.0
        self.max_pusher = 0.0

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
            VehicleStatus, "/fmu/out/vehicle_status", self._vehicle_status, qos
        )
        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v4",
            self._vehicle_status,
            qos,
        )
        self.create_subscription(
            VtolNmpcAllocationStatus,
            "/fmu/out/vtol_nmpc_allocation_status",
            self._allocation_status,
            qos,
        )
        self.create_subscription(
            VtolVehicleStatus,
            "/fmu/out/vtol_vehicle_status",
            self._vtol_status,
            qos,
        )
        self.create_subscription(
            AirspeedValidated,
            "/fmu/out/airspeed_validated_v1",
            self._airspeed_status,
            qos,
        )
        self.proposed_control_publisher = self.create_publisher(
            Float64MultiArray,
            "/standard_vtol_robust_shadow/proposed_control",
            10,
        )
        self.create_service(
            Trigger,
            "/standard_vtol_robust_shadow/capture_hover_reference",
            self._capture_hover_reference,
        )
        self.create_service(
            Trigger,
            "/standard_vtol_robust_shadow/clear_reference",
            self._clear_reference,
        )
        self.create_service(
            Trigger,
            "/standard_vtol_robust_shadow/status",
            self._status,
        )
        self.create_service(
            Trigger,
            "/standard_vtol_robust_shadow/enable_hover_offboard",
            self._enable_hover_offboard,
        )
        self.create_service(
            Trigger,
            "/standard_vtol_robust_shadow/enable_allocation_l1",
            self._enable_allocation_l1,
        )
        self.create_service(
            Trigger,
            "/standard_vtol_robust_shadow/disable",
            self._disable_output,
        )
        self.offboard_publisher = None
        self.rates_publisher = None
        self.command_publisher = None
        self.allocation_publisher = None
        if self.allow_output:
            self.offboard_publisher = self.create_publisher(
                OffboardControlMode, "/fmu/in/offboard_control_mode", qos
            )
            self.rates_publisher = self.create_publisher(
                VehicleRatesSetpoint, "/fmu/in/vehicle_rates_setpoint", qos
            )
            self.command_publisher = self.create_publisher(
                VehicleCommand, "/fmu/in/vehicle_command", qos
            )
            self.allocation_publisher = self.create_publisher(
                VtolNmpcAllocationSetpoint,
                "/fmu/in/vtol_nmpc_allocation_setpoint",
                qos,
            )
        self.create_timer(0.05, self._update)
        if self.allow_l1_output:
            self.get_logger().info(
                "Robust Standard VTOL NMPC started in guarded L1 allocation "
                "mode (5 m/s, lambda >= 0.8, MC only)"
            )
        elif self.allow_hover_output:
            self.get_logger().info(
                "Robust Standard VTOL NMPC started in guarded 5 s hover mode"
            )
        else:
            self.get_logger().info(
                "Robust Standard VTOL NMPC started in read-only shadow mode; "
                "no /fmu/in publisher exists in this node"
            )

    def _vehicle_status(self, message: VehicleStatus) -> None:
        self.status_message = message

    def _allocation_status(self, message: VtolNmpcAllocationStatus) -> None:
        self.allocation_status = message
        self.allocation_ever_active |= bool(message.active)
        self.allocation_ever_valid |= bool(message.setpoint_valid)

    def _vtol_status(self, message: VtolVehicleStatus) -> None:
        self.vtol_status = message

    def _airspeed_status(self, message: AirspeedValidated) -> None:
        self.airspeed = message

    def _odometry(self, message: VehicleOdometry) -> None:
        if message.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            return
        if message.velocity_frame != VehicleOdometry.VELOCITY_FRAME_NED:
            return
        values = np.r_[
            message.position,
            message.velocity,
            message.q,
            message.angular_velocity,
        ]
        if not np.all(np.isfinite(values)):
            return
        self.state = np.r_[
            ned_to_enu(np.asarray(message.position)),
            ned_to_enu(np.asarray(message.velocity)),
            px4_quaternion_to_gazebo(np.asarray(message.q)),
            frd_to_flu(np.asarray(message.angular_velocity)),
            self.surface_state,
        ]
        self.state_received_ns = self.get_clock().now().nanoseconds

    def _state_age(self) -> float:
        if self.state_received_ns == 0:
            return math.inf
        return (
            self.get_clock().now().nanoseconds - self.state_received_ns
        ) * 1.0e-9

    @staticmethod
    def _yaw(quaternion: np.ndarray) -> float:
        qw, qx, qy, qz = quaternion
        return math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )

    @classmethod
    def _level_yaw_quaternion(cls, quaternion: np.ndarray) -> np.ndarray:
        yaw = cls._yaw(quaternion)
        return np.array([math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)])

    def _capture_hover_reference(self, _request, response):
        if self.state is None or self._state_age() > self.max_state_age:
            response.success = False
            response.message = "odometry_stale"
            return response
        if (
            self.status_message is None
            or self.status_message.arming_state
            != VehicleStatus.ARMING_STATE_ARMED
        ):
            response.success = False
            response.message = "vehicle_not_armed"
            return response
        self.reference = self.state.copy()
        self.reference[3:6] = 0.0
        self.reference[6:10] = self._level_yaw_quaternion(self.state[6:10])
        self.reference[10:16] = 0.0
        yaw = self._yaw(self.reference[6:10])
        self.reference_forward = np.array([math.cos(yaw), math.sin(yaw)])
        self.reference_lateral = np.array([-math.sin(yaw), math.cos(yaw)])
        self.surface_state[:] = 0.0
        self.last_control = self.controller.model.hover_control()
        self.last_solver_status = -1
        self.solver_failures = 0
        self.solve_count = 0
        self.solve_times_ms.clear()
        self.abort_reason = "none"
        self.allocation_ever_active = False
        self.allocation_ever_valid = False
        self.max_forward_speed = 0.0
        self.max_cross_track = 0.0
        self.max_altitude_error = 0.0
        self.min_applied_lambda = 1.0
        self.max_pusher = 0.0
        response.success = True
        response.message = "hover reference captured"
        return response

    def _clear_reference(self, _request, response):
        if self.output_requested or self._offboard_active():
            self._abort("reference_clear_requested")
        self.reference = None
        response.success = True
        response.message = "shadow reference cleared"
        return response

    def _status(self, _request, response):
        armed = (
            self.status_message is not None
            and self.status_message.arming_state
            == VehicleStatus.ARMING_STATE_ARMED
        )
        nav_state = (
            int(self.status_message.nav_state)
            if self.status_message is not None
            else -1
        )
        p99 = (
            float(np.percentile(self.solve_times_ms, 99))
            if self.solve_times_ms
            else math.nan
        )
        state_age = self._state_age()
        offboard = self._offboard_active()
        vtol_state = (
            int(self.vtol_status.vehicle_vtol_state)
            if self.vtol_status is not None
            else -1
        )
        forward_speed = (
            self._forward_speed()
            if self.state is not None and self.reference is not None
            else math.nan
        )
        cross_track = (
            self._cross_track()
            if self.state is not None and self.reference is not None
            else math.nan
        )
        calibrated_airspeed = (
            float(self.airspeed.calibrated_airspeed_m_s)
            if self.airspeed is not None
            else math.nan
        )
        if self.allocation_status is None:
            allocation = "unavailable"
        else:
            allocation = (
                f"requested={self.allocation_status.requested_weight:.3f},"
                f"applied={self.allocation_status.applied_weight:.3f},"
                f"active={self.allocation_status.active},"
                f"valid={self.allocation_status.setpoint_valid},"
                f"ever_active={self.allocation_ever_active},"
                f"ever_valid={self.allocation_ever_valid}"
            )
        response.success = (
            self.reference is not None
            and state_age <= self.max_state_age
            and self.last_solver_status == 0
            and self.solver_failures == 0
            and p99 <= 40.0
        )
        warmup_remaining = max(0, self.warmup_solve_count - self.solve_count)
        response.message = (
            f"read_only={not self.allow_output},"
            f"publishes_fmu={self.allow_output},"
            f"output_requested={self.output_requested},offboard={offboard},"
            f"test_mode={self.test_mode},vtol_state={vtol_state},"
            f"reference_captured={self.reference is not None},"
            f"armed={armed},nav_state={nav_state},state_age={state_age:.3f}s,"
            f"solver_status={self.last_solver_status},"
            f"solver_failures={self.solver_failures},"
            f"warmup_remaining={warmup_remaining},"
            f"solve_time={self.last_solve_time_ms:.2f}ms,"
            f"solve_time_p99={p99:.2f}ms,"
            f"last_offboard_duration={self.last_offboard_duration:.2f}s,"
            f"abort_reason={self.abort_reason},"
            f"allocation=[{allocation}],"
            f"motion=[forward_speed={forward_speed:.3f},"
            f"cross_track={cross_track:.3f},airspeed={calibrated_airspeed:.3f}],"
            f"maxima=[forward_speed={self.max_forward_speed:.3f},"
            f"cross_track={self.max_cross_track:.3f},"
            f"altitude={self.max_altitude_error:.3f},"
            f"pusher={self.max_pusher:.3f},"
            f"min_lambda={self.min_applied_lambda:.3f}],"
            f"control={np.round(self.last_control, 4).tolist()},"
            f"published_control={np.round(self.published_control, 4).tolist()}"
        )
        return response

    def _offboard_active(self) -> bool:
        return (
            self.status_message is not None
            and self.status_message.nav_state
            == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )

    def _enable_hover_offboard(self, _request, response):
        if not self.allow_hover_output:
            response.success = False
            response.message = "launch guarded hover output mode first"
            return response
        if self.state is None or self._state_age() > self.max_state_age:
            response.success = False
            response.message = "odometry_stale"
            return response
        if self.reference is None:
            response.success = False
            response.message = "capture_hover_reference_first"
            return response
        p99 = (
            float(np.percentile(self.solve_times_ms, 99))
            if self.solve_times_ms
            else math.inf
        )
        if (
            self.solve_count < self.warmup_solve_count
            or self.last_solver_status != 0
            or self.solver_failures != 0
            or p99 > 40.0
        ):
            response.success = False
            response.message = "solver_not_warmed_or_realtime_unready"
            return response
        if (
            self.status_message is None
            or self.status_message.arming_state
            != VehicleStatus.ARMING_STATE_ARMED
        ):
            response.success = False
            response.message = "vehicle_not_armed"
            return response
        if self._offboard_active():
            response.success = False
            response.message = "vehicle_already_offboard"
            return response
        if np.linalg.norm(self.state[3:5]) > 0.5:
            response.success = False
            response.message = "horizontal_speed_too_high"
            return response
        if abs(self.state[5]) > 0.15:
            response.success = False
            response.message = "vertical_speed_too_high"
            return response
        self.output_requested = True
        self.test_mode = "hover"
        self.ever_offboard = False
        self.prestream_count = 0
        self.offboard_start_ns = 0
        self.last_offboard_duration = 0.0
        self.abort_reason = "none"
        self.allocation_ever_active = False
        self.allocation_ever_valid = False
        self.published_control = self.controller.model.hover_control()
        response.success = True
        response.message = (
            "guarded robust hover prestream started; PX4 Offboard follows "
            "after 1 s and returns to Position after 5 s"
        )
        return response

    def _enable_allocation_l1(self, _request, response):
        if not self.allow_l1_output:
            response.success = False
            response.message = "launch guarded L1 allocation mode first"
            return response
        if self.state is None or self._state_age() > self.max_state_age:
            response.success = False
            response.message = "odometry_stale"
            return response
        if self.reference is None:
            response.success = False
            response.message = "capture_hover_reference_first"
            return response
        p99 = (
            float(np.percentile(self.solve_times_ms, 99))
            if self.solve_times_ms
            else math.inf
        )
        if (
            self.solve_count < self.warmup_solve_count
            or self.last_solver_status != 0
            or self.solver_failures != 0
            or p99 > 40.0
        ):
            response.success = False
            response.message = "solver_not_warmed_or_realtime_unready"
            return response
        if (
            self.status_message is None
            or self.status_message.arming_state
            != VehicleStatus.ARMING_STATE_ARMED
        ):
            response.success = False
            response.message = "vehicle_not_armed"
            return response
        if (
            self.vtol_status is None
            or self.vtol_status.vehicle_vtol_state
            != VtolVehicleStatus.VEHICLE_VTOL_STATE_MC
        ):
            response.success = False
            response.message = "vehicle_not_in_mc_mode"
            return response
        if self._offboard_active():
            response.success = False
            response.message = "vehicle_already_offboard"
            return response
        if abs(self.state[5]) > 0.15 or np.linalg.norm(self.state[3:5]) > 0.5:
            response.success = False
            response.message = "vehicle_not_settled_for_l1"
            return response
        self.output_requested = True
        self.test_mode = "allocation_l1"
        self.ever_offboard = False
        self.prestream_count = 0
        self.offboard_start_ns = 0
        self.last_offboard_duration = 0.0
        self.abort_reason = "none"
        self.published_control = self.controller.model.hover_control()
        self.allocation_ever_active = False
        self.allocation_ever_valid = False
        self.max_forward_speed = 0.0
        self.max_cross_track = 0.0
        self.max_altitude_error = 0.0
        self.min_applied_lambda = 1.0
        self.max_pusher = 0.0
        response.success = True
        response.message = (
            "guarded L1 prestream started; MC-only 0->5->0 m/s, "
            "lambda 1->0.8->1, then Position"
        )
        return response

    def _disable_output(self, _request, response):
        self._abort("manual_disable")
        response.success = True
        response.message = "Position mode requested; robust output disabled"
        return response

    def _request_nav_state(self, offboard: bool) -> None:
        if self.command_publisher is None:
            return
        message = VehicleCommand()
        message.timestamp = self.get_clock().now().nanoseconds // 1000
        message.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        message.param1 = 1.0
        message.param2 = 6.0 if offboard else 3.0
        message.target_system = 1
        message.target_component = 1
        message.source_system = 1
        message.source_component = 1
        message.from_external = True
        self.command_publisher.publish(message)

    def _publish_setpoint(self, control: np.ndarray) -> None:
        if (
            self.offboard_publisher is None
            or self.rates_publisher is None
            or self.allocation_publisher is None
        ):
            return
        timestamp = self.get_clock().now().nanoseconds // 1000
        mode = OffboardControlMode()
        mode.timestamp = timestamp
        mode.body_rate = True
        self.offboard_publisher.publish(mode)
        message = VehicleRatesSetpoint()
        message.timestamp = timestamp
        message.roll = float(control[2])
        message.pitch = float(-control[3])
        message.yaw = float(-control[4])
        message.thrust_body = [
            float(control[1]), 0.0, float(-control[0])
        ]
        self.rates_publisher.publish(message)
        allocation = VtolNmpcAllocationSetpoint()
        allocation.timestamp = timestamp
        allocation.transition_weight = float(np.clip(control[5], 0.0, 1.0))
        self.allocation_publisher.publish(allocation)

    def _tilt_degrees(self) -> float:
        qw, qx, qy, qz = self.state[6:10]
        roll = math.atan2(
            2.0 * (qw * qx + qy * qz),
            1.0 - 2.0 * (qx * qx + qy * qy),
        )
        pitch = math.asin(
            np.clip(2.0 * (qw * qy - qz * qx), -1.0, 1.0)
        )
        return math.degrees(max(abs(roll), abs(pitch)))

    def _forward_speed(self) -> float:
        return float(np.dot(self.state[3:5], self.reference_forward))

    def _cross_track(self) -> float:
        return float(
            np.dot(self.state[0:2] - self.reference[0:2], self.reference_lateral)
        )

    def _l1_speed_reference(self, elapsed: float) -> float:
        start_delay = 1.0
        accelerate = self.l1_target_speed / self.l1_acceleration
        brake_rate = 0.5
        brake = self.l1_target_speed / brake_rate
        time = max(0.0, float(elapsed) - start_delay)
        if time < accelerate:
            return self.l1_acceleration * time
        time -= accelerate
        if time < self.l1_hold_seconds:
            return self.l1_target_speed
        time -= self.l1_hold_seconds
        if time < brake:
            return self.l1_target_speed - brake_rate * time
        return 0.0

    def _l1_total_seconds(self) -> float:
        return (
            1.0
            + self.l1_target_speed / self.l1_acceleration
            + self.l1_hold_seconds
            + self.l1_target_speed / 0.5
            + 3.0
        )

    def _l1_references(self, elapsed: float):
        x_ref = np.zeros(
            (self.controller.N + 1, self.controller.model.state_size)
        )
        u_ref = np.zeros(
            (self.controller.N, self.controller.model.control_size)
        )
        parameters = np.zeros(
            (self.controller.N + 1, self.controller.model.parameter_size)
        )
        along = float(np.dot(self.state[0:2], self.reference_forward))
        cross_origin = float(np.dot(self.reference[0:2], self.reference_lateral))
        level_quaternion = self.reference[6:10]
        hover = self.controller.model.plant.hover_command
        for stage in range(self.controller.N + 1):
            stage_time = elapsed + stage * self.controller.dt
            speed = self._l1_speed_reference(stage_time)
            if stage:
                along += speed * self.controller.dt
            position_xy = (
                along * self.reference_forward
                + cross_origin * self.reference_lateral
            )
            x_ref[stage, 0:3] = [
                position_xy[0], position_xy[1], self.reference[2]
            ]
            x_ref[stage, 3:5] = speed * self.reference_forward
            x_ref[stage, 6:10] = level_quaternion
            if stage < self.controller.N:
                fraction = speed / self.l1_target_speed
                allocation = 1.0 - (1.0 - self.l1_min_lambda) * fraction
                u_ref[stage] = [
                    np.clip(hover / allocation, 0.0, 0.68),
                    self.l1_pusher_max * fraction,
                    0.0,
                    0.0,
                    0.0,
                    allocation,
                ]
        return x_ref, u_ref, parameters

    def _safety_reason(self) -> str | None:
        if self.state is None or self._state_age() > self.max_state_age:
            return "odometry_stale"
        if (
            self.status_message is None
            or self.status_message.arming_state
            != VehicleStatus.ARMING_STATE_ARMED
        ):
            return "vehicle_not_armed"
        if self.reference is None:
            return "reference_missing"
        l1 = self.test_mode == "allocation_l1"
        if abs(self.state[2] - self.reference[2]) > (0.75 if l1 else 0.50):
            return "altitude_error"
        if abs(self.state[5]) > (0.65 if l1 else 0.50):
            return "vertical_speed_limit"
        if (
            not l1
            and np.linalg.norm(self.state[0:2] - self.reference[0:2]) > 0.75
        ):
            return "horizontal_position_error"
        if not l1 and np.linalg.norm(self.state[3:5]) > 0.75:
            return "horizontal_speed_limit"
        if l1 and self._forward_speed() > self.l1_target_speed + 0.8:
            return "l1_forward_speed_limit"
        if l1 and abs(self._cross_track()) > 1.5:
            return "l1_cross_track_limit"
        if self._tilt_degrees() > (12.0 if l1 else 10.0):
            return "tilt_limit"
        if l1 and self.vtol_status is not None and (
            self.vtol_status.vehicle_vtol_state
            != VtolVehicleStatus.VEHICLE_VTOL_STATE_MC
        ):
            return "l1_left_mc_mode"
        if self.solver_failures > 0 or self.last_solver_status != 0:
            return "solver_failure"
        return None

    def _abort(self, reason: str) -> None:
        was_active = self.output_requested or self._offboard_active()
        if self.offboard_start_ns:
            self.last_offboard_duration = (
                self.get_clock().now().nanoseconds - self.offboard_start_ns
            ) * 1.0e-9
        self.output_requested = False
        self.abort_reason = reason
        if was_active:
            self._request_nav_state(False)
            self.get_logger().info(
                f"Robust hover stopped: {reason}; Position mode requested"
            )

    def _update(self) -> None:
        if (
            self.state is None
            or self.reference is None
            or self._state_age() > self.max_state_age
        ):
            if self.output_requested or self._offboard_active():
                self._abort("odometry_or_reference_stale")
            return
        elapsed = 0.0
        if self.offboard_start_ns:
            elapsed = (
                self.get_clock().now().nanoseconds - self.offboard_start_ns
            ) * 1.0e-9
        if self.test_mode == "allocation_l1":
            x_ref, u_ref, parameters = self._l1_references(elapsed)
        else:
            x_ref = np.tile(self.reference, (self.controller.N + 1, 1))
            u_ref = np.tile(
                self.controller.model.hover_control(), (self.controller.N, 1)
            )
            parameters = np.zeros(
                (self.controller.N + 1, self.controller.model.parameter_size)
            )
        try:
            solution = self.controller.solve(
                self.state, x_ref, u_ref, parameters
            )
        except Exception as error:
            self.last_solver_status = -2
            self.solver_failures += 1
            self.get_logger().error(f"Robust NMPC solve exception: {error}")
            if self.output_requested or self._offboard_active():
                self._abort("solver_exception")
            return
        self.last_solver_status = solution.status
        self.last_solve_time_ms = 1000.0 * solution.solve_time
        self.solve_count += 1
        if self.solve_count > self.warmup_solve_count:
            self.solve_times_ms.append(self.last_solve_time_ms)
        if solution.status != 0:
            self.solver_failures += 1
            if self.output_requested or self._offboard_active():
                self._abort("solver_failure")
            return
        self.last_control = solution.control.copy()

        now_ns = self.get_clock().now().nanoseconds
        dt = 0.05 if self.last_update_ns == 0 else np.clip(
            (now_ns - self.last_update_ns) * 1.0e-9, 0.0, 0.10
        )
        self.last_update_ns = now_ns
        derivative = np.asarray(
            self.model_function(
                self.state,
                self.last_control,
                np.zeros(self.controller.model.parameter_size),
            ),
            dtype=float,
        ).reshape(-1)
        self.surface_state += dt * derivative[13:16]

        message = Float64MultiArray()
        message.data = [
            *self.last_control.tolist(),
            float(solution.status),
            self.last_solve_time_ms,
        ]
        self.proposed_control_publisher.publish(message)
        self._update_output(float(dt))

    def _update_output(self, dt: float) -> None:
        if not self.output_requested:
            return
        offboard = self._offboard_active()
        if self.ever_offboard and not offboard:
            self._abort("px4_left_offboard")
            return
        reason = self._safety_reason()
        if reason is not None:
            self._abort(reason)
            return

        hover_control = self.controller.model.hover_control()
        if not offboard:
            self.published_control = hover_control.copy()
            self._publish_setpoint(self.published_control)
            self.prestream_count += 1
            if (
                20 <= self.prestream_count <= 50
                and self.prestream_count % 10 == 0
            ):
                self._request_nav_state(True)
            elif self.prestream_count > 70:
                self._abort("px4_did_not_enter_offboard")
            return

        self.ever_offboard = True
        now_ns = self.get_clock().now().nanoseconds
        if self.offboard_start_ns == 0:
            self.offboard_start_ns = now_ns
        elapsed = (now_ns - self.offboard_start_ns) * 1.0e-9
        if (
            self.test_mode == "allocation_l1"
            and elapsed >= self._l1_total_seconds()
        ):
            if self.max_forward_speed < 4.0:
                self._abort("l1_insufficient_forward_speed")
            elif self.max_pusher < 0.05:
                self._abort("l1_pusher_not_exercised")
            elif self.min_applied_lambda > 0.90:
                self._abort("l1_allocation_not_exercised")
            else:
                self._abort("allocation_l1_test_timeout")
            return
        if (
            self.test_mode != "allocation_l1"
            and elapsed >= self.hover_test_seconds
        ):
            self._abort("robust_hover_test_timeout")
            return

        if self.test_mode == "allocation_l1":
            if elapsed > 1.0 and (
                self.allocation_status is None
                or not self.allocation_status.active
                or not self.allocation_status.setpoint_valid
            ):
                self._abort("allocation_channel_inactive")
                return
            requested = self.last_control.copy()
            requested[5] = np.clip(requested[5], self.l1_min_lambda, 1.0)
            altitude_error = self.state[2] - self.reference[2]
            base_lift = vertical_hover_lift(
                self.controller.model.plant,
                altitude_error,
                self.state[5],
            )
            requested[0] = np.clip(base_lift / requested[5], 0.48, 0.68)
            requested[1] = np.clip(requested[1], 0.0, self.l1_pusher_max)
            requested[2:5] = np.clip(
                np.array([0.35, 0.50, 0.35]) * requested[2:5],
                [-0.10, -0.12, -0.08],
                [0.10, 0.12, 0.08],
            )
            slew = np.array([0.10, 0.05, 0.20, 0.20, 0.15, 0.05])
            limited = self.published_control + np.clip(
                requested - self.published_control,
                -slew * dt,
                slew * dt,
            )
            reference_speed = self._l1_speed_reference(elapsed)
            if self._forward_speed() > reference_speed + 0.40:
                limited[1] = max(
                    0.0, self.published_control[1] - 0.15 * dt
                )
            self.published_control = limited
            if elapsed < 0.50:
                self.published_control = hover_control.copy()
            self.max_forward_speed = max(
                self.max_forward_speed, self._forward_speed()
            )
            self.max_cross_track = max(
                self.max_cross_track, abs(self._cross_track())
            )
            self.max_altitude_error = max(
                self.max_altitude_error,
                abs(self.state[2] - self.reference[2]),
            )
            self.max_pusher = max(
                self.max_pusher, float(self.published_control[1])
            )
            if self.allocation_status is not None:
                self.min_applied_lambda = min(
                    self.min_applied_lambda,
                    float(self.allocation_status.applied_weight),
                )
            self._publish_setpoint(self.published_control)
            return

        requested = self.last_control.copy()
        altitude_error = self.state[2] - self.reference[2]
        requested[0] = vertical_hover_lift(
            self.controller.model.plant,
            altitude_error,
            self.state[5],
        )
        requested[1] = 0.0
        requested[5] = 1.0
        limited = limit_mc_command(
            self.published_control[0:5], requested[0:5], dt
        )
        limited[2:5] = np.clip(
            limited[2:5], [-0.08, -0.08, -0.05], [0.08, 0.08, 0.05]
        )
        self.published_control = np.r_[limited, 1.0]
        if elapsed < 0.50:
            self.published_control = hover_control.copy()
        self._publish_setpoint(self.published_control)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StandardVtolRobustShadow()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
