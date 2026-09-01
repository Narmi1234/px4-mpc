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
        self.declare_parameter("active_state_stale_abort_seconds", 0.45)
        self.declare_parameter("allow_hover_output", False)
        self.declare_parameter("allow_l1_output", False)
        self.declare_parameter("allow_l2_output", False)
        self.declare_parameter("hover_test_seconds", 5.0)
        self.declare_parameter("l1_target_speed", 5.0)
        self.declare_parameter("l1_acceleration", 0.4)
        self.declare_parameter("l1_hold_seconds", 3.0)
        self.declare_parameter("l1_min_lambda", 0.8)
        self.declare_parameter("l1_pusher_max", 0.25)
        self.declare_parameter("l2_target_speed", 9.0)
        self.declare_parameter("l2_acceleration", 0.4)
        self.declare_parameter("l2_hold_seconds", 3.0)
        self.declare_parameter("l2_min_lambda", 0.5)
        self.declare_parameter("l2_pusher_max", 0.35)
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
        self.active_state_stale_abort = float(
            self.get_parameter("active_state_stale_abort_seconds").value
        )
        if not (
            self.max_state_age < self.active_state_stale_abort <= 0.75
        ):
            raise ValueError(
                "active_state_stale_abort_seconds must be greater than "
                "max_state_age_seconds and no greater than 0.75 seconds"
            )
        self.allow_hover_output = bool(
            self.get_parameter("allow_hover_output").value
        )
        self.allow_l1_output = bool(
            self.get_parameter("allow_l1_output").value
        )
        self.allow_l2_output = bool(
            self.get_parameter("allow_l2_output").value
        )
        self.allow_output = (
            self.allow_hover_output
            or self.allow_l1_output
            or self.allow_l2_output
        )
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
        self.l2_target_speed = float(
            self.get_parameter("l2_target_speed").value
        )
        self.l2_acceleration = float(
            self.get_parameter("l2_acceleration").value
        )
        self.l2_hold_seconds = float(
            self.get_parameter("l2_hold_seconds").value
        )
        self.l2_min_lambda = float(
            self.get_parameter("l2_min_lambda").value
        )
        self.l2_pusher_max = float(
            self.get_parameter("l2_pusher_max").value
        )
        if not (
            8.0 <= self.l2_target_speed <= 10.0
            and 0.2 <= self.l2_acceleration <= 0.4
            and 2.0 <= self.l2_hold_seconds <= 5.0
            and 0.5 <= self.l2_min_lambda <= 0.7
            and 0.25 <= self.l2_pusher_max <= 0.35
        ):
            raise ValueError("L2 parameters exceed the guarded envelope")
        self.state: np.ndarray | None = None
        self.surface_state = np.zeros(3)
        self.reference: np.ndarray | None = None
        self.status_message: VehicleStatus | None = None
        self.vtol_status: VtolVehicleStatus | None = None
        self.airspeed: AirspeedValidated | None = None
        self.airspeed_received_ns = 0
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
        self.max_airspeed = 0.0
        self.max_state_gap = 0.0

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
            "/standard_vtol_robust_shadow/enable_allocation_l2",
            self._enable_allocation_l2,
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
        if self.allow_l2_output:
            self.get_logger().info(
                "Robust Standard VTOL NMPC started in guarded L2 allocation "
                "mode (9 m/s, lambda >= 0.5, MC only)"
            )
        elif self.allow_l1_output:
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
        self.airspeed_received_ns = self.get_clock().now().nanoseconds

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

    def _airspeed_age(self) -> float:
        if self.airspeed_received_ns == 0:
            return math.inf
        return (
            self.get_clock().now().nanoseconds - self.airspeed_received_ns
        ) * 1.0e-9

    def _calibrated_airspeed(self) -> float:
        if self.airspeed is None:
            return math.nan
        return float(self.airspeed.calibrated_airspeed_m_s)

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
        self.max_airspeed = 0.0
        self.max_state_gap = 0.0
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
        calibrated_airspeed = self._calibrated_airspeed()
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
            f"airspeed={self.max_airspeed:.3f},"
            f"pusher={self.max_pusher:.3f},"
            f"min_lambda={self.min_applied_lambda:.3f},"
            f"state_gap={self.max_state_gap:.3f}],"
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
        self.max_airspeed = 0.0
        self.max_state_gap = 0.0
        response.success = True
        response.message = (
            "guarded L1 prestream started; MC-only 0->5->0 m/s, "
            "lambda 1->0.8->1, then Position"
        )
        return response

    def _enable_allocation_l2(self, _request, response):
        if not self.allow_l2_output:
            response.success = False
            response.message = "launch guarded L2 allocation mode first"
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
        if (
            self.airspeed is None
            or self._airspeed_age() > 0.5
            or not np.isfinite(self._calibrated_airspeed())
            or self.airspeed.airspeed_source
            == AirspeedValidated.SOURCE_DISABLED
        ):
            response.success = False
            response.message = "airspeed_stream_unavailable_for_l2"
            return response
        if self._offboard_active():
            response.success = False
            response.message = "vehicle_already_offboard"
            return response
        if abs(self.state[5]) > 0.15 or np.linalg.norm(self.state[3:5]) > 0.5:
            response.success = False
            response.message = "vehicle_not_settled_for_l2"
            return response
        self.output_requested = True
        self.test_mode = "allocation_l2"
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
        self.max_airspeed = 0.0
        self.max_state_gap = 0.0
        response.success = True
        response.message = (
            "guarded L2 prestream started; MC-only 0->9->0 m/s, "
            "lambda 1->0.5->1, then Position"
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

    def _allocation_configuration(self):
        if getattr(self, "test_mode", "allocation_l1") == "allocation_l2":
            return (
                self.l2_target_speed,
                self.l2_acceleration,
                self.l2_hold_seconds,
                self.l2_min_lambda,
                self.l2_pusher_max,
            )
        return (
            self.l1_target_speed,
            self.l1_acceleration,
            self.l1_hold_seconds,
            self.l1_min_lambda,
            self.l1_pusher_max,
        )

    def _l1_speed_reference(self, elapsed: float) -> float:
        target_speed, acceleration, hold_seconds, _, _ = (
            self._allocation_configuration()
        )
        start_delay = 1.0
        accelerate = target_speed / acceleration
        brake_rate = 0.5
        brake = target_speed / brake_rate
        time = max(0.0, float(elapsed) - start_delay)
        if time < accelerate:
            return acceleration * time
        time -= accelerate
        if time < hold_seconds:
            return target_speed
        time -= hold_seconds
        if time < brake:
            return target_speed - brake_rate * time
        return 0.0

    def _l1_total_seconds(self) -> float:
        target_speed, acceleration, hold_seconds, _, _ = (
            self._allocation_configuration()
        )
        return (
            1.0
            + target_speed / acceleration
            + hold_seconds
            + target_speed / 0.5
            + (4.0 if getattr(self, "test_mode", "") == "allocation_l2" else 3.0)
        )

    @staticmethod
    def _yaw_pitch_quaternion(yaw: float, pitch: float) -> np.ndarray:
        cy, sy = math.cos(0.5 * yaw), math.sin(0.5 * yaw)
        cp, sp = math.cos(0.5 * pitch), math.sin(0.5 * pitch)
        return np.array([cy * cp, -sy * sp, cy * sp, sy * cp])

    @staticmethod
    def _l2_pitch_reference(speed: float, target_speed: float) -> float:
        nodes = target_speed / 15.0 * np.array(
            [0.0, 4.0, 7.0, 9.0, 12.0, 15.0]
        )
        degrees = np.array([0.0, -3.0, -6.0, -8.0, -4.0, -1.36])
        return float(np.deg2rad(np.interp(speed, nodes, degrees)))

    def _l1_references(self, elapsed: float):
        target_speed, _, _, minimum_lambda, pusher_max = (
            self._allocation_configuration()
        )
        l2 = self.test_mode == "allocation_l2"
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
        yaw = self._yaw(self.reference[6:10])
        hover = self.controller.model.plant.hover_command
        corridor_speed = np.arange(10.0)
        corridor_lift = np.array(
            [
                0.520119535,
                0.515250356,
                0.493080651,
                0.459032148,
                0.435379671,
                0.435132797,
                0.397462199,
                0.349616361,
                0.280231964,
                0.186055563,
            ]
        )
        corridor_pusher = np.array(
            [
                0.0,
                0.246113549,
                0.338041363,
                0.354536687,
                0.323434863,
                0.290171366,
                0.290014802,
                0.289258326,
                0.288015620,
                0.282533714,
            ]
        )
        corridor_elevator = np.deg2rad(
            [0.0, 0.0, 0.0, 0.0, 0.0, 43.3170, 43.4304, 43.2066, 44.2501, 43.8752]
        )
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
            pitch = self._l2_pitch_reference(speed, target_speed) if l2 else 0.0
            x_ref[stage, 6:10] = self._yaw_pitch_quaternion(yaw, pitch)
            if stage < self.controller.N:
                fraction = speed / target_speed
                allocation = 1.0 - (1.0 - minimum_lambda) * fraction
                if l2:
                    lift = float(np.interp(speed, corridor_speed, corridor_lift))
                    collective = np.clip(lift / allocation, 0.0, 0.68)
                    pusher = min(
                        pusher_max,
                        float(np.interp(speed, corridor_speed, corridor_pusher)),
                    )
                    elevator = float(
                        np.interp(speed, corridor_speed, corridor_elevator)
                    )
                    x_ref[stage, 15] = (1.0 - allocation) * elevator
                else:
                    collective = np.clip(hover / allocation, 0.0, 0.68)
                    pusher = pusher_max * fraction
                u_ref[stage] = [
                    collective,
                    pusher,
                    0.0,
                    0.0,
                    0.0,
                    allocation,
                ]
        return x_ref, u_ref, parameters

    def _allocation_control_bounds(self, u_ref: np.ndarray):
        """Keep allocation inside the scheduled NMPC transition corridor."""
        _, _, _, minimum_lambda, _ = self._allocation_configuration()
        lower = np.tile(
            np.array([0.0, 0.0, -0.45, -0.45, -0.30, minimum_lambda]),
            (self.controller.N, 1),
        )
        upper = np.tile(
            np.array([0.70, 0.70, 0.45, 0.45, 0.30, 1.0]),
            (self.controller.N, 1),
        )
        # Lambda remains an NMPC decision, but it may not evade the requested
        # authority-transfer experiment by staying arbitrarily close to one.
        # The 0.05 band leaves optimization freedom around the slow schedule.
        upper[:, 5] = np.clip(u_ref[:, 5] + 0.05, minimum_lambda, 1.0)
        return lower, upper

    def _safety_reason(self) -> str | None:
        # State freshness is handled before solving in _update().  Rechecking
        # it here after a 15-30 ms solve caused false aborts whenever a sample
        # was near the nominal age limit at the start of the timer callback.
        if self.state is None:
            return "odometry_missing"
        if (
            self.status_message is None
            or self.status_message.arming_state
            != VehicleStatus.ARMING_STATE_ARMED
        ):
            return "vehicle_not_armed"
        if self.reference is None:
            return "reference_missing"
        allocation_gate = self.test_mode in (
            "allocation_l1", "allocation_l2"
        )
        l2 = self.test_mode == "allocation_l2"
        altitude_limit = 1.0 if l2 else (0.75 if allocation_gate else 0.50)
        vertical_speed_limit = 0.8 if l2 else (0.65 if allocation_gate else 0.50)
        if abs(self.state[2] - self.reference[2]) > altitude_limit:
            return "altitude_error"
        if abs(self.state[5]) > vertical_speed_limit:
            return "vertical_speed_limit"
        if (
            not allocation_gate
            and np.linalg.norm(self.state[0:2] - self.reference[0:2]) > 0.75
        ):
            return "horizontal_position_error"
        if not allocation_gate and np.linalg.norm(self.state[3:5]) > 0.75:
            return "horizontal_speed_limit"
        target_speed, _, _, _, _ = self._allocation_configuration()
        if allocation_gate and self._forward_speed() > target_speed + (1.0 if l2 else 0.8):
            return f"{'l2' if l2 else 'l1'}_forward_speed_limit"
        if allocation_gate and abs(self._cross_track()) > (2.0 if l2 else 1.5):
            return f"{'l2' if l2 else 'l1'}_cross_track_limit"
        if self._tilt_degrees() > (15.0 if l2 else (12.0 if allocation_gate else 10.0)):
            return "tilt_limit"
        if allocation_gate and self.vtol_status is not None and (
            self.vtol_status.vehicle_vtol_state
            != VtolVehicleStatus.VEHICLE_VTOL_STATE_MC
        ):
            return f"{'l2' if l2 else 'l1'}_left_mc_mode"
        if l2 and self._airspeed_age() > 0.5:
            return "l2_airspeed_stale"
        if l2 and self._forward_speed() > 3.0 and self._calibrated_airspeed() < 0.0:
            return "l2_airspeed_invalid"
        if self.solver_failures > 0 or self.last_solver_status != 0:
            return "solver_failure"
        return None

    def _state_freshness_action(self, state_age: float) -> str:
        """Select solve, bounded hold, or abort without using stale state."""
        if state_age <= self.max_state_age:
            return "solve"
        if state_age < self.active_state_stale_abort:
            return "hold"
        return "abort"

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
        if self.state is None:
            if self.output_requested or self._offboard_active():
                self._abort("odometry_missing")
            return
        if self.reference is None:
            if self.output_requested or self._offboard_active():
                self._abort("reference_missing")
            return
        state_age = self._state_age()
        self.max_state_gap = max(self.max_state_gap, state_age)
        freshness_action = self._state_freshness_action(state_age)
        if freshness_action != "solve":
            if self.output_requested or self._offboard_active():
                if self.ever_offboard and not self._offboard_active():
                    self._abort("px4_left_offboard")
                elif freshness_action == "abort":
                    self._abort("odometry_stale_continuous")
                else:
                    # Preserve the Offboard heartbeat during one short ROS DDS
                    # delivery gap.  The command is already slew- and
                    # envelope-limited; no new NMPC solve uses stale state.
                    self._publish_setpoint(self.published_control)
            return
        elapsed = 0.0
        if self.offboard_start_ns:
            elapsed = (
                self.get_clock().now().nanoseconds - self.offboard_start_ns
            ) * 1.0e-9
        if self.test_mode in ("allocation_l1", "allocation_l2"):
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
            lower_bounds = None
            upper_bounds = None
            if self.test_mode in ("allocation_l1", "allocation_l2"):
                lower_bounds, upper_bounds = self._allocation_control_bounds(
                    u_ref
                )
            solution = self.controller.solve(
                self.state,
                x_ref,
                u_ref,
                parameters,
                control_lower_bounds=lower_bounds,
                control_upper_bounds=upper_bounds,
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
        allocation_gate = self.test_mode in (
            "allocation_l1", "allocation_l2"
        )
        l2 = self.test_mode == "allocation_l2"
        if allocation_gate and elapsed >= self._l1_total_seconds():
            if self.max_forward_speed < (8.0 if l2 else 4.0):
                self._abort(f"{'l2' if l2 else 'l1'}_insufficient_forward_speed")
            elif l2 and self.max_airspeed < 8.0:
                self._abort("l2_insufficient_airspeed")
            elif self.max_pusher < (0.10 if l2 else 0.05):
                self._abort(f"{'l2' if l2 else 'l1'}_pusher_not_exercised")
            elif self.min_applied_lambda > (0.60 if l2 else 0.90):
                self._abort(f"{'l2' if l2 else 'l1'}_allocation_not_exercised")
            elif l2 and abs(self._forward_speed()) > 0.5:
                self._abort("l2_did_not_settle")
            else:
                self._abort(
                    "allocation_l2_test_timeout"
                    if l2
                    else "allocation_l1_test_timeout"
                )
            return
        if (
            not allocation_gate
            and elapsed >= self.hover_test_seconds
        ):
            self._abort("robust_hover_test_timeout")
            return

        if allocation_gate:
            if elapsed > 1.0 and (
                self.allocation_status is None
                or not self.allocation_status.active
                or not self.allocation_status.setpoint_valid
            ):
                self._abort("allocation_channel_inactive")
                return
            requested = self.last_control.copy()
            _, _, _, minimum_lambda, pusher_max = (
                self._allocation_configuration()
            )
            requested[5] = np.clip(requested[5], minimum_lambda, 1.0)
            altitude_error = self.state[2] - self.reference[2]
            base_lift = vertical_hover_lift(
                self.controller.model.plant,
                altitude_error,
                self.state[5],
            )
            if l2:
                correction = (
                    base_lift - self.controller.model.plant.hover_command
                ) / requested[5]
                requested[0] = np.clip(
                    requested[0] + correction, 0.30, 0.70
                )
                requested[2:5] = np.clip(
                    np.array([0.40, 1.00, 0.40]) * requested[2:5],
                    [-0.12, -0.18, -0.10],
                    [0.12, 0.18, 0.10],
                )
            else:
                requested[0] = np.clip(
                    base_lift / requested[5], 0.48, 0.68
                )
                requested[2:5] = np.clip(
                    np.array([0.35, 0.50, 0.35]) * requested[2:5],
                    [-0.10, -0.12, -0.08],
                    [0.10, 0.12, 0.08],
                )
            requested[1] = np.clip(requested[1], 0.0, pusher_max)
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
            calibrated_airspeed = self._calibrated_airspeed()
            if np.isfinite(calibrated_airspeed):
                self.max_airspeed = max(
                    self.max_airspeed, calibrated_airspeed
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
