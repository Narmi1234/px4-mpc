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
    OffboardControlMode,
    VehicleCommand,
    VehicleOdometry,
    VehicleRatesSetpoint,
    VehicleStatus,
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
        self.declare_parameter("horizon_steps", 25)
        self.declare_parameter("horizon_seconds", 2.0)
        self.declare_parameter("max_state_age_seconds", 0.20)
        self.declare_parameter("allow_hover_output", False)
        self.declare_parameter("hover_test_seconds", 5.0)
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
        self.hover_test_seconds = float(
            self.get_parameter("hover_test_seconds").value
        )
        if self.hover_test_seconds <= 0.0 or self.hover_test_seconds > 5.0:
            raise ValueError("hover_test_seconds must be in (0, 5] seconds")
        self.state: np.ndarray | None = None
        self.surface_state = np.zeros(3)
        self.reference: np.ndarray | None = None
        self.status_message: VehicleStatus | None = None
        self.state_received_ns = 0
        self.last_update_ns = 0
        self.last_control = self.controller.model.hover_control()
        self.last_solver_status = -1
        self.last_solve_time_ms = math.nan
        self.solver_failures = 0
        self.solve_count = 0
        self.warmup_solve_count = 40
        self.solve_times_ms: deque[float] = deque(maxlen=2000)
        self.model_function = self.controller.model.function()
        self.output_requested = False
        self.ever_offboard = False
        self.prestream_count = 0
        self.offboard_start_ns = 0
        self.last_offboard_duration = 0.0
        self.abort_reason = "none"
        self.published_control = self.controller.model.hover_control()

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
            "/standard_vtol_robust_shadow/disable",
            self._disable_output,
        )
        self.offboard_publisher = None
        self.rates_publisher = None
        self.command_publisher = None
        if self.allow_hover_output:
            self.offboard_publisher = self.create_publisher(
                OffboardControlMode, "/fmu/in/offboard_control_mode", qos
            )
            self.rates_publisher = self.create_publisher(
                VehicleRatesSetpoint, "/fmu/in/vehicle_rates_setpoint", qos
            )
            self.command_publisher = self.create_publisher(
                VehicleCommand, "/fmu/in/vehicle_command", qos
            )
        self.create_timer(0.05, self._update)
        if self.allow_hover_output:
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
        self.reference[10:16] = 0.0
        self.surface_state[:] = 0.0
        self.last_control = self.controller.model.hover_control()
        self.last_solver_status = -1
        self.solver_failures = 0
        self.solve_count = 0
        self.solve_times_ms.clear()
        self.abort_reason = "none"
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
        response.success = (
            self.reference is not None
            and state_age <= self.max_state_age
            and self.last_solver_status == 0
            and self.solver_failures == 0
            and p99 <= 40.0
        )
        warmup_remaining = max(0, self.warmup_solve_count - self.solve_count)
        response.message = (
            f"read_only={not self.allow_hover_output},"
            f"publishes_fmu={self.allow_hover_output},"
            f"output_requested={self.output_requested},offboard={offboard},"
            f"reference_captured={self.reference is not None},"
            f"armed={armed},nav_state={nav_state},state_age={state_age:.3f}s,"
            f"solver_status={self.last_solver_status},"
            f"solver_failures={self.solver_failures},"
            f"warmup_remaining={warmup_remaining},"
            f"solve_time={self.last_solve_time_ms:.2f}ms,"
            f"solve_time_p99={p99:.2f}ms,"
            f"last_offboard_duration={self.last_offboard_duration:.2f}s,"
            f"abort_reason={self.abort_reason},"
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
        self.ever_offboard = False
        self.prestream_count = 0
        self.offboard_start_ns = 0
        self.last_offboard_duration = 0.0
        self.abort_reason = "none"
        self.published_control = self.controller.model.hover_control()
        response.success = True
        response.message = (
            "guarded robust hover prestream started; PX4 Offboard follows "
            "after 1 s and returns to Position after 5 s"
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
        if self.offboard_publisher is None or self.rates_publisher is None:
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
        message.thrust_body = [0.0, 0.0, float(-control[0])]
        self.rates_publisher.publish(message)

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
        if abs(self.state[2] - self.reference[2]) > 0.50:
            return "altitude_error"
        if abs(self.state[5]) > 0.50:
            return "vertical_speed_limit"
        if np.linalg.norm(self.state[0:2] - self.reference[0:2]) > 0.75:
            return "horizontal_position_error"
        if np.linalg.norm(self.state[3:5]) > 0.75:
            return "horizontal_speed_limit"
        if self._tilt_degrees() > 10.0:
            return "tilt_limit"
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
        if elapsed >= self.hover_test_seconds:
            self._abort("robust_hover_test_timeout")
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
