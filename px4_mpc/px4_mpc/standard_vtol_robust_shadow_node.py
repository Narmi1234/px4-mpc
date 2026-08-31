"""Read-only ROS 2 shadow node for the 16-state Standard VTOL NMPC."""

from __future__ import annotations

from collections import deque
import math
from pathlib import Path

import numpy as np
from px4_mpc.controllers.standard_vtol_robust_nmpc import (
    StandardVtolRobustNmpc,
)
from px4_mpc.models.frames import (
    frd_to_flu,
    ned_to_enu,
    px4_quaternion_to_gazebo,
)
from px4_msgs.msg import VehicleOdometry, VehicleStatus
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
    """Run the robust OCP at 20 Hz without publishing anything to PX4."""

    def __init__(self) -> None:
        super().__init__("standard_vtol_robust_shadow")
        self.declare_parameter("horizon_steps", 25)
        self.declare_parameter("horizon_seconds", 2.0)
        self.declare_parameter("max_state_age_seconds", 0.20)
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
        self.create_timer(0.05, self._update)
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
        response.success = True
        response.message = (
            "hover reference captured; read-only shadow solve enabled; "
            "no Offboard command will be sent"
        )
        return response

    def _clear_reference(self, _request, response):
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
        response.success = (
            self.reference is not None
            and state_age <= self.max_state_age
            and self.last_solver_status == 0
            and self.solver_failures == 0
            and p99 <= 40.0
        )
        warmup_remaining = max(0, self.warmup_solve_count - self.solve_count)
        response.message = (
            "read_only=True,publishes_fmu=False,"
            f"reference_captured={self.reference is not None},"
            f"armed={armed},nav_state={nav_state},state_age={state_age:.3f}s,"
            f"solver_status={self.last_solver_status},"
            f"solver_failures={self.solver_failures},"
            f"warmup_remaining={warmup_remaining},"
            f"solve_time={self.last_solve_time_ms:.2f}ms,"
            f"solve_time_p99={p99:.2f}ms,"
            f"control={np.round(self.last_control, 4).tolist()}"
        )
        return response

    def _update(self) -> None:
        if (
            self.state is None
            or self.reference is None
            or self._state_age() > self.max_state_age
        ):
            return
        x_ref = np.tile(self.reference, (self.controller.N + 1, 1))
        u_ref = np.tile(
            self.controller.model.hover_control(), (self.controller.N, 1)
        )
        parameters = np.zeros(
            (self.controller.N + 1, self.controller.model.parameter_size)
        )
        solution = self.controller.solve(self.state, x_ref, u_ref, parameters)
        self.last_solver_status = solution.status
        self.last_solve_time_ms = 1000.0 * solution.solve_time
        self.solve_count += 1
        if self.solve_count > self.warmup_solve_count:
            self.solve_times_ms.append(self.last_solve_time_ms)
        if solution.status != 0:
            self.solver_failures += 1
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
