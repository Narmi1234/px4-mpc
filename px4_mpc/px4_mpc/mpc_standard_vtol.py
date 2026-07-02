#!/usr/bin/env python3
"""PX4 offboard NMPC node for the Gazebo standard VTOL model."""

from __future__ import annotations

import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from visualization_msgs.msg import Marker

from mpc_msgs.srv import SetPose
from px4_msgs.msg import (
    OffboardControlMode,
    VehicleAngularVelocity,
    VehicleAttitude,
    VehicleLocalPosition,
    VehicleOdometry,
    VehicleRatesSetpoint,
    VehicleStatus,
)
from standard_vtol_nmpc.model import StandardVtolModel
from standard_vtol_nmpc.mpc_casadi import MpcConfig, StandardVtolNMPC


def _timestamp_us(node: Node) -> int:
    return int(node.get_clock().now().nanoseconds / 1000)


def _px4_topic(namespace: str, topic: str) -> str:
    prefix = f"/{namespace.strip('/')}" if namespace else ""
    normalized_topic = topic if topic.startswith("/") else f"/{topic}"
    return f"{prefix}{normalized_topic}"


def _px4_qos(durability: QoSDurabilityPolicy) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=durability,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


def _px4_quat_to_model(q_px4) -> np.ndarray:
    q = np.asarray(q_px4, dtype=float)
    if not np.isfinite(q).all() or float(np.linalg.norm(q)) < 1e-6:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

    converted = np.array([q[0], q[1], -q[2], -q[3]], dtype=float)
    norm = float(np.linalg.norm(converted))
    if norm < 1e-6:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    converted = converted / norm
    if converted[0] < 0.0:
        converted = -converted
    return converted


def _pose_msg(
    frame_id: str,
    position: np.ndarray,
    attitude: np.ndarray,
) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(position[0])
    msg.pose.position.y = float(position[1])
    msg.pose.position.z = float(position[2])
    msg.pose.orientation.w = float(attitude[0])
    msg.pose.orientation.x = float(attitude[1])
    msg.pose.orientation.y = float(attitude[2])
    msg.pose.orientation.z = float(attitude[3])
    return msg


class StandardVtolMPC(Node):
    """Run the simplified standard VTOL NMPC against PX4 offboard topics.

    The NMPC model uses a NWU-like local world convention: x is PX4 local
    north, y is negative PX4 east, and z is positive altitude.
    """

    def __init__(self) -> None:
        super().__init__("standard_vtol_mpc")

        self.namespace = self.declare_parameter("namespace", "").value
        self.control_dt = float(
            self.declare_parameter("control_dt", 0.05).value
        )
        self.heartbeat_dt = float(
            self.declare_parameter("heartbeat_dt", 0.05).value
        )
        self.publish_before_offboard = bool(
            self.declare_parameter("publish_before_offboard", True).value
        )
        self.control_mode = str(
            self.declare_parameter("control_mode", "nmpc").value
        ).strip().lower()
        self.manual_lift = float(
            self.declare_parameter("manual_lift", 0.0).value
        )
        self.manual_pusher = float(
            self.declare_parameter("manual_pusher", 0.0).value
        )
        self.manual_roll_rate = float(
            self.declare_parameter("manual_roll_rate", 0.0).value
        )
        self.manual_pitch_rate = float(
            self.declare_parameter("manual_pitch_rate", 0.0).value
        )
        self.manual_yaw_rate = float(
            self.declare_parameter("manual_yaw_rate", 0.0).value
        )
        self.altitude_hold_hover_thrust = float(
            self.declare_parameter("altitude_hold_hover_thrust", 0.5195).value
        )
        self.altitude_hold_gain = float(
            self.declare_parameter("altitude_hold_gain", 0.02).value
        )
        self.altitude_hold_velocity_gain = float(
            self.declare_parameter("altitude_hold_velocity_gain", 0.08).value
        )
        self.altitude_hold_min_thrust = float(
            self.declare_parameter("altitude_hold_min_thrust", 0.45).value
        )
        self.altitude_hold_max_thrust = float(
            self.declare_parameter("altitude_hold_max_thrust", 0.60).value
        )
        self.reference_altitude = float(
            self.declare_parameter("reference_altitude", 20.0).value
        )
        self.reference_forward_speed = float(
            self.declare_parameter("reference_forward_speed", 0.0).value
        )
        self.rate_setpoint_limit = float(
            self.declare_parameter("rate_setpoint_limit", 2.0).value
        )
        self.nmpc_max_body_rate = float(
            self.declare_parameter("nmpc_max_body_rate", 2.0).value
        )
        self.max_tilt_deg = float(
            self.declare_parameter("max_tilt_deg", 70.0).value
        )
        self.max_safe_speed = float(
            self.declare_parameter("max_safe_speed", 15.0).value
        )
        self.speed_safety_min_altitude = float(
            self.declare_parameter("speed_safety_min_altitude", 1.0).value
        )
        self.max_safe_tilt_deg = float(
            self.declare_parameter("max_safe_tilt_deg", 95.0).value
        )
        self.max_odometry_position_norm = float(
            self.declare_parameter("max_odometry_position_norm", 1000.0).value
        )
        self.fallback_hover_thrust = float(
            self.declare_parameter("fallback_hover_thrust", 0.56).value
        )
        self.fallback_altitude_gain = float(
            self.declare_parameter("fallback_altitude_gain", 0.03).value
        )
        self.fallback_vertical_velocity_gain = float(
            self.declare_parameter(
                "fallback_vertical_velocity_gain",
                0.12,
            ).value
        )
        self.fallback_attitude_gain = float(
            self.declare_parameter("fallback_attitude_gain", 1.8).value
        )
        self.fallback_min_thrust = float(
            self.declare_parameter("fallback_min_thrust", 0.15).value
        )
        self.fallback_max_thrust = float(
            self.declare_parameter("fallback_max_thrust", 0.75).value
        )
        self.reference_topic = self.declare_parameter(
            "reference_topic", "px4_mpc/standard_vtol/reference"
        ).value

        mpc_config = MpcConfig(
            dt=self.control_dt,
            horizon_steps=int(
                self.declare_parameter("horizon_steps", 8).value
            ),
            max_ipopt_iter=int(
                self.declare_parameter("max_ipopt_iter", 80).value
            ),
            max_tilt_deg=self.max_tilt_deg,
            max_body_rate=self.nmpc_max_body_rate,
        )

        self.model = StandardVtolModel()
        self.mpc = StandardVtolNMPC(self.model, mpc_config)
        self.previous_solution: dict[str, np.ndarray] | None = None
        self.odometry_origin: np.ndarray | None = None

        self.vehicle_attitude = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self.vehicle_local_position = np.zeros(3, dtype=float)
        self.vehicle_local_velocity = np.zeros(3, dtype=float)
        self.vehicle_angular_velocity = np.zeros(3, dtype=float)
        self.reference_state = np.zeros(self.model.nx, dtype=float)
        self.reference_state[2] = self.reference_altitude
        self.reference_state[3] = self.reference_forward_speed
        self.reference_state[6] = 1.0

        self.have_attitude = False
        self.have_local_position = False
        self.have_body_rates = False
        self.have_reference = False
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.last_solve_status = "not_started"

        pub_qos = _px4_qos(QoSDurabilityPolicy.TRANSIENT_LOCAL)
        sub_qos = _px4_qos(QoSDurabilityPolicy.VOLATILE)

        self.create_subscription(
            VehicleStatus,
            _px4_topic(self.namespace, "/fmu/out/vehicle_status"),
            self.vehicle_status_callback,
            sub_qos,
        )
        self.create_subscription(
            VehicleAttitude,
            _px4_topic(self.namespace, "/fmu/out/vehicle_attitude"),
            self.vehicle_attitude_callback,
            sub_qos,
        )
        self.create_subscription(
            VehicleAngularVelocity,
            _px4_topic(self.namespace, "/fmu/out/vehicle_angular_velocity"),
            self.vehicle_angular_velocity_callback,
            sub_qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            _px4_topic(self.namespace, "/fmu/out/vehicle_local_position"),
            self.vehicle_local_position_callback,
            sub_qos,
        )
        self.create_subscription(
            VehicleOdometry,
            _px4_topic(self.namespace, "/fmu/out/vehicle_odometry"),
            self.vehicle_odometry_callback,
            sub_qos,
        )
        self.create_subscription(
            Odometry,
            _px4_topic(self.namespace, self.reference_topic),
            self.reference_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            _px4_topic(self.namespace, "/px4_mpc/standard_vtol/setpoint_pose"),
            self.setpoint_pose_callback,
            10,
        )

        self.set_pose_srv = self.create_service(
            SetPose,
            _px4_topic(self.namespace, "/px4_mpc/standard_vtol/set_pose"),
            self.set_pose_callback,
        )

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            _px4_topic(self.namespace, "/fmu/in/offboard_control_mode"),
            pub_qos,
        )
        self.rates_setpoint_pub = self.create_publisher(
            VehicleRatesSetpoint,
            _px4_topic(self.namespace, "/fmu/in/vehicle_rates_setpoint"),
            pub_qos,
        )
        self.predicted_path_pub = self.create_publisher(
            Path,
            _px4_topic(
                self.namespace,
                "/px4_mpc/standard_vtol/predicted_path",
            ),
            10,
        )
        self.reference_marker_pub = self.create_publisher(
            Marker,
            _px4_topic(
                self.namespace,
                "/px4_mpc/standard_vtol/reference_marker",
            ),
            10,
        )

        self.create_timer(self.heartbeat_dt, self.publish_offboard_mode)
        self.create_timer(self.control_dt, self.control_loop)

        self.get_logger().info(
            "standard VTOL NMPC ready: "
            f"mode={self.control_mode}, "
            f"dt={self.control_dt:.3f}s, horizon={mpc_config.horizon_steps}, "
            f"reference="
            f"{_px4_topic(self.namespace, self.reference_topic)}"
        )
        if self.control_mode == "manual_rates":
            self.get_logger().warn(
                "manual rate/thrust test mode active: "
                f"lift={self.manual_lift:.3f}, "
                f"pusher={self.manual_pusher:.3f}, "
                f"rates=[{self.manual_roll_rate:.3f}, "
                f"{self.manual_pitch_rate:.3f}, "
                f"{self.manual_yaw_rate:.3f}] rad/s"
            )
        elif self.control_mode == "altitude_hold":
            self.get_logger().warn(
                "altitude hold test mode active: "
                f"target={self.reference_altitude:.2f} m, "
                f"hover={self.altitude_hold_hover_thrust:.4f}, "
                f"kp={self.altitude_hold_gain:.3f}, "
                f"kd={self.altitude_hold_velocity_gain:.3f}, "
                f"limits=[{self.altitude_hold_min_thrust:.3f}, "
                f"{self.altitude_hold_max_thrust:.3f}]"
            )

    def vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def vehicle_attitude_callback(self, msg: VehicleAttitude) -> None:
        self.vehicle_attitude = _px4_quat_to_model(msg.q)
        self.have_attitude = True

    def vehicle_angular_velocity_callback(
        self,
        msg: VehicleAngularVelocity,
    ) -> None:
        self.vehicle_angular_velocity[0] = msg.xyz[0]
        self.vehicle_angular_velocity[1] = -msg.xyz[1]
        self.vehicle_angular_velocity[2] = -msg.xyz[2]
        self.have_body_rates = True

    def vehicle_local_position_callback(
        self,
        msg: VehicleLocalPosition,
    ) -> None:
        if not (
            msg.xy_valid
            and msg.z_valid
            and msg.v_xy_valid
            and msg.v_z_valid
        ):
            return

        self.vehicle_local_position[0] = msg.x
        self.vehicle_local_position[1] = -msg.y
        self.vehicle_local_position[2] = -msg.z
        self.vehicle_local_velocity[0] = msg.vx
        self.vehicle_local_velocity[1] = -msg.vy
        self.vehicle_local_velocity[2] = -msg.vz
        self.have_local_position = True

        if not self.have_reference:
            self.reference_state[0] = self.vehicle_local_position[0]
            self.reference_state[1] = self.vehicle_local_position[1]

    def vehicle_odometry_callback(self, msg: VehicleOdometry) -> None:
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            return

        position = np.asarray(msg.position, dtype=float)
        if np.isfinite(position).all():
            if self.odometry_origin is None:
                self.odometry_origin = position.copy()
                self.get_logger().info(
                    "using current PX4 odometry sample as local origin"
                )
            local_position = position - self.odometry_origin
            if float(np.linalg.norm(local_position)) > self.max_odometry_position_norm:
                self.get_logger().warn(
                    "ignoring implausible relative PX4 odometry position: "
                    f"norm={float(np.linalg.norm(local_position)):.1f} m",
                    throttle_duration_sec=2.0,
                )
                return
            self.vehicle_local_position[0] = local_position[0]
            self.vehicle_local_position[1] = -local_position[1]
            self.vehicle_local_position[2] = -local_position[2]
            self.have_local_position = True
        else:
            self.get_logger().warn(
                "ignoring non-finite PX4 odometry position",
                throttle_duration_sec=2.0,
            )

        velocity = np.asarray(msg.velocity, dtype=float)
        if (
            msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_NED
            and np.isfinite(velocity).all()
        ):
            self.vehicle_local_velocity[0] = velocity[0]
            self.vehicle_local_velocity[1] = -velocity[1]
            self.vehicle_local_velocity[2] = -velocity[2]

        attitude = np.asarray(msg.q, dtype=float)
        if np.isfinite(attitude).all() and float(np.linalg.norm(attitude)) > 1e-6:
            self.vehicle_attitude = _px4_quat_to_model(attitude)
            self.have_attitude = True

        if not self.have_reference:
            self.reference_state[0] = self.vehicle_local_position[0]
            self.reference_state[1] = self.vehicle_local_position[1]

    def reference_callback(self, msg: Odometry) -> None:
        self.reference_state[0] = msg.pose.pose.position.x
        self.reference_state[1] = msg.pose.pose.position.y
        self.reference_state[2] = msg.pose.pose.position.z
        self.reference_state[3] = msg.twist.twist.linear.x
        self.reference_state[4] = msg.twist.twist.linear.y
        self.reference_state[5] = msg.twist.twist.linear.z
        self.reference_state[6:10] = self._quat_from_pose(
            msg.pose.pose.orientation
        )
        self.reference_state[10] = msg.twist.twist.angular.x
        self.reference_state[11] = msg.twist.twist.angular.y
        self.reference_state[12] = msg.twist.twist.angular.z
        self.have_reference = True

    def setpoint_pose_callback(self, msg: PoseStamped) -> None:
        self.reference_state[0] = msg.pose.position.x
        self.reference_state[1] = msg.pose.position.y
        self.reference_state[2] = msg.pose.position.z
        self.reference_state[3:6] = 0.0
        self.reference_state[6:10] = self._quat_from_pose(msg.pose.orientation)
        self.reference_state[10:13] = 0.0
        self.have_reference = True

    def set_pose_callback(
        self,
        request: SetPose.Request,
        response: SetPose.Response,
    ):
        self.reference_state[0] = request.pose.position.x
        self.reference_state[1] = request.pose.position.y
        self.reference_state[2] = request.pose.position.z
        self.reference_state[3:6] = 0.0
        self.reference_state[6:10] = self._quat_from_pose(
            request.pose.orientation
        )
        self.reference_state[10:13] = 0.0
        self.have_reference = True
        response.result = True
        return response

    def publish_offboard_mode(self) -> None:
        msg = OffboardControlMode()
        msg.timestamp = _timestamp_us(self)
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = True
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self.offboard_mode_pub.publish(msg)

    def control_loop(self) -> None:
        if self.control_mode == "manual_rates":
            if self._should_publish_setpoint():
                self.publish_manual_rate_setpoint()
            return

        if self.control_mode == "altitude_hold":
            if not self._state_ready():
                return
            if self._should_publish_setpoint():
                self.publish_altitude_hold_setpoint()
            return

        if not self._state_ready():
            return

        x0 = np.concatenate(
            (
                self.vehicle_local_position,
                self.vehicle_local_velocity,
                self.vehicle_attitude,
                self.vehicle_angular_velocity,
            )
        )
        x_ref = self.reference_state.copy()

        if self._unsafe_for_nmpc():
            self.previous_solution = None
            u0 = self._hover_fallback_control()
            self.get_logger().warn(
                self._safety_fallback_message(),
                throttle_duration_sec=1.0,
            )
        else:
            solution = self.mpc.solve(x0, x_ref, self.previous_solution)
            self.last_solve_status = str(solution["status"])
            solve_succeeded = self.last_solve_status == "Solve_Succeeded"

            if solve_succeeded:
                x_pred = np.asarray(solution["x_pred"], dtype=float)
                self.previous_solution = {
                    "x_pred": x_pred,
                    "u_pred": np.asarray(solution["u_pred"], dtype=float),
                }
                u0 = np.asarray(solution["u0"], dtype=float)
                self.publish_prediction(x_pred)
            else:
                u0 = self._hover_fallback_control()
                self.get_logger().warn(
                    self._solver_failure_message(),
                    throttle_duration_sec=2.0,
                )

        self.publish_reference_marker(x_ref)

        if self._should_publish_setpoint():
            self.publish_rate_setpoint(u0)

    def _should_publish_setpoint(self) -> bool:
        return (
            self.publish_before_offboard
            or self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )

    def publish_manual_rate_setpoint(self) -> None:
        msg = VehicleRatesSetpoint()
        msg.timestamp = _timestamp_us(self)
        msg.roll = float(self.manual_roll_rate)
        msg.pitch = float(self.manual_pitch_rate)
        msg.yaw = float(self.manual_yaw_rate)
        msg.thrust_body[0] = self._clip01(self.manual_pusher)
        msg.thrust_body[1] = 0.0
        msg.thrust_body[2] = -self._clip01(self.manual_lift)
        msg.reset_integral = False
        self.rates_setpoint_pub.publish(msg)

    def publish_altitude_hold_setpoint(self) -> None:
        altitude_error = (
            self.reference_altitude - float(self.vehicle_local_position[2])
        )
        vertical_velocity = float(self.vehicle_local_velocity[2])
        lift = self.altitude_hold_hover_thrust + (
            self.altitude_hold_gain * altitude_error
        ) - (
            self.altitude_hold_velocity_gain * vertical_velocity
        )
        lift = float(
            np.clip(
                lift,
                self.altitude_hold_min_thrust,
                self.altitude_hold_max_thrust,
            )
        )

        msg = VehicleRatesSetpoint()
        msg.timestamp = _timestamp_us(self)
        msg.roll = 0.0
        msg.pitch = 0.0
        msg.yaw = 0.0
        msg.thrust_body[0] = self._clip01(self.manual_pusher)
        msg.thrust_body[1] = 0.0
        msg.thrust_body[2] = -self._clip01(lift)
        msg.reset_integral = False
        self.rates_setpoint_pub.publish(msg)

    def publish_rate_setpoint(self, u: np.ndarray) -> None:
        params = self.model.params
        lift = self._clip01(u[0] / params.max_lift_thrust)
        pusher = self._clip01(u[1] / params.max_pusher_thrust)

        roll_rate = self._moment_to_rate(
            u[2],
            params.max_roll_moment,
            sign=1.0,
        )
        pitch_rate = self._moment_to_rate(
            u[3],
            params.max_pitch_moment,
            sign=-1.0,
        )
        yaw_rate = self._moment_to_rate(u[4], params.max_yaw_moment, sign=-1.0)

        msg = VehicleRatesSetpoint()
        msg.timestamp = _timestamp_us(self)
        msg.roll = float(roll_rate)
        msg.pitch = float(pitch_rate)
        msg.yaw = float(yaw_rate)
        msg.thrust_body[0] = float(pusher)
        msg.thrust_body[1] = 0.0
        msg.thrust_body[2] = float(-lift)
        msg.reset_integral = False
        self.rates_setpoint_pub.publish(msg)

    def _hover_fallback_control(self) -> np.ndarray:
        fallback = np.zeros(self.model.nu, dtype=float)
        altitude_error = (
            float(self.reference_state[2]) - float(self.vehicle_local_position[2])
        )
        normalized_lift = self.fallback_hover_thrust + (
            self.fallback_altitude_gain * altitude_error
        ) - (
            self.fallback_vertical_velocity_gain
            * float(self.vehicle_local_velocity[2])
        )
        normalized_lift = float(
            np.clip(
                normalized_lift,
                self.fallback_min_thrust,
                self.fallback_max_thrust,
            )
        )
        fallback[0] = normalized_lift * self.model.params.max_lift_thrust
        fallback[1] = 0.0
        fallback[2:5] = self._leveling_moments()
        return fallback

    def _leveling_moments(self) -> np.ndarray:
        params = self.model.params
        if self.rate_setpoint_limit <= 0.0:
            return np.zeros(3, dtype=float)

        roll, pitch, _ = self.model.quaternion_to_euler(self.vehicle_attitude)
        desired_rates = np.array(
            [
                -self.fallback_attitude_gain * roll,
                -self.fallback_attitude_gain * pitch,
                0.0,
            ],
            dtype=float,
        )
        desired_rates = np.clip(
            desired_rates,
            -self.rate_setpoint_limit,
            self.rate_setpoint_limit,
        )
        return np.array(
            [
                desired_rates[0] / self.rate_setpoint_limit
                * params.max_roll_moment,
                desired_rates[1] / self.rate_setpoint_limit
                * params.max_pitch_moment,
                desired_rates[2] / self.rate_setpoint_limit
                * params.max_yaw_moment,
            ],
            dtype=float,
        )

    def _unsafe_for_nmpc(self) -> bool:
        speed_safety_active = (
            self.vehicle_local_position[2] > self.speed_safety_min_altitude
        )
        speed_unsafe = speed_safety_active and self._speed() > self.max_safe_speed
        tilt_unsafe = self._tilt_deg() > self.max_safe_tilt_deg
        return speed_unsafe or tilt_unsafe

    def _safety_fallback_message(self) -> str:
        return (
            "NMPC safety fallback active; "
            f"tilt={self._tilt_deg():.1f}/{self.max_safe_tilt_deg:.1f} deg, "
            f"speed={self._speed():.1f}/{self.max_safe_speed:.1f} m/s, "
            f"velocity=[{self.vehicle_local_velocity[0]:.1f}, "
            f"{self.vehicle_local_velocity[1]:.1f}, "
            f"{self.vehicle_local_velocity[2]:.1f}] m/s, "
            f"altitude={self.vehicle_local_position[2]:.1f}/"
            f"{self.reference_state[2]:.1f} m, "
            f"speed_safety_min_altitude={self.speed_safety_min_altitude:.1f} m"
        )

    def _solver_failure_message(self) -> str:
        return (
            f"NMPC status: {self.last_solve_status}; "
            "publishing hover fallback; "
            f"tilt={self._tilt_deg():.1f}/{self.max_tilt_deg:.1f} deg, "
            f"speed={self._speed():.1f} m/s, "
            f"altitude={self.vehicle_local_position[2]:.1f}/"
            f"{self.reference_state[2]:.1f} m, "
            f"body_rates={self._body_rates_label()}"
        )

    def _speed(self) -> float:
        return float(np.linalg.norm(self.vehicle_local_velocity))

    def _body_rates_label(self) -> str:
        return (
            f"[{self.vehicle_angular_velocity[0]:.2f}, "
            f"{self.vehicle_angular_velocity[1]:.2f}, "
            f"{self.vehicle_angular_velocity[2]:.2f}]/"
            f"{self.nmpc_max_body_rate:.2f} rad/s"
        )

    def _tilt_deg(self) -> float:
        q = self.model.quaternion_normalize(self.vehicle_attitude)
        body_z_world_z = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])
        body_z_world_z = float(np.clip(body_z_world_z, -1.0, 1.0))
        return float(np.rad2deg(np.arccos(body_z_world_z)))

    def publish_prediction(self, x_pred: np.ndarray) -> None:
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for k in range(x_pred.shape[1]):
            pose = _pose_msg("map", x_pred[0:3, k], x_pred[6:10, k])
            pose.header.stamp = msg.header.stamp
            msg.poses.append(pose)
        self.predicted_path_pub.publish(msg)

    def publish_reference_marker(self, reference: np.ndarray) -> None:
        msg = Marker()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.action = Marker.ADD
        msg.ns = "standard_vtol_reference"
        msg.id = 1
        msg.type = Marker.SPHERE
        msg.scale.x = 0.75
        msg.scale.y = 0.75
        msg.scale.z = 0.75
        msg.color.r = 0.0
        msg.color.g = 0.4
        msg.color.b = 1.0
        msg.color.a = 0.9
        msg.pose.position.x = float(reference[0])
        msg.pose.position.y = float(reference[1])
        msg.pose.position.z = float(reference[2])
        msg.pose.orientation.w = 1.0
        self.reference_marker_pub.publish(msg)

    def _state_ready(self) -> bool:
        if self.have_local_position and self.have_attitude:
            return True
        missing = []
        if not self.have_local_position:
            missing.append("local position")
        if not self.have_attitude:
            missing.append("attitude")
        self.get_logger().info(
            f"waiting for PX4 {' and '.join(missing)}",
            throttle_duration_sec=2.0,
        )
        return False

    def _quat_from_pose(self, orientation) -> np.ndarray:
        q = np.array(
            [orientation.w, orientation.x, orientation.y, orientation.z],
            dtype=float,
        )
        if not np.isfinite(q).all() or float(np.linalg.norm(q)) < 1e-6:
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        return self.model.quaternion_normalize(q)

    def _moment_to_rate(
        self,
        moment: float,
        max_moment: float,
        sign: float,
    ) -> float:
        if max_moment <= 0.0:
            return 0.0
        normalized = float(np.clip(moment / max_moment, -1.0, 1.0))
        return sign * normalized * self.rate_setpoint_limit

    @staticmethod
    def _clip01(value: float) -> float:
        if not math.isfinite(value):
            return 0.0
        return float(np.clip(value, 0.0, 1.0))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StandardVtolMPC()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
