#!/usr/bin/env python3
"""Reference generator for the standard VTOL NMPC offboard node."""

from __future__ import annotations

import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry


def _px4_topic(namespace: str, topic: str) -> str:
    prefix = f"/{namespace.strip('/')}" if namespace else ""
    normalized_topic = topic if topic.startswith("/") else f"/{topic}"
    return f"{prefix}{normalized_topic}"


def _px4_sub_qos() -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


def _px4_quat_to_model(q_px4) -> np.ndarray:
    q = np.asarray(q_px4, dtype=float)
    if not np.isfinite(q).all() or float(np.linalg.norm(q)) < 1e-6:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

    converted = np.array([q[0], q[1], -q[2], -q[3]], dtype=float)
    converted = converted / float(np.linalg.norm(converted))
    if converted[0] < 0.0:
        converted = -converted
    return converted


class StandardVtolReference(Node):
    """Publish a moving NMPC reference in the controller's NWU world frame."""

    def __init__(self) -> None:
        super().__init__("standard_vtol_reference")

        self.namespace = self.declare_parameter("namespace", "").value
        self.profile = self.declare_parameter("profile", "transition").value
        self.rate_hz = float(self.declare_parameter("rate_hz", 10.0).value)
        self.altitude = float(self.declare_parameter("altitude", 20.0).value)
        self.forward_speed = float(
            self.declare_parameter("forward_speed", 16.0).value
        )
        self.transition_delay = float(
            self.declare_parameter("transition_delay", 5.0).value
        )
        self.speed_ramp_time = float(
            self.declare_parameter("speed_ramp_time", 8.0).value
        )
        self.lookahead_time = float(
            self.declare_parameter("lookahead_time", 2.7).value
        )
        self.min_lookahead = float(
            self.declare_parameter("min_lookahead", 10.0).value
        )
        self.max_odometry_position_norm = float(
            self.declare_parameter("max_odometry_position_norm", 1000.0).value
        )
        self.x_offset = float(self.declare_parameter("x_offset", 0.0).value)
        self.y_offset = float(self.declare_parameter("y_offset", 0.0).value)
        self.reference_topic = self.declare_parameter(
            "reference_topic", "px4_mpc/standard_vtol/reference"
        ).value

        self.current_position = np.zeros(3, dtype=float)
        self.current_attitude = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self.odometry_origin: np.ndarray | None = None
        self.have_position = False
        self.have_attitude = False
        self.start_time = self.get_clock().now()

        self.create_subscription(
            VehicleLocalPosition,
            _px4_topic(self.namespace, "/fmu/out/vehicle_local_position"),
            self.local_position_callback,
            _px4_sub_qos(),
        )
        self.create_subscription(
            VehicleOdometry,
            _px4_topic(self.namespace, "/fmu/out/vehicle_odometry"),
            self.vehicle_odometry_callback,
            _px4_sub_qos(),
        )
        self.reference_pub = self.create_publisher(
            Odometry,
            _px4_topic(self.namespace, self.reference_topic),
            10,
        )
        self.create_timer(1.0 / max(self.rate_hz, 1.0), self.publish_reference)

        self.get_logger().info(
            "standard VTOL reference ready: "
            f"profile={self.profile}, altitude={self.altitude:.1f} m, "
            f"speed={self.forward_speed:.1f} m/s"
        )

    def local_position_callback(self, msg: VehicleLocalPosition) -> None:
        if not (msg.xy_valid and msg.z_valid):
            return
        self.current_position[0] = msg.x
        self.current_position[1] = -msg.y
        self.current_position[2] = -msg.z
        self.have_position = True

    def vehicle_odometry_callback(self, msg: VehicleOdometry) -> None:
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            return

        position = np.asarray(msg.position, dtype=float)
        if np.isfinite(position).all():
            if self.odometry_origin is None:
                self.odometry_origin = position.copy()
            local_position = position - self.odometry_origin
            if float(np.linalg.norm(local_position)) > self.max_odometry_position_norm:
                return
            self.current_position[0] = local_position[0]
            self.current_position[1] = -local_position[1]
            self.current_position[2] = -local_position[2]
            self.have_position = True

        attitude = np.asarray(msg.q, dtype=float)
        if np.isfinite(attitude).all() and float(np.linalg.norm(attitude)) > 1e-6:
            self.current_attitude = _px4_quat_to_model(attitude)
            self.have_attitude = True

    def publish_reference(self) -> None:
        elapsed = (
            self.get_clock().now().nanoseconds - self.start_time.nanoseconds
        ) * 1e-9
        position, velocity = self._reference_at(elapsed)
        attitude = self._attitude_reference()

        msg = Odometry()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.child_frame_id = "standard_vtol_reference"
        msg.pose.pose.position.x = float(position[0])
        msg.pose.pose.position.y = float(position[1])
        msg.pose.pose.position.z = float(position[2])
        msg.pose.pose.orientation.w = float(attitude[0])
        msg.pose.pose.orientation.x = float(attitude[1])
        msg.pose.pose.orientation.y = float(attitude[2])
        msg.pose.pose.orientation.z = float(attitude[3])
        msg.twist.twist.linear.x = float(velocity[0])
        msg.twist.twist.linear.y = float(velocity[1])
        msg.twist.twist.linear.z = float(velocity[2])
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = 0.0
        self.reference_pub.publish(msg)

    def _reference_at(self, elapsed: float) -> tuple[np.ndarray, np.ndarray]:
        if self.have_position:
            base = self.current_position.copy()
        else:
            base = np.zeros(3)
        base[2] = self.altitude

        if self.profile == "hover":
            speed = 0.0
            lookahead = 0.0
        elif self.profile == "static":
            speed = self.forward_speed
            lookahead = self.x_offset
            base[1] = self.y_offset
        else:
            ramp = self._smooth_ramp(
                (elapsed - self.transition_delay)
                / max(self.speed_ramp_time, 1e-3)
            )
            speed = self.forward_speed * ramp
            lookahead = max(
                speed * self.lookahead_time,
                self.min_lookahead * ramp,
            )

        position = np.array(
            [
                base[0] + self.x_offset + lookahead,
                base[1] + self.y_offset,
                self.altitude,
            ],
            dtype=float,
        )
        velocity = np.array([speed, 0.0, 0.0], dtype=float)
        return position, velocity

    def _attitude_reference(self) -> np.ndarray:
        if self.profile == "hover" and self.have_attitude:
            return self.current_attitude.copy()
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

    @staticmethod
    def _smooth_ramp(value: float) -> float:
        value = float(np.clip(value, 0.0, 1.0))
        if not math.isfinite(value):
            return 0.0
        return value * value * (3.0 - 2.0 * value)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StandardVtolReference()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
