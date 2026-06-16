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

from px4_msgs.msg import VehicleLocalPosition


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
        self.x_offset = float(self.declare_parameter("x_offset", 0.0).value)
        self.y_offset = float(self.declare_parameter("y_offset", 0.0).value)
        self.reference_topic = self.declare_parameter(
            "reference_topic", "px4_mpc/standard_vtol/reference"
        ).value

        self.current_position = np.zeros(3, dtype=float)
        self.have_position = False
        self.start_time = self.get_clock().now()

        self.create_subscription(
            VehicleLocalPosition,
            _px4_topic(self.namespace, "/fmu/out/vehicle_local_position"),
            self.local_position_callback,
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

    def publish_reference(self) -> None:
        elapsed = (
            self.get_clock().now().nanoseconds - self.start_time.nanoseconds
        ) * 1e-9
        position, velocity = self._reference_at(elapsed)

        msg = Odometry()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.child_frame_id = "standard_vtol_reference"
        msg.pose.pose.position.x = float(position[0])
        msg.pose.pose.position.y = float(position[1])
        msg.pose.pose.position.z = float(position[2])
        msg.pose.pose.orientation.w = 1.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
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
