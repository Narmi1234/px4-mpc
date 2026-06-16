#!/usr/bin/env python3
"""Arm PX4 and switch the standard VTOL into offboard mode."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from px4_msgs.msg import (
    OffboardControlMode,
    VehicleCommand,
    VehicleCommandAck,
    VehicleStatus,
)


PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6.0

COMMAND_RESULT_LABELS = {
    VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED: "accepted",
    VehicleCommandAck.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED: "temporary reject",
    VehicleCommandAck.VEHICLE_CMD_RESULT_DENIED: "denied",
    VehicleCommandAck.VEHICLE_CMD_RESULT_UNSUPPORTED: "unsupported",
    VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED: "failed",
    VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS: "in progress",
    VehicleCommandAck.VEHICLE_CMD_RESULT_CANCELLED: "cancelled",
}


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


class StandardVtolCommander(Node):
    """Continuously stream offboard heartbeat, then request offboard + arm."""

    def __init__(self) -> None:
        super().__init__("standard_vtol_commander")

        self.namespace = self.declare_parameter("namespace", "").value
        self.heartbeat_dt = float(
            self.declare_parameter("heartbeat_dt", 0.05).value
        )
        self.command_interval = float(
            self.declare_parameter("command_interval", 1.0).value
        )
        self.startup_heartbeats = int(
            self.declare_parameter("startup_heartbeats", 20).value
        )
        self.arm = bool(self.declare_parameter("arm", True).value)
        self.offboard = bool(self.declare_parameter("offboard", True).value)
        self.require_preflight_checks = bool(
            self.declare_parameter("require_preflight_checks", True).value
        )
        self.target_system = int(
            self.declare_parameter("target_system", 1).value
        )
        self.target_component = int(
            self.declare_parameter("target_component", 1).value
        )
        self.source_system = int(
            self.declare_parameter("source_system", 1).value
        )
        self.source_component = int(
            self.declare_parameter("source_component", 1).value
        )

        pub_qos = _px4_qos(QoSDurabilityPolicy.TRANSIENT_LOCAL)
        sub_qos = _px4_qos(QoSDurabilityPolicy.VOLATILE)

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            _px4_topic(self.namespace, "/fmu/in/offboard_control_mode"),
            pub_qos,
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            _px4_topic(self.namespace, "/fmu/in/vehicle_command"),
            pub_qos,
        )
        self.create_subscription(
            VehicleStatus,
            _px4_topic(self.namespace, "/fmu/out/vehicle_status"),
            self.vehicle_status_callback,
            sub_qos,
        )
        self.create_subscription(
            VehicleCommandAck,
            _px4_topic(self.namespace, "/fmu/out/vehicle_command_ack"),
            self.vehicle_command_ack_callback,
            sub_qos,
        )

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.pre_flight_checks_pass = False
        self.failsafe = False
        self.heartbeat_count = 0
        self.last_command_time = self.get_clock().now()
        self.reported_ready = False

        self.create_timer(self.heartbeat_dt, self.timer_callback)
        self.get_logger().info(
            "standard VTOL commander streaming offboard heartbeat"
        )

    def vehicle_status_callback(self, msg: VehicleStatus) -> None:
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.pre_flight_checks_pass = msg.pre_flight_checks_pass
        self.failsafe = msg.failsafe

    def vehicle_command_ack_callback(self, msg: VehicleCommandAck) -> None:
        if msg.command not in (
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
        ):
            return

        label = COMMAND_RESULT_LABELS.get(msg.result, f"result {msg.result}")
        self.get_logger().info(
            "PX4 command ack: "
            f"command={msg.command}, result={label}, "
            f"param1={msg.result_param1}, param2={msg.result_param2}"
        )

    def timer_callback(self) -> None:
        self.publish_offboard_mode()
        self.heartbeat_count += 1

        if self.heartbeat_count < self.startup_heartbeats:
            return

        now = self.get_clock().now()
        elapsed = (now.nanoseconds - self.last_command_time.nanoseconds) * 1e-9
        if elapsed < self.command_interval:
            return
        self.last_command_time = now

        if self.require_preflight_checks and not self._health_ready():
            self.get_logger().warn(
                "waiting for PX4 health: "
                f"pre_flight_checks_pass={self.pre_flight_checks_pass}, "
                f"failsafe={self.failsafe}",
                throttle_duration_sec=2.0,
            )
            return

        if (
            self.offboard
            and self.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD
        ):
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                param1=1.0,
                param2=PX4_CUSTOM_MAIN_MODE_OFFBOARD,
            )
            self.get_logger().info("requested PX4 offboard mode")

        if self.arm and self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=float(VehicleCommand.ARMING_ACTION_ARM),
            )
            self.get_logger().info("requested arm")

        if (
            not self.reported_ready
            and self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
            and (
                not self.arm
                or self.arming_state == VehicleStatus.ARMING_STATE_ARMED
            )
        ):
            self.reported_ready = True
            self.get_logger().info("vehicle is armed and in offboard mode")

    def _health_ready(self) -> bool:
        return self.pre_flight_checks_pass and not self.failsafe

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

    def publish_vehicle_command(
        self,
        command: int,
        *,
        param1: float = 0.0,
        param2: float = 0.0,
        param3: float = 0.0,
        param4: float = 0.0,
        param5: float = 0.0,
        param6: float = 0.0,
        param7: float = 0.0,
    ) -> None:
        msg = VehicleCommand()
        msg.timestamp = _timestamp_us(self)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)
        msg.command = int(command)
        msg.target_system = self.target_system
        msg.target_component = self.target_component
        msg.source_system = self.source_system
        msg.source_component = self.source_component
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StandardVtolCommander()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
