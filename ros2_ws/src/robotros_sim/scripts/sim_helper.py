#!/usr/bin/env python3
"""Convenience for interactive simulation sessions (tools/sim_start.sh, launch argument auto:=true).

- auto_arm: calls /hw/set_arm once at start (repeats until the simulated Nano answers ARMED), then stops.
  It never arms again: after an E-stop or a watchdog stop the operator arms by hand, as on the robot.
- initial_pose (mode nav): publishes /initialpose at the true robot pose (/sim/ground_truth) until AMCL
  answers on /amcl_pose. Valid because the map frame of a map made in this world starts at the spawn
  pose of the mapping run (world origin by default).
Simulation only; it is not started on the robot.
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from robotros_interfaces.srv import SetArm


class SimHelper(Node):
    def __init__(self) -> None:
        super().__init__("sim_helper")
        self.declare_parameter("auto_arm", True)
        self.declare_parameter("initial_pose", False)
        self.arm_pending = bool(self.get_parameter("auto_arm").value)
        self.pose_pending = bool(self.get_parameter("initial_pose").value)
        self.arm_future = None
        self.truth: Odometry | None = None
        self.arm_client = self.create_client(SetArm, "hw/set_arm")
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
        latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE
        )
        self.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self.on_amcl, latched)
        self.create_subscription(Odometry, "sim/ground_truth", self.on_truth, qos_profile_sensor_data)
        self.create_timer(1.0, self.tick)  # simulation time

    def on_truth(self, msg: Odometry) -> None:
        self.truth = msg

    def on_amcl(self, _msg: PoseWithCovarianceStamped) -> None:
        if self.pose_pending:
            self.get_logger().info("AMCL has a pose: initial pose done")
        self.pose_pending = False

    def tick(self) -> None:
        if self.arm_pending:
            self.try_arm()
        if self.pose_pending and self.truth is not None:
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = Time().to_msg()  # 0 = use the latest transform
            msg.pose.pose = self.truth.pose.pose
            msg.pose.covariance[0] = msg.pose.covariance[7] = 0.05**2
            msg.pose.covariance[35] = 0.05**2
            self.pub_init.publish(msg)

    def try_arm(self) -> None:
        if self.arm_future is not None:
            if not self.arm_future.done():
                return
            res = self.arm_future.result()
            self.arm_future = None
            if res is not None and res.success:
                self.arm_pending = False
                self.get_logger().info("robot armed (simulation)")
                return
        if self.arm_client.service_is_ready():
            self.arm_future = self.arm_client.call_async(SetArm.Request(arm=True))


def main() -> None:
    rclpy.init()
    node = SimHelper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
