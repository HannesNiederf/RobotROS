#!/usr/bin/env python3
"""Clock relay for the simulation (ADR 0006): Gazebo's clock (/sim/clock, every physics step) -> /clock at most
rate_hz times per simulated second.

ROS nodes only need the clock at the resolution of their own timers (here at most 100 Hz). Gazebo's clock at
every step would cross the WLAN a thousand times per second in a --pi session. Gating uses the simulated time of
the incoming messages, not the wall clock.
"""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


class ClockRelay(Node):
    def __init__(self) -> None:
        super().__init__("clock_relay")
        self.declare_parameter("rate_hz", 100.0)
        self.period_ns = int(1e9 / float(self.get_parameter("rate_hz").value))
        self.last_ns: int | None = None
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(Clock, "/clock", 10)
        self.create_subscription(Clock, "/sim/clock", self.on_clock, qos)

    def on_clock(self, msg: Clock) -> None:
        now = msg.clock.sec * 1_000_000_000 + msg.clock.nanosec
        if self.last_ns is None or now - self.last_ns >= self.period_ns or now < self.last_ns:
            self.last_ns = now
            self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ClockRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
