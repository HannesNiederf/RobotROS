#!/usr/bin/env python3
"""Measure the real time factor of the running simulation: simulated seconds on /clock per wall-clock second.

Usage: sim_rtf.py [SECONDS] (default 10). 1.0 = real time, 0.1 = the simulation runs ten times slower than real.
The wall clock is used only for this measurement, not for any robot logic.
"""

from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


class RtfProbe(Node):
    def __init__(self) -> None:
        super().__init__("sim_rtf")
        self.first: tuple[float, float] | None = None
        self.last: tuple[float, float] | None = None
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Clock, "/clock", self.on_clock, qos)

    def on_clock(self, msg: Clock) -> None:
        sample = (msg.clock.sec + msg.clock.nanosec / 1e9, time.monotonic())
        if self.first is None:
            self.first = sample
        self.last = sample


def main() -> int:
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    rclpy.init()
    node = RtfProbe()
    end = time.monotonic() + seconds + 15.0
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.first and node.last and node.last[1] - node.first[1] >= seconds:
                break
        if node.first is None or node.last is None or node.last[1] <= node.first[1]:
            print("no /clock received - is the simulation running (tools/sim_start.sh mapping|nav)?")
            return 1
        sim = node.last[0] - node.first[0]
        wall = node.last[1] - node.first[1]
        print(f"real time factor {sim / wall:.2f} ({sim:.1f} s simulated in {wall:.1f} s)")
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
