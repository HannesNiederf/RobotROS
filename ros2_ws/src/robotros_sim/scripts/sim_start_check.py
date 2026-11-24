#!/usr/bin/env python3
"""Check of an interactive session started by tools/sim_start.sh (used with --check in CI).

  mapping: sim_helper armed the robot, /map arrives, /cmd_vel_manual moves the robot.
  nav:     sim_helper armed the robot and set the initial pose (/amcl_pose), Nav2 is active, a goal on
           /goal_pose (what RViz's "2D Goal Pose" publishes) makes the robot drive.
Exit code 0 = pass. Durations in simulation time, the wall clock only bounds the waiting.
"""

from __future__ import annotations

import math
import sys
import time
from collections.abc import Callable

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from robotros_interfaces.msg import SafetyState
from std_srvs.srv import Trigger

WAIT_S = 240.0


class StartCheck(Node):
    def __init__(self) -> None:
        super().__init__("sim_start_check", parameter_overrides=[Parameter("use_sim_time", value=True)])
        latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE
        )
        self.safety: SafetyState | None = None
        self.map: OccupancyGrid | None = None
        self.amcl: PoseWithCovarianceStamped | None = None
        self.truth: Odometry | None = None
        self.create_subscription(SafetyState, "/hw/safety", lambda m: setattr(self, "safety", m), 10)
        self.create_subscription(OccupancyGrid, "/map", lambda m: setattr(self, "map", m), latched)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", lambda m: setattr(self, "amcl", m), latched)
        self.create_subscription(
            Odometry, "/sim/ground_truth", lambda m: setattr(self, "truth", m), qos_profile_sensor_data
        )
        self.manual = self.create_publisher(Twist, "/cmd_vel_manual", 10)
        self.goal = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.failures = 0

    def report(self, ok: bool, text: str) -> None:
        print(("ok    " if ok else "FAIL  ") + text, flush=True)
        self.failures += 0 if ok else 1

    def spin_until(self, cond: Callable[[], bool], wall_s: float) -> bool:
        deadline = time.monotonic() + wall_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if cond():
                return True
        return False

    def moved_from(self, x0: float, y0: float) -> float:
        p = self.truth.pose.pose.position
        return math.hypot(p.x - x0, p.y - y0)

    def run(self, mode: str) -> int:
        ok = self.spin_until(lambda: self.truth is not None and self.get_clock().now().nanoseconds > 0, WAIT_S)
        self.report(ok, "simulation running")
        if not ok:
            return 1
        ok = self.spin_until(lambda: self.safety is not None and self.safety.state == SafetyState.ARMED, WAIT_S)
        self.report(ok, f"sim_helper armed the robot (state {self.safety and self.safety.state})")
        p0 = self.truth.pose.pose.position
        x0, y0 = p0.x, p0.y
        if mode == "mapping":
            ok = self.spin_until(lambda: self.map is not None, WAIT_S)
            self.report(ok, "/map from slam_toolbox")
            twist = Twist()
            twist.linear.x = 0.15
            end = self.get_clock().now() + Duration(seconds=2.0)
            deadline = time.monotonic() + 60.0
            while self.get_clock().now() < end and time.monotonic() < deadline:
                self.manual.publish(twist)
                self.spin_until(lambda: False, 0.05)
            self.manual.publish(Twist())
            d = self.moved_from(x0, y0)
            self.report(d > 0.15, f"/cmd_vel_manual (keyboard path) moves the robot ({d:.2f} m)")
        else:
            ok = self.spin_until(lambda: self.amcl is not None, WAIT_S)
            self.report(ok, "sim_helper set the initial pose (/amcl_pose received)")
            if ok:
                a = self.amcl.pose.pose.position
                self.report(math.hypot(a.x - x0, a.y - y0) < 0.2, f"AMCL pose at the robot ({a.x:.2f}, {a.y:.2f})")
            active = self.create_client(Trigger, "/lifecycle_manager_navigation/is_active")
            ok = False
            deadline = time.monotonic() + WAIT_S
            while not ok and time.monotonic() < deadline and active.wait_for_service(timeout_sec=5.0):
                future = active.call_async(Trigger.Request())
                self.spin_until(future.done, 10.0)
                ok = bool(future.done() and future.result().success)
                if not ok:
                    self.spin_until(lambda: False, 1.0)
            self.report(ok, "Nav2 active")
            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.pose.position.x, goal.pose.orientation.z, goal.pose.orientation.w = -1.0, 1.0, 0.0
            for _ in range(3):
                self.goal.publish(goal)
                self.spin_until(lambda: False, 0.5)
            moved = self.spin_until(lambda: self.moved_from(x0, y0) > 0.2, 120.0)
            d = self.moved_from(x0, y0)
            self.report(moved, f"goal on /goal_pose (RViz '2D Goal Pose') makes the robot drive ({d:.2f} m)")
        return 1 if self.failures else 0


def main() -> int:
    mode = sys.argv[1] if len(sys.argv) > 1 else "nav"
    rclpy.init()
    node = StartCheck()
    try:
        code = node.run(mode)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    print(f"SIM START ({mode}): " + ("PASS" if code == 0 else "FAIL"), flush=True)
    return code


if __name__ == "__main__":
    sys.exit(main())
