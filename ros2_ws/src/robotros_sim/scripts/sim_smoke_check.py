#!/usr/bin/env python3
"""Smoke test against a running robotROS simulation (robotros_sim sim.launch.py).

Checks: sim clock runs, /scan (frame, beam count, ranges), /joint_states, TF chain odom -> laser,
driving straight and turning via /cmd_vel measured on /odom. Durations of the drive commands are
measured in simulation time; the wall clock only bounds how long the check waits.

Only meant for the simulation: run it with an isolated ROS_DOMAIN_ID (tools/sim_smoke.sh does that).
Exit code 0 when every check passes.
"""

from __future__ import annotations

import math
import sys
import time
from collections.abc import Callable

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import Buffer, TransformListener

WAIT_S = 120.0  # wall-clock bound for the whole start-up (Gazebo can be slow on CI)


def yaw_of(odom: Odometry) -> float:
    q = odom.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SmokeCheck(Node):
    def __init__(self) -> None:
        super().__init__("sim_smoke_check", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.scan: LaserScan | None = None
        self.odom: Odometry | None = None
        self.joints: JointState | None = None
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.failures = 0

    def _on_scan(self, msg: LaserScan) -> None:
        self.scan = msg

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def _on_joints(self, msg: JointState) -> None:
        self.joints = msg

    def report(self, ok: bool, text: str) -> None:
        print(("ok    " if ok else "FAIL  ") + text, flush=True)
        if not ok:
            self.failures += 1

    def spin_until(self, cond: Callable[[], bool], wall_s: float) -> bool:
        deadline = time.monotonic() + wall_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if cond():
                return True
        return False

    @staticmethod
    def beam(scan: LaserScan, angle: float) -> float:
        i = round((angle - scan.angle_min) / scan.angle_increment) % len(scan.ranges)
        return float(scan.ranges[i])

    def sim_now(self) -> Time:
        return self.get_clock().now()

    def drive(self, linear: float, angular: float, sim_s: float) -> bool:
        """Publish a twist at 20 Hz for sim_s seconds of simulation time, then stop."""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        end = self.sim_now() + Duration(seconds=sim_s)
        deadline = time.monotonic() + sim_s * 20 + 10  # generous bound for a slow real-time factor
        next_pub = 0.0
        while self.sim_now() < end:
            if time.monotonic() > deadline:
                return False
            if time.monotonic() >= next_pub:
                self.cmd.publish(twist)
                next_pub = time.monotonic() + 0.05
            rclpy.spin_once(self, timeout_sec=0.01)
        self.cmd.publish(Twist())
        settle = self.sim_now() + Duration(seconds=1.0)
        self.spin_until(lambda: self.sim_now() >= settle, 30.0)
        self.cmd.publish(Twist())
        return True

    def run(self) -> int:
        ok = self.spin_until(lambda: self.sim_now().nanoseconds > 0, WAIT_S)
        self.report(ok, "simulation clock runs (/clock)")
        if not ok:
            return 1

        ok = self.spin_until(lambda: self.scan is not None, WAIT_S)
        self.report(ok, "/scan received")
        if ok:
            s = self.scan
            self.report(s.header.frame_id == "laser", f"/scan frame_id is 'laser' (got '{s.header.frame_id}')")
            n = len(s.ranges)
            self.report(n == 720, f"/scan has 720 beams (got {n})")
            self.report(
                abs(s.range_min - 0.15) < 1e-3 and abs(s.range_max - 12.0) < 1e-3,
                f"/scan range 0.15-12 m (got {s.range_min:.3f}-{s.range_max:.3f})",
            )
            hits = [r for r in s.ranges if math.isfinite(r) and s.range_min <= r <= s.range_max]
            # In the closed 7 x 7 m room nearly every beam hits something.
            self.report(len(hits) >= 0.9 * max(n, 1), f"/scan sees the room ({len(hits)} of {n} beams hit)")
            if hits:
                self.report(max(hits) < 5.5, f"/scan farthest hit {max(hits):.2f} m (room diagonal from centre ~4.9 m)")
            # Robot spawns at (0, 0) facing +x. The glass wall at y = -2 (x -1.5..-0.5) has no visual, so the beam
            # towards (-1, -2) passes it and hits the south wall (~3.9 m); the obstacle at (-1.5, 1) is at ~1.6 m.
            glass = self.beam(s, math.atan2(-2.0, -1.0))
            self.report(glass > 3.0, f"glass wall invisible to the lidar (beam towards it: {glass:.2f} m, > 3.0)")
            obstacle = self.beam(s, math.atan2(1.0, -1.5))
            self.report(1.3 < obstacle < 1.7, f"static obstacle seen (beam towards it: {obstacle:.2f} m, 1.3-1.7)")

        ok = self.spin_until(lambda: self.joints is not None, 30.0)
        names = set(self.joints.name) if self.joints else set()
        self.report(
            ok and {"left_wheel_joint", "right_wheel_joint"} <= names,
            f"/joint_states has both wheel joints (got {sorted(names)})",
        )

        ok = self.spin_until(lambda: self.tf_buffer.can_transform("odom", "laser", Time()), 30.0)
        self.report(ok, "TF odom -> base_footprint -> base_link -> laser available")

        ok = self.spin_until(lambda: self.odom is not None, 30.0)
        self.report(ok, "/odom received")
        if not ok:
            return 1
        self.report(
            self.odom.header.frame_id == "odom" and self.odom.child_frame_id == "base_footprint",
            f"/odom frames odom -> base_footprint (got {self.odom.header.frame_id} -> {self.odom.child_frame_id})",
        )

        x0, y0 = self.odom.pose.pose.position.x, self.odom.pose.pose.position.y
        ok = self.drive(0.2, 0.0, 2.0)
        dx = self.odom.pose.pose.position.x - x0
        dy = self.odom.pose.pose.position.y - y0
        self.report(ok and 0.3 <= dx <= 0.5 and abs(dy) < 0.05, f"drive 0.2 m/s for 2 s: dx={dx:.3f} m dy={dy:.3f} m")

        a0 = yaw_of(self.odom)
        ok = self.drive(0.0, 0.5, 2.0)
        da = math.atan2(math.sin(yaw_of(self.odom) - a0), math.cos(yaw_of(self.odom) - a0))
        self.report(ok and 0.75 <= da <= 1.25, f"turn 0.5 rad/s for 2 s: dyaw={da:.3f} rad")

        return 1 if self.failures else 0


def main() -> int:
    rclpy.init()
    node = SmokeCheck()
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print("SIM SMOKE: " + ("PASS" if code == 0 else "FAIL"), flush=True)
    return code


if __name__ == "__main__":
    sys.exit(main())
