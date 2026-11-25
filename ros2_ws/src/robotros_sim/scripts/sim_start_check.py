#!/usr/bin/env python3
"""Check of an interactive session started by tools/sim_start.sh (used with --check in CI).

  mapping: sim_helper armed the robot, /map arrives, /cmd_vel_manual moves the robot, the wheel odometry
           (/odom) follows the true motion (/sim/ground_truth) when turning and driving.
  nav:     sim_helper armed the robot and set the initial pose (/amcl_pose), Nav2 is active, the recovery's
           BackUp reverses the robot (ADR 0002), a goal on
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
from nav2_msgs.action import BackUp
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from robotros_interfaces.msg import SafetyState
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger

WAIT_S = 240.0
ODOM_TURN_TOLERANCE = 0.03  # relative yaw error of the wheel odometry on a turn in place


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
        self.odom: Odometry | None = None
        self.create_subscription(Odometry, "/odom", lambda m: setattr(self, "odom", m), qos_profile_sensor_data)
        self.scan_ages: list[float] = []
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
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

    def _on_scan(self, msg: LaserScan) -> None:
        now = self.get_clock().now().nanoseconds
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if now > 0:
            self.scan_ages.append((now - stamp) / 1e9)

    @staticmethod
    def yaw_of(msg: Odometry) -> float:
        q = msg.pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def drive(self, vx: float, wz: float, sim_s: float) -> tuple[float, float, float, float]:
        """Drive with (vx, wz) for sim_s; return unwrapped yaw change and distance from /odom and the truth."""
        twist = Twist()
        twist.linear.x, twist.angular.z = vx, wz
        last = {"odom": self.yaw_of(self.odom), "truth": self.yaw_of(self.truth)}
        acc = {"odom": 0.0, "truth": 0.0}
        p_o0, p_t0 = self.odom.pose.pose.position, self.truth.pose.pose.position
        o0, t0 = (p_o0.x, p_o0.y), (p_t0.x, p_t0.y)
        end = self.get_clock().now() + Duration(seconds=sim_s)
        deadline = time.monotonic() + 120.0
        settle = None
        while time.monotonic() < deadline:
            now = self.get_clock().now()
            if now < end:
                self.manual.publish(twist)
            else:
                self.manual.publish(Twist())
                settle = settle or now + Duration(seconds=1.5)
                if now > settle:
                    break
            self.spin_until(lambda: False, 0.02)
            for key, msg in (("odom", self.odom), ("truth", self.truth)):
                y = self.yaw_of(msg)
                acc[key] += math.atan2(math.sin(y - last[key]), math.cos(y - last[key]))
                last[key] = y
        p_o, p_t = self.odom.pose.pose.position, self.truth.pose.pose.position
        return (acc["odom"], acc["truth"], math.hypot(p_o.x - o0[0], p_o.y - o0[1]),
                math.hypot(p_t.x - t0[0], p_t.y - t0[1]))

    def odom_consistency(self) -> None:
        """Measurement: does the simulated wheel odometry follow the true motion?"""
        ok = self.spin_until(lambda: self.odom is not None, 30.0)
        self.report(ok, "/odom received")
        if not ok:
            return
        wall0, sim0 = time.monotonic(), self.get_clock().now().nanoseconds
        for vx, wz, secs in ((0.0, 0.6, 2 * math.pi / 0.6), (0.0, -0.3, 2 * math.pi / 0.3),
                             (0.15, 0.0, 3.0), (0.10, 0.4, 4.0)):
            yo, yt, do, dt = self.drive(vx, wz, secs)
            print(f"MEASURE drive vx={vx:.2f} wz={wz:+.2f} {secs:.1f}s: yaw odom {math.degrees(yo):+.1f} deg, "
                  f"truth {math.degrees(yt):+.1f} deg, error {math.degrees(yo - yt):+.1f} deg; "
                  f"dist odom {do:.3f} m, truth {dt:.3f} m", flush=True)
            if vx == 0.0:
                # Turning in place: the wheel odometry must follow the true turn (gazebo_track_scale, sim_hw.yaml).
                rel = abs(yo - yt) / max(abs(yt), 1e-6)
                self.report(rel <= ODOM_TURN_TOLERANCE, f"odometry follows a turn in place at {wz:+.1f} rad/s "
                            f"(error {100 * rel:.1f} %, limit {100 * ODOM_TURN_TOLERANCE:.0f} %)")
            else:
                self.report(abs(do - dt) <= 0.03 * dt + 0.01, f"odometry follows the drive (vx {vx:.2f})")
        rtf = (self.get_clock().now().nanoseconds - sim0) / 1e9 / max(time.monotonic() - wall0, 1e-6)
        ages = sorted(self.scan_ages) or [float("nan")]
        print(f"MEASURE real time factor {rtf:.2f}; /scan stamp age at receipt: median "
              f"{ages[len(ages) // 2]:.3f} s, max {ages[-1]:.3f} s, n={len(self.scan_ages)}", flush=True)

    def backup_check(self) -> None:
        """The recovery's BackUp (ADR 0002) reaches the wheels through smoother, twist_mux and collision monitor."""
        client = ActionClient(self, BackUp, "/backup")
        if not client.wait_for_server(timeout_sec=30.0):
            self.report(False, "behavior_server /backup available")
            return
        p0, yaw = self.truth.pose.pose.position, self.yaw_of(self.truth)
        goal = BackUp.Goal()
        goal.target.x, goal.speed, goal.time_allowance.sec = 0.10, 0.05, 10
        sent = client.send_goal_async(goal)
        self.spin_until(sent.done, 10.0)
        handle = sent.result()
        if handle is None or not handle.accepted:
            self.report(False, "BackUp goal accepted")
            return
        res = handle.get_result_async()
        self.spin_until(res.done, 60.0)
        self.spin_until(lambda: False, 1.0)
        p = self.truth.pose.pose.position
        along = (p.x - p0.x) * math.cos(yaw) + (p.y - p0.y) * math.sin(yaw)
        self.report(-0.13 <= along <= -0.07, f"recovery BackUp 0.10 m reverses the robot (moved {along:+.3f} m)")

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
            self.odom_consistency()
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
            self.backup_check()
            p0 = self.truth.pose.pose.position
            x0, y0 = p0.x, p0.y
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
