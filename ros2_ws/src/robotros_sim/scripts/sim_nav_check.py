#!/usr/bin/env python3
"""Mapping and navigation check against a running simulation (tools/sim_nav.sh starts it).

  sim_nav_check.py mapping --map-out DIR/name   sim with mode:=mapping: arm, drive a short pattern via
                                                /cmd_vel_manual (twist_mux A -> collision_monitor -> twist_mux B),
                                                check /map covers the 7 x 7 m room, save it (map_saver_cli)
  sim_nav_check.py nav --goal X Y YAW           sim with mode:=nav map:=...: initial pose at the spawn pose,
                                                arm, NavigateToPose, check the ground truth at the end

Durations are simulation time; the wall clock only bounds how long the check waits. Exit code 0 = pass.
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time
from collections.abc import Callable

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from robotros_interfaces.msg import SafetyState
from robotros_interfaces.srv import SetArm
from std_srvs.srv import Trigger

WAIT_S = 180.0  # wall-clock bound for start-up of Gazebo plus the stack on a slow CI runner


def yaw_of(q) -> float:  # noqa: ANN001
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class NavCheck(Node):
    def __init__(self) -> None:
        super().__init__("sim_nav_check", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.failures = 0
        self.map: OccupancyGrid | None = None
        self.truth: Odometry | None = None
        self.safety: SafetyState | None = None
        latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, latched)
        self.create_subscription(Odometry, "/sim/ground_truth", self._on_truth, qos_profile_sensor_data)
        self.create_subscription(SafetyState, "/hw/safety", self._on_safety, 10)
        self.manual = self.create_publisher(Twist, "/cmd_vel_manual", 10)
        self.initial = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.arm_client = self.create_client(SetArm, "/hw/set_arm")
        self.nav = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map = msg

    def _on_truth(self, msg: Odometry) -> None:
        self.truth = msg

    def _on_safety(self, msg: SafetyState) -> None:
        self.safety = msg

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

    def sim_now(self) -> Time:
        return self.get_clock().now()

    def sim_sleep(self, s: float) -> None:
        end = self.sim_now() + Duration(seconds=s)
        self.spin_until(lambda: self.sim_now() >= end, s * 20 + 10)

    def drive(self, v: float, w: float, sim_s: float) -> None:
        twist = Twist()
        twist.linear.x, twist.angular.z = v, w
        end = self.sim_now() + Duration(seconds=sim_s)
        deadline = time.monotonic() + sim_s * 20 + 10
        next_pub = 0.0
        while self.sim_now() < end and time.monotonic() < deadline:
            if time.monotonic() >= next_pub:
                self.manual.publish(twist)
                next_pub = time.monotonic() + 0.05
            rclpy.spin_once(self, timeout_sec=0.01)
        self.manual.publish(Twist())
        self.sim_sleep(0.5)

    def start(self) -> bool:
        ok = self.spin_until(lambda: self.sim_now().nanoseconds > 0 and self.truth is not None, WAIT_S)
        self.report(ok, "simulation running (/clock, /sim/ground_truth)")
        if not ok:
            return False
        ok = self.arm_client.wait_for_service(timeout_sec=60.0)
        res = None
        # sim_hw_adapter answers "not armed" until its own clock runs; retry like an operator would.
        for _ in range(20 if ok else 0):
            future = self.arm_client.call_async(SetArm.Request(arm=True))
            self.spin_until(future.done, 10.0)
            res = future.result() if future.done() else None
            if res is not None and res.success:
                break
            self.sim_sleep(1.0)
        self.report(res is not None and res.success, f"/hw/set_arm -> ARMED (got {res and (res.success, res.state)})")
        self.sim_sleep(0.5)
        return res is not None and res.success

    # ------------------------------------------------------------------ mapping
    def mapping(self, map_out: str) -> int:
        if not self.start():
            return 1
        ok = self.spin_until(lambda: self.map is not None, 120.0)
        self.report(ok, "/map from slam_toolbox received")
        # Pattern: full turn, 0.4 m forward, half turn, 0.4 m back towards the start, then stop.
        self.drive(0.0, 0.5, 13.0)
        x0 = self.truth.pose.pose.position.x
        self.drive(0.15, 0.0, 2.7)
        dx = self.truth.pose.pose.position.x - x0
        self.report(dx > 0.25, f"/cmd_vel_manual -> twist_mux -> collision_monitor -> robot (dx={dx:.3f})")
        self.drive(0.0, 0.5, 6.5)
        self.drive(0.15, 0.0, 2.7)
        self.sim_sleep(5.0)  # map_update_interval 2 s
        m = self.map
        if m is None:
            return 1
        res = m.info.resolution
        w_m, h_m = m.info.width * res, m.info.height * res
        self.report(res == 0.05 or abs(res - 0.05) < 1e-6, f"map resolution 0.05 m (got {res})")
        self.report(w_m >= 6.8 and h_m >= 6.8, f"map covers the room (size {w_m:.2f} x {h_m:.2f} m, >= 6.8)")
        ox, oy = m.info.origin.position.x, m.info.origin.position.y
        occ = [(ox + (i % m.info.width + 0.5) * res, oy + (i // m.info.width + 0.5) * res)
               for i, v in enumerate(m.data) if v >= 65]
        free = sum(1 for v in m.data if 0 <= v < 25)
        sides = {
            "east": any(x > 3.3 and abs(y) < 3.0 for x, y in occ),
            "west": any(x < -3.3 and abs(y) < 3.0 for x, y in occ),
            "north": any(y > 3.3 and abs(x) < 3.0 for x, y in occ),
            "south": any(y < -3.3 and abs(x) < 3.0 for x, y in occ),
        }
        self.report(all(sides.values()), f"all four walls in the map ({sides})")
        self.report(free * res * res > 20.0, f"free area {free * res * res:.1f} m^2 (> 20)")
        obstacle = any(abs(x + 1.5) < 0.3 and abs(y - 1.0) < 0.3 for x, y in occ)
        self.report(obstacle, "static obstacle at (-1.5, 1.0) in the map")
        glass = any(abs(y + 2.0) < 0.1 and -1.4 < x < -0.6 for x, y in occ)
        self.report(not glass, "glass wall not in the lidar map (expected: invisible to the lidar)")
        cmd = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", map_out]
        # Wall-clock timeout of map_saver (default 2 s is one slam map_update_interval; too short on CI).
        cmd += ["--ros-args", "-p", "save_map_timeout:=20.0", "-p", "map_subscribe_transient_local:=true"]
        done = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
        self.report(done.returncode == 0, f"map saved to {map_out}.yaml (map_saver_cli rc {done.returncode})")
        if done.returncode != 0:
            print(done.stdout[-2000:], done.stderr[-2000:])
        return 1 if self.failures else 0

    # ------------------------------------------------------------------ nav
    def navigate(self, gx: float, gy: float, gyaw: float) -> int:
        if not self.start():
            return 1
        p = self.truth.pose.pose
        init = PoseWithCovarianceStamped()
        init.header.frame_id = "map"
        init.pose.pose = p
        init.pose.covariance[0] = init.pose.covariance[7] = 0.05**2
        init.pose.covariance[35] = 0.05**2
        # The action server exists before bt_navigator is active (it rejects goals until then): wait for the
        # lifecycle manager to report all navigation nodes active.
        active = self.create_client(Trigger, "/lifecycle_manager_navigation/is_active")
        ok = False
        deadline = time.monotonic() + WAIT_S
        while not ok and time.monotonic() < deadline and active.wait_for_service(timeout_sec=5.0):
            # The global costmap only activates once map -> odom exists, i.e. after AMCL got an initial pose
            # (set_initial_pose: false): keep publishing it at the spawn pose until everything is active.
            init.header.stamp = self.sim_now().to_msg()
            self.initial.publish(init)
            future = active.call_async(Trigger.Request())
            self.spin_until(future.done, 10.0)
            ok = bool(future.done() and future.result().success)
            if not ok:
                self.sim_sleep(1.0)
        self.report(ok, "lifecycle_manager_navigation: all Nav2 nodes active (after the initial pose)")
        for _ in range(3):  # once more after activation, so AMCL surely has it
            init.header.stamp = self.sim_now().to_msg()
            self.initial.publish(init)
            self.sim_sleep(1.0)
        ok = ok and self.nav.wait_for_server(timeout_sec=60.0)
        self.report(ok, "NavigateToPose action server available")
        if not ok:
            return 1
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.sim_now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = gx, gy
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
        t0 = self.sim_now()
        send = self.nav.send_goal_async(goal)
        self.spin_until(send.done, 30.0)
        handle = send.result() if send.done() else None
        self.report(handle is not None and handle.accepted, "goal accepted")
        if handle is None or not handle.accepted:
            return 1
        result = handle.get_result_async()
        self.spin_until(result.done, 600.0)
        status = result.result().status if result.done() else None
        took = (self.sim_now() - t0).nanoseconds / 1e9
        self.report(status == 4, f"NavigateToPose succeeded (status {status}, 4 = SUCCEEDED) in {took:.1f} s sim")
        self.sim_sleep(1.0)
        q = self.truth.pose.pose
        err = math.hypot(q.position.x - gx, q.position.y - gy)
        dyaw = abs(math.atan2(math.sin(yaw_of(q.orientation) - gyaw), math.cos(yaw_of(q.orientation) - gyaw)))
        # Acceptance criterion: 0.10 m / 10 deg (spez_parameter_tests.md, spec 7.2 goal checker is tighter).
        self.report(err <= 0.10, f"true position error {err:.3f} m (<= 0.10)")
        self.report(dyaw <= math.radians(10), f"true heading error {math.degrees(dyaw):.1f} deg (<= 10)")
        return 1 if self.failures else 0


def main() -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="phase", required=True)
    m = sub.add_parser("mapping")
    m.add_argument("--map-out", required=True)
    n = sub.add_parser("nav")
    n.add_argument("--goal", nargs=3, type=float, required=True, metavar=("X", "Y", "YAW"))
    args = ap.parse_args()
    rclpy.init()
    node = NavCheck()
    try:
        code = node.mapping(args.map_out) if args.phase == "mapping" else node.navigate(*args.goal)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    print(f"SIM NAV ({args.phase}): " + ("PASS" if code == 0 else "FAIL"), flush=True)
    return code


if __name__ == "__main__":
    sys.exit(main())
