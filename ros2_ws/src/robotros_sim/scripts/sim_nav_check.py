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
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import OccupancyGrid, Odometry, Path
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
# Narrow passage of worlds/generic.sdf (0.45 m, spec section 14): wall x = 1.5, free between these y values.
PASSAGE_X = 1.5
PASSAGE_Y = (-1.0, -0.55)


def yaw_of(q) -> float:  # noqa: ANN001
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class NavCheck(Node):
    def __init__(self) -> None:
        super().__init__("sim_nav_check", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.failures = 0
        self.map: OccupancyGrid | None = None
        self.truth: Odometry | None = None
        self.safety: SafetyState | None = None
        self.amcl: PoseWithCovarianceStamped | None = None
        latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, latched)
        self.create_subscription(Odometry, "/sim/ground_truth", self._on_truth, qos_profile_sensor_data)
        self.create_subscription(SafetyState, "/hw/safety", self._on_safety, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, latched)
        self.manual = self.create_publisher(Twist, "/cmd_vel_manual", 10)
        self.initial = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.arm_client = self.create_client(SetArm, "/hw/set_arm")
        self.nav = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        # Diagnostics while a goal runs (MEASURE lines): AMCL corrections, planned path, collision monitor.
        self.track: dict | None = None
        self.create_subscription(Path, "/plan", self._on_plan, 10)
        self.create_subscription(CollisionMonitorState, "/collision_monitor_state", self._on_cm, 10)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map = msg

    def _on_truth(self, msg: Odometry) -> None:
        self.truth = msg

    def _on_safety(self, msg: SafetyState) -> None:
        self.safety = msg

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        if self.track is not None and self.truth is not None:
            a, t = msg.pose.pose.position, self.truth.pose.pose.position
            err = math.hypot(a.x - t.x, a.y - t.y)
            prev = self.track.get("last")
            if prev is not None:
                # Jump of one AMCL update = change of the AMCL pose minus the true motion in the same interval.
                jump = math.hypot((a.x - prev[0]) - (t.x - prev[2]), (a.y - prev[1]) - (t.y - prev[3]))
                self.track["jumps"].append(jump)
            self.track["last"] = (a.x, a.y, t.x, t.y)
            self.track["errs"].append(err)
        self.amcl = msg

    def _on_plan(self, msg: Path) -> None:
        if self.track is not None and msg.poses:
            self.track["plans"] += 1
            gap = [p.pose.position for p in msg.poses if abs(p.pose.position.x - PASSAGE_X) < 0.06]
            if gap and all(PASSAGE_Y[0] < p.y < PASSAGE_Y[1] for p in gap):
                self.track["plans_through_passage"] += 1
                centre = (PASSAGE_Y[0] + PASSAGE_Y[1]) / 2
                self.track["plan_offset"] = max(self.track["plan_offset"], max(abs(p.y - centre) for p in gap))

    def _on_cm(self, msg: CollisionMonitorState) -> None:
        if self.track is not None and msg.action_type != 0:
            self.track["cm_events"] += 1

    def track_start(self) -> None:
        self.track = {"jumps": [], "errs": [], "last": None, "plans": 0, "plans_through_passage": 0,
                      "cm_events": 0, "min_clear": math.inf, "crossed": False, "plan_offset": 0.0}

    def track_truth(self) -> None:
        """Clearance of the true footprint centre to the passage walls while the robot is inside the passage."""
        if self.track is None or self.truth is None:
            return
        t = self.truth.pose.pose.position
        if abs(t.x - PASSAGE_X) < 0.2 and PASSAGE_Y[0] - 0.1 < t.y < PASSAGE_Y[1] + 0.1:
            self.track["min_clear"] = min(self.track["min_clear"], t.y - PASSAGE_Y[0], PASSAGE_Y[1] - t.y)
            self.track["crossed"] = True

    def track_report(self, run: int) -> bool:
        """Print the diagnostics of one goal; return whether the robot drove through the narrow passage."""
        tr, self.track = self.track, None
        if tr is None:
            return False
        jumps, errs = sorted(tr["jumps"]) or [math.nan], sorted(tr["errs"]) or [math.nan]
        print(f"MEASURE run {run}: AMCL updates {len(tr['jumps'])}, jump median {jumps[len(jumps) // 2]:.3f} m, "
              f"p90 {jumps[int(0.9 * (len(jumps) - 1))]:.3f} m, max {jumps[-1]:.3f} m; AMCL-truth median "
              f"{errs[len(errs) // 2]:.3f} m, max {errs[-1]:.3f} m", flush=True)
        print(f"MEASURE run {run}: plans {tr['plans']}, through the passage {tr['plans_through_passage']}; "
              f"collision monitor events {tr['cm_events']}; in passage {tr['crossed']}, "
              f"min clearance of the centre {tr['min_clear']:.3f} m (centred: "
              f"{(PASSAGE_Y[1] - PASSAGE_Y[0]) / 2:.3f}), plan max offset from the passage centre "
              f"{tr['plan_offset']:.3f} m", flush=True)
        return bool(tr["crossed"])

    def amcl_vs_truth(self, label: str) -> None:
        """Diagnostics: AMCL pose (map frame) against the ground truth (world frame)."""
        if self.amcl is None or self.truth is None:
            print(f"info  {label}: no AMCL pose yet", flush=True)
            return
        a, t = self.amcl.pose.pose, self.truth.pose.pose
        cov = self.amcl.pose.covariance
        print(
            f"info  {label}: AMCL ({a.position.x:.3f}, {a.position.y:.3f}, {yaw_of(a.orientation):.3f}) "
            f"truth ({t.position.x:.3f}, {t.position.y:.3f}, {yaw_of(t.orientation):.3f}) "
            f"diff {math.hypot(a.position.x - t.position.x, a.position.y - t.position.y):.3f} m, "
            f"sigma_xy {math.sqrt(max(cov[0], cov[7])):.3f} m",
            flush=True,
        )

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

    def goto(self, gx: float, gy: float, v_max: float = 0.15) -> bool:
        """Test driver for mapping: drive to (gx, gy) in the world frame on the ground truth via /cmd_vel_manual
        (the keyboard path: twist_mux A, collision monitor, simulated Nano). Not navigation software."""
        deadline = time.monotonic() + 600.0
        next_pub = 0.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            if time.monotonic() < next_pub:
                continue
            next_pub = time.monotonic() + 0.05
            p = self.truth.pose.pose
            dx, dy = gx - p.position.x, gy - p.position.y
            dist = math.hypot(dx, dy)
            if dist < 0.05:
                self.manual.publish(Twist())
                self.sim_sleep(0.3)
                return True
            err = math.atan2(math.sin(math.atan2(dy, dx) - yaw_of(p.orientation)),
                             math.cos(math.atan2(dy, dx) - yaw_of(p.orientation)))
            twist = Twist()
            if abs(err) > 0.25:
                twist.angular.z = math.copysign(0.4, err)
            else:
                twist.linear.x = min(v_max, 0.5 * dist + 0.03)
                twist.angular.z = max(-0.5, min(0.5, 2.0 * err))
            self.manual.publish(twist)
        self.manual.publish(Twist())
        return False

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
        # Tour of the room so that the map also covers the area behind the narrow passage (east of x = 1.5):
        # north of inner wall b, down the east side, west through the passage (centre y = -0.775), back.
        centre = (PASSAGE_Y[0] + PASSAGE_Y[1]) / 2
        tour = [(1.0, 2.5), (2.5, 2.5), (2.5, centre), (2.1, centre), (0.6, centre), (0.0, 0.0)]
        reached = all(self.goto(x, y, 0.08 if abs(y - centre) < 1e-6 else 0.15) for x, y in tour)
        self.report(reached, "mapping tour through the room and the narrow passage (driven on the ground truth)")
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
        behind = sum(1 for i, v in enumerate(m.data) if 0 <= v < 25
                     and ox + (i % m.info.width + 0.5) * res > 1.6 and oy + (i // m.info.width + 0.5) * res < -1.1)
        area = behind * res * res
        self.report(area > 2.5, f"area behind the passage mapped ({area:.1f} m^2 free, > 2.5)")
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
    def one_goal(self, run: int, gx: float, gy: float, gyaw: float) -> tuple[float, float, float] | None:
        """Send one goal; return (true position error, true heading error, AMCL-vs-truth) or None."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.sim_now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = gx, gy
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(gyaw / 2), math.cos(gyaw / 2)
        t0 = self.sim_now()
        start_x = self.truth.pose.pose.position.x
        self.track_start()
        send = self.nav.send_goal_async(goal)
        self.spin_until(send.done, 30.0)
        handle = send.result() if send.done() else None
        self.report(handle is not None and handle.accepted, f"run {run}: goal ({gx}, {gy}, {gyaw:.2f}) accepted")
        if handle is None or not handle.accepted:
            return None
        result = handle.get_result_async()
        self.spin_until(lambda: (self.track_truth(), result.done())[1], 600.0)
        status = result.result().status if result.done() else None
        took = (self.sim_now() - t0).nanoseconds / 1e9
        crossed = self.track_report(run)
        if (start_x - PASSAGE_X) * (gx - PASSAGE_X) < 0 and abs(gy - sum(PASSAGE_Y) / 2) < 0.8:
            # Start and goal on different sides of the inner wall, next to the passage: it must be used.
            self.report(crossed, f"run {run}: drove through the 0.45 m passage")
        self.report(status == 4, f"run {run}: NavigateToPose SUCCEEDED (status {status}) in {took:.1f} s sim")
        self.sim_sleep(1.0)
        self.amcl_vs_truth(f"run {run} end")
        q = self.truth.pose.pose
        err = math.hypot(q.position.x - gx, q.position.y - gy)
        dyaw = abs(math.atan2(math.sin(yaw_of(q.orientation) - gyaw), math.cos(yaw_of(q.orientation) - gyaw)))
        loc = math.inf
        if self.amcl is not None:
            a = self.amcl.pose.pose.position
            loc = math.hypot(a.x - q.position.x, a.y - q.position.y)
        print(f"info  run {run}: true error {err:.3f} m, {math.degrees(dyaw):.1f} deg; end pose "
              f"({q.position.x:.2f}, {q.position.y:.2f}, {math.degrees(yaw_of(q.orientation)):.0f} deg)", flush=True)
        return err, dyaw, loc

    def navigate(self, goals: list[tuple[float, float, float]], runs: int, acceptance: bool) -> int:
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
            init.header.stamp = Time().to_msg()  # 0 = latest transform (avoids extrapolation into the future)
            self.initial.publish(init)
            future = active.call_async(Trigger.Request())
            self.spin_until(future.done, 10.0)
            ok = bool(future.done() and future.result().success)
            if not ok:
                self.sim_sleep(1.0)
        self.report(ok, "lifecycle_manager_navigation: all Nav2 nodes active (after the initial pose)")
        for _ in range(3):  # once more after activation, so AMCL surely has it
            init.header.stamp = Time().to_msg()  # 0 = latest transform (avoids extrapolation into the future)
            self.initial.publish(init)
            self.sim_sleep(1.0)
        ok = ok and self.nav.wait_for_server(timeout_sec=60.0)
        self.report(ok, "NavigateToPose action server available")
        if not ok:
            return 1
        self.amcl_vs_truth("before the goals")
        within = 0
        for run in range(runs):
            gx, gy, gyaw = goals[run % len(goals)]
            res = self.one_goal(run + 1, gx, gy, gyaw)
            if res is None:
                return 1
            err, dyaw, loc = res
            # spez_parameter_tests.md M1-03: end error <= 0.10 m and <= 10 deg in >= 9 of 10 runs.
            m103 = err <= 0.10 and dyaw <= math.radians(10)
            within += int(m103)
            print(f"info  run {run + 1}: M1-03 limit 0.10 m / 10 deg {'met' if m103 else 'NOT met'}", flush=True)
            if not acceptance:
                # Single CI run: gross regression limits (chosen here, wider than M1-03, which is a 9-of-10 test).
                self.report(err <= 0.15, f"run {run + 1}: true position error {err:.3f} m (<= 0.15)")
                deg = math.degrees(dyaw)
                self.report(dyaw <= math.radians(10), f"run {run + 1}: true heading error {deg:.1f} deg (<= 10)")
                self.report(loc <= 0.10, f"run {run + 1}: AMCL vs truth {loc:.3f} m (<= 0.10)")
        if acceptance:
            text = f"M1-03 in simulation: {within} of {runs} runs within 0.10 m / 10 deg (>= 90 %)"
            self.report(within >= math.ceil(0.9 * runs), text)
        return 1 if self.failures else 0


def main() -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="phase", required=True)
    m = sub.add_parser("mapping")
    m.add_argument("--map-out", required=True)
    n = sub.add_parser("nav")
    n.add_argument("--goal", nargs=3, type=float, action="append", required=True, metavar=("X", "Y", "YAW"),
                   help="repeat for several goals; runs cycle through them")
    n.add_argument("--runs", type=int, default=0, help="number of goals to drive (default: one per --goal)")
    n.add_argument("--acceptance", action="store_true", help="M1-03 rule: >= 90 %% of runs within 0.10 m / 10 deg")
    args = ap.parse_args()
    rclpy.init()
    node = NavCheck()
    try:
        if args.phase == "mapping":
            code = node.mapping(args.map_out)
        else:
            goals = [tuple(g) for g in args.goal]
            code = node.navigate(goals, args.runs or len(goals), args.acceptance)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    print(f"SIM NAV ({args.phase}): " + ("PASS" if code == 0 else "FAIL"), flush=True)
    return code


if __name__ == "__main__":
    sys.exit(main())
