#!/usr/bin/env python3
"""Measure scan, odometry and localization while the robot turns on the spot (running nav simulation).

Hannes saw the laser scan rotated against the map after turning (27.11.2026). This script separates the causes
with the ground truth of the simulation (/sim/ground_truth, world frame):
  odom_scale  turned angle in /odom divided by the true angle (1.0 = odometry right)
  scan_delta  rotation that best aligns each scan with the world when it is drawn at the true pose of its time
              stamp (0 = stamp and data agree); scan_lag_ms = scan_delta / omega
  amcl_err    heading of map -> base_footprint minus the true heading, right after stopping and 3 s later
Prints one MEASURE line per turn rate. Exit code 0 unless the measurement itself fails (no data);
--max-* options turn the values into checks.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from robotros_sim.world_map import boxes_from_sdf, rasterize
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

RES = 0.03
HIT_RADIUS_CELLS = 2  # a scan point within 6 cm of an obstacle counts as a hit


def yaw_of(q) -> float:  # noqa: ANN001
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Track:
    """Time series of (stamp, x, y, unwrapped yaw)."""

    def __init__(self) -> None:
        self.t: list[float] = []
        self.p: list[tuple[float, float, float]] = []

    def add(self, t: float, x: float, y: float, yaw: float) -> None:
        if self.p:
            prev = self.p[-1][2]
            yaw = prev + math.atan2(math.sin(yaw - prev), math.cos(yaw - prev))
        self.t.append(t)
        self.p.append((x, y, yaw))

    def at(self, t: float) -> tuple[float, float, float] | None:
        if len(self.t) < 2 or t < self.t[0] or t > self.t[-1]:
            return None
        i = int(np.searchsorted(self.t, t))
        i = min(max(i, 1), len(self.t) - 1)
        t0, t1 = self.t[i - 1], self.t[i]
        a = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
        p0, p1 = self.p[i - 1], self.p[i]
        return tuple(p0[k] + a * (p1[k] - p0[k]) for k in range(3))  # type: ignore[return-value]


class RotationCheck(Node):
    def __init__(self, world: Path) -> None:
        super().__init__("rotation_check", parameter_overrides=[Parameter("use_sim_time", value=True)])
        boxes = boxes_from_sdf(world.read_text(encoding="utf-8"))
        self.origin = (-4.0, -4.0)
        grid = np.array(rasterize(boxes, 0.12, self.origin, (8.0, 8.0), RES), dtype=bool)
        hit = grid.copy()
        r = HIT_RADIUS_CELLS
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy <= r * r:
                    hit |= np.roll(np.roll(grid, dy, axis=0), dx, axis=1)
        self.hit = hit
        self.gt, self.odom = Track(), Track()
        self.scans: list[LaserScan] = []
        self.recording = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, "/sim/ground_truth", self.on_gt, 50)
        self.create_subscription(Odometry, "/odom", self.on_odom, 50)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, "/cmd_vel_manual", 10)

    @staticmethod
    def stamp(msg) -> float:  # noqa: ANN001
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_gt(self, m: Odometry) -> None:
        p = m.pose.pose
        self.gt.add(self.stamp(m), p.position.x, p.position.y, yaw_of(p.orientation))

    def on_odom(self, m: Odometry) -> None:
        p = m.pose.pose
        self.odom.add(self.stamp(m), p.position.x, p.position.y, yaw_of(p.orientation))

    def on_scan(self, m: LaserScan) -> None:
        if self.recording:
            self.scans.append(m)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_sim(self, seconds: float, twist: Twist | None = None, wall_limit: float = 120.0) -> None:
        start, wall = self.now(), time.monotonic()
        while self.now() - start < seconds and time.monotonic() - wall < wall_limit:
            if twist is not None:
                self.pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

    def score(self, scan: LaserScan, x: float, y: float, yaw: float) -> float:
        r = np.asarray(scan.ranges, dtype=float)
        a = scan.angle_min + scan.angle_increment * np.arange(len(r))
        ok = np.isfinite(r) & (r > scan.range_min) & (r < scan.range_max)
        r, a = r[ok], a[ok]
        if len(r) < 20:
            return 0.0
        px, py = x + r * np.cos(yaw + a), y + r * np.sin(yaw + a)
        c = np.floor((px - self.origin[0]) / RES).astype(int)
        rr = np.floor((py - self.origin[1]) / RES).astype(int)
        inside = (c >= 0) & (c < self.hit.shape[1]) & (rr >= 0) & (rr < self.hit.shape[0])
        hits = np.zeros(len(r), dtype=bool)
        hits[inside] = self.hit[rr[inside], c[inside]]
        return float(hits.mean())

    def best_delta(self, scan: LaserScan, pose: tuple[float, float, float]) -> tuple[float, float, float]:
        """(delta, score at delta, score at 0)."""
        x, y, yaw = pose
        deltas = np.arange(-0.25, 0.2501, 0.005)
        scores = [self.score(scan, x, y, yaw + d) for d in deltas]
        i = int(np.argmax(scores))
        return float(deltas[i]), scores[i], self.score(scan, x, y, yaw)

    def map_yaw(self) -> float | None:
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_footprint", Time())
        except TransformException:
            return None
        return yaw_of(tf.transform.rotation)

    def turn(self, omega: float) -> dict:
        self.spin_sim(1.0)
        self.scans.clear()
        g0, o0 = self.gt.p[-1][2], self.odom.p[-1][2]
        tw = Twist()
        tw.angular.z = omega
        self.recording = True
        t_turn = 2 * math.pi / abs(omega)
        t_start = self.now()
        self.spin_sim(t_turn, tw, wall_limit=t_turn * 5 + 30)
        self.recording = False
        self.pub.publish(Twist())
        self.spin_sim(0.3, Twist())
        g1, o1 = self.gt.p[-1][2], self.odom.p[-1][2]
        # scans in the steady part of the turn (skip the first second: acceleration)
        deltas, lags, s0, s1 = [], [], [], []
        for sc in self.scans:
            ts = self.stamp(sc)
            if ts < t_start + 1.0:
                continue
            pose = self.gt.at(ts)
            if pose is None:
                continue
            d, best, zero = self.best_delta(sc, pose)
            deltas.append(d)
            s0.append(zero)
            s1.append(best)
        w_true = (g1 - g0) / max(t_turn, 1e-3)
        for d in deltas:
            lags.append(-d / w_true * 1000.0 if abs(w_true) > 1e-3 else math.nan)
        yaw_true = self.gt.p[-1][2]
        my = self.map_yaw()
        err0 = math.nan if my is None else math.atan2(math.sin(my - yaw_true), math.cos(my - yaw_true))
        self.spin_sim(3.0)
        my = self.map_yaw()
        yaw_true = self.gt.p[-1][2]
        err3 = math.nan if my is None else math.atan2(math.sin(my - yaw_true), math.cos(my - yaw_true))
        return {
            "omega_cmd": omega,
            "omega_true": w_true,
            "odom_scale": (o1 - o0) / (g1 - g0) if abs(g1 - g0) > 1e-3 else math.nan,
            "scans": len(deltas),
            "scan_delta_deg": math.degrees(float(np.median(deltas))) if deltas else math.nan,
            "scan_lag_ms": float(np.median(lags)) if lags else math.nan,
            "score_at_stamp": float(np.median(s0)) if s0 else math.nan,
            "score_best": float(np.median(s1)) if s1 else math.nan,
            "amcl_err_deg_stop": math.degrees(err0),
            "amcl_err_deg_3s": math.degrees(err3),
        }

    def static_check(self) -> dict:
        """Scan alignment while standing still (reference)."""
        self.scans.clear()
        self.recording = True
        self.spin_sim(2.0)
        self.recording = False
        vals = []
        for sc in self.scans:
            pose = self.gt.at(self.stamp(sc))
            if pose is not None:
                vals.append(self.best_delta(sc, pose))
        return {"scans": len(vals), "scan_delta_deg": math.degrees(float(np.median([v[0] for v in vals])))
                if vals else math.nan, "score_at_stamp": float(np.median([v[2] for v in vals])) if vals else math.nan}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--omegas", default="0.3,0.6,1.0")
    ap.add_argument("--world", default="")
    ap.add_argument("--max-scan-delta-deg", type=float, default=None)
    ap.add_argument("--max-odom-error", type=float, default=None, help="allowed |odom_scale - 1|")
    args = ap.parse_args()
    share = Path(get_package_share_directory("robotros_sim"))
    world = Path(args.world) if args.world else share / "worlds" / "generic.sdf"
    rclpy.init()
    node = RotationCheck(world)
    code = 0
    try:
        wall = time.monotonic()
        while (not node.gt.p or not node.odom.p) and time.monotonic() - wall < 30:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not node.gt.p or not node.odom.p:
            print("FAIL  no /sim/ground_truth or /odom", flush=True)
            return 1
        st = node.static_check()
        print("MEASURE rotation standing " + " ".join(f"{k}={v:.3f}" if isinstance(v, float) else f"{k}={v}"
                                                      for k, v in st.items()), flush=True)
        for w in (float(v) for v in args.omegas.split(",")):
            res = node.turn(w)
            print("MEASURE rotation " + " ".join(f"{k}={v:.3f}" if isinstance(v, float) else f"{k}={v}"
                                                 for k, v in res.items()), flush=True)
            if args.max_scan_delta_deg is not None and not abs(res["scan_delta_deg"]) <= args.max_scan_delta_deg:
                print(f"FAIL  scan rotated {res['scan_delta_deg']:.2f} deg against the truth at {w} rad/s", flush=True)
                code = 1
            if args.max_odom_error is not None and not abs(res["odom_scale"] - 1.0) <= args.max_odom_error:
                print(f"FAIL  odometry scale {res['odom_scale']:.3f} at {w} rad/s", flush=True)
                code = 1
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()
    print("ROTATION CHECK: " + ("PASS" if code == 0 else "FAIL"), flush=True)
    return code


if __name__ == "__main__":
    sys.exit(main())
