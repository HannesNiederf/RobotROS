#!/usr/bin/env python3
"""Check of mission_manager part 1 in a running nav simulation (ADR 0007), run by tools/sim_start.sh nav --check.

Stations: two storage places are taught over /map/upsert_station and appear on /web/stations.
Order checks: E303, E301, a valid TRANSPORT, E301 again for the reserved source.
TRANSPORT to a taught place without a real cart: the robot drives to the pre-check point, the slot check of the
docking server sees no cart and the order is DEFERRED with E301 (ADR 0009).
Real TRANSPORT from S1 (cart C1 in the world) to a free place S3 near the north wall: turn, reverse under the cart,
lift, drive to S3, reverse in, lower, drive out; the order completes and the cart is booked at S3.
(The Gazebo cart does not move yet.)
GOTO_POINT: a software E-stop while driving pauses the order (safety not ARMED), /mission/ack_safety RESUME arms
the robot again and the order completes. A queued order can be cancelled.
Exit code 0 = pass. The wall clock only bounds the waiting.
"""

from __future__ import annotations

import math
import sys
import time
from collections.abc import Callable

import rclpy
from geometry_msgs.msg import Pose2D, PoseStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robotros_interfaces.msg import MissionStatus, OperatorAlert, OrderList, SafetyState, Station, StationList
from robotros_interfaces.srv import AckSafety, CreateOrder, OrderAction, UpsertStation
from std_srvs.srv import Trigger

# Station poses: the pre-check point P_pre = S + 0.70 m along the station heading is a goal that sim_nav.sh reaches.
S1 = (0.0, -1.5, 0.0)    # P_pre (0.7, -1.5)
S2 = (-3.0, 0.0, 0.0)    # P_pre (-2.3, 0.0)
POINT = (-1.0, 0.0, 0.0)
# Places of the world generic.sdf: S1 with cart C1 (feet); the destination is a free place in the same (west) part
# of the room, so the check does not depend on the 0.45 m passage, whose planning is known to fail now and then
# (ADR 0003; seen as Nav2 error 208 in 2 of about 6 runs on 27.11.2026). Front pointing out of the box (D-33).
WORLD_S1 = (-3.25, 0.0, 0.0)
WORLD_S3 = (-2.5, 3.25, -math.pi / 2)


class MissionCheck(Node):
    def __init__(self) -> None:
        super().__init__("mission_check", parameter_overrides=[Parameter("use_sim_time", value=True)])
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.status: MissionStatus | None = None
        self.states: list[str] = []
        self.orders: OrderList | None = None
        self.stations: StationList | None = None
        self.safety: SafetyState | None = None
        self.pose: PoseStamped | None = None
        self.alerts: list[OperatorAlert] = []
        self.create_subscription(MissionStatus, "/mission/status", self._on_status, 10)
        self.create_subscription(OrderList, "/mission/orders", lambda m: setattr(self, "orders", m), latched)
        self.create_subscription(StationList, "/web/stations", lambda m: setattr(self, "stations", m), latched)
        self.create_subscription(SafetyState, "/hw/safety", lambda m: setattr(self, "safety", m), 10)
        self.create_subscription(PoseStamped, "/robot_pose", lambda m: setattr(self, "pose", m), 10)
        self.create_subscription(OperatorAlert, "/mission/alerts", self.alerts.append, 10)
        self.failures = 0

    def _on_status(self, msg: MissionStatus) -> None:
        self.status = msg
        if not self.states or self.states[-1] != msg.state:
            self.states.append(msg.state)
            print(f"info  /mission/status: {msg.state} order {msg.active_order_id} step {msg.step} "
                  f"waiting '{msg.waiting_reason}' error '{msg.error_code}'", flush=True)

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

    def call(self, srv_type, name: str, req, wall_s: float = 10.0):  # noqa: ANN001, ANN201
        client = self.create_client(srv_type, name)
        if not client.wait_for_service(timeout_sec=wall_s):
            self.report(False, f"service {name} available")
            return None
        fut = client.call_async(req)
        self.spin_until(fut.done, wall_s)
        return fut.result() if fut.done() else None

    def order(self, order_id: int):  # noqa: ANN201
        for o in (self.orders.orders if self.orders else []):
            if o.id == order_id:
                return o
        return None

    def order_state(self, order_id: int) -> str:
        o = self.order(order_id)
        return o.state if o else ""

    def create(self, type_: str, src: int = 0, dst: int = 0, goal: tuple = (0.0, 0.0, 0.0)) -> CreateOrder.Response:
        req = CreateOrder.Request(type=type_, priority=5, source_station_id=src, dest_station_id=dst,
                                  goal=Pose2D(x=goal[0], y=goal[1], theta=goal[2]))
        res = self.call(CreateOrder, "/mission/create_order", req)
        return res if res is not None else CreateOrder.Response(success=False, error_code="no answer")

    def dist_to(self, x: float, y: float) -> float:
        if self.pose is None:
            return math.inf
        p = self.pose.pose.position
        return math.hypot(p.x - x, p.y - y)

    def run(self) -> int:
        idle = self.spin_until(lambda: self.status is not None and self.status.state == "IDLE", 90.0)
        self.report(idle, f"mission_manager reaches IDLE (states {self.states})")
        if not idle:
            return 1
        # ---- stations ----
        ids = []
        for name, pose, occupied in (("check_S1", S1, True), ("check_S2", S2, False)):
            st = Station(id=0, name=name, kind="STORAGE", pose=Pose2D(x=pose[0], y=pose[1], theta=pose[2]),
                         approach_distance=0.40, enabled=True, occupied=occupied, cart_name="")
            res = self.call(UpsertStation, "/map/upsert_station", UpsertStation.Request(station=st))
            ok = res is not None and res.success
            self.report(ok, f"/map/upsert_station {name} (occupied={occupied})")
            ids.append(res.station_id if ok else 0)
        s1, s2 = ids
        listed = self.spin_until(
            lambda: self.stations is not None and {s.id: s.occupied for s in self.stations.stations}.get(s1) is True
            and {s.id: s.occupied for s in self.stations.stations}.get(s2) is False, 15.0)
        self.report(listed, "/web/stations lists both stations, S1 with a cart, S2 free")
        # ---- order checks ----
        self.report(self.create("TRANSPORT", s1, s1).error_code == "E303", "TRANSPORT S1 -> S1 refused with E303")
        self.report(self.create("TRANSPORT", s2, s1).error_code == "E301", "TRANSPORT from empty S2 refused (E301)")
        res = self.create("TRANSPORT", s1, s2)
        self.report(res.success, f"TRANSPORT S1 -> S2 accepted (order {res.order_id})")
        tid = res.order_id
        self.report(self.create("TRANSPORT", s1, s2).error_code == "E301", "second order from reserved S1: E301")
        # ---- transport until cart handling ----
        pickup = self.spin_until(lambda: "GOTO_PICKUP" in self.states, 20.0)
        self.report(pickup, "mission drives to the pickup station (GOTO_PICKUP)")
        deferred = self.spin_until(lambda: self.order_state(tid) == "DEFERRED", 180.0)
        o = self.order(tid)
        self.report(deferred and o is not None and o.error_code == "E301",
                    f"slot check sees no cart: order DEFERRED with E301 ({o.state if o else '-'} "
                    f"{o.error_code if o else ''})")
        d = self.dist_to(S1[0] + 0.7, S1[1])
        self.report(d < 0.35, f"robot stopped at the pre-check point of S1 ({d:.2f} m away)")
        self.report(self.spin_until(lambda: any(a.code == "E301" for a in self.alerts), 5.0),
                    "alert E301 on /mission/alerts")
        still = {s.id: s.occupied for s in self.stations.stations} if self.stations else {}
        self.report(still.get(s1) is True and still.get(s2) is False, "cart still at S1, S2 still free")
        # ---- GOTO_POINT with software E-stop and resume ----
        res = self.create("GOTO_POINT", goal=POINT)
        self.report(res.success, f"GOTO_POINT accepted (order {res.order_id})")
        gid = res.order_id
        driving = self.spin_until(lambda: self.status is not None and self.status.state == "GOTO_POINT"
                                  and self.dist_to(S1[0] + 0.7, S1[1]) > 0.15, 60.0)
        self.report(driving, "robot drives to the point")
        stop = self.call(Trigger, "/mission/software_estop", Trigger.Request())
        self.report(stop is not None and stop.success, "/mission/software_estop answered success")
        paused = self.spin_until(lambda: self.safety is not None and self.safety.state != SafetyState.ARMED
                                 and self.status is not None and self.status.state == "PAUSED_SAFETY", 10.0)
        self.report(paused, "after the software E-stop: robot not armed, mission PAUSED_SAFETY")
        self.report(self.order_state(gid) == "ACTIVE", "the order stays ACTIVE while paused")
        ack = self.call(AckSafety, "/mission/ack_safety", AckSafety.Request(resume_mode="RESUME"))
        self.report(ack is not None and ack.success, "/mission/ack_safety RESUME arms the robot")
        done = self.spin_until(lambda: self.order_state(gid) == "COMPLETED", 180.0)
        d = self.dist_to(POINT[0], POINT[1])
        self.report(done and d < 0.35, f"GOTO_POINT completed after resuming ({d:.2f} m from the point)")
        # ---- real transport S1 -> S3 ----
        self.real_transport()
        # ---- cancel ----
        self.call(Trigger, "/mission/software_estop", Trigger.Request())
        self.spin_until(lambda: self.status is not None and self.status.state == "LOCKED", 10.0)
        res = self.create("GOTO_POINT", goal=(0.0, 0.0, 0.0))
        cancel = self.call(OrderAction, "/mission/order_action", OrderAction.Request(order_id=res.order_id,
                                                                                    action="CANCEL"))
        ok = cancel is not None and cancel.success
        self.report(ok and self.spin_until(lambda: self.order_state(res.order_id) == "CANCELLED", 10.0),
                    "a queued order can be cancelled")
        self.call(AckSafety, "/mission/ack_safety", AckSafety.Request(resume_mode="RESUME"))
        return 1 if self.failures else 0


    def upsert(self, name: str, pose: tuple, occupied: bool) -> int:
        st = Station(id=0, name=name, kind="STORAGE", pose=Pose2D(x=pose[0], y=pose[1], theta=pose[2]),
                     approach_distance=0.40, enabled=True, occupied=occupied, cart_name="")
        res = self.call(UpsertStation, "/map/upsert_station", UpsertStation.Request(station=st))
        ok = res is not None and res.success
        self.report(ok, f"/map/upsert_station {name}")
        return res.station_id if ok else 0

    def real_transport(self) -> bool:
        s1, s3 = self.upsert("world_S1", WORLD_S1, True), self.upsert("world_S3", WORLD_S3, False)
        res = self.create("TRANSPORT", s1, s3)
        self.report(res.success, f"TRANSPORT world_S1 -> world_S3 accepted (order {res.order_id})")
        if not res.success:
            return False
        oid = res.order_id
        closest = [math.inf]
        steps: list[str] = []

        def watch() -> bool:
            closest[0] = min(closest[0], self.dist_to(WORLD_S1[0], WORLD_S1[1]))
            if self.status is not None and self.status.active_order_id == oid and (
                    not steps or steps[-1] != self.status.step):
                steps.append(self.status.step)
            return self.order_state(oid) in ("COMPLETED", "DEFERRED", "CANCELLED") or (
                self.status is not None and self.status.state in ("NEEDS_OPERATOR", "HELD_WITH_CART"))

        self.spin_until(watch, 480.0)
        o = self.order(oid)
        print(f"info  transport steps {steps}, closest to S1 {closest[0]:.3f} m, "
              f"state {o.state if o else '-'} {o.error_code if o else ''} {self.status.state if self.status else ''}",
              flush=True)
        self.report(closest[0] <= 0.05, f"robot reversed under the cart to S1 ({closest[0]:.3f} m)")
        done = o is not None and o.state == "COMPLETED"
        self.report(done, f"transport COMPLETED (steps {steps})")
        d = self.dist_to(WORLD_S3[0], WORLD_S3[1])
        self.report(0.30 <= d <= 0.50, f"robot drove out of S3 to P_a ({d:.2f} m from S3)")
        def occ() -> dict:
            return {s.id: s.cart_name for s in self.stations.stations} if self.stations else {}

        booked = self.spin_until(lambda: occ().get(s1) == "" and occ().get(s3) == "cart@world_S1", 5.0)
        self.report(booked, f"cart booked at S3 ({occ()})")
        return done


def main() -> int:
    rclpy.init()
    node = MissionCheck()
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    print("MISSION CHECK: " + ("PASS" if code == 0 else "FAIL"), flush=True)
    return code


if __name__ == "__main__":
    sys.exit(main())
