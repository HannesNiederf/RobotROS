"""mission_manager, part 1 (softwarespezifikation.md section 9, ADR 0007).

ROS wrapper around core.mission.Mission and core.store.Store (the only writer of the database, D-28):
services /mission/create_order, /mission/order_action, /mission/ack_safety, /mission/resolve_alert,
/mission/software_estop, /mission/set_cart_location, /map/upsert_station, /map/delete_station; topics
/mission/status (status_rate_hz + on change), /mission/orders and /web/stations (on change + every
republish_period_s), /mission/alerts. Navigation goes to Nav2 (navigate_to_pose); cart handling to
/docking/engage_cart and /docking/release_cart (not implemented yet: the order is then DEFERRED with E305).
"""

from __future__ import annotations

import math
import threading
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose2D, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robotros_interfaces.action import EngageCart, ReleaseCart
from robotros_interfaces.msg import MissionStatus, OperatorAlert, OrderList, PowerState, SafetyState, StationList
from robotros_interfaces.msg import Order as OrderMsg
from robotros_interfaces.msg import Station as StationMsg
from robotros_interfaces.srv import (
    AckSafety,
    CreateOrder,
    DeleteStation,
    OrderAction,
    ResolveAlert,
    SetArm,
    SetCartLocation,
    UpsertStation,
)
from std_srvs.srv import Trigger

from .core.mission import (
    Alert,
    CancelDocking,
    CancelNav,
    Disarm,
    Engage,
    Mission,
    MissionConfig,
    Navigate,
    Release,
    Result,
)
from .core.store import Station, Store, StoreError


def yaw_to_quat(yaw: float) -> tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class MissionManager(Node):
    def __init__(self) -> None:
        super().__init__("mission_manager")
        p = {
            "db_path": "/var/lib/robotros/robotros.db",
            "mode": "NORMAL",
            "map_yaml": "",
            "min_order_v": 11.8,
            "nav_timeout_s": 180.0,
            "turn_distance": 0.70,
            "tick_hz": 5.0,
            "status_rate_hz": 2.0,
            "republish_period_s": 5.0,
            "localized_timeout_s": 2.0,
            "free_threshold": 25,
            "arm_timeout_s": 1.0,
            "server_wait_s": 2.0,
        }
        for k, v in p.items():
            self.declare_parameter(k, v)
        g = self.get_parameter
        self.lock = threading.RLock()
        self.store = Store(g("db_path").value)
        mode = str(g("mode").value).upper()
        map_yaml = str(g("map_yaml").value)
        if mode != "MAPPING":
            name = Path(map_yaml).stem if map_yaml else "default"
            self.store.activate_map(name, map_yaml)
        cfg = MissionConfig(g("min_order_v").value, g("nav_timeout_s").value, g("turn_distance").value)
        self.mission = Mission(self.store, cfg, mode)
        self.free_threshold = int(g("free_threshold").value)
        self.localized_timeout = float(g("localized_timeout_s").value)
        self.arm_timeout = float(g("arm_timeout_s").value)
        self.server_wait = float(g("server_wait_s").value)
        self.last_pose_time: float | None = None
        self.nav_handle = None
        self.dock_handle = None
        self.last_status: tuple | None = None

        cb = ReentrantCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_status = self.create_publisher(MissionStatus, "mission/status", reliable)
        self.pub_orders = self.create_publisher(OrderList, "mission/orders", latched)
        self.pub_alerts = self.create_publisher(OperatorAlert, "mission/alerts", reliable)
        self.pub_stations = self.create_publisher(StationList, "web/stations", latched)
        self.create_subscription(SafetyState, "hw/safety", self.on_safety, reliable, callback_group=cb)
        self.create_subscription(PowerState, "hw/power", self.on_power, reliable, callback_group=cb)
        self.create_subscription(OccupancyGrid, "map", self.on_map, latched, callback_group=cb)
        self.create_subscription(PoseStamped, "robot_pose", self.on_pose, 10, callback_group=cb)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=cb)
        self.engage = ActionClient(self, EngageCart, "docking/engage_cart", callback_group=cb)
        self.release = ActionClient(self, ReleaseCart, "docking/release_cart", callback_group=cb)
        self.set_arm = self.create_client(SetArm, "hw/set_arm", callback_group=cb)
        self.create_service(CreateOrder, "mission/create_order", self.on_create_order, callback_group=cb)
        self.create_service(OrderAction, "mission/order_action", self.on_order_action, callback_group=cb)
        self.create_service(AckSafety, "mission/ack_safety", self.on_ack_safety, callback_group=cb)
        self.create_service(ResolveAlert, "mission/resolve_alert", self.on_resolve_alert, callback_group=cb)
        self.create_service(Trigger, "mission/software_estop", self.on_software_estop, callback_group=cb)
        self.create_service(SetCartLocation, "mission/set_cart_location", self.on_set_cart, callback_group=cb)
        self.create_service(UpsertStation, "map/upsert_station", self.on_upsert_station, callback_group=cb)
        self.create_service(DeleteStation, "map/delete_station", self.on_delete_station, callback_group=cb)
        self.create_timer(1.0 / g("tick_hz").value, self.on_tick, callback_group=self.timer_group)
        self.create_timer(1.0 / g("status_rate_hz").value, lambda: self.publish_status(force=True),
                          callback_group=self.timer_group)
        self.create_timer(g("republish_period_s").value, self.republish, callback_group=self.timer_group)
        self.get_logger().info(f"mission_manager: mode {mode}, database {g('db_path').value}")

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ---- inputs ---------------------------------------------------------------------------------
    def on_safety(self, msg: SafetyState) -> None:
        with self.lock:
            self.mission.set_safety(msg.state == SafetyState.ARMED)

    def on_power(self, msg: PowerState) -> None:
        with self.lock:
            self.mission.v_batt = float(msg.v_batt)

    def on_pose(self, _msg: PoseStamped) -> None:
        self.last_pose_time = self.now()

    def on_map(self, msg: OccupancyGrid) -> None:
        info, data, thr = msg.info, msg.data, self.free_threshold
        ox, oy, res, w, h = info.origin.position.x, info.origin.position.y, info.resolution, info.width, info.height

        def free(x: float, y: float) -> bool:
            col, row = int(math.floor((x - ox) / res)), int(math.floor((y - oy) / res))
            if not (0 <= col < w and 0 <= row < h):
                return False
            v = data[row * w + col]
            return 0 <= v <= thr

        with self.lock:
            self.mission.set_map(True, free)

    # ---- services -------------------------------------------------------------------------------
    def on_create_order(self, req: CreateOrder.Request, res: CreateOrder.Response) -> CreateOrder.Response:
        with self.lock:
            g = req.goal
            try:
                res.success, res.order_id, res.error_code, res.detail = self.mission.create_order(
                    req.type, int(req.priority), int(req.source_station_id), int(req.dest_station_id),
                    (g.x, g.y, g.theta))
            except StoreError as e:
                res.success, res.error_code, res.detail = False, "E401", str(e)
            self.after_change()
        return res

    def on_order_action(self, req: OrderAction.Request, res: OrderAction.Response) -> OrderAction.Response:
        with self.lock:
            res.success, res.error_code, res.detail = self.mission.order_action(int(req.order_id), req.action,
                                                                                req.value)
            self.after_change()
        return res

    def on_resolve_alert(self, req: ResolveAlert.Request, res: ResolveAlert.Response) -> ResolveAlert.Response:
        with self.lock:
            res.success, res.error_code = self.mission.resolve_alert(int(req.alert_id), req.option)
            self.after_change()
        return res

    def on_software_estop(self, _req: Trigger.Request, res: Trigger.Response) -> Trigger.Response:
        with self.lock:
            self.mission.software_estop()
            self.execute(self.mission.tick(self.now()))
        ok = self.call_arm(False)
        res.success = ok is not None
        res.message = "disarmed" if res.success else "no answer from /hw/set_arm (E102)"
        if not res.success:
            self.get_logger().error("software E-stop: /hw/set_arm did not answer")
        return res

    def on_ack_safety(self, req: AckSafety.Request, res: AckSafety.Response) -> AckSafety.Response:
        if req.resume_mode not in ("RESUME", "KEEP_PAUSED", "CANCEL_ORDER"):
            res.success, res.error_code = False, "E309"
            return res
        state = self.call_arm(True)
        if state != SafetyState.ARMED:
            res.success, res.error_code = False, "E101" if state == SafetyState.ESTOP else "E102"
            return res
        with self.lock:
            self.mission.set_safety(True)
            self.mission.ack(req.resume_mode)
            self.after_change()
        res.success = True
        return res

    def call_arm(self, arm: bool) -> int | None:
        """Call /hw/set_arm and wait up to arm_timeout_s; returns the reported SafetyState.state or None."""
        if not self.set_arm.wait_for_service(timeout_sec=self.arm_timeout):
            return None
        done = threading.Event()
        future = self.set_arm.call_async(SetArm.Request(arm=arm))
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(self.arm_timeout + 1.0) or future.result() is None:
            return None
        return int(future.result().state)

    def on_set_cart(self, req: SetCartLocation.Request, res: SetCartLocation.Response) -> SetCartLocation.Response:
        with self.lock:
            if not req.cart_name:
                res.success, res.error_code = False, "E309"
            elif req.station_id and self.store.station(req.station_id) is None:
                res.success, res.error_code = False, "E304"
            elif req.station_id and self.store.station_in_use(req.station_id):
                res.success, res.error_code = False, "E309"
            else:
                try:
                    self.store.set_cart_location(req.cart_name, int(req.station_id))
                    res.success = True
                except StoreError:
                    res.success, res.error_code = False, "E401"
                self.mission.stations_changed = True
            self.after_change()
        return res

    def on_upsert_station(self, req: UpsertStation.Request, res: UpsertStation.Response) -> UpsertStation.Response:
        s = req.station
        with self.lock:
            map_id = self.store.active_map_id()
            if not map_id:
                res.success, res.error_code = False, "E310"
                return res
            if s.kind not in ("STORAGE", "DOCK") or not s.name.strip():
                res.success, res.error_code = False, "E309"
                return res
            if s.id and self.store.station(s.id) is None:
                res.success, res.error_code = False, "E304"
                return res
            if s.id and self.store.station_in_use(s.id):
                res.success, res.error_code = False, "E309"
                return res
            approach = float(s.approach_distance) if s.approach_distance > 0 else 0.40
            try:
                sid = self.store.upsert_station(Station(int(s.id), map_id, s.name.strip(), s.kind, s.pose.x,
                                                        s.pose.y, s.pose.theta, approach, bool(s.enabled)))
                if s.kind == "STORAGE" and s.occupied:
                    self.store.set_cart_location(s.cart_name or f"cart@{s.name.strip()}", sid)
                elif s.kind == "STORAGE" and not s.occupied:
                    current = self.store.station(sid)
                    if current is not None and current.cart_name:
                        self.store.set_cart_location(current.cart_name, 0)
                res.success, res.station_id = True, sid
            except StoreError as e:
                res.success, res.error_code = False, "E401"
                self.get_logger().error(f"upsert_station: {e}")
            self.mission.stations_changed = True
            self.after_change()
        return res

    def on_delete_station(self, req: DeleteStation.Request, res: DeleteStation.Response) -> DeleteStation.Response:
        with self.lock:
            if self.store.station(req.station_id) is None:
                res.success, res.error_code = False, "E304"
            elif self.store.station_in_use(req.station_id):
                res.success, res.error_code = False, "E309"
            else:
                try:
                    self.store.delete_station(int(req.station_id))
                    res.success = True
                except StoreError:
                    res.success, res.error_code = False, "E401"
                self.mission.stations_changed = True
            self.after_change()
        return res

    # ---- cycle ----------------------------------------------------------------------------------
    def on_tick(self) -> None:
        with self.lock:
            self.mission.localized = (self.last_pose_time is not None
                                      and self.now() - self.last_pose_time < self.localized_timeout)
            # A command can finish at once (e.g. no action server): tick again so that its result, the alert
            # and the new state go out in this cycle and not 1/tick_hz later.
            for _ in range(3):
                commands = self.mission.tick(self.now())
                if not commands:
                    break
                self.execute(commands)
            self.after_change()

    def after_change(self) -> None:
        if self.mission.orders_changed:
            self.mission.orders_changed = False
            self.publish_orders()
        if self.mission.stations_changed:
            self.mission.stations_changed = False
            self.publish_stations()
        self.publish_status(force=False)

    def republish(self) -> None:
        with self.lock:
            self.publish_orders()
            self.publish_stations()

    def execute(self, commands: list) -> None:
        for c in commands:
            if isinstance(c, Navigate):
                self.send_nav(c)
            elif isinstance(c, CancelNav):
                if self.nav_handle is not None:
                    self.nav_handle.cancel_goal_async()
                    self.nav_handle = None
            elif isinstance(c, Engage | Release):
                self.send_docking(c)
            elif isinstance(c, CancelDocking):
                if self.dock_handle is not None:
                    self.dock_handle.cancel_goal_async()
                    self.dock_handle = None
            elif isinstance(c, Disarm):
                pass  # the software E-stop service calls /hw/set_arm itself
            elif isinstance(c, Alert):
                msg = OperatorAlert(id=c.id, severity=c.severity, code=c.code, detail=c.detail,
                                    needs_action=c.needs_action, options=c.options)
                self.pub_alerts.publish(msg)
                # rclpy refuses one call site with changing severities ("Logger severity cannot be changed").
                if c.severity == "ERROR":
                    self.get_logger().error(f"alert {c.code}: {c.detail}")
                else:
                    self.get_logger().warning(f"alert {c.code}: {c.detail}")

    def send_nav(self, c: Navigate) -> None:
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.pose.position.x, goal.pose.pose.position.y = c.x, c.y
        yaw = 0.0 if math.isnan(c.yaw) else c.yaw
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = yaw_to_quat(yaw)
        seq = c.seq
        if not self.nav.wait_for_server(timeout_sec=self.server_wait):
            self.mission.nav_result(seq, Result(False, "E203", "navigation is not running"))
            return

        def on_accept(fut) -> None:  # noqa: ANN001
            handle = fut.result()
            with self.lock:
                if handle is None or not handle.accepted:
                    self.mission.nav_result(seq, Result(False, "E203", "goal rejected by Nav2"))
                    return
                if seq != self.mission.seq:
                    handle.cancel_goal_async()  # stopped while the goal was on its way
                    return
                self.nav_handle = handle
            handle.get_result_async().add_done_callback(lambda f: on_result(f))

        def on_result(fut) -> None:  # noqa: ANN001
            r = fut.result()
            with self.lock:
                if r.status == GoalStatus.STATUS_SUCCEEDED:
                    self.mission.nav_result(seq, Result(True))
                elif r.status != GoalStatus.STATUS_CANCELED:
                    code = int(getattr(r.result, "error_code", 0))
                    self.mission.nav_result(seq, Result(False, "E203", f"navigation failed (Nav2 error {code})"))
                if self.nav_handle is not None and seq == self.mission.seq:
                    self.nav_handle = None
                self.after_change()

        self.nav.send_goal_async(goal).add_done_callback(on_accept)

    def send_docking(self, c: Engage | Release) -> None:
        client = self.engage if isinstance(c, Engage) else self.release
        goal = EngageCart.Goal() if isinstance(c, Engage) else ReleaseCart.Goal()
        goal.station_id = c.station_id
        seq = c.seq
        if not client.wait_for_server(timeout_sec=self.server_wait):
            self.mission.docking_result(seq, Result(False, "", "cart handling (docking) is not running"),
                                        started=False)
            return

        def on_accept(fut) -> None:  # noqa: ANN001
            handle = fut.result()
            with self.lock:
                if handle is None or not handle.accepted:
                    self.mission.docking_result(seq, Result(False, "", "goal rejected"), started=False)
                    return
                if seq != self.mission.seq:
                    handle.cancel_goal_async()
                    return
                self.dock_handle = handle
            handle.get_result_async().add_done_callback(lambda f: on_result(f))

        def on_result(fut) -> None:  # noqa: ANN001
            r = fut.result()
            with self.lock:
                ok = r.status == GoalStatus.STATUS_SUCCEEDED and r.result.success
                if r.status != GoalStatus.STATUS_CANCELED:
                    self.mission.docking_result(seq, Result(ok, r.result.error_code, ""))
                self.dock_handle = None
                self.after_change()

        client.send_goal_async(goal).add_done_callback(on_accept)

    # ---- outputs --------------------------------------------------------------------------------
    def publish_status(self, force: bool) -> None:
        with self.lock:
            s = self.mission.status()
            key = tuple(s.values())
            if not force and key == self.last_status:
                return
            self.last_status = key
            msg = MissionStatus()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.mode, msg.state, msg.step = s["mode"], s["state"], s["step"]
            msg.active_order_id = s["active_order_id"]
            msg.progress = float(s["progress"])
            msg.message_key = f"mission.state.{s['state']}"
            msg.error_code = s["error_code"]
            msg.lifted = s["lifted"]
            msg.loc_sigma_xy = -1.0  # not measured yet
            msg.waiting_reason = s["waiting_reason"]
            msg.queue_length = s["queue_length"]
            self.pub_status.publish(msg)

    def publish_orders(self) -> None:
        msg = OrderList()
        for o in self.store.orders():
            m = OrderMsg(id=o.id, type=o.type, priority=o.priority, source_station_id=o.source_station_id,
                         dest_station_id=o.dest_station_id, state=o.state, step=o.step, error_code=o.error_code,
                         error_detail=o.error_detail, attempts=min(o.attempts, 255), created_at=o.created_at,
                         origin=o.origin)
            m.goal = Pose2D(x=0.0 if math.isnan(o.goal_x) else o.goal_x, y=0.0 if math.isnan(o.goal_y) else o.goal_y,
                            theta=0.0 if math.isnan(o.goal_yaw) else o.goal_yaw)
            msg.orders.append(m)
        self.pub_orders.publish(msg)

    def publish_stations(self) -> None:
        msg = StationList()
        for s in self.store.stations():
            msg.stations.append(StationMsg(id=s.id, name=s.name, kind=s.kind, pose=Pose2D(x=s.x, y=s.y, theta=s.yaw),
                                           approach_distance=s.approach_distance, enabled=s.enabled,
                                           occupied=s.occupied, cart_name=s.cart_name))
        self.pub_stations.publish(msg)


def main() -> None:
    rclpy.init()
    node = MissionManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
