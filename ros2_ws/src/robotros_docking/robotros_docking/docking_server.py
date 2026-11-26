"""docking_server, part 1: /docking/engage_cart and /docking/release_cart (ÄM-01 3.1/3.2, ADR 0009).

The maneuver logic is in core/maneuver.py. This node: slot check with the raw /scan at P_pre (D-37), then the
maneuver at control_hz on /cmd_vel_docking (twist_mux B, priority over the collision monitor output), the hub over
/hw/cmd/hub, the scan mask on /mission/scan_mask (latched, D-30) and the buzzer. Station poses come from
/web/stations (mission_manager). Map pose from TF map -> base_footprint, odometry from /odom.
"""

from __future__ import annotations

import math
import threading

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from robotros_interfaces.action import EngageCart, ReleaseCart
from robotros_interfaces.msg import BuzzerCommand, HubCommand, HubState, SafetyState, StationList
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener

from .core.maneuver import CartManeuver, DockingConfig, Pose, Station, slot_points

PHASE_PROGRESS = {"SLOT_CHECK": 0.05, "TURN_AROUND": 0.15, "REVERSE_IN": 0.4, "LIFT": 0.7, "LOWER": 0.7,
                  "EXIT_FWD": 0.85, "DONE": 1.0}


def yaw_of(q) -> float:  # noqa: ANN001
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class DockingServer(Node):
    def __init__(self) -> None:
        super().__init__("docking_server")
        defaults = DockingConfig()
        for k, v in defaults.__dict__.items():
            self.declare_parameter(k, v)
        extra = {
            "control_hz": 20.0,
            "map_frame": "map",
            "base_frame": "base_footprint",
            "slot_margin_m": 0.04,
            "release_slot_margin_m": 0.0,
            "cart_half_size_m": 0.20,
            "slot_free_max_points": 3,
            "cart_present_min_points": 8,
            "cart_check_enabled": True,
            "range_max": 8.0,
            "hub_pwm": 255,
            "pose_max_age_s": 1.0,
        }
        for k, v in extra.items():
            self.declare_parameter(k, v)
        g = self.get_parameter
        self.cfg = DockingConfig(**{k: g(k).value for k in defaults.__dict__})
        self.p = {k: g(k).value for k in extra}
        self.lock = threading.Lock()
        self.stations: dict[int, Station] = {}
        self.odom: Pose | None = None
        self.v_meas = 0.0
        self.hub_state = HubState.IDLE
        self.armed = False
        self.scan: LaserScan | None = None
        self.mask = False
        self.busy = False

        cb = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub_cmd = self.create_publisher(Twist, "cmd_vel_docking", 10)
        self.pub_hub = self.create_publisher(HubCommand, "hw/cmd/hub", 10)
        self.pub_buzzer = self.create_publisher(BuzzerCommand, "hw/cmd/buzzer", 10)
        self.pub_mask = self.create_publisher(Bool, "mission/scan_mask", latched)
        self.create_subscription(StationList, "web/stations", self.on_stations, latched, callback_group=cb)
        self.create_subscription(Odometry, "odom", self.on_odom, 20, callback_group=cb)
        self.create_subscription(HubState, "hw/hub_state", self.on_hub, 10, callback_group=cb)
        self.create_subscription(SafetyState, "hw/safety", self.on_safety, 10, callback_group=cb)
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data, callback_group=cb)
        for name, action in (("docking/engage_cart", EngageCart), ("docking/release_cart", ReleaseCart)):
            ActionServer(self, action, name, execute_callback=self.execute, goal_callback=self.on_goal,
                         cancel_callback=lambda _g: CancelResponse.ACCEPT, callback_group=cb)
        self.publish_mask(False)
        self.get_logger().info("docking_server: engage_cart, release_cart")

    # ---- inputs ---------------------------------------------------------------------------------
    def on_stations(self, msg: StationList) -> None:
        with self.lock:
            self.stations = {s.id: Station(s.pose.x, s.pose.y, s.pose.theta, s.approach_distance or 0.40)
                             for s in msg.stations if s.kind == "STORAGE"}

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        with self.lock:
            self.odom = Pose(p.position.x, p.position.y, yaw_of(p.orientation))
            self.v_meas = msg.twist.twist.linear.x

    def on_hub(self, msg: HubState) -> None:
        self.hub_state = HubState.TOP if msg.limit_top else HubState.BOTTOM if msg.limit_bottom else msg.state
        if msg.state == HubState.FAULT:
            self.hub_state = HubState.FAULT

    def on_safety(self, msg: SafetyState) -> None:
        self.armed = msg.state == SafetyState.ARMED

    def on_scan(self, msg: LaserScan) -> None:
        self.scan = msg

    def map_pose(self) -> Pose | None:
        try:
            tf = self.tf_buffer.lookup_transform(self.p["map_frame"], self.p["base_frame"], Time(),
                                                 timeout=Duration(seconds=0.0))
        except TransformException:
            return None
        age = (self.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
        if age > self.p["pose_max_age_s"]:
            return None
        t = tf.transform
        return Pose(t.translation.x, t.translation.y, yaw_of(t.rotation))

    def publish_mask(self, on: bool) -> None:
        self.mask = on
        self.pub_mask.publish(Bool(data=on))

    # ---- action ---------------------------------------------------------------------------------
    def on_goal(self, _goal) -> GoalResponse:  # noqa: ANN001
        return GoalResponse.REJECT if self.busy else GoalResponse.ACCEPT

    def execute(self, handle):  # noqa: ANN001, ANN201
        kind = "ENGAGE" if isinstance(handle.request, EngageCart.Goal) else "RELEASE"
        result = EngageCart.Result() if kind == "ENGAGE" else ReleaseCart.Result()
        feedback = EngageCart.Feedback() if kind == "ENGAGE" else ReleaseCart.Feedback()
        fail_code = "E305" if kind == "ENGAGE" else "E306"
        self.busy = True
        try:
            with self.lock:
                st = self.stations.get(int(handle.request.station_id))
            if st is None:
                result.success, result.error_code = False, "E304"
                handle.abort()
                return result
            # SLOT_CHECK (D-37) at P_pre with the raw scan.
            feedback.phase, feedback.progress = "SLOT_CHECK", PHASE_PROGRESS["SLOT_CHECK"]
            handle.publish_feedback(feedback)
            code = self.slot_check(kind, st)
            if code:
                self.get_logger().warning(f"{kind} station {handle.request.station_id}: slot check {code}")
                result.success, result.error_code = False, code
                handle.abort()
                return result
            m = CartManeuver(kind, st, self.cfg, carrying=(kind == "RELEASE"))
            rate = self.create_rate(self.p["control_hz"])  # ROS clock (sim time in the simulation)
            last_phase = ""
            while rclpy.ok():
                if handle.is_cancel_requested:
                    self.stop()
                    self.publish_mask(m.mask_after())
                    handle.canceled()
                    result.success, result.error_code = False, fail_code
                    return result
                with self.lock:
                    odom, v = self.odom, self.v_meas
                if odom is None:
                    rate.sleep()
                    continue
                now = self.get_clock().now().nanoseconds * 1e-9
                out = m.step(now, self.map_pose(), odom, v, self.hub_state, self.armed)
                self.pub_cmd.publish(Twist(linear=self._vec(out.v), angular=self._vec(0.0, out.w)))
                if out.hub is not None:
                    self.pub_hub.publish(HubCommand(command=out.hub, pwm=int(self.p["hub_pwm"])))
                if out.mask != self.mask:
                    self.publish_mask(out.mask)
                if out.phase != last_phase:
                    last_phase = out.phase
                    self.pub_buzzer.publish(BuzzerCommand(pattern=out.buzzer))
                    self.get_logger().info(f"{kind} station {handle.request.station_id}: {out.phase}")
                    feedback.phase, feedback.progress = out.phase, PHASE_PROGRESS.get(out.phase, 0.0)
                    handle.publish_feedback(feedback)
                if m.result is not None:
                    break
                rate.sleep()
            self.destroy_rate(rate)
            self.stop()
            self.publish_mask(m.mask_after())
            ok, code, detail = m.result if m.result is not None else (False, fail_code, "shutdown")
            if not ok:
                self.get_logger().warning(f"{kind} station {handle.request.station_id} failed: {code} {detail}")
            result.success, result.error_code = ok, code
            if ok:
                handle.succeed()
            else:
                handle.abort()
            return result
        finally:
            self.busy = False

    def slot_check(self, kind: str, st: Station) -> str:
        """'' = ok, else the error code (D-37)."""
        scan, robot = self.scan, self.map_pose()
        if scan is None or robot is None:
            return ""  # no data: the maneuver's own stall detection still protects
        margin = self.p["slot_margin_m"] if kind == "ENGAGE" else self.p["release_slot_margin_m"]
        n = slot_points(list(scan.ranges), scan.angle_min, scan.angle_increment, robot, st,
                        self.p["cart_half_size_m"] + margin, self.p["range_max"])
        self.get_logger().info(f"slot check {kind}: {n} scan points in the slot")
        if kind == "RELEASE" and n > self.p["slot_free_max_points"]:
            return "E302"
        if kind == "ENGAGE" and self.p["cart_check_enabled"] and n < self.p["cart_present_min_points"]:
            return "E301"
        return ""

    @staticmethod
    def _vec(x: float, z: float = 0.0):  # noqa: ANN205
        from geometry_msgs.msg import Vector3

        return Vector3(x=float(x), y=0.0, z=float(z))

    def stop(self) -> None:
        self.pub_cmd.publish(Twist())
        self.pub_buzzer.publish(BuzzerCommand(pattern=BuzzerCommand.PATTERN_OFF))


def main() -> None:
    rclpy.init()
    node = DockingServer()
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
