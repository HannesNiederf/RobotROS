"""nano_bridge: ROS 2 node between the Nano boards (serial, protocol v1) and the /hw/* topics.

softwarespezifikation.md 3.2, 5.2, 5.3; spez_firmware_protokoll.md 2.6. Logic lives in core/; this file
only does serial I/O, ROS publishing and parameters. Time for logic is monotonic.
"""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field

import rclpy
import serial
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry as OdometryMsg
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robotros_interfaces.msg import (
    BumperState,
    BuzzerCommand,
    HubCommand,
    HubState,
    LedCommand,
    PowerState,
    RangeState,
    SafetyState,
)
from robotros_interfaces.srv import SetArm, SetRelay
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

from .core import convert
from .core import protocol as p
from .core.kinematics import Geometry, Odometry, twist_to_tps, yaw_to_quaternion
from .core.link import DeviceLink, LinkEvents, LinkState, LinkTimings


def mono_ms() -> int:
    return time.monotonic_ns() // 1_000_000


@dataclass
class Device:
    cfg: convert.DeviceConfig
    link: DeviceLink
    port: serial.Serial | None = None
    next_open_ms: int = 0
    acks: list[p.Message] = field(default_factory=list)


class NanoBridge(Node):
    def __init__(self, **node_kwargs: object) -> None:
        super().__init__("nano_bridge", **node_kwargs)
        try:
            share = get_package_share_directory("robotros_bringup")
        except PackageNotFoundError:
            share = "."  # only usable with explicit *_yaml parameters
        self.declare_parameter("bridge_yaml", f"{share}/config/bridge.yaml")
        self.declare_parameter("robot_yaml", f"{share}/config/robot.yaml")
        self.declare_parameter("firmware_yaml", f"{share}/config/firmware.yaml")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")

        self.bridge_cfg = convert.load_yaml(self.get_parameter("bridge_yaml").value)
        robot_cfg = convert.load_yaml(self.get_parameter("robot_yaml").value)
        firmware_cfg = convert.load_yaml(self.get_parameter("firmware_yaml").value)
        self.geom = Geometry.from_robot_yaml(robot_cfg)
        self.latency_s = float(robot_cfg.get("serial_latency_s", 0.0))
        self.cov_xy = float(robot_cfg.get("odom_cov_xy", 0.02))
        self.cov_yaw = float(robot_cfg.get("odom_cov_yaw", 0.05))
        self.odom = Odometry(self.geom)
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        bc = self.bridge_cfg
        timings = LinkTimings(
            hello_timeout_ms=int(float(bc["hello_timeout_s"]) * 1000),
            cfg_ack_timeout_ms=int(float(bc["cfg_ack_timeout_s"]) * 1000),
            link_timeout_ms=int(float(bc["link_timeout_s"]) * 1000),
        )
        self.reconnect_ms = int(float(bc["reconnect_period_s"]) * 1000)
        self.devices: dict[str, Device] = {}
        for dc in convert.devices_from_bridge_cfg(bc):
            link = DeviceLink(dc.name, dc.role, convert.cfg_for_role(firmware_cfg, dc.role), timings)
            self.devices[dc.name] = Device(dc, link)
        if not self.geom.calibrated:
            self.get_logger().warn("geometry_calibrated is false: wheel speed limited to 0.05 m/s (D-24)")

        self.lock = threading.Lock()
        self.cmd_vel = (0.0, 0.0)
        self.cmd_vel_ms = -(10**9)
        self.vel_seq = 0
        self.ping_seq = 0
        self.hub_cmd: HubCommand | None = None
        self.hub_cmd_ms = -(10**9)
        self.last_safety: convert.SafetyValues | None = None
        self.last_hub_state = 0

        io_group = MutuallyExclusiveCallbackGroup()
        srv_group = ReentrantCallbackGroup()
        self.pub_odom = self.create_publisher(OdometryMsg, "odom", qos_profile_sensor_data)
        self.pub_joints = self.create_publisher(JointState, "joint_states", qos_profile_sensor_data)
        self.pub_safety = self.create_publisher(SafetyState, "hw/safety", 10)
        self.pub_bumper = self.create_publisher(BumperState, "hw/bumper", 10)
        self.pub_range = self.create_publisher(RangeState, "hw/range", qos_profile_sensor_data)
        self.pub_power = self.create_publisher(PowerState, "hw/power", 10)
        self.pub_hub = self.create_publisher(HubState, "hw/hub_state", 10)
        self.tf = TransformBroadcaster(self) if self.get_parameter("publish_tf").value else None
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10, callback_group=io_group)
        self.create_subscription(HubCommand, "hw/cmd/hub", self.on_hub, 10, callback_group=io_group)
        self.create_subscription(LedCommand, "hw/cmd/led", self.on_led, 10, callback_group=io_group)
        self.create_subscription(BuzzerCommand, "hw/cmd/buzzer", self.on_buzzer, 10, callback_group=io_group)
        self.create_service(SetArm, "hw/set_arm", self.on_set_arm, callback_group=srv_group)
        self.create_service(SetRelay, "hw/set_relay", self.on_set_relay, callback_group=srv_group)
        self.create_timer(0.005, self.io_step, callback_group=io_group)
        self.create_timer(1.0 / float(bc["vel_rate_hz"]), self.send_vel, callback_group=io_group)
        self.create_timer(1.0 / float(bc["ping_rate_hz"]), self.send_ping, callback_group=io_group)
        self.create_timer(1.0 / float(bc["hub_rate_hz"]), self.send_hub, callback_group=io_group)

    # ------------------------------------------------------------- routing
    def _device_with(self, role: p.Role) -> Device | None:
        for dev in self.devices.values():
            if dev.cfg.role in (role, p.Role.ALL):
                return dev
        return None

    def _command(self, role: p.Role, name: str, **fields: int | str) -> bool:
        dev = self._device_with(role)
        if dev is None:
            self.get_logger().warn(f"no device for role {role.name}: {name} dropped", throttle_duration_sec=5.0)
            return False
        with self.lock:
            return dev.link.send_command(name, **fields)

    # ------------------------------------------------------------------- io
    def io_step(self) -> None:
        now = mono_ms()
        for dev in self.devices.values():
            self._io_device(dev, now)

    def _io_device(self, dev: Device, now: int) -> None:
        link = dev.link
        if dev.port is None:
            if now < dev.next_open_ms or link.state == LinkState.INCOMPATIBLE:
                return
            try:
                dev.port = serial.Serial(dev.cfg.port, dev.cfg.baud, timeout=0, write_timeout=0.05)
            except (serial.SerialException, OSError) as err:
                dev.next_open_ms = now + self.reconnect_ms
                self.get_logger().warn(f"{dev.cfg.name}: cannot open {dev.cfg.port}: {err}", throttle_duration_sec=10.0)
                return
            with self.lock:
                link.connect(now)
            self.get_logger().info(f"{dev.cfg.name}: opened {dev.cfg.port}")
        try:
            data = dev.port.read(dev.port.in_waiting or 1)
        except (serial.SerialException, OSError) as err:
            self._close(dev, now, f"read error: {err}")
            return
        with self.lock:
            events = [link.feed(now, data), link.tick(now)]
            out = link.take_output()
        if out:
            try:
                dev.port.write(out)
            except (serial.SerialException, OSError) as err:
                self._close(dev, now, f"write error: {err}")
                return
        for ev in events:
            self._handle_events(dev, ev)

    def _close(self, dev: Device, now: int, reason: str) -> None:
        self.get_logger().error(f"{dev.cfg.name}: link lost ({reason}), E103")
        try:
            if dev.port is not None:
                dev.port.close()
        finally:
            dev.port = None
            dev.next_open_ms = now + self.reconnect_ms
            with self.lock:
                dev.link.disconnect()
            self._publish_link_lost(dev)

    def _handle_events(self, dev: Device, ev: LinkEvents) -> None:
        log = self.get_logger()
        for warning in ev.warnings:
            log.warn(warning, throttle_duration_sec=2.0)
        if dev.link.error_ratio > float(self.bridge_cfg["checksum_warn_ratio"]) and dev.link.frames_bad >= 5:
            log.warn(f"{dev.cfg.name}: telegram error rate {dev.link.error_ratio:.1%}", throttle_duration_sec=10.0)
        if ev.incompatible:
            log.error(f"E104 {ev.incompatible}")
        if ev.nano_reset:
            log.error(f"{dev.cfg.name}: unexpected HELLO, Nano was reset; treated as BOOT_LOCKED")
            if dev.cfg.role in (p.Role.DRIVE, p.Role.ALL):
                self.odom.reset_reference()
        if ev.became_ready:
            log.info(f"{dev.cfg.name}: ready (fw {dev.link.fw_version}, caps 0x{dev.link.caps:02X})")
        if ev.lost:
            self._close(dev, mono_ms(), f"no data for {dev.link.t.link_timeout_ms} ms")
        for msg in ev.messages:
            self._publish(dev, msg)

    # -------------------------------------------------------------- publish
    def _stamp(self) -> Time:
        now = self.get_clock().now().nanoseconds - int(self.latency_s * 1e9)
        return Time(sec=now // 1_000_000_000, nanosec=now % 1_000_000_000)

    def _publish(self, dev: Device, msg: p.Message) -> None:
        name = msg.name
        if name in ("ACK", "ERR"):
            with self.lock:
                dev.acks.append(msg)
                del dev.acks[:-20]
            return
        stamp = self._stamp()
        if name == "ODO":
            self._publish_odom(stamp, msg)
        elif name == "SAF":
            values = convert.safety_from_saf(msg)
            self.last_safety = values
            self._publish_safety(stamp, values)
        elif name == "SEN":
            left, middle, right = convert.bumper_from_mask(int(msg["bump"]))
            self.pub_bumper.publish(BumperState(header=self._header(stamp), left=left, middle=middle, right=right))
            self.pub_range.publish(
                RangeState(
                    header=self._header(stamp),
                    front_mm=int(msg["us_front_mm"]),
                    rear_mm=int(msg["us_rear_mm"]),
                    front_class=int(msg["cls_front"]),
                    rear_class=int(msg["cls_rear"]),
                )
            )
        elif name == "PWR":
            pv = convert.power_from_pwr(msg, self.bridge_cfg)
            self.pub_power.publish(
                PowerState(
                    header=self._header(stamp),
                    v_batt=pv.v_batt,
                    v_charger=pv.v_charger,
                    v_rail=pv.v_rail,
                    i_charge=pv.i_charge,
                    charger_present=pv.charger_present,
                    charge_relay_on=False,  # chosen here: unknown until a relay state telegram exists
                    soc_estimate=-1.0,
                )
            )
        elif name == "HUBST":
            state, top, bottom, fault = convert.hub_from_hubst(msg)
            self.last_hub_state = state
            self.pub_hub.publish(
                HubState(header=self._header(stamp), state=state, limit_top=top, limit_bottom=bottom, fault=fault)
            )
        elif name == "EVT":
            self.get_logger().info(f"{dev.cfg.name}: EVT {msg['type']} arg {msg['arg']}")

    def _header(self, stamp: Time, frame: str = "base_link") -> Header:
        return Header(stamp=stamp, frame_id=frame)

    def _publish_safety(self, stamp: Time, v: convert.SafetyValues) -> None:
        self.pub_safety.publish(
            SafetyState(
                header=self._header(stamp),
                state=v.state,
                estop_pressed=v.estop_pressed,
                relay_on=v.relay_on,
                block_forward=v.block_forward,
                block_backward=v.block_backward,
                pi_link_lost=v.pi_link_lost,
                software_stop=v.software_stop,
                flags=v.flags,
            )
        )

    def _publish_link_lost(self, dev: Device) -> None:
        if dev.cfg.role not in (p.Role.DRIVE, p.Role.ALL):
            return
        last = self.last_safety
        values = convert.SafetyValues(
            state=last.state if last else SafetyState.BOOT_LOCKED,
            estop_pressed=last.estop_pressed if last else False,
            relay_on=False,
            block_forward=last.block_forward if last else False,
            block_backward=last.block_backward if last else False,
            pi_link_lost=True,
            software_stop=last.software_stop if last else False,
            flags=last.flags if last else 0,
        )
        self._publish_safety(self._stamp(), values)

    def _publish_odom(self, stamp: Time, msg: p.Message) -> None:
        s = self.odom.update(int(msg["t_ms"]), int(msg["l_ticks"]), int(msg["r_ticks"]))
        qx, qy, qz, qw = yaw_to_quaternion(s.yaw)
        odom = OdometryMsg()
        odom.header = self._header(stamp, self.odom_frame)
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = s.x
        odom.pose.pose.position.y = s.y
        odom.pose.pose.orientation.x, odom.pose.pose.orientation.y = qx, qy
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
        odom.twist.twist.linear.x = s.v
        odom.twist.twist.angular.z = s.w
        cov = [0.0] * 36
        cov[0] = cov[7] = self.cov_xy
        cov[35] = self.cov_yaw
        for i in (14, 21, 28):
            cov[i] = 1e6  # z, roll, pitch not measured
        odom.pose.covariance = cov
        odom.twist.covariance = cov
        self.pub_odom.publish(odom)
        joints = JointState()
        joints.header = self._header(stamp)
        joints.name = ["left_wheel_joint", "right_wheel_joint"]  # chosen here, must match the URDF
        joints.position = [s.left_rad, s.right_rad]
        joints.velocity = [
            int(msg["l_tps"]) * 2 * math.pi / self.geom.ticks_per_wheel_rev,
            int(msg["r_tps"]) * 2 * math.pi / self.geom.ticks_per_wheel_rev,
        ]
        self.pub_joints.publish(joints)
        if self.tf is not None:
            tf = TransformStamped()
            tf.header = odom.header
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = s.x
            tf.transform.translation.y = s.y
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf.sendTransform(tf)

    # ------------------------------------------------------------- commands
    def on_cmd_vel(self, msg: Twist) -> None:
        self.cmd_vel = (float(msg.linear.x), float(msg.angular.z))
        self.cmd_vel_ms = mono_ms()

    def send_vel(self) -> None:
        v, w = self.cmd_vel
        if mono_ms() - self.cmd_vel_ms > float(self.bridge_cfg["cmd_vel_timeout_s"]) * 1000:
            v, w = 0.0, 0.0
        left, right = twist_to_tps(v, w, self.geom)
        self.vel_seq = (self.vel_seq + 1) % 2**32
        self._command(p.Role.DRIVE, "VEL", seq=self.vel_seq, l_tps=left, r_tps=right)

    def send_ping(self) -> None:
        dev = self._device_with(p.Role.AUX)
        if dev is not None and dev.cfg.role == p.Role.AUX:
            self.ping_seq = (self.ping_seq + 1) % 2**32
            self._command(p.Role.AUX, "PING", seq=self.ping_seq)

    def on_hub(self, msg: HubCommand) -> None:
        self.hub_cmd = msg
        self.hub_cmd_ms = mono_ms()
        self.send_hub()

    def send_hub(self) -> None:
        cmd = self.hub_cmd
        if cmd is None:
            return
        expired = mono_ms() - self.hub_cmd_ms > float(self.bridge_cfg["hub_cmd_timeout_s"]) * 1000
        done = (
            (cmd.command == HubCommand.UP and self.last_hub_state == HubState.TOP)
            or (cmd.command == HubCommand.DOWN and self.last_hub_state == HubState.BOTTOM)
            or self.last_hub_state == HubState.FAULT
        )
        if cmd.command == HubCommand.STOP or expired or done:
            self._command(p.Role.AUX, "HUB", command=0, pwm=0)
            self.hub_cmd = None
            return
        self._command(p.Role.AUX, "HUB", command=int(cmd.command), pwm=int(cmd.pwm))

    def on_led(self, msg: LedCommand) -> None:
        self._command(p.Role.AUX, "LED", strip=msg.strip, pattern=msg.pattern, r=msg.r, g=msg.g, b=msg.b)

    def on_buzzer(self, msg: BuzzerCommand) -> None:
        self._command(p.Role.AUX, "BUZ", pattern=msg.pattern)

    # ------------------------------------------------------------- services
    def _wait_for(self, predicate: Callable[[], bool], timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.005)
        return predicate()

    def on_set_arm(self, request: SetArm.Request, response: SetArm.Response) -> SetArm.Response:
        wanted_armed = bool(request.arm)
        sent = self._command(p.Role.DRIVE, "ARM", arm=int(wanted_armed))
        timeout = float(self.bridge_cfg["arm_timeout_s"])

        def reached() -> bool:
            s = self.last_safety
            return s is not None and (s.state == SafetyState.ARMED) == wanted_armed

        response.success = sent and self._wait_for(reached, timeout)
        response.state = self.last_safety.state if self.last_safety else SafetyState.BOOT_LOCKED
        if not response.success:
            self.get_logger().warn(f"set_arm({wanted_armed}) failed (E102), state {response.state}")
        return response

    def on_set_relay(self, request: SetRelay.Request, response: SetRelay.Response) -> SetRelay.Response:
        dev = self._device_with(p.Role.AUX)
        if dev is None or not 1 <= request.channel <= 3:
            response.success, response.message = False, "no AUX device or invalid channel"
            return response
        with self.lock:
            dev.acks.clear()
        if not self._command(p.Role.AUX, "REL", channel=int(request.channel), on=int(request.on)):
            response.success, response.message = False, "AUX link not ready"
            return response

        def acked() -> bool:
            with self.lock:
                return any(m.name == "ACK" and m["command"] == "REL" for m in dev.acks)

        response.success = self._wait_for(acked, 0.5)
        response.message = "ok" if response.success else "no ACK within 0.5 s"
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = NanoBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
