"""Hardware model behind ``sim_hw_adapter`` (softwarespezifikation.md section 14). No ROS imports.

The Nano side is the same ``FakeNano`` (role ALL) that tests ``nano_bridge``: its safety state machine,
watchdogs, direction locks, cliff confirmation and hub logic are tested row by row against
``firmware/test/vectors/safety_fsm.csv``. This module talks protocol v1 to it in-process, the way
``nano_bridge`` would over USB, and adds what the real robot gets from physics:

- the wheel speeds the Nano allows (after safety, locks and its PT1 lag) become the body twist that
  Gazebo's diff drive executes (``wheel_twist``),
- ultrasonic distances from the ground under the sensors (cliff polygons of the world = no echo),
- a 12 V lead-acid battery with open-circuit voltage curve, internal resistance and charging at the dock.

Not in here: odometry (Gazebo publishes it) and the D-24 speed cap of the bridge (the simulated
geometry is exact, so the model treats it as calibrated).
"""

from __future__ import annotations

import bisect
import math
import xml.etree.ElementTree as ET
from collections.abc import Sequence
from dataclasses import dataclass, field

from robotros_bridge.core import convert
from robotros_bridge.core import protocol as p
from robotros_bridge.core.fake_nano import FakeNano
from robotros_bridge.core.kinematics import Geometry, twist_to_tps
from robotros_bridge.core.safety import NO_ECHO_MM, SafetyState

Polygon = Sequence[tuple[float, float]]


def point_in_polygon(x: float, y: float, poly: Polygon) -> bool:
    """Even-odd rule; points on an edge may go either way."""
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            x_cross = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < x_cross:
                inside = not inside
    return inside


def cliff_polygons_from_sdf(sdf_text: str, prefix: str = "cliff") -> list[list[tuple[float, float]]]:
    """Rectangles of the world models whose name starts with ``prefix`` (pose x y yaw, first box visual)."""
    world = ET.fromstring(sdf_text).find("world")
    polys: list[list[tuple[float, float]]] = []
    for model in world.findall("model") if world is not None else []:
        if not str(model.get("name", "")).startswith(prefix):
            continue
        mx, my, _, _, _, myaw = (float(v) for v in model.findtext("pose", "0 0 0 0 0 0").split())
        box = model.find(".//visual")
        size = box.findtext("geometry/box/size") if box is not None else None
        if size is None:
            continue
        vx, vy, _, _, _, vyaw = (float(v) for v in box.findtext("pose", "0 0 0 0 0 0").split())
        sx, sy = (float(v) for v in size.split()[:2])
        c, s_ = math.cos(myaw), math.sin(myaw)
        cx, cy, yaw = mx + c * vx - s_ * vy, my + s_ * vx + c * vy, myaw + vyaw
        cy_, sy_ = math.cos(yaw), math.sin(yaw)
        corners = [(-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2)]
        polys.append([(cx + cy_ * u - sy_ * v, cy + sy_ * u + cy_ * v) for u, v in corners])
    return polys


@dataclass
class BatteryConfig:
    capacity_ah: float
    initial_soc: float
    r_internal_ohm: float
    ocv_curve: list[tuple[float, float]]  # (soc 0..1, open-circuit volts), ascending soc
    idle_current_a: float
    drive_current_a_per_mps: float
    hub_current_a: float
    charger_v: float
    charge_limit_a: float
    charge_taper_soc: float

    @classmethod
    def from_dict(cls, d: dict) -> BatteryConfig:
        curve = sorted((float(s), float(v)) for s, v in d["ocv_curve"])
        return cls(
            capacity_ah=float(d["capacity_ah"]),
            initial_soc=float(d["initial_soc"]),
            r_internal_ohm=float(d["r_internal_ohm"]),
            ocv_curve=curve,
            idle_current_a=float(d["idle_current_a"]),
            drive_current_a_per_mps=float(d["drive_current_a_per_mps"]),
            hub_current_a=float(d["hub_current_a"]),
            charger_v=float(d["charger_v"]),
            charge_limit_a=float(d["charge_limit_a"]),
            charge_taper_soc=float(d["charge_taper_soc"]),
        )


class Battery:
    """Lead-acid battery: OCV(soc) by linear interpolation, terminal voltage = OCV -/+ I * R."""

    def __init__(self, cfg: BatteryConfig) -> None:
        self.cfg = cfg
        self.soc = min(1.0, max(0.0, cfg.initial_soc))
        self.v_terminal = self.ocv()
        self.i_charge = 0.0

    def ocv(self) -> float:
        curve = self.cfg.ocv_curve
        socs = [s for s, _ in curve]
        i = bisect.bisect_left(socs, self.soc)
        if i <= 0:
            return curve[0][1]
        if i >= len(curve):
            return curve[-1][1]
        (s0, v0), (s1, v1) = curve[i - 1], curve[i]
        return v0 + (v1 - v0) * (self.soc - s0) / (s1 - s0)

    def step(self, dt_s: float, load_a: float, charging: bool) -> None:
        """Charging: current limit, then a linear taper to 0 A at full charge (stands in for the CV phase).

        The load is supplied by the charger while charging (the charger current is the battery current).
        """
        cfg = self.cfg
        self.i_charge = 0.0
        if charging:
            taper = (1.0 - self.soc) / max(1e-6, 1.0 - cfg.charge_taper_soc)
            self.i_charge = cfg.charge_limit_a * max(0.0, min(1.0, taper))
            net_a = self.i_charge
        else:
            net_a = -load_a
        self.soc = min(1.0, max(0.0, self.soc + net_a * dt_s / 3600.0 / cfg.capacity_ah))
        if charging:
            in_cv = self.i_charge < cfg.charge_limit_a
            self.v_terminal = cfg.charger_v if in_cv else min(cfg.charger_v, self.ocv() + net_a * cfg.r_internal_ohm)
        else:
            self.v_terminal = self.ocv() + net_a * cfg.r_internal_ohm


@dataclass
class SimConfig:
    geometry: Geometry
    nano_cfg: dict[str, int]  # firmware.yaml drive + aux
    bridge_cfg: dict  # bridge.yaml (rates, timeouts, voltage conversion)
    battery: BatteryConfig
    us_ground_x: float  # ground point of the front sensor in base_footprint (rear: mirrored)
    cliff_polygons: list[Polygon] = field(default_factory=list)  # world frame
    dock_xy: tuple[float, float] = (0.0, 0.0)  # world frame
    dock_radius: float = 0.1
    motor_tau_ms: int = 100
    hub_travel_ms: int = 3000


@dataclass
class Outputs:
    """Messages from the simulated Nano since the last ``take_outputs``."""

    safety: list[convert.SafetyValues] = field(default_factory=list)
    sensors: list[p.Message] = field(default_factory=list)  # $SEN
    power: list[convert.PowerValues] = field(default_factory=list)
    hub: list[tuple[int, bool, bool, int]] = field(default_factory=list)
    events: list[p.Message] = field(default_factory=list)


class SimHardware:
    def __init__(self, cfg: SimConfig, now_ms: int) -> None:
        self.cfg = cfg
        self.geom = Geometry(
            wheel_radius_m=cfg.geometry.wheel_radius_m,
            track_width_m=cfg.geometry.track_width_m,
            ticks_per_wheel_rev=cfg.geometry.ticks_per_wheel_rev,
            wheel_radius_scale=cfg.geometry.wheel_radius_scale,
            track_scale=cfg.geometry.track_scale,
            calibrated=True,
        )
        bc = cfg.bridge_cfg
        self.vel_period_ms = 1000.0 / float(bc["vel_rate_hz"])
        self.ping_period_ms = 1000.0 / float(bc["ping_rate_hz"])
        self.hub_period_ms = 1000.0 / float(bc["hub_rate_hz"])
        self.cmd_timeout_ms = float(bc["cmd_vel_timeout_s"]) * 1000.0
        self.hub_cmd_timeout_ms = float(bc["hub_cmd_timeout_s"]) * 1000.0
        self.battery = Battery(cfg.battery)
        self.nano = FakeNano(role=p.Role.ALL, motor_tau_ms=cfg.motor_tau_ms, hub_travel_ms=cfg.hub_travel_ms)
        self._parser = p.StreamParser()
        self._acks: list[p.Message] = []
        self.out = Outputs()
        self.cmd = (0.0, 0.0)
        self.cmd_ms = -1e18
        self.hub_cmd: int | None = None
        self.hub_cmd_ms = -1e18
        self.pose_world = (0.0, 0.0, 0.0)
        self.last_ms = now_ms
        self.seq = 0
        self.vel_seq = 0
        self.hub_pwm = 0
        self.nano.power_on(now_ms)
        for key, value in cfg.nano_cfg.items():
            self._send(now_ms, "CFG", key=key, value=int(value))
        self._update_inputs(0.0)
        self._collect()
        self._acks.clear()
        self.next_vel = self.next_ping = self.next_hub = float(now_ms)

    # ------------------------------------------------------------------ protocol
    def _send(self, now_ms: int, name: str, **fields: int | str) -> None:
        self.nano.feed(now_ms, p.encode(name, **fields))

    def _collect(self) -> None:
        for msg in self._parser.feed(self.nano.read_output()):
            if isinstance(msg, p.ProtocolError):
                continue
            if msg.name in ("ACK", "ERR"):
                self._acks.append(msg)
            elif msg.name == "SAF":
                self.out.safety.append(convert.safety_from_saf(msg))
            elif msg.name == "SEN":
                self.out.sensors.append(msg)
            elif msg.name == "PWR":
                self.out.power.append(convert.power_from_pwr(msg, self.cfg.bridge_cfg))
            elif msg.name == "HUBST":
                self.out.hub.append(convert.hub_from_hubst(msg))
            elif msg.name == "EVT":
                self.out.events.append(msg)

    def take_outputs(self) -> Outputs:
        out, self.out = self.out, Outputs()
        return out

    def _command_reply(self, now_ms: int, name: str, **fields: int | str) -> p.Message | None:
        self._acks.clear()
        self._send(now_ms, name, **fields)
        self._collect()
        return self._acks[-1] if self._acks else None

    # ------------------------------------------------------------------ commands
    def set_cmd_vel(self, now_ms: int, v: float, w: float) -> None:
        self.cmd = (v, w)
        self.cmd_ms = now_ms

    def set_hub(self, now_ms: int, command: int, pwm: int) -> None:
        self.hub_cmd = command
        self.hub_pwm = pwm
        self.hub_cmd_ms = now_ms
        self._send(now_ms, "HUB", command=command, pwm=pwm)
        self._collect()

    def arm(self, now_ms: int, arm: bool) -> tuple[bool, int]:
        """Like /hw/set_arm: success when the Nano is ARMED (arm) or no longer ARMED (disarm)."""
        reply = self._command_reply(now_ms, "ARM", arm=int(arm))
        state = self.safety_state
        ok = reply is not None and reply.name == "ACK" and ((state == SafetyState.ARMED) == arm)
        return ok, state

    def set_relay(self, now_ms: int, channel: int, on: bool) -> tuple[bool, str]:
        reply = self._command_reply(now_ms, "REL", channel=channel, on=int(on))
        if reply is None:
            return False, "no reply"
        return reply.name == "ACK", "ok" if reply.name == "ACK" else f"ERR {reply['code']}"

    def set_estop(self, now_ms: int, pressed: bool) -> None:
        self.nano.set_estop(now_ms, pressed)
        self._collect()

    def set_bumper(self, mask: int) -> None:
        self.nano.set_bumper(mask)

    def set_pose_world(self, x: float, y: float, yaw: float) -> None:
        self.pose_world = (x, y, yaw)

    @property
    def safety_state(self) -> int:
        return int(self.nano.safety.state)

    # ------------------------------------------------------------------ world
    def ground_point(self, front: bool) -> tuple[float, float]:
        x, y, yaw = self.pose_world
        d = self.cfg.us_ground_x if front else -self.cfg.us_ground_x
        return x + d * math.cos(yaw), y + d * math.sin(yaw)

    def us_mm(self, front: bool) -> int:
        gx, gy = self.ground_point(front)
        if any(point_in_polygon(gx, gy, poly) for poly in self.cfg.cliff_polygons):
            return NO_ECHO_MM
        return int(self.cfg.nano_cfg.get("US_FLOOR_F" if front else "US_FLOOR_R", 120))

    def at_dock(self) -> bool:
        x, y, _ = self.pose_world
        return math.hypot(x - self.cfg.dock_xy[0], y - self.cfg.dock_xy[1]) <= self.cfg.dock_radius

    def charging(self) -> bool:
        return self.at_dock() and bool(self.nano.relays.get(1))

    def _update_inputs(self, dt_s: float) -> None:
        self.nano.set_us_mm(front=self.us_mm(True), rear=self.us_mm(False))
        v, _ = self.wheel_twist()
        load = self.battery.cfg.idle_current_a + abs(v) * self.battery.cfg.drive_current_a_per_mps
        if self.nano.hub_state in (1, 2):  # moving up or down
            load += self.battery.cfg.hub_current_a
        self.battery.step(dt_s, load, self.charging())
        bc = self.cfg.bridge_cfg

        def adc_mv(volts: float, key: str) -> int:
            raw = (volts - float(bc["voltage_offset"][key])) / float(bc["voltage_gain"][key])
            return max(0, round(raw / float(bc["divider_ratio"]) * 1000.0))

        v_batt = self.battery.v_terminal
        v_chg = self.battery.cfg.charger_v if self.at_dock() else 0.0
        v_rail = v_batt if self.nano.safety.arming else 0.0
        self.nano.set_power(
            adc_mv(v_batt, "batt"), adc_mv(v_chg, "chg"), adc_mv(v_rail, "rail"), round(self.battery.i_charge * 1000)
        )

    # ------------------------------------------------------------------ time
    def step(self, now_ms: int) -> None:
        dt_ms = now_ms - self.last_ms
        if dt_ms < 0:
            raise ValueError("time went backwards")
        self.last_ms = now_ms
        if now_ms >= self.next_vel:
            v, w = self.cmd if now_ms - self.cmd_ms <= self.cmd_timeout_ms else (0.0, 0.0)
            l_tps, r_tps = twist_to_tps(v, w, self.geom)
            self.vel_seq = (self.vel_seq + 1) % 65536
            self._send(now_ms, "VEL", seq=self.vel_seq, l_tps=l_tps, r_tps=r_tps)
            self.next_vel = max(self.next_vel + self.vel_period_ms, float(now_ms))
        if now_ms >= self.next_ping:
            self.seq = (self.seq + 1) % 65536
            self._send(now_ms, "PING", seq=self.seq)
            self.next_ping = max(self.next_ping + self.ping_period_ms, float(now_ms))
        if self.hub_cmd and now_ms >= self.next_hub:
            if now_ms - self.hub_cmd_ms <= self.hub_cmd_timeout_ms:
                self._send(now_ms, "HUB", command=self.hub_cmd, pwm=self.hub_pwm)
            self.next_hub = max(self.next_hub + self.hub_period_ms, float(now_ms))
        self._update_inputs(dt_ms / 1000.0)
        self.nano.step(now_ms)
        self._collect()
        self._acks.clear()

    def wheel_twist(self) -> tuple[float, float]:
        """Body twist (m/s, rad/s) from the wheel speeds the simulated Nano drives."""
        mpt = self.geom.metres_per_tick
        v_l, v_r = self.nano.tps[0] * mpt, self.nano.tps[1] * mpt
        return (v_l + v_r) / 2.0, (v_r - v_l) / self.geom.track


def load_sim_config(robot_cfg: dict, firmware_cfg: dict, bridge_cfg: dict, sim_cfg: dict, world_sdf: str) -> SimConfig:
    """SimConfig from the parsed YAML files and the world SDF text."""
    return SimConfig(
        geometry=Geometry.from_robot_yaml(robot_cfg),
        nano_cfg=convert.cfg_for_role(firmware_cfg, p.Role.ALL),
        bridge_cfg=bridge_cfg,
        battery=BatteryConfig.from_dict(sim_cfg["battery"]),
        us_ground_x=float(robot_cfg["us_ground_x"]),
        cliff_polygons=cliff_polygons_from_sdf(world_sdf),
        dock_xy=(float(sim_cfg["dock"]["xy"][0]), float(sim_cfg["dock"]["xy"][1])),
        dock_radius=float(sim_cfg["dock"]["radius"]),
        motor_tau_ms=int(sim_cfg["motor_tau_ms"]),
        hub_travel_ms=int(sim_cfg["hub_travel_ms"]),
    )
