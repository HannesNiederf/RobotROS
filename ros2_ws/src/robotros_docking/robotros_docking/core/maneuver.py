"""Cart maneuvers EngageCart and ReleaseCart (spez_aenderung_01_rueckwaerts.md 3.1 / 3.2, ADR 0009).

Pure Python, no ROS. The node calls step() at control_hz with the robot pose in the map (AMCL, may be None), the
pose in the odom frame, the measured forward speed, the hub state and the time (ROS clock). step() returns the
velocity command, hub command, scan mask and buzzer pattern, and the current phase; `result` is set at the end.

Phases: TURN_AROUND -> REVERSE_IN (R1 free, R2 guided) -> LIFT | LOWER -> EXIT_FWD -> DONE.
The slot check (D-37) is done by the node before the maneuver starts (slot_points below).
Distances are measured with the odometry (spec 10: never by time); time only bounds the hub and the stall check.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# HubState.state
HUB_TOP, HUB_BOTTOM, HUB_FAULT = 3, 4, 5
# HubCommand.command
HUB_STOP, HUB_UP, HUB_DOWN = 0, 1, 2
# BuzzerCommand.pattern
BUZ_OFF, BUZ_REVERSE, BUZ_SHORT = 0, 1, 2


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class DockingConfig:
    """docking.yaml (ÄM-01 section 5)."""

    turn_distance: float = 0.70
    reverse_fast_speed: float = 0.10
    reverse_slow_speed: float = 0.05
    exit_fwd_speed: float = 0.08
    turn_speed: float = 0.30
    turn_speed_cart: float = 0.25
    turn_tolerance: float = 0.05
    engage_stop_tol: float = 0.01
    engage_hard_limit_extra: float = 0.03
    stall_speed: float = 0.005       # m/s, measured speed below this counts as stalled (stall_tps converted)
    stall_time_s: float = 0.5
    stall_ok_distance: float = 0.04  # a stall this close to S counts as end stop (3.1 step 4c)
    reverse_k_theta: float = 2.0
    reverse_k_y: float = 8.0   # spec start value 1.5 corrects 4 cm only over ~1.3 m; see ADR 0009
    max_omega_reverse: float = 0.30
    hub_timeout_s: float = 6.0
    min_turn_omega: float = 0.08


@dataclass
class Pose:
    x: float
    y: float
    yaw: float


@dataclass
class Station:
    x: float
    y: float
    yaw: float
    approach_distance: float


@dataclass
class Output:
    v: float = 0.0
    w: float = 0.0
    hub: int | None = None   # HubCommand to send this cycle, None = nothing
    mask: bool = False
    buzzer: int = BUZ_OFF
    phase: str = ""


def axis_errors(p: Pose, st: Station) -> tuple[float, float, float]:
    """(s, e_y, e_theta) of a pose relative to the station axis: s = distance from S along û (positive = outside
    the box), e_y = lateral offset (left of û positive), e_theta = heading error to û."""
    ux, uy = math.cos(st.yaw), math.sin(st.yaw)
    dx, dy = p.x - st.x, p.y - st.y
    return dx * ux + dy * uy, -dx * uy + dy * ux, wrap(p.yaw - st.yaw)


def slot_points(ranges: list[float], angle_min: float, angle_inc: float, robot: Pose, st: Station,
                half_size: float, range_max: float, back_trim: float = 0.0) -> int:
    """Number of scan points inside the slot around S (D-37): ±half_size across, from -(half_size - back_trim)
    (back of the place, away from the robot) to +half_size along û. back_trim keeps a wall right behind the
    place out of the count."""
    n = 0
    c, s = math.cos(st.yaw), math.sin(st.yaw)
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r <= 0.0 or r >= range_max:
            continue
        a = robot.yaw + angle_min + i * angle_inc
        px, py = robot.x + r * math.cos(a) - st.x, robot.y + r * math.sin(a) - st.y
        lx, ly = px * c + py * s, -px * s + py * c
        if -(half_size - back_trim) <= lx <= half_size and abs(ly) <= half_size:
            n += 1
    return n


class CartManeuver:
    def __init__(self, kind: str, st: Station, cfg: DockingConfig, carrying: bool) -> None:
        assert kind in ("ENGAGE", "RELEASE")
        self.kind = kind
        self.st = st
        self.cfg = cfg
        self.carrying = carrying       # cart on board (RELEASE): slower turn, mask already on
        self.phase = "TURN_AROUND"
        self.result: tuple[bool, str, str] | None = None
        self.phase_start_odom: Pose | None = None
        self.phase_start_t = 0.0
        self.r2_start_odom: Pose | None = None
        self.stall_since: float | None = None
        self.turn_dir = 0.0
        self.lift_started = False

    @property
    def fail_code(self) -> str:
        return "E305" if self.kind == "ENGAGE" else "E306"

    def _enter(self, phase: str, odom: Pose, t: float) -> None:
        self.phase = phase
        self.phase_start_odom = Pose(odom.x, odom.y, odom.yaw)
        self.phase_start_t = t
        self.stall_since = None

    def _fail(self, detail: str) -> Output:
        self.result = (False, self.fail_code, detail)
        self.phase = "FAILED"
        return Output(mask=self.mask_after(), phase="FAILED")

    def mask_after(self) -> bool:
        """Scan mask once the maneuver has ended (D-30): on while a cart is or may be on board."""
        if self.result is not None and self.result[0]:
            return self.kind == "ENGAGE"
        if self.kind == "RELEASE":
            return True  # failed before the cart was down: still on board
        return self.lift_started  # ENGAGE failed: masked if the lift may have moved

    def _odom_dist(self, odom: Pose) -> float:
        p0 = self.phase_start_odom or odom
        return math.hypot(odom.x - p0.x, odom.y - p0.y)

    def _stalled(self, v_cmd: float, v_meas: float, t: float) -> bool:
        if abs(v_cmd) > 1e-3 and abs(v_meas) < self.cfg.stall_speed and t - self.phase_start_t > 0.5:
            if self.stall_since is None:
                self.stall_since = t
            return t - self.stall_since >= self.cfg.stall_time_s
        self.stall_since = None
        return False

    def step(self, t: float, map_pose: Pose | None, odom: Pose, v_meas: float, hub_state: int,
             armed: bool = True) -> Output:
        cfg, st = self.cfg, self.st
        if self.result is not None:
            return Output(mask=self.mask_after(), phase=self.phase)
        if self.phase_start_odom is None:
            self._enter(self.phase, odom, t)
        if not armed:
            return self._fail("robot not armed during the cart maneuver")

        if self.phase == "TURN_AROUND":
            # Turn on the spot until the front points along û (away from the station).
            yaw = map_pose.yaw if map_pose is not None else odom.yaw
            target = st.yaw if map_pose is not None else wrap(self.phase_start_odom.yaw + math.pi)
            err = wrap(target - yaw)
            speed = cfg.turn_speed_cart if self.carrying else cfg.turn_speed
            if abs(err) <= cfg.turn_tolerance:
                self._enter("REVERSE_IN", odom, t)
                return Output(mask=True, buzzer=BUZ_REVERSE, phase="REVERSE_IN")
            if self.turn_dir == 0.0:
                self.turn_dir = 1.0  # counter-clockwise (3.1 step 3 default; cost-based side: not yet)
            if abs(err) > math.pi / 2:
                w = self.turn_dir * speed
            else:
                w = max(cfg.min_turn_omega, min(speed, 2.0 * abs(err))) * math.copysign(1.0, err)
            return Output(w=w, mask=self.carrying, buzzer=BUZ_SHORT, phase="TURN_AROUND")

        if self.phase == "REVERSE_IN":
            total = self._odom_dist(odom)
            r1_len = max(0.0, cfg.turn_distance - st.approach_distance)
            if map_pose is not None:
                s, e_y, e_th = axis_errors(map_pose, st)
            else:
                s, e_y, e_th = cfg.turn_distance - total, 0.0, 0.0  # odometry only (3.1 step 4 R2)
            in_r2 = s <= st.approach_distance or total >= r1_len
            if in_r2 and self.r2_start_odom is None:
                self.r2_start_odom = Pose(odom.x, odom.y, odom.yaw)
            travel_r2 = math.hypot(odom.x - self.r2_start_odom.x, odom.y - self.r2_start_odom.y) \
                if self.r2_start_odom is not None else 0.0
            if s <= cfg.engage_stop_tol or travel_r2 >= st.approach_distance + cfg.engage_hard_limit_extra:
                return self._end_reverse(odom, t)
            v = -(cfg.reverse_slow_speed if in_r2 else cfg.reverse_fast_speed)
            w = -cfg.reverse_k_theta * e_th + cfg.reverse_k_y * e_y
            w = max(-cfg.max_omega_reverse, min(cfg.max_omega_reverse, w))
            if self._stalled(v, v_meas, t):
                if s <= cfg.stall_ok_distance:
                    return self._end_reverse(odom, t)
                return self._fail(f"blocked {s:.2f} m before the end position")
            return Output(v=v, w=w, mask=True, buzzer=BUZ_REVERSE, phase="REVERSE_IN")

        if self.phase in ("LIFT", "LOWER"):
            want = HUB_TOP if self.phase == "LIFT" else HUB_BOTTOM
            if hub_state == want:
                if self.phase == "LOWER":
                    # Cart is down: mask off before driving out (3.2 step 6).
                    self._enter("EXIT_FWD", odom, t)
                    return Output(hub=HUB_STOP, mask=False, phase="EXIT_FWD")
                self._enter("EXIT_FWD", odom, t)
                return Output(hub=HUB_STOP, mask=True, phase="EXIT_FWD")
            if hub_state == HUB_FAULT or t - self.phase_start_t > cfg.hub_timeout_s:
                out = self._fail("lift did not reach its end position (E107)")
                out.hub = HUB_STOP
                return out
            return Output(hub=HUB_UP if self.phase == "LIFT" else HUB_DOWN, mask=True, phase=self.phase)

        if self.phase == "EXIT_FWD":
            mask = self.kind == "ENGAGE"
            if self._odom_dist(odom) >= st.approach_distance:
                self.result = (True, "", "")
                self.phase = "DONE"
                return Output(mask=mask, phase="DONE")
            w = 0.0
            if map_pose is not None:
                _, e_y, e_th = axis_errors(map_pose, st)
                w = max(-cfg.max_omega_reverse, min(cfg.max_omega_reverse,
                                                     -cfg.reverse_k_theta * e_th - cfg.reverse_k_y * e_y))
            return Output(v=cfg.exit_fwd_speed, w=w, mask=mask, phase="EXIT_FWD")

        return Output(mask=self.mask_after(), phase=self.phase)

    def _end_reverse(self, odom: Pose, t: float) -> Output:
        phase = "LIFT" if self.kind == "ENGAGE" else "LOWER"
        self.lift_started = True
        self._enter(phase, odom, t)
        return Output(hub=HUB_UP if phase == "LIFT" else HUB_DOWN, mask=True, phase=phase)
