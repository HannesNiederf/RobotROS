"""Differential-drive kinematics and wheel odometry for nano_bridge (spec FW section 2.6, D-03, D-24).

Pure logic. Geometry comes from robot.yaml; no values are hard-coded here except the D-24 limit.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

UNCALIBRATED_SPEED_LIMIT = 0.05  # m/s per wheel while geometry_calibrated is false (D-24)


@dataclass(frozen=True)
class Geometry:
    wheel_radius_m: float
    track_width_m: float
    ticks_per_wheel_rev: int
    wheel_radius_scale: float = 1.0
    track_scale: float = 1.0
    calibrated: bool = False

    @classmethod
    def from_robot_yaml(cls, cfg: dict) -> Geometry:
        return cls(
            wheel_radius_m=float(cfg["wheel_radius_m"]),
            track_width_m=float(cfg["track_width_m"]),
            ticks_per_wheel_rev=int(cfg["ticks_per_wheel_rev"]),
            wheel_radius_scale=float(cfg.get("wheel_radius_scale", 1.0)),
            track_scale=float(cfg.get("track_scale", 1.0)),
            calibrated=bool(cfg.get("geometry_calibrated", False)),
        )

    @property
    def radius(self) -> float:
        return self.wheel_radius_m * self.wheel_radius_scale

    @property
    def track(self) -> float:
        return self.track_width_m * self.track_scale

    @property
    def metres_per_tick(self) -> float:
        return 2.0 * math.pi * self.radius / self.ticks_per_wheel_rev


def twist_to_tps(v: float, w: float, geom: Geometry) -> tuple[int, int]:
    """Body velocity (m/s, rad/s) to wheel setpoints in ticks/s for ``$VEL``.

    While the geometry is not calibrated, both wheel speeds are scaled down together so that neither
    exceeds UNCALIBRATED_SPEED_LIMIT (keeps the path shape, D-24).
    """
    v_left = v - w * geom.track / 2.0
    v_right = v + w * geom.track / 2.0
    if not geom.calibrated:
        peak = max(abs(v_left), abs(v_right))
        if peak > UNCALIBRATED_SPEED_LIMIT:
            scale = UNCALIBRATED_SPEED_LIMIT / peak
            v_left *= scale
            v_right *= scale
    k = 1.0 / geom.metres_per_tick
    return round(v_left * k), round(v_right * k)


def wrap_i32(delta: int) -> int:
    """Difference of two int32 counters, correct across overflow."""
    return (delta + 2**31) % 2**32 - 2**31


def wrap_u32(delta: int) -> int:
    """Difference of two uint32 millisecond stamps (``t_ms``), correct across overflow."""
    return delta % 2**32


@dataclass
class OdomState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v: float = 0.0
    w: float = 0.0
    left_rad: float = 0.0  # wheel angle for joint_states
    right_rad: float = 0.0


class Odometry:
    """Integrates ``$ODO`` telegrams (cumulative ticks, Nano time) with midpoint integration."""

    def __init__(self, geom: Geometry) -> None:
        self.geom = geom
        self.state = OdomState()
        self._last: tuple[int, int, int] | None = None

    def reset_reference(self) -> None:
        """Forget the last sample (after a Nano reset its counters restart); the pose is kept."""
        self._last = None

    def update(self, t_ms: int, l_ticks: int, r_ticks: int) -> OdomState:
        if self._last is None:
            self._last = (t_ms, l_ticks, r_ticks)
            return self.state
        t0, l0, r0 = self._last
        self._last = (t_ms, l_ticks, r_ticks)
        dt = wrap_u32(t_ms - t0) / 1000.0
        dl = wrap_i32(l_ticks - l0) * self.geom.metres_per_tick
        dr = wrap_i32(r_ticks - r0) * self.geom.metres_per_tick
        ds = (dl + dr) / 2.0
        dyaw = (dr - dl) / self.geom.track
        s = self.state
        mid = s.yaw + dyaw / 2.0
        s.x += ds * math.cos(mid)
        s.y += ds * math.sin(mid)
        s.yaw = math.atan2(math.sin(s.yaw + dyaw), math.cos(s.yaw + dyaw))
        if dt > 0:
            s.v = ds / dt
            s.w = dyaw / dt
        per_tick = 2.0 * math.pi / self.geom.ticks_per_wheel_rev
        s.left_rad += wrap_i32(l_ticks - l0) * per_tick
        s.right_rad += wrap_i32(r_ticks - r0) * per_tick
        return s


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """(x, y, z, w) of a rotation about z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)
