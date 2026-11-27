"""Cart maneuvers against a simple unicycle model (ÄM-01 3.1/3.2, ADR 0009). No ROS needed."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robotros_docking.core.maneuver import (  # noqa: E402
    BUZ_REVERSE,
    HUB_BOTTOM,
    HUB_DOWN,
    HUB_FAULT,
    HUB_TOP,
    HUB_UP,
    CartManeuver,
    DockingConfig,
    Pose,
    Station,
    axis_errors,
    slot_points,
)

DT = 0.05


def run(m: CartManeuver, start: Pose, hub_reaches_after: float = 1.0, wall: float | None = None,
        t_max: float = 60.0, hub_fault: bool = False) -> tuple[Pose, list[str], list]:
    """Integrate the commands; the odom frame equals the map frame here. `wall`: along-axis position s at which
    the robot is blocked (stall)."""
    p = Pose(start.x, start.y, start.yaw)
    t, phases, outs = 0.0, [], []
    hub_state, hub_since = 0, None
    v_meas = 0.0
    while m.result is None and t < t_max:
        out = m.step(t, p, p, v_meas, hub_state)
        outs.append(out)
        if not phases or phases[-1] != out.phase:
            phases.append(out.phase)
        if out.hub in (HUB_UP, HUB_DOWN) and hub_since is None:
            hub_since = t
        if hub_since is not None and t - hub_since >= hub_reaches_after:
            hub_state = HUB_FAULT if hub_fault else (HUB_TOP if out.hub == HUB_UP or m.phase == "LIFT" else HUB_BOTTOM)
            hub_since = None
        nx = p.x + out.v * math.cos(p.yaw) * DT
        ny = p.y + out.v * math.sin(p.yaw) * DT
        if wall is not None and axis_errors(Pose(nx, ny, p.yaw), m.st)[0] < wall:
            nx, ny = p.x, p.y  # blocked
        v_meas = math.hypot(nx - p.x, ny - p.y) / DT * (1 if out.v >= 0 else -1)
        p = Pose(nx, ny, p.yaw + out.w * DT)
        t += DT
    return p, phases, outs


ST = Station(x=-3.25, y=0.0, yaw=0.0, approach_distance=0.40)


def p_pre(dx: float = 0.0, dy: float = 0.0, dyaw: float = 0.0) -> Pose:
    return Pose(ST.x + 0.70 + dx, ST.y + dy, math.pi + dyaw)


def test_axis_errors() -> None:
    s, e_y, e_th = axis_errors(Pose(-2.55, 0.02, 0.1), ST)
    assert (s, e_y, e_th) == pytest.approx((0.70, 0.02, 0.1))


def test_engage_reaches_station_and_lifts() -> None:
    m = CartManeuver("ENGAGE", ST, DockingConfig(), carrying=False)
    end, phases, outs = run(m, p_pre(0.02, 0.03, 0.08))
    assert m.result == (True, "", "")
    assert phases == ["TURN_AROUND", "REVERSE_IN", "LIFT", "EXIT_FWD", "DONE"]
    # the lift happened at S (within 1 cm along, 1 cm across), then 0.40 m forward to P_a
    lift = [o for o in outs if o.phase == "LIFT"]
    assert lift and lift[0].hub == HUB_UP
    s, e_y, e_th = axis_errors(end, ST)
    assert s == pytest.approx(0.40, abs=0.02)
    assert abs(e_y) < 0.01 and abs(e_th) < 0.05
    assert outs[-1].mask is True  # cart on board: mask stays on
    assert any(o.buzzer == BUZ_REVERSE for o in outs)
    assert all(o.v <= 0 for o in outs if o.phase == "REVERSE_IN")
    assert max(abs(o.w) for o in outs if o.phase == "REVERSE_IN") <= 0.30 + 1e-9


def test_lateral_error_is_corrected_while_reversing() -> None:
    m = CartManeuver("ENGAGE", ST, DockingConfig(), carrying=False)
    positions = []
    p = p_pre(0.0, 0.04, 0.0)
    t = 0.0
    while m.result is None and t < 60:
        out = m.step(t, p, p, out_v if t > 0 else 0.0, HUB_TOP if m.phase == "LIFT" else 0)  # noqa: F821
        out_v = out.v  # noqa: F841
        if out.phase == "REVERSE_IN":
            positions.append(axis_errors(p, ST))
        p = Pose(p.x + out.v * math.cos(p.yaw) * DT, p.y + out.v * math.sin(p.yaw) * DT, p.yaw + out.w * DT)
        t += DT
    s_end, e_y_end, _ = positions[-1]
    assert s_end <= 0.02 and abs(e_y_end) < 0.01  # 4 cm lateral offset removed before the end position


def test_release_lowers_and_mask_off() -> None:
    m = CartManeuver("RELEASE", ST, DockingConfig(), carrying=True)
    _, phases, outs = run(m, p_pre())
    assert m.result == (True, "", "")
    assert phases == ["TURN_AROUND", "REVERSE_IN", "LOWER", "EXIT_FWD", "DONE"]
    assert outs[0].mask is True and outs[-1].mask is False
    assert all(not o.mask for o in outs if o.phase == "EXIT_FWD")
    turn = [abs(o.w) for o in outs if o.phase == "TURN_AROUND"]
    assert max(turn) <= 0.25 + 1e-9  # slower turn with the cart


def test_stall_near_end_counts_as_end_stop_far_fails() -> None:
    m = CartManeuver("ENGAGE", ST, DockingConfig(), carrying=False)
    _, phases, _ = run(m, p_pre(), wall=0.03)
    assert m.result[0] and "LIFT" in phases
    m = CartManeuver("ENGAGE", ST, DockingConfig(), carrying=False)
    run(m, p_pre(), wall=0.20)
    assert m.result[0] is False and m.result[1] == "E305" and "blocked" in m.result[2]
    assert m.mask_after() is False  # nothing lifted


def test_hub_fault_and_not_armed() -> None:
    m = CartManeuver("RELEASE", ST, DockingConfig(), carrying=True)
    run(m, p_pre(), hub_fault=True)
    assert m.result[:2] == (False, "E306") and m.mask_after() is True
    m = CartManeuver("ENGAGE", ST, DockingConfig(), carrying=False)
    out = m.step(0.0, p_pre(), p_pre(), 0.0, 0, armed=False)
    assert m.result[0] is False and out.v == 0 and out.w == 0


def test_without_map_pose_uses_odometry() -> None:
    m = CartManeuver("ENGAGE", ST, DockingConfig(), carrying=False)
    p, t = p_pre(), 0.0
    while m.result is None and t < 60:
        out = m.step(t, None, p, out.v if t > 0 else 0.0, HUB_TOP if m.phase == "LIFT" else 0)  # noqa: F821
        p = Pose(p.x + out.v * math.cos(p.yaw) * DT, p.y + out.v * math.sin(p.yaw) * DT, p.yaw + out.w * DT)
        t += DT
    assert m.result[0]


def test_slot_points() -> None:
    robot = p_pre()  # 0.70 m in front of S, looking at it
    n = 720
    inc = 2 * math.pi / n
    ranges = [math.inf] * n
    # four feet at (±0.19, ±0.19) around S
    for fx, fy in ((0.19, 0.19), (0.19, -0.19), (-0.19, 0.19), (-0.19, -0.19)):
        wx, wy = ST.x + fx, ST.y + fy
        a = math.atan2(wy - robot.y, wx - robot.x) - robot.yaw
        i = round((a + math.pi) / inc) % n
        ranges[i] = math.hypot(wx - robot.x, wy - robot.y)
    assert slot_points(ranges, -math.pi, inc, robot, ST, 0.24, 8.0) == 4
    assert slot_points([math.inf] * n, -math.pi, inc, robot, ST, 0.24, 8.0) == 0


def test_slot_back_trim_ignores_a_wall_behind_the_place() -> None:
    robot = p_pre()
    n = 720
    inc = 2 * math.pi / n
    ranges = [math.inf] * n
    # wall points 0.22 m behind S (a few cm of pose error closer than the real 0.25 m)
    for k in range(-10, 11):
        wx, wy = ST.x - 0.22, ST.y + 0.02 * k
        a = math.atan2(wy - robot.y, wx - robot.x) - robot.yaw
        i = round((a + math.pi) / inc) % n
        ranges[i] = math.hypot(wx - robot.x, wy - robot.y)
    assert slot_points(ranges, -math.pi, inc, robot, ST, 0.20, 8.0) == 0  # outside 0.20 anyway
    ranges2 = [math.inf] * n
    for k in range(-10, 11):
        wx, wy = ST.x - 0.17, ST.y + 0.02 * k
        a = math.atan2(wy - robot.y, wx - robot.x) - robot.yaw
        ranges2[round((a + math.pi) / inc) % n] = math.hypot(wx - robot.x, wy - robot.y)
    assert slot_points(ranges2, -math.pi, inc, robot, ST, 0.20, 8.0) > 3
    assert slot_points(ranges2, -math.pi, inc, robot, ST, 0.20, 8.0, back_trim=0.06) == 0
