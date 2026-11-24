"""SimHardware (sim_hw_adapter core) without ROS: safety gating, locks, cliff, battery, hub."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
SRC = PKG.parent
sys.path[:0] = [str(PKG), str(SRC / "robotros_bridge")]

from robotros_bridge.core.safety import SafetyState  # noqa: E402
from robotros_sim.hw_model import (  # noqa: E402
    Battery,
    BatteryConfig,
    SimHardware,
    cliff_polygons_from_sdf,
    load_sim_config,
    point_in_polygon,
)

CONFIG = SRC / "robotros_bringup" / "config"


def load(name: str) -> dict:
    return yaml.safe_load((CONFIG / name).read_text())


SIM_CFG = yaml.safe_load((PKG / "config" / "sim_hw.yaml").read_text())
WORLD = (PKG / "worlds" / "generic.sdf").read_text()


def make(now: int = 0) -> SimHardware:
    cfg = load_sim_config(load("robot.yaml"), load("firmware.yaml"), load("bridge.yaml"), SIM_CFG, WORLD)
    return SimHardware(cfg, now)


def run(hw: SimHardware, t0: int, t1: int, v: float | None = None, w: float = 0.0, step: int = 10) -> int:
    t = t0
    while t < t1:
        t += step
        if v is not None:
            hw.set_cmd_vel(t, v, w)
        hw.step(t)
    return t


def armed(hw: SimHardware, t: int) -> int:
    t = run(hw, t, t + 100)
    ok, state = hw.arm(t, True)
    assert ok and state == SafetyState.ARMED
    return run(hw, t, t + 250)  # 200 ms drive delay after arming


def test_boot_locked_does_not_move() -> None:
    hw = make()
    run(hw, 0, 1000, v=0.2)
    assert hw.safety_state == SafetyState.BOOT_LOCKED
    assert hw.wheel_twist() == (0.0, 0.0)


def test_armed_follows_cmd_vel_with_lag() -> None:
    hw = make()
    t = armed(hw, 0)
    t = run(hw, t, t + 1000, v=0.2)
    v, w = hw.wheel_twist()
    assert v == pytest.approx(0.2, abs=0.005)
    assert w == pytest.approx(0.0, abs=1e-6)
    t = run(hw, t, t + 1000, v=0.0, w=0.5)
    v, w = hw.wheel_twist()
    assert v == pytest.approx(0.0, abs=0.005)
    assert w == pytest.approx(0.5, abs=0.02)


def test_cmd_vel_timeout_stops() -> None:
    hw = make()
    t = armed(hw, 0)
    t = run(hw, t, t + 500, v=0.2)
    t = run(hw, t, t + 1500)  # no more cmd_vel: bridge timeout 0.5 s -> zero setpoints
    assert hw.wheel_twist()[0] == pytest.approx(0.0, abs=1e-9)
    assert hw.safety_state == SafetyState.ARMED  # watchdog fed by VEL/PING, nothing tripped


def test_disarm_brakes() -> None:
    hw = make()
    t = armed(hw, 0)
    t = run(hw, t, t + 500, v=0.2)
    ok, state = hw.arm(t, False)
    assert ok and state == SafetyState.WAIT_ACK
    run(hw, t, t + 20, v=0.2)
    assert hw.wheel_twist() == (0.0, 0.0)


def test_estop_stops_and_needs_release_and_arm() -> None:
    hw = make()
    t = armed(hw, 0)
    t = run(hw, t, t + 500, v=0.2)
    hw.set_estop(t, True)
    t = run(hw, t, t + 20, v=0.2)
    assert hw.safety_state == SafetyState.ESTOP and hw.wheel_twist() == (0.0, 0.0)
    ok, _ = hw.arm(t, True)
    assert not ok  # ERR5 while pressed
    hw.set_estop(t, False)
    assert hw.safety_state == SafetyState.WAIT_ACK
    t = armed(hw, t)
    t = run(hw, t, t + 500, v=0.2)
    assert hw.wheel_twist()[0] > 0.1


def test_cliff_brakes_and_blocks_forward_only() -> None:
    hw = make()
    poly = hw.cfg.cliff_polygons[0]
    t = armed(hw, 0)
    # Robot 0.3 m before the cliff edge (x = 1.0), facing it: ground point x + 0.21 is still on the floor.
    hw.set_pose_world(0.7, 0.0, 0.0)
    t = run(hw, t, t + 500, v=0.2)
    assert hw.wheel_twist()[0] > 0.1
    hw.set_pose_world(0.8, 0.0, 0.0)  # ground point 1.01: inside the cliff
    assert point_in_polygon(*hw.ground_point(True), poly)
    t = run(hw, t, t + 200, v=0.2)  # two confirmations at about 60 ms each
    assert hw.wheel_twist() == (0.0, 0.0)
    saf = hw.take_outputs().safety
    assert saf and saf[-1].block_forward and not saf[-1].block_backward
    t = run(hw, t, t + 500, v=-0.1)  # backwards is allowed
    assert hw.wheel_twist()[0] < -0.05


def test_bumper_blocks_forward() -> None:
    hw = make()
    t = armed(hw, 0)
    hw.set_bumper(2)
    t = run(hw, t, t + 300, v=0.2)
    assert hw.wheel_twist()[0] == pytest.approx(0.0)
    t = run(hw, t, t + 500, v=-0.1)
    assert hw.wheel_twist()[0] < -0.05


def test_relay_and_charging_at_dock() -> None:
    hw = make()
    t = run(hw, 0, 100)
    soc0 = hw.battery.soc
    t = run(hw, t, t + 60_000, step=100)
    assert hw.battery.soc < soc0  # idle load drains
    hw.set_pose_world(*SIM_CFG["dock"]["xy"], 0.0)
    assert not hw.charging()  # relay still open
    ok, msg = hw.set_relay(t, 1, True)
    assert ok, msg
    assert hw.charging()
    t = run(hw, t, t + 2000, step=100)
    power = hw.take_outputs().power[-1]
    assert power.charger_present
    expected = SIM_CFG["battery"]["charge_limit_a"] * min(1.0, (1 - hw.battery.soc) / 0.2)
    assert power.i_charge == pytest.approx(expected, abs=0.01)
    assert power.v_batt == pytest.approx(SIM_CFG["battery"]["charger_v"], abs=0.05)


def test_power_voltage_round_trip() -> None:
    hw = make()
    run(hw, 0, 600)
    power = hw.take_outputs().power[-1]
    assert power.v_batt == pytest.approx(hw.battery.v_terminal, abs=0.01)
    assert not power.charger_present
    assert power.v_rail == pytest.approx(0.0)  # safety relay open before arming


def test_hub_moves_to_top() -> None:
    hw = make()
    t = run(hw, 0, 100)
    for _ in range(8):  # the client repeats /hw/cmd/hub (bridge hub_cmd_timeout 2 s, Nano lease 0.5 s)
        hw.set_hub(t, 1, 200)
        t = run(hw, t, t + 500)
    hub = hw.take_outputs().hub
    assert hub and hub[-1][0] == 3 and hub[-1][1]  # TOP, limit_top


def test_hub_stops_without_repeated_command() -> None:
    hw = make()
    t = run(hw, 0, 100)
    hw.set_hub(t, 1, 200)
    run(hw, t, t + 3500)  # sent for 2 s, then the lease runs out: stops before the top (travel 3 s)
    assert hw.nano.hub_state != 3 and 0 < hw.nano.hub_pos_ms < hw.cfg.hub_travel_ms


def test_battery_curve_and_taper() -> None:
    cfg = BatteryConfig.from_dict(dict(SIM_CFG["battery"], initial_soc=0.5))
    b = Battery(cfg)
    assert b.ocv() == pytest.approx(12.1)
    b.step(1.0, 2.0, charging=False)
    assert b.v_terminal == pytest.approx(b.ocv() - 2.0 * cfg.r_internal_ohm)
    b.soc = 0.95
    b.step(1.0, 2.0, charging=True)
    assert b.i_charge == pytest.approx(cfg.charge_limit_a * 0.25, rel=0.01)


def test_cliff_polygon_from_world() -> None:
    polys = cliff_polygons_from_sdf(WORLD)
    assert len(polys) == 1
    xs = sorted({round(x, 6) for x, _ in polys[0]})
    ys = sorted({round(y, 6) for _, y in polys[0]})
    assert xs == [1.0, 1.4] and ys == [-0.3, 0.3]
    assert point_in_polygon(1.2, 0.0, polys[0]) and not point_in_polygon(0.9, 0.0, polys[0])
