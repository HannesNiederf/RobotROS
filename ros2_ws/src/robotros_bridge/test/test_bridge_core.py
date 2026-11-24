"""Tests of the ROS-free parts of nano_bridge: kinematics, link handshake against fake_nano, conversions."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robotros_bridge.core import convert  # noqa: E402
from robotros_bridge.core import protocol as p  # noqa: E402
from robotros_bridge.core.fake_nano import FakeNano  # noqa: E402
from robotros_bridge.core.kinematics import (  # noqa: E402
    UNCALIBRATED_SPEED_LIMIT,
    Geometry,
    Odometry,
    twist_to_tps,
    wrap_i32,
    wrap_u32,
)
from robotros_bridge.core.link import DeviceLink, LinkState  # noqa: E402

REPO = Path(__file__).resolve().parents[4]
CONFIG = REPO / "ros2_ws" / "src" / "robotros_bringup" / "config"
GEOM_CAL = Geometry(0.0325, 0.16, 576, calibrated=True)
GEOM_UNCAL = Geometry(0.0325, 0.16, 576, calibrated=False)


# ------------------------------------------------------------------ kinematics
def test_twist_to_tps_straight_and_rotation() -> None:
    per_m = 576 / (2 * math.pi * 0.0325)
    assert twist_to_tps(0.1, 0.0, GEOM_CAL) == (round(0.1 * per_m), round(0.1 * per_m))
    left, right = twist_to_tps(0.0, 1.0, GEOM_CAL)
    assert left == -right and right == round(0.08 * per_m)


def test_uncalibrated_limit_keeps_ratio() -> None:
    left, right = twist_to_tps(0.15, 0.6, GEOM_UNCAL)
    m_per_tick = 2 * math.pi * 0.0325 / 576
    assert max(abs(left), abs(right)) * m_per_tick <= UNCALIBRATED_SPEED_LIMIT + m_per_tick
    raw_l, raw_r = 0.15 - 0.6 * 0.08, 0.15 + 0.6 * 0.08
    assert left / right == pytest.approx(raw_l / raw_r, rel=0.02)
    assert twist_to_tps(0.03, 0.0, GEOM_UNCAL) == twist_to_tps(0.03, 0.0, GEOM_CAL)


def test_wraps() -> None:
    assert wrap_i32((-(2**31) + 10) - (2**31 - 10)) == 20  # counter went from 2^31-10 to -2^31+10
    assert wrap_i32(5 - 10) == -5
    assert wrap_u32(5 - (2**32 - 5)) == 10


def test_odometry_straight_and_turn() -> None:
    odo = Odometry(GEOM_CAL)
    ticks_per_m = 576 / (2 * math.pi * 0.0325)
    odo.update(0, 0, 0)
    s = odo.update(1000, round(ticks_per_m), round(ticks_per_m))
    assert s.x == pytest.approx(1.0, abs=1e-3) and s.y == pytest.approx(0.0, abs=1e-9)
    assert s.v == pytest.approx(1.0, abs=1e-3)
    quarter = round(math.pi / 4 * 0.16 * ticks_per_m)  # wheel arc for +90 deg in place
    s = odo.update(2000, round(ticks_per_m) - quarter, round(ticks_per_m) + quarter)
    assert s.yaw == pytest.approx(math.pi / 2, abs=0.01)
    assert s.x == pytest.approx(1.0, abs=1e-3)


def test_odometry_velocity_from_measured_rate() -> None:
    odo = Odometry(GEOM_CAL)
    odo.update(0, 0, 0, 100, 100)
    s = odo.update(20, 3, 2, 141, 141)  # tick difference is quantised, measured rate is not
    assert s.v == pytest.approx(141 * GEOM_CAL.metres_per_tick)
    assert s.w == pytest.approx(0.0)
    assert s.x == pytest.approx(2.5 * GEOM_CAL.metres_per_tick)


def test_odometry_across_counter_overflow() -> None:
    odo = Odometry(GEOM_CAL)
    odo.update(2**32 - 10, 2**31 - 100, 2**31 - 100)
    s = odo.update(10, -(2**31) + 100, -(2**31) + 100)  # +200 ticks, +20 ms
    assert s.x == pytest.approx(200 * GEOM_CAL.metres_per_tick)
    assert s.v == pytest.approx(200 * GEOM_CAL.metres_per_tick / 0.02)


def test_odometry_reset_reference_keeps_pose() -> None:
    odo = Odometry(GEOM_CAL)
    odo.update(0, 0, 0)
    odo.update(100, 1000, 1000)
    x = odo.state.x
    odo.reset_reference()
    odo.update(5, 0, 0)  # Nano restarted at zero
    assert odo.update(105, 10, 10).x == pytest.approx(x + 10 * GEOM_CAL.metres_per_tick)


# ------------------------------------------------------------- link + fake_nano
class Wire:
    """Connects a DeviceLink and a FakeNano in memory with a shared clock."""

    def __init__(self, role: p.Role, cfg: dict[str, int], start_nano: bool = True) -> None:
        self.t = 0
        self.nano = FakeNano(role)
        if start_nano:
            self.nano.power_on(self.t)
        self.link = DeviceLink("dev", role, cfg)
        self.link.connect(self.t)
        self.events = []

    def run(self, ms: int, step: int = 5) -> None:
        end = self.t + ms
        while self.t < end:
            self.t += step
            self.nano.feed(self.t, self.link.take_output())
            self.nano.step(self.t)
            self.events.append(self.link.feed(self.t, self.nano.read_output()))
            self.events.append(self.link.tick(self.t))


def test_handshake_and_config_reach_ready() -> None:
    fw = convert.load_yaml(CONFIG / "firmware.yaml")
    cfg = convert.cfg_for_role(fw, p.Role.DRIVE)
    w = Wire(p.Role.DRIVE, cfg)
    w.run(500)
    assert w.link.state == LinkState.READY
    assert any(e.became_ready for e in w.events)
    for key, value in cfg.items():
        assert w.nano.cfg[key] == value
    assert w.link.frames_bad == 0


def test_wrong_role_is_incompatible() -> None:
    w = Wire(p.Role.DRIVE, {})
    w.link.role = p.Role.AUX
    w.run(100)
    assert w.link.state == LinkState.INCOMPATIBLE
    assert any("role" in e.incompatible for e in w.events)


def test_nano_reset_is_detected_and_reconfigured() -> None:
    w = Wire(p.Role.DRIVE, {"WD_SOFT_MS": 400})
    w.run(200)
    assert w.link.state == LinkState.READY
    w.nano.power_on(w.t)  # reset: cfg back to defaults, HELLO again
    w.nano.cfg["WD_SOFT_MS"] = 500
    w.run(200)
    assert any(e.nano_reset for e in w.events)
    assert w.link.state == LinkState.READY and w.nano.cfg["WD_SOFT_MS"] == 400


def test_link_lost_when_silent() -> None:
    w = Wire(p.Role.DRIVE, {})
    w.run(100)
    w.nano.powered = False  # stops talking
    w.run(400)
    assert w.link.state == LinkState.LOST
    assert any(e.lost for e in w.events)


def test_hello_retried_when_nano_starts_late() -> None:
    w = Wire(p.Role.AUX, {}, start_nano=False)
    w.run(1500)
    assert w.link.state == LinkState.HANDSHAKE
    w.nano.power_on(w.t)
    w.run(200)
    assert w.link.state == LinkState.READY


def test_unknown_cfg_key_is_skipped_with_warning() -> None:
    w = Wire(p.Role.DRIVE, {"NOPE": 1, "KP": 7})
    w.run(300)
    assert w.link.state == LinkState.READY and w.nano.cfg["KP"] == 7
    assert any("NOPE" in msg for e in w.events for msg in e.warnings)


def test_commands_only_when_ready() -> None:
    link = DeviceLink("x", p.Role.DRIVE, {})
    assert not link.send_command("PING", seq=1)


# ------------------------------------------------------------------ conversion
def test_config_files_load() -> None:
    bridge = convert.load_yaml(CONFIG / "bridge.yaml")
    devices = convert.devices_from_bridge_cfg(bridge)
    assert [(d.name, d.role) for d in devices] == [("drive", p.Role.DRIVE), ("aux", p.Role.AUX)]
    geom = Geometry.from_robot_yaml(convert.load_yaml(CONFIG / "robot.yaml"))
    assert not geom.calibrated
    fw = convert.load_yaml(CONFIG / "firmware.yaml")
    assert "HUB_TIMEOUT_MS" in convert.cfg_for_role(fw, p.Role.AUX)
    assert "HUB_TIMEOUT_MS" not in convert.cfg_for_role(fw, p.Role.DRIVE)
    assert set(convert.cfg_for_role(fw, p.Role.ALL)) >= {"KP", "HUB_PWM"}


def test_firmware_yaml_keys_are_known_to_the_nano() -> None:
    from robotros_bridge.core.fake_nano import CFG_DEFAULTS

    fw = convert.load_yaml(CONFIG / "firmware.yaml")
    for section in ("drive", "aux"):
        assert set(fw[section]) <= set(CFG_DEFAULTS), section


def test_power_conversion() -> None:
    bridge = convert.load_yaml(CONFIG / "bridge.yaml")
    msg = p.decode_frame(p.encode("PWR", t_ms=1, adc_batt_mv=2500, adc_chg_mv=2760, adc_rail_mv=0, i_ma=450).decode())
    pv = convert.power_from_pwr(msg, bridge)
    assert pv.v_batt == pytest.approx(12.5) and pv.v_charger == pytest.approx(13.8)
    assert pv.charger_present and pv.i_charge == pytest.approx(0.45)
    msg = p.decode_frame(p.encode("PWR", t_ms=1, adc_batt_mv=0, adc_chg_mv=0, adc_rail_mv=0, i_ma=-32768).decode())
    assert math.isnan(convert.power_from_pwr(msg, bridge).i_charge)


def test_safety_and_hub_conversion() -> None:
    saf = p.decode_frame(p.encode("SAF", t_ms=1, state=3, flags=0b10_0010_0110).decode())
    v = convert.safety_from_saf(saf)
    assert v.state == 3 and v.relay_on and v.block_forward and not v.estop_pressed and v.software_stop
    assert convert.safety_from_saf(saf, link_lost=True).pi_link_lost
    hub = p.decode_frame(p.encode("HUBST", t_ms=1, state=3, limits=1, fault=0).decode())
    assert convert.hub_from_hubst(hub) == (3, True, False, 0)
    assert convert.bumper_from_mask(5) == (True, False, True)
