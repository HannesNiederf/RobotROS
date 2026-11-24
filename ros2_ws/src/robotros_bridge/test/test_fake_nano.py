"""Behaviour of fake_nano (simulated Nano) through its byte interface and through a real pty."""

from __future__ import annotations

import os
import select
import subprocess
import sys
import time
from pathlib import Path

import pytest

PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_ROOT))

from robotros_bridge.core import protocol as p  # noqa: E402
from robotros_bridge.core.fake_nano import FakeNano, HubState  # noqa: E402
from robotros_bridge.core.safety import SafetyState  # noqa: E402


class Harness:
    def __init__(self, role: p.Role = p.Role.DRIVE, estop: bool = False, **kw: int) -> None:
        self.t = 0
        self.nano = FakeNano(role, **kw)
        self.nano.power_on(self.t, estop_pressed=estop)
        self.parser = p.StreamParser()
        self.log: list[p.Message] = []
        self.collect()

    def collect(self) -> list[p.Message]:
        new = [m for m in self.parser.feed(self.nano.read_output()) if isinstance(m, p.Message)]
        self.log.extend(new)
        return new

    def send(self, name: str, **fields: int | str) -> list[p.Message]:
        self.nano.feed(self.t, p.encode(name, **fields))
        return self.collect()

    def raw(self, data: bytes) -> list[p.Message]:
        self.nano.feed(self.t, data)
        return self.collect()

    def run(self, ms: int, step: int = 5, keepalive: bool = False) -> list[p.Message]:
        out: list[p.Message] = []
        end = self.t + ms
        seq = 0
        while self.t < end:
            self.t += step
            if keepalive and self.t % 100 == 0:
                seq += 1
                self.nano.feed(self.t, p.encode("PING", seq=seq))
            self.nano.step(self.t)
            out += self.collect()
        return out

    def arm(self) -> None:
        assert any(m.name == "ACK" for m in self.send("ARM", arm=1))
        self.run(200, keepalive=True)

    def last(self, name: str) -> p.Message:
        return [m for m in self.log if m.name == name][-1]


def names(msgs: list[p.Message]) -> list[str]:
    return [m.name for m in msgs]


def test_hello_at_power_on_and_repeated_until_first_command() -> None:
    h = Harness()
    hello = h.log[0]
    assert hello.name == "HELLO" and hello["proto"] == p.PROTO_VER and hello["role"] == 1 and hello["caps"] == 7
    assert names(h.run(2000)).count("HELLO") == 1
    h.send("PING", seq=1)
    assert "HELLO" not in names(h.run(4000))
    assert names(h.send("WHO")) == ["HELLO"]


def test_aux_caps_without_ina219() -> None:
    h = Harness(p.Role.AUX, ina219_present=False)
    assert h.log[0]["caps"] == 0b0111_1000


def test_telemetry_rates_drive() -> None:
    h = Harness()
    out = h.run(1000, keepalive=True)
    n = names(out)
    assert 48 <= n.count("ODO") <= 51
    assert 15 <= n.count("SEN") <= 18
    assert 5 <= n.count("SAF") <= 7


def test_protocol_errors_are_answered() -> None:
    h = Harness()
    assert h.raw(b"$VEL,1,2,3*00\n")[0].fields == {"code": 1, "detail": "VEL"}
    assert h.send("HUB", command=1, pwm=100)[0].fields == {"code": 4, "detail": "HUB"}
    assert h.raw(p.encode_payload("FOO,1"))[0]["code"] == 2
    assert h.send("CFG", key="NOPE", value=1)[0]["code"] == 3
    assert h.send("CFG", key="HUB_PWM", value=1)[0]["code"] == 4
    assert h.send("CFG", key="KP", value=1500)[0].fields == {"command": "CFG", "arg": "KP"}


def test_no_motion_before_arming_and_during_delay() -> None:
    h = Harness()
    h.send("VEL", seq=1, l_tps=500, r_tps=500)
    h.run(300, keepalive=True)
    assert h.last("ODO")["l_ticks"] == 0
    h.send("ARM", arm=1)
    h.t += 100
    h.send("VEL", seq=2, l_tps=500, r_tps=500)  # within 200 ms after arming: ignored
    h.run(50)
    assert h.last("ODO")["l_tps"] == 0


def test_drives_when_armed_and_odometry_counts() -> None:
    h = Harness()
    h.arm()
    for i in range(50):
        h.send("VEL", seq=i, l_tps=400, r_tps=400)
        h.run(20)
    odo = h.last("ODO")
    assert odo["l_tps"] >= 390 and 250 <= odo["l_ticks"] <= 400
    assert h.last("SAF")["state"] == int(SafetyState.ARMED)


def test_soft_then_hard_watchdog() -> None:
    h = Harness()
    h.arm()
    h.send("VEL", seq=1, l_tps=300, r_tps=300)
    out = h.run(600)
    evts = [m["type"] for m in out if m.name == "EVT"]
    assert 50 in evts
    assert h.last("ODO")["l_tps"] == 0
    assert h.last("SAF")["flags"] & (1 << 4)
    out = h.run(4500)
    assert h.last("SAF")["state"] == int(SafetyState.WAIT_ACK)
    assert not h.last("SAF")["flags"] & (1 << 1)  # relay off


def test_estop_stops_and_needs_ack() -> None:
    h = Harness()
    h.arm()
    h.send("VEL", seq=1, l_tps=300, r_tps=300)
    h.run(200, keepalive=True)
    h.nano.set_estop(h.t, True)
    out = h.collect() + h.run(50, keepalive=True)
    assert any(m.name == "EVT" and m["type"] == 10 for m in out)
    assert h.last("SAF")["state"] == int(SafetyState.ESTOP)
    assert h.last("ODO")["l_tps"] == 0
    assert h.send("ARM", arm=1)[0].fields == {"code": 5, "detail": "ARM"}
    h.nano.set_estop(h.t, False)
    assert any(m.name == "EVT" and m["type"] == 11 for m in h.collect())
    h.run(20, keepalive=True)
    assert h.last("SAF")["state"] == int(SafetyState.WAIT_ACK)
    h.arm()
    assert h.last("SAF")["state"] == int(SafetyState.ARMED)


def test_software_stop_flag() -> None:
    h = Harness()
    h.arm()
    h.send("ARM", arm=0)
    saf = h.last("SAF")
    assert saf["state"] == int(SafetyState.WAIT_ACK) and saf["flags"] & (1 << 5)


def test_bumper_blocks_forward_but_allows_reverse_and_rotation() -> None:
    h = Harness()
    h.arm()
    h.send("VEL", seq=1, l_tps=300, r_tps=300)
    h.run(200, keepalive=True)
    h.nano.set_bumper(0b010)
    out = h.run(20, keepalive=True)
    assert any(m.name == "EVT" and m["type"] == 30 and m["arg"] == 2 for m in out)
    assert h.last("ODO")["l_tps"] == 0
    assert h.last("SAF")["flags"] & (1 << 2)
    h.send("VEL", seq=2, l_tps=300, r_tps=300)
    h.run(200, keepalive=True)
    assert h.last("ODO")["l_tps"] == 0
    h.send("VEL", seq=3, l_tps=-200, r_tps=-200)
    h.run(300, keepalive=True)
    assert h.last("ODO")["l_tps"] < -150
    h.send("VEL", seq=4, l_tps=-100, r_tps=100)
    h.run(300, keepalive=True)
    odo = h.last("ODO")
    assert odo["l_tps"] < -80 and odo["r_tps"] > 80


def test_cliff_front_confirmed_after_two_readings() -> None:
    h = Harness()
    h.arm()
    h.send("VEL", seq=1, l_tps=300, r_tps=300)
    h.run(200, keepalive=True)
    h.nano.set_us_mm(front=65535)
    out = h.run(200, keepalive=True)
    assert any(m.name == "EVT" and m["type"] == 20 for m in out)
    saf = h.last("SAF")
    assert saf["flags"] & (1 << 6) and saf["flags"] & (1 << 2)
    assert h.last("SEN")["cls_front"] == 2
    assert h.last("ODO")["l_tps"] == 0
    h.nano.set_us_mm(front=120)
    out = h.run(400, keepalive=True)
    assert any(m.name == "EVT" and m["type"] == 21 for m in out)
    assert not h.last("SAF")["flags"] & (1 << 6)


def test_obstacle_class_does_not_block() -> None:
    h = Harness()
    h.arm()
    h.nano.set_us_mm(front=40)
    h.send("VEL", seq=1, l_tps=300, r_tps=300)
    for i in range(20):
        h.send("VEL", seq=i + 2, l_tps=300, r_tps=300)
        h.run(20)
    assert h.last("SEN")["cls_front"] == 3
    assert h.last("ODO")["l_tps"] > 250


def test_encoder_fault_event() -> None:
    h = Harness()
    h.arm()
    h.nano.set_wheel_stall(True)
    out: list[p.Message] = []
    for i in range(60):
        h.send("VEL", seq=i, l_tps=300, r_tps=300)
        out += h.run(20)
    assert [m["type"] for m in out if m.name == "EVT"].count(52) == 1


def hub_h(**kw: int) -> Harness:
    h = Harness(p.Role.AUX, hub_travel_ms=1000, **kw)
    h.send("PING", seq=1)
    return h


def hub_run(h: Harness, command: int, ms: int) -> list[p.Message]:
    out: list[p.Message] = []
    for _ in range(ms // 100):
        out += h.send("HUB", command=command, pwm=200)
        out += h.run(100)
    return out


def test_hub_up_and_down_with_lease() -> None:
    h = hub_h()
    out = hub_run(h, 1, 1200)
    assert any(m.name == "EVT" and m["type"] == 40 for m in out)
    assert h.last("HUBST")["state"] == HubState.TOP and h.last("HUBST")["limits"] == 1
    out = hub_run(h, 2, 1200)
    assert any(m.name == "EVT" and m["type"] == 41 for m in out)
    assert h.last("HUBST")["state"] == HubState.BOTTOM


def test_hub_stops_when_lease_expires() -> None:
    h = hub_h()
    hub_run(h, 1, 300)
    pos = h.nano.hub_pos_ms
    h.run(1000)  # no more $HUB
    assert h.nano.hub_cmd == 0 and h.nano.hub_pos_ms <= pos + 600


def test_hub_timeout_fault_and_clear() -> None:
    h = hub_h()
    h.send("CFG", key="HUB_TIMEOUT_MS", value=500)
    h.nano.set_hub_power(False)  # motor without supply never reaches the limit
    out = hub_run(h, 1, 800)
    assert any(m.name == "EVT" and m["type"] == 42 for m in out)
    assert h.last("HUBST")["state"] == HubState.FAULT and h.last("HUBST")["fault"] == 1
    hub_run(h, 1, 300)
    assert h.nano.hub_state == HubState.FAULT  # stays until CLR
    assert h.send("CLR")[0].fields == {"command": "CLR"}
    assert h.nano.hub_state != HubState.FAULT


def test_hub_both_limits_fault() -> None:
    h = hub_h()
    h.nano.hub_limits_forced = 3
    hub_run(h, 1, 200)
    assert h.last("HUBST")["fault"] == 2


def test_aux_power_and_relay() -> None:
    h = hub_h()
    h.nano.set_power(2600, 2750, 2500, 480)
    out = h.run(1000)
    pwr = [m for m in out if m.name == "PWR"]
    assert 2 <= len(pwr) <= 3 and pwr[-1]["i_ma"] == 480
    assert h.send("REL", channel=1, on=1)[0].fields == {"command": "REL", "arg": "1"}
    assert h.nano.relays[1] == 1
    assert h.send("VEL", seq=1, l_tps=1, r_tps=1)[0]["code"] == 4


@pytest.mark.skipif(not sys.platform.startswith("linux"), reason="pty test needs Linux")
def test_pty_end_to_end() -> None:
    env = dict(os.environ, PYTHONPATH=str(PKG_ROOT))
    proc = subprocess.Popen(
        [sys.executable, "-m", "robotros_bridge.fake_nano_main", "--role", "drive"],
        stdout=subprocess.PIPE,
        text=True,
        env=env,
    )
    try:
        assert proc.stdout is not None
        path = proc.stdout.readline().strip()
        fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
        try:
            os.write(fd, p.encode("WHO") + p.encode("PING", seq=42))
            parser = p.StreamParser()
            got: list[p.Message] = []
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and not any(m.name == "PONG" for m in got):
                ready, _, _ = select.select([fd], [], [], 0.1)
                if ready:
                    got += [m for m in parser.feed(os.read(fd, 4096)) if isinstance(m, p.Message)]
            assert any(m.name == "HELLO" for m in got)
            assert any(m.name == "PONG" and m["seq"] == 42 for m in got)
            assert any(m.name == "ODO" for m in got)
        finally:
            os.close(fd)
    finally:
        proc.terminate()
        proc.wait(timeout=5)
