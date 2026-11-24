"""Simulated Nano Every that speaks serial protocol version 1 (softwarespezifikation.md D-19, section 14).

``FakeNano`` is pure logic: bytes go in through ``feed``, time advances through ``step`` (monotonic
ms supplied by the caller), and frames come out through ``read_output``. ``fake_nano_main`` wraps it
in a pseudo-terminal so nano_bridge can open it like a real serial port.

It follows spez_firmware_protokoll.md: telegrams and rates (section 2), safety state machine and
watchdogs (section 5, via ``safety.SafetyFsm``), direction locks (5.3), ultrasonic classification
and cliff confirmation (5.4, 6.1), hub lease, limits and timeout (section 7), CFG keys (section 10).

Simulation only (not from the spec): wheels follow their setpoint with a first-order lag
(``motor_tau_ms``), the hub needs ``hub_travel_ms`` between the limit switches, the test harness
sets sensor values directly (``set_us_mm``, ``set_bumper``, ``set_estop``, ``set_hub_power``,
``set_wheel_stall``, ``set_power``). Ticks per second here are the setpoint units of ``$VEL``.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from . import protocol as p
from .safety import (
    NO_ECHO_MM,
    CliffDebounce,
    SafetyFsm,
    SafetyOutput,
    SafetyState,
    UsClass,
    apply_direction_lock,
    classify_us,
)

FW_VERSION = "fake-0.1"

CAPS = {
    p.Role.DRIVE: 0b0000_0111,  # encoder, ultrasonic, bumper
    p.Role.AUX: 0b1111_1000,  # hub, LED, relay, voltage, INA219
    p.Role.ALL: 0b1111_1111,
}

# CFG keys and the role that owns them (section 10). Values are the firmware start values.
CFG_DEFAULTS: dict[str, tuple[set[p.Role], int]] = {
    "KP": ({p.Role.DRIVE}, 0),
    "KI": ({p.Role.DRIVE}, 0),
    "KD": ({p.Role.DRIVE}, 0),
    "KFF": ({p.Role.DRIVE}, 0),
    "PWM_MIN_L": ({p.Role.DRIVE}, 60),
    "PWM_MIN_R": ({p.Role.DRIVE}, 60),
    "PWM_MAX": ({p.Role.DRIVE}, 255),
    "ACC": ({p.Role.DRIVE}, 1500),
    "TPS_MAX": ({p.Role.DRIVE}, 2000),
    "STOP_TPS": ({p.Role.DRIVE}, 5),
    "MOT_INV": ({p.Role.DRIVE}, 0),
    "ENC_INV": ({p.Role.DRIVE}, 0),
    "US_FLOOR_F": ({p.Role.DRIVE}, 120),
    "US_FLOOR_R": ({p.Role.DRIVE}, 120),
    "US_CLIFF_MARGIN": ({p.Role.DRIVE}, 40),
    "US_OBST_MARGIN": ({p.Role.DRIVE}, 40),
    "US_CONFIRM_ON": ({p.Role.DRIVE}, 2),
    "US_CONFIRM_OFF": ({p.Role.DRIVE}, 3),
    "BUMP_DEBOUNCE_MS": ({p.Role.DRIVE, p.Role.AUX}, 5),
    "WD_SOFT_MS": ({p.Role.DRIVE}, 500),
    "WD_HARD_MS": ({p.Role.DRIVE}, 5000),
    "HUB_TIMEOUT_MS": ({p.Role.AUX}, 8000),
    "HUB_PWM": ({p.Role.AUX}, 200),
    "LED_BRIGHT": ({p.Role.AUX}, 64),
    "INA_SHUNT_MOHM": ({p.Role.AUX}, 100),
}

# Commands per role (section 2.2). ALL accepts everything.
ROLE_COMMANDS: dict[p.Role, set[str]] = {
    p.Role.DRIVE: {"WHO", "PING", "VEL", "ARM", "CLR", "CFG"},
    p.Role.AUX: {"WHO", "PING", "CLR", "CFG", "HUB", "LED", "BUZ", "REL"},
    p.Role.ALL: set(p.PI_TO_NANO),
}

HELLO_PERIOD_MS = 2000
ODO_PERIOD_MS = 20  # 50 Hz
US_SLOT_MS = 30  # one sensor per slot, each sensor every 60 ms
SEN_PERIOD_MS = 60  # about 16 Hz
SAF_PERIOD_MS = 200  # 5 Hz
PWR_PERIOD_MS = 500  # 2 Hz
HUBST_PERIOD_MS = 500  # 2 Hz while moving
HUB_LEASE_MS = 500
ENC_FAULT_MS = 1000


class HubState:
    IDLE, UP, DOWN, TOP, BOTTOM, FAULT = range(6)


@dataclass
class _Periodic:
    period_ms: int
    next_ms: int = 0

    def due(self, now_ms: int) -> bool:
        if now_ms >= self.next_ms:
            self.next_ms = max(self.next_ms + self.period_ms, now_ms - self.period_ms + 1)
            return True
        return False


class FakeNano:
    """One simulated board. Construct, then call ``power_on(now_ms)``."""

    def __init__(
        self,
        role: p.Role = p.Role.DRIVE,
        motor_tau_ms: int = 100,
        hub_travel_ms: int = 3000,
        ina219_present: bool = True,
        log: Callable[[str], None] | None = None,
    ) -> None:
        self.role = role
        self.motor_tau_ms = motor_tau_ms
        self.hub_travel_ms = hub_travel_ms
        self.ina219_present = ina219_present
        self._log = log or (lambda _msg: None)
        self._parser = p.StreamParser()
        self._out = bytearray()
        self.cfg = {key: value for key, (roles, value) in CFG_DEFAULTS.items() if self._owns(roles)}
        # Inputs set by the test harness.
        self.us_mm = {"front": 120, "rear": 120}
        self.bumper = 0
        self.wheel_stall = False
        self.hub_power = True  # supply of the hub motor (cut by the safety relay)
        self.hub_limits_forced: int | None = None  # force a limit pattern, e.g. 3 = both active
        self.power_mv = {"batt": 2500, "chg": 0, "rail": 2500}
        self.i_ma = 0
        self.powered = False

    # ----------------------------------------------------------------- helpers
    def _owns(self, roles: set[p.Role]) -> bool:
        return self.role == p.Role.ALL or self.role in roles

    @property
    def has_drive(self) -> bool:
        return self.role in (p.Role.DRIVE, p.Role.ALL)

    @property
    def has_aux(self) -> bool:
        return self.role in (p.Role.AUX, p.Role.ALL)

    def _send(self, name: str, **fields: int | str) -> None:
        self._out += p.encode(name, **fields)

    def _evt(self, now_ms: int, typ: int, arg: int = 0) -> None:
        self._send("EVT", t_ms=now_ms & p.UINT32_MAX, type=typ, arg=arg)

    def read_output(self) -> bytes:
        data = bytes(self._out)
        self._out.clear()
        return data

    # ------------------------------------------------------------- power / reset
    def power_on(self, now_ms: int, estop_pressed: bool = False) -> None:
        self.powered = True
        self._parser = p.StreamParser()
        self.safety = SafetyFsm(wd_soft_ms=self.cfg.get("WD_SOFT_MS", 500), wd_hard_ms=self.cfg.get("WD_HARD_MS", 5000))
        self.safety.reset(now_ms, estop_pressed)
        self.got_pi_command = False
        self._hello = _Periodic(HELLO_PERIOD_MS, now_ms + HELLO_PERIOD_MS)
        self._send_hello()
        # Drive
        self.target = [0, 0]  # accepted setpoints after locks
        self.tps = [0.0, 0.0]
        self.ticks = [0.0, 0.0]
        self.vel_since_ms = now_ms
        self.enc_fault_sent = False
        self.cliff = {"front": CliffDebounce(), "rear": CliffDebounce()}
        self.us_cls = {"front": UsClass.UNKNOWN, "rear": UsClass.UNKNOWN}
        self.us_slot_front = True
        self._odo = _Periodic(ODO_PERIOD_MS, now_ms)
        self._us = _Periodic(US_SLOT_MS, now_ms + US_SLOT_MS)
        self._sen = _Periodic(SEN_PERIOD_MS, now_ms)
        self._saf = _Periodic(SAF_PERIOD_MS, now_ms)
        self._last_saf: tuple[int, int] | None = None
        self._last_bumper = self.bumper
        self._last_cliff = (False, False)
        # Aux
        self.hub_state = HubState.BOTTOM
        self.hub_pos_ms = 0  # 0 = bottom, hub_travel_ms = top
        self.hub_fault = 0
        self.hub_cmd = 0
        self.hub_lease_until = now_ms
        self.hub_move_start = now_ms
        self.relays = {1: 0, 2: 0, 3: 0}
        self._pwr = _Periodic(PWR_PERIOD_MS, now_ms)
        self._hubst = _Periodic(HUBST_PERIOD_MS, now_ms)
        self._last_hubst: tuple[int, int, int] | None = None
        self.last_ms = now_ms

    def _send_hello(self) -> None:
        caps = CAPS[self.role]
        if self.has_aux and not self.ina219_present:
            caps &= ~(1 << 7)
        self._send("HELLO", proto=p.PROTO_VER, fw=FW_VERSION, role=int(self.role), caps=caps)

    # ------------------------------------------------------------ harness inputs
    def set_estop(self, now_ms: int, pressed: bool) -> None:
        if not self.has_drive:
            return
        out = self.safety.set_estop(now_ms, pressed)
        self._apply(now_ms, out)

    def set_bumper(self, mask: int) -> None:
        self.bumper = mask & 7

    def set_us_mm(self, front: int | None = None, rear: int | None = None) -> None:
        if front is not None:
            self.us_mm["front"] = front
        if rear is not None:
            self.us_mm["rear"] = rear

    def set_wheel_stall(self, stalled: bool) -> None:
        self.wheel_stall = stalled

    def set_hub_power(self, on: bool) -> None:
        self.hub_power = on

    def set_power(self, batt_mv: int, chg_mv: int, rail_mv: int, i_ma: int) -> None:
        self.power_mv = {"batt": batt_mv, "chg": chg_mv, "rail": rail_mv}
        self.i_ma = i_ma

    # ------------------------------------------------------------------ input
    def feed(self, now_ms: int, data: bytes) -> None:
        if not self.powered:
            return
        for result in self._parser.feed(data):
            if isinstance(result, p.ProtocolError):
                self._log(f"protocol error: {result}")
                self._out += p.err_reply(result)
                continue
            self._handle(now_ms, result)

    def _handle(self, now_ms: int, msg: p.Message) -> None:
        name = msg.name
        if name not in p.PI_TO_NANO:
            self._send("ERR", code=int(p.ErrCode.UNKNOWN_COMMAND), detail=name)
            return
        if name not in ROLE_COMMANDS[self.role]:
            self._send("ERR", code=int(p.ErrCode.ROLE_NOT_SUPPORTED), detail=name)
            return
        self.got_pi_command = True
        if name in ("VEL", "PING") and self.has_drive:
            self._apply(now_ms, self.safety.valid_pi_message(now_ms))
        if name == "WHO":
            self._send_hello()
        elif name == "PING":
            self._send("PONG", seq=msg["seq"])
        elif name == "VEL":
            self._vel(now_ms, int(msg["l_tps"]), int(msg["r_tps"]))
        elif name == "ARM":
            self._apply(now_ms, self.safety.arm_request(now_ms, bool(msg["arm"])))
        elif name == "CLR":
            if self.has_aux and self.hub_state == HubState.FAULT:
                self.hub_fault = 0
                self.hub_state = HubState.IDLE
            self._send("ACK", command="CLR")
        elif name == "CFG":
            self._cfg(str(msg["key"]), int(msg["value"]))
        elif name == "HUB":
            self._hub_cmd(now_ms, int(msg["command"]))
        elif name in ("LED", "BUZ"):
            self._send("ACK", command=name)
        elif name == "REL":
            self.relays[int(msg["channel"])] = int(msg["on"])
            self._send("ACK", command="REL", arg=str(msg["channel"]))

    def _apply(self, now_ms: int, out: SafetyOutput) -> None:
        for action in out.actions:
            if action == "ACK_ARM":
                self._send("ACK", command="ARM")
            elif action == "ERR5":
                self._send("ERR", code=int(p.ErrCode.NOT_ALLOWED_IN_STATE), detail="ARM")
            elif action == "SETPOINTS_ZERO":
                self.target = [0, 0]
            elif action == "BRAKE":
                # Safety stop bypasses the ramp (section 4.2); the simulation brakes to zero at once.
                self.target = [0, 0]
                if self.has_drive:
                    self.tps = [0.0, 0.0]
        for typ, arg in out.events:
            self._evt(now_ms, typ, arg)
        self._send_saf_if_changed(now_ms)

    def _cfg(self, key: str, value: int) -> None:
        if key not in CFG_DEFAULTS:
            self._send("ERR", code=int(p.ErrCode.BAD_ARGUMENT), detail=key)
            return
        if not self._owns(CFG_DEFAULTS[key][0]):
            self._send("ERR", code=int(p.ErrCode.ROLE_NOT_SUPPORTED), detail=key)
            return
        self.cfg[key] = value
        if key == "WD_SOFT_MS":
            self.safety.wd_soft_ms = value
        elif key == "WD_HARD_MS":
            self.safety.wd_hard_ms = value
        elif key in ("US_CONFIRM_ON", "US_CONFIRM_OFF"):
            for deb in self.cliff.values():
                deb.confirm_on = self.cfg["US_CONFIRM_ON"]
                deb.confirm_off = self.cfg["US_CONFIRM_OFF"]
        self._send("ACK", command="CFG", arg=key)

    # ------------------------------------------------------------------ drive
    def _locks(self) -> tuple[bool, bool]:
        block_fwd = self.cliff["front"].active or self.bumper != 0
        block_back = self.cliff["rear"].active
        return block_fwd, block_back

    def _vel(self, now_ms: int, l_tps: int, r_tps: int) -> None:
        if not self.safety.drive_allowed(now_ms):
            self.target = [0, 0]
            return
        tps_max = self.cfg["TPS_MAX"]
        l_tps = max(-tps_max, min(tps_max, l_tps))
        r_tps = max(-tps_max, min(tps_max, r_tps))
        block_fwd, block_back = self._locks()
        new = list(apply_direction_lock(l_tps, r_tps, block_fwd, block_back))
        if new != self.target and any(new):
            self.vel_since_ms = now_ms
            self.enc_fault_sent = False
        self.target = new

    def _drive_step(self, now_ms: int, dt_ms: int) -> None:
        # A new lock stops the robot at once (no ramp), section 5.3.
        block_fwd, block_back = self._locks()
        target_lin = self.target[0] + self.target[1]
        speed_lin = self.tps[0] + self.tps[1]
        if (block_fwd and (target_lin > 0 or speed_lin > 0)) or (block_back and (target_lin < 0 or speed_lin < 0)):
            self.target = list(apply_direction_lock(self.target[0], self.target[1], block_fwd, block_back))
            self.tps = [0.0, 0.0]
        if self.safety.state != SafetyState.ARMED or self.safety.pi_link_lost:
            self.target = [0, 0]
        for i in range(2):
            if self.wheel_stall or not self.safety.arming:
                self.tps[i] = 0.0
                continue
            alpha = min(1.0, dt_ms / self.motor_tau_ms) if self.motor_tau_ms > 0 else 1.0
            self.tps[i] += (self.target[i] - self.tps[i]) * alpha
            if self.target[i] == 0 and abs(self.tps[i]) < self.cfg["STOP_TPS"]:
                self.tps[i] = 0.0
            self.ticks[i] += self.tps[i] * dt_ms / 1000.0
        # Encoder fault: setpoint not zero but no ticks for 1 s (EVT 52).
        moving = any(abs(v) >= 1.0 for v in self.tps)
        if any(self.target) and not moving and not self.enc_fault_sent and now_ms - self.vel_since_ms >= ENC_FAULT_MS:
            self.enc_fault_sent = True
            self._evt(now_ms, 52, 0)
        if moving:
            self.vel_since_ms = now_ms

    def _us_step(self, now_ms: int) -> None:
        side = "front" if self.us_slot_front else "rear"
        self.us_slot_front = not self.us_slot_front
        floor = self.cfg["US_FLOOR_F" if side == "front" else "US_FLOOR_R"]
        cls = classify_us(self.us_mm[side], floor, self.cfg["US_CLIFF_MARGIN"], self.cfg["US_OBST_MARGIN"])
        self.us_cls[side] = cls
        before = self.cliff[side].active
        after = self.cliff[side].update(cls)
        if before != after:
            base = 20 if side == "front" else 22
            self._evt(now_ms, base if after else base + 1, 0)

    def _saf_tuple(self) -> tuple[int, int]:
        block_fwd, block_back = self._locks()
        flags = self.safety.flags(block_fwd, block_back, self.cliff["front"].active, self.cliff["rear"].active)
        return int(self.safety.state), flags

    def _send_saf_if_changed(self, now_ms: int) -> None:
        if not self.has_drive:
            return
        current = self._saf_tuple()
        if current != self._last_saf:
            self._last_saf = current
            self._send("SAF", t_ms=now_ms & p.UINT32_MAX, state=current[0], flags=current[1])

    # -------------------------------------------------------------------- hub
    def _hub_cmd(self, now_ms: int, command: int) -> None:
        self.hub_lease_until = now_ms + HUB_LEASE_MS
        if self.hub_state == HubState.FAULT:
            return
        if command == 0:
            self.hub_cmd = 0
        elif command != self.hub_cmd:
            self.hub_cmd = command
            self.hub_move_start = now_ms
        self._hub_eval(now_ms)
        self._send_hubst_if_changed(now_ms)

    def _limits(self) -> int:
        if self.hub_limits_forced is not None:
            return self.hub_limits_forced
        top = 1 if self.hub_pos_ms >= self.hub_travel_ms else 0
        bottom = 2 if self.hub_pos_ms <= 0 else 0
        return top | bottom

    def _hub_eval(self, now_ms: int) -> None:
        limits = self._limits()
        if limits == 3:
            self.hub_state, self.hub_fault, self.hub_cmd = HubState.FAULT, 2, 0
            return
        if self.hub_cmd == 1:
            self.hub_state = HubState.TOP if limits & 1 else HubState.UP
        elif self.hub_cmd == 2:
            self.hub_state = HubState.BOTTOM if limits & 2 else HubState.DOWN
        else:
            self.hub_state = HubState.TOP if limits & 1 else HubState.BOTTOM if limits & 2 else HubState.IDLE

    def _hub_step(self, now_ms: int, dt_ms: int) -> None:
        if self.hub_state == HubState.FAULT:
            return
        if self.hub_cmd and now_ms > self.hub_lease_until:
            self.hub_cmd = 0  # lease expired: stop
        if self.hub_state in (HubState.UP, HubState.DOWN) and self.hub_power:
            step = dt_ms if self.hub_state == HubState.UP else -dt_ms
            self.hub_pos_ms = max(0, min(self.hub_travel_ms, self.hub_pos_ms + step))
        before = self.hub_state
        self._hub_eval(now_ms)
        if (
            self.hub_state in (HubState.UP, HubState.DOWN)
            and now_ms - self.hub_move_start >= self.cfg["HUB_TIMEOUT_MS"]
        ):
            self.hub_state, self.hub_fault, self.hub_cmd = HubState.FAULT, 1, 0
            self._evt(now_ms, 42, 0)
        elif before in (HubState.UP, HubState.DOWN) and self.hub_state == HubState.TOP:
            self._evt(now_ms, 40, 0)
        elif before in (HubState.UP, HubState.DOWN) and self.hub_state == HubState.BOTTOM:
            self._evt(now_ms, 41, 0)

    def _hubst_tuple(self) -> tuple[int, int, int]:
        return self.hub_state, self._limits(), self.hub_fault

    def _send_hubst_if_changed(self, now_ms: int, force: bool = False) -> None:
        current = self._hubst_tuple()
        if force or current != self._last_hubst:
            self._last_hubst = current
            self._send("HUBST", t_ms=now_ms & p.UINT32_MAX, state=current[0], limits=current[1], fault=current[2])

    # ------------------------------------------------------------------- time
    def step(self, now_ms: int) -> None:
        """Advance the simulation to ``now_ms`` (monotonic, must not go backwards)."""
        if not self.powered:
            return
        dt_ms = now_ms - self.last_ms
        if dt_ms < 0:
            raise ValueError("time went backwards")
        self.last_ms = now_ms
        if not self.got_pi_command and self._hello.due(now_ms):
            self._send_hello()
        if self.has_drive:
            self._apply(now_ms, self.safety.tick(now_ms))
            if self._us.due(now_ms):
                self._us_step(now_ms)
            if self.bumper != self._last_bumper:
                self._last_bumper = self.bumper
                self._evt(now_ms, 30, self.bumper)
            self._drive_step(now_ms, dt_ms)
            if self._odo.due(now_ms):
                ticks = [max(p.INT32_MIN, min(p.INT32_MAX, round(t))) for t in self.ticks]
                self._send(
                    "ODO",
                    t_ms=now_ms & p.UINT32_MAX,
                    l_ticks=ticks[0],
                    r_ticks=ticks[1],
                    l_tps=round(self.tps[0]),
                    r_tps=round(self.tps[1]),
                )
            if self._sen.due(now_ms):
                self._send(
                    "SEN",
                    t_ms=now_ms & p.UINT32_MAX,
                    us_front_mm=min(self.us_mm["front"], NO_ECHO_MM),
                    us_rear_mm=min(self.us_mm["rear"], NO_ECHO_MM),
                    cls_front=int(self.us_cls["front"]),
                    cls_rear=int(self.us_cls["rear"]),
                    bump=self.bumper,
                )
            if self._saf.due(now_ms):
                self._last_saf = None  # periodic SAF is always sent
            self._send_saf_if_changed(now_ms)
        if self.has_aux:
            self._hub_step(now_ms, dt_ms)
            moving = self.hub_state in (HubState.UP, HubState.DOWN)
            periodic = self._hubst.due(now_ms) and moving
            self._send_hubst_if_changed(now_ms, force=periodic)
            if self._pwr.due(now_ms):
                self._send(
                    "PWR",
                    t_ms=now_ms & p.UINT32_MAX,
                    adc_batt_mv=self.power_mv["batt"],
                    adc_chg_mv=self.power_mv["chg"],
                    adc_rail_mv=self.power_mv["rail"],
                    i_ma=self.i_ma if self.ina219_present else -32768,
                )
