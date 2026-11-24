"""Safety state machine of the DRIVE role (spez_firmware_protokoll.md section 5).

Pure logic without I/O or clocks: the caller passes the monotonic time in ms.
Used by fake_nano now; the firmware core implements the same table in C++
(firmware/test/vectors/safety_fsm.csv is the shared test table).

Choices made here (the spec table leaves them open, documented for review):
- Events that the table does not list for a state change nothing. ``ARM,1`` while ARMED and
  ``ARM,0`` outside ARMED are acknowledged (ACK) without effect; ``ARM,1`` with the E-stop
  pressed gives ``ERR,5`` in every state.
- EVT 50 is sent when the soft watchdog (WD_SOFT_MS) sets ``pi_link_lost`` and again when the
  hard watchdog (WD_HARD_MS) disarms (table row 11). EVT 51 is sent when a valid message clears
  ``pi_link_lost``.
- Direction locks act on the linear part of the wheel setpoints only; the rotation part is kept
  and truncated towards zero to whole ticks/s.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum


class SafetyState(IntEnum):
    BOOT_LOCKED = 0
    ESTOP = 1
    WAIT_ACK = 2
    ARMED = 3


class Flag(IntEnum):
    """Bits of the ``$SAF`` flags field (section 2.3)."""

    ESTOP_PRESSED = 1 << 0
    RELAY_ON = 1 << 1
    BLOCK_FORWARD = 1 << 2
    BLOCK_BACKWARD = 1 << 3
    PI_LINK_LOST = 1 << 4
    SOFTWARE_STOP = 1 << 5
    CLIFF_FRONT = 1 << 6
    CLIFF_REAR = 1 << 7
    HUB_FAULT = 1 << 8


ARM_DRIVE_DELAY_MS = 200


@dataclass
class SafetyOutput:
    """What the caller has to do after an event."""

    actions: list[str] = field(default_factory=list)
    events: list[tuple[int, int]] = field(default_factory=list)  # (EVT type, arg)


@dataclass
class SafetyFsm:
    """State machine plus watchdogs. Times are monotonic milliseconds supplied by the caller."""

    wd_soft_ms: int = 500
    wd_hard_ms: int = 5000
    state: SafetyState = SafetyState.BOOT_LOCKED
    estop_pressed: bool = False
    arming: bool = False
    software_stop: bool = False
    pi_link_lost: bool = False
    last_valid_ms: int = 0
    armed_at_ms: int = 0

    def reset(self, now_ms: int, estop_pressed: bool) -> SafetyOutput:
        """Power-on or reset (rows 1-2)."""
        self.estop_pressed = estop_pressed
        self.state = SafetyState.ESTOP if estop_pressed else SafetyState.BOOT_LOCKED
        self.arming = False
        self.software_stop = False
        self.pi_link_lost = False
        self.last_valid_ms = now_ms
        return SafetyOutput(actions=["HELLO"])

    def _arm(self, now_ms: int) -> SafetyOutput:
        self.state = SafetyState.ARMED
        self.arming = True
        self.software_stop = False
        self.armed_at_ms = now_ms
        self.last_valid_ms = now_ms
        return SafetyOutput(actions=["ACK_ARM", "SETPOINTS_ZERO", "DRIVE_DELAY_200MS"])

    def arm_request(self, now_ms: int, arm: bool) -> SafetyOutput:
        """``$ARM,1`` / ``$ARM,0`` (rows 3, 4, 8, 10)."""
        if arm:
            if self.estop_pressed:
                return SafetyOutput(actions=["ERR5"])
            if self.state in (SafetyState.BOOT_LOCKED, SafetyState.WAIT_ACK):
                return self._arm(now_ms)
            return SafetyOutput(actions=["ACK_ARM"])  # already ARMED: no change
        if self.state == SafetyState.ARMED:
            self.state = SafetyState.WAIT_ACK
            self.arming = False
            self.software_stop = True
            return SafetyOutput(actions=["ACK_ARM", "BRAKE", "SW_STOP_FLAG"])
        return SafetyOutput(actions=["ACK_ARM"])

    def set_estop(self, now_ms: int, pressed: bool) -> SafetyOutput:
        """E-stop input changed (rows 5, 6, 7, 9). Debouncing is the caller's job."""
        if pressed == self.estop_pressed:
            return SafetyOutput()
        self.estop_pressed = pressed
        if pressed:
            previous = self.state
            self.state = SafetyState.ESTOP
            self.arming = False
            if previous == SafetyState.ARMED:
                return SafetyOutput(actions=["BRAKE"], events=[(10, 0)])
            return SafetyOutput()
        if self.state == SafetyState.ESTOP:
            self.state = SafetyState.WAIT_ACK
            return SafetyOutput(events=[(11, 0)])
        return SafetyOutput()

    def valid_pi_message(self, now_ms: int) -> SafetyOutput:
        """A VEL or PING with correct checksum arrived."""
        self.last_valid_ms = now_ms
        if self.pi_link_lost:
            self.pi_link_lost = False
            return SafetyOutput(events=[(51, 0)])
        return SafetyOutput()

    def tick(self, now_ms: int) -> SafetyOutput:
        """Evaluate the watchdogs (row 11 and the soft watchdog)."""
        out = SafetyOutput()
        if self.state != SafetyState.ARMED:
            return out
        silent = now_ms - self.last_valid_ms
        if silent >= self.wd_hard_ms:
            self.state = SafetyState.WAIT_ACK
            self.arming = False
            out.actions.append("BRAKE")
            out.events.append((50, 0))
            return out
        if silent >= self.wd_soft_ms and not self.pi_link_lost:
            self.pi_link_lost = True
            out.actions.append("BRAKE")
            out.events.append((50, 0))
        return out

    def drive_allowed(self, now_ms: int) -> bool:
        """Wheel setpoints other than zero are accepted only in ARMED, 200 ms after arming, with a live link."""
        return (
            self.state == SafetyState.ARMED
            and now_ms - self.armed_at_ms >= ARM_DRIVE_DELAY_MS
            and not self.pi_link_lost
        )

    def flags(self, block_forward: bool, block_backward: bool, cliff_front: bool, cliff_rear: bool) -> int:
        value = 0
        for bit, on in (
            (Flag.ESTOP_PRESSED, self.estop_pressed),
            (Flag.RELAY_ON, self.arming),
            (Flag.BLOCK_FORWARD, block_forward),
            (Flag.BLOCK_BACKWARD, block_backward),
            (Flag.PI_LINK_LOST, self.pi_link_lost),
            (Flag.SOFTWARE_STOP, self.software_stop),
            (Flag.CLIFF_FRONT, cliff_front),
            (Flag.CLIFF_REAR, cliff_rear),
        ):
            if on:
                value |= int(bit)
        return value


def apply_direction_lock(l_tps: int, r_tps: int, block_forward: bool, block_backward: bool) -> tuple[int, int]:
    """Clamp the linear part of a wheel setpoint pair (section 5.3); rotation in place stays allowed."""
    linear2 = l_tps + r_tps  # twice the linear component
    if not ((block_forward and linear2 > 0) or (block_backward and linear2 < 0)):
        return l_tps, r_tps
    rot = int((r_tps - l_tps) / 2)  # truncated towards zero
    return -rot, rot


class UsClass(IntEnum):
    UNKNOWN = 0
    FLOOR = 1
    CLIFF = 2
    OBSTACLE = 3


NO_ECHO_MM = 65535


def classify_us(distance_mm: int, floor_mm: int, cliff_margin_mm: int, obst_margin_mm: int) -> UsClass:
    """Raw class of one ultrasonic reading (section 6.1)."""
    if distance_mm == NO_ECHO_MM or distance_mm > floor_mm + cliff_margin_mm:
        return UsClass.CLIFF
    if distance_mm < floor_mm - obst_margin_mm:
        return UsClass.OBSTACLE
    return UsClass.FLOOR


@dataclass
class CliffDebounce:
    """Cliff confirmation: on after ``confirm_on`` cliff readings, off after ``confirm_off`` floor readings."""

    confirm_on: int = 2
    confirm_off: int = 3
    active: bool = False
    _count: int = 0

    def update(self, cls: UsClass) -> bool:
        if not self.active:
            self._count = self._count + 1 if cls == UsClass.CLIFF else 0
            if self._count >= self.confirm_on:
                self.active, self._count = True, 0
        else:
            self._count = self._count + 1 if cls == UsClass.FLOOR else 0
            if self._count >= self.confirm_off:
                self.active, self._count = False, 0
        return self.active
