"""Safety state machine tests driven by firmware/test/vectors/safety_fsm.csv (spec FW section 5)."""

from __future__ import annotations

import csv
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robotros_bridge.core.safety import (  # noqa: E402
    CliffDebounce,
    SafetyFsm,
    SafetyState,
    UsClass,
    apply_direction_lock,
    classify_us,
)

REPO_ROOT = Path(__file__).resolve().parents[4]
CSV_PATH = REPO_ROOT / "firmware" / "test" / "vectors" / "safety_fsm.csv"


def load_rows() -> list[dict[str, str]]:
    lines = [line for line in CSV_PATH.read_text().splitlines() if line and not line.startswith("#")]
    return list(csv.DictReader(lines))


ROWS = load_rows()


def fsm_in(state: str, pressed: bool) -> SafetyFsm:
    """Build an FSM that is in ``state`` with the given E-stop input, through legal transitions only."""
    fsm = SafetyFsm()
    if state == "START":
        return fsm
    if state == "ESTOP":
        fsm.reset(0, True)
    else:
        fsm.reset(0, False)
        if state in ("ARMED", "WAIT_ACK"):
            fsm.arm_request(0, True)
        if state == "WAIT_ACK":
            fsm.arm_request(0, False)
    fsm.estop_pressed = pressed  # rows 4 and similar: input level at the moment of the event
    assert fsm.state == SafetyState[state]
    return fsm


def test_table_has_all_rows_of_the_spec() -> None:
    assert [int(r["row"]) for r in ROWS] == list(range(1, 12))


@pytest.mark.parametrize("row", ROWS, ids=lambda r: f"row{r['row']}-{r['state']}-{r['event']}")
def test_table_row(row: dict[str, str]) -> None:
    pressed = row["estop_pressed"] == "1"
    event = row["event"]
    if event == "RESET":
        fsm = SafetyFsm()
        out = fsm.reset(1000, pressed)
    else:
        before_pressed = pressed if event not in ("ESTOP_PRESS", "ESTOP_RELEASE") else not pressed
        fsm = fsm_in(row["state"], before_pressed)
        if event == "ARM1":
            out = fsm.arm_request(1000, True)
        elif event == "ARM0":
            out = fsm.arm_request(1000, False)
        elif event == "ESTOP_PRESS":
            out = fsm.set_estop(1000, True)
        elif event == "ESTOP_RELEASE":
            out = fsm.set_estop(1000, False)
        elif event == "WD_HARD":
            out = fsm.tick(fsm.last_valid_ms + fsm.wd_hard_ms)
        else:
            raise AssertionError(event)
    assert fsm.state == SafetyState[row["next_state"]]
    assert fsm.arming == (row["arming"] == "1")
    expected = set(row["actions"].split(";")) - {"NONE"}
    got = set(out.actions) | {f"EVT{typ}" for typ, _ in out.events}
    if "SW_STOP_FLAG" in expected:
        assert fsm.software_stop
        got.add("SW_STOP_FLAG")
    if event == "WD_HARD":
        got.discard("BRAKE")  # braking at hard timeout is implied by disarming
    assert got == expected


def test_drive_delay_after_arming() -> None:
    fsm = SafetyFsm()
    fsm.reset(0, False)
    fsm.arm_request(1000, True)
    assert not fsm.drive_allowed(1199)
    fsm.valid_pi_message(1199)
    assert fsm.drive_allowed(1200)


def test_soft_watchdog_sets_flag_and_recovers() -> None:
    fsm = SafetyFsm()
    fsm.reset(0, False)
    fsm.arm_request(0, True)
    assert fsm.tick(499).events == []
    out = fsm.tick(500)
    assert fsm.pi_link_lost and fsm.state == SafetyState.ARMED and (50, 0) in out.events
    assert not fsm.drive_allowed(600)
    assert fsm.valid_pi_message(600).events == [(51, 0)]
    assert fsm.drive_allowed(600)


def test_hard_watchdog_needs_new_ack() -> None:
    fsm = SafetyFsm()
    fsm.reset(0, False)
    fsm.arm_request(0, True)
    fsm.tick(500)
    fsm.tick(5000)
    assert fsm.state == SafetyState.WAIT_ACK and not fsm.arming
    fsm.valid_pi_message(5100)  # a message alone does not re-arm
    assert fsm.state == SafetyState.WAIT_ACK


def test_arm_with_estop_pressed_is_refused_everywhere() -> None:
    fsm = SafetyFsm()
    fsm.reset(0, True)
    assert fsm.arm_request(10, True).actions == ["ERR5"]
    assert fsm.state == SafetyState.ESTOP


def test_watchdog_inactive_outside_armed() -> None:
    fsm = SafetyFsm()
    fsm.reset(0, False)
    assert fsm.tick(100_000).actions == []
    assert fsm.state == SafetyState.BOOT_LOCKED


@pytest.mark.parametrize(
    ("left", "right", "fwd", "back", "expected"),
    [
        (100, 100, True, False, (0, 0)),  # straight forward blocked
        (-100, -100, True, False, (-100, -100)),  # backward still allowed
        (-100, 100, True, True, (-100, 100)),  # rotation in place allowed
        (50, 150, True, False, (-50, 50)),  # forward arc: keep rotation only
        (-150, -50, False, True, (-50, 50)),  # backward arc with rear lock
        (0, 3, True, False, (-1, 1)),  # odd difference truncated towards zero
        (100, 100, False, False, (100, 100)),
    ],
)
def test_direction_lock(left: int, right: int, fwd: bool, back: bool, expected: tuple[int, int]) -> None:
    assert apply_direction_lock(left, right, fwd, back) == expected


def test_us_classification() -> None:
    assert classify_us(65535, 120, 40, 40) == UsClass.CLIFF
    assert classify_us(161, 120, 40, 40) == UsClass.CLIFF
    assert classify_us(160, 120, 40, 40) == UsClass.FLOOR
    assert classify_us(80, 120, 40, 40) == UsClass.FLOOR
    assert classify_us(79, 120, 40, 40) == UsClass.OBSTACLE


def test_cliff_debounce_two_on_three_off() -> None:
    deb = CliffDebounce()
    assert not deb.update(UsClass.CLIFF)
    assert deb.update(UsClass.CLIFF)
    assert deb.update(UsClass.FLOOR)
    assert deb.update(UsClass.FLOOR)
    assert not deb.update(UsClass.FLOOR)
    deb2 = CliffDebounce()
    deb2.update(UsClass.CLIFF)
    deb2.update(UsClass.FLOOR)  # interrupted: count restarts
    assert not deb2.update(UsClass.CLIFF)
