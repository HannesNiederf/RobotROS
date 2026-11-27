"""Mission state machine and store (spec section 9 and 12, ADR 0007). No ROS needed."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from robotros_mission.core.mission import (  # noqa: E402
    Alert,
    CancelDocking,
    CancelNav,
    Disarm,
    Engage,
    Mission,
    MissionConfig,
    Navigate,
    Release,
    Result,
    approach_pose,
)
from robotros_mission.core.store import Station, Store  # noqa: E402


def make(tmp_path: Path | None = None, cart_at: tuple[str, ...] = ("A",)) -> tuple[Mission, dict[str, int]]:
    store = Store(str(tmp_path / "m.db") if tmp_path else ":memory:")
    map_id = store.activate_map("generic", "/maps/generic.yaml", 0.05)
    ids = {}
    for name, x in (("A", 1.0), ("B", 2.0), ("C", 3.0)):
        ids[name] = store.upsert_station(Station(0, map_id, name, "STORAGE", x, 0.0, 0.0, 0.4, True))
    ids["D"] = store.upsert_station(Station(0, map_id, "D", "DOCK", 0.0, 1.0, 0.0, 0.3, True))
    for i, name in enumerate(cart_at):
        store.set_cart_location(f"cart{i + 1}", ids[name])
    m = Mission(store, MissionConfig(min_order_v=11.8, nav_timeout_s=180.0, turn_distance=0.7))
    m.set_map(True)
    m.set_safety(True)
    m.localized = True
    return m, ids


def cmds(m: Mission, now: float = 0.0, kind: type | None = None) -> list:
    out = m.tick(now)
    return [c for c in out if kind is None or isinstance(c, kind)]


def test_schema_and_migrations(tmp_path: Path) -> None:
    store = Store(str(tmp_path / "x.db"))
    assert store.schema_version() == 2
    again = Store(str(tmp_path / "x.db"))  # reopening applies nothing twice
    assert again.schema_version() == 2
    mode = again.db.execute("PRAGMA journal_mode").fetchone()[0]
    assert mode == "wal"


def test_approach_pose_storage_and_dock() -> None:
    st = Station(1, 1, "S", "STORAGE", 1.0, 2.0, 0.0, 0.4, True)
    x, y, yaw = approach_pose(st, 0.7)
    assert (x, y) == pytest.approx((1.7, 2.0))
    assert abs(abs(yaw) - math.pi) < 1e-9  # looks back at the station
    dock = Station(2, 1, "D", "DOCK", 0.0, 0.0, math.pi / 2, 0.3, True)
    x, y, yaw = approach_pose(dock, 0.7)
    assert (x, y, yaw) == pytest.approx((0.0, -0.3, math.pi / 2))


def test_create_order_checks() -> None:
    m, ids = make(cart_at=("A", "C"))
    assert m.create_order("TRANSPORT", 5, ids["A"], 999)[2] == "E304"
    assert m.create_order("TRANSPORT", 5, ids["A"], ids["A"])[2] == "E303"
    assert m.create_order("TRANSPORT", 5, ids["B"], ids["A"])[2] == "E301"   # B has no cart
    assert m.create_order("TRANSPORT", 5, ids["A"], ids["C"])[2] == "E302"   # C is occupied
    assert m.create_order("TRANSPORT", 5, ids["D"], ids["B"])[2] == "E301"   # dock is no storage place
    assert m.create_order("TRANSPORT", 10, ids["A"], ids["B"])[2] == "E309"
    ok, oid, code, _ = m.create_order("TRANSPORT", 5, ids["A"], ids["B"])
    assert ok and oid and code == ""
    # A and B are reserved now
    assert m.create_order("TRANSPORT", 5, ids["C"], ids["B"])[2] == "E302"
    assert m.create_order("TRANSPORT", 5, ids["A"], ids["B"])[2] == "E301"
    m.set_map(False)
    assert m.create_order("GOTO_POINT", 5, goal=(0.0, 0.0, 0.0))[2] == "E310"
    m.set_map(True, free_cell=lambda x, y: x < 5)
    assert m.create_order("GOTO_POINT", 5, goal=(9.0, 0.0, 0.0))[2] == "E203"
    assert m.create_order("GOTO_POINT", 5, goal=(1.0, 0.0, 0.0))[0]
    assert m.create_order("GOTO_DOCK", 5)[2] == "E309"


def test_transport_full_run() -> None:
    m, ids = make()
    _, oid, _, _ = m.create_order("TRANSPORT", 5, ids["A"], ids["B"])
    nav = cmds(m, 0.0, Navigate)
    assert m.state == "GOTO_PICKUP" and len(nav) == 1
    assert (nav[0].x, nav[0].y) == pytest.approx((1.7, 0.0))
    assert cmds(m, 1.0) == []  # nothing new while navigating
    m.nav_result(nav[0].seq, Result(True))
    eng = cmds(m, 2.0, Engage)
    assert m.state == "ENGAGE_CART" and eng[0].station_id == ids["A"]
    m.docking_result(eng[0].seq, Result(True))
    assert m.lifted and m.store.station(ids["A"]).cart_name == ""
    nav = cmds(m, 3.0, Navigate)
    assert m.state == "GOTO_DEST" and (nav[0].x, nav[0].y) == pytest.approx((2.7, 0.0))
    m.nav_result(nav[0].seq, Result(True))
    rel = cmds(m, 4.0, Release)
    assert m.state == "RELEASE_CART" and rel[0].station_id == ids["B"]
    m.docking_result(rel[0].seq, Result(True))
    cmds(m, 5.0)
    assert m.state == "IDLE" and not m.lifted
    assert m.store.order(oid).state == "COMPLETED"
    assert m.store.station(ids["B"]).cart_name == "cart1"
    assert [e for e, _ in m.store.events(oid)] == ["CREATED", "STARTED", "STEP", "STEP", "STEP", "COMPLETED"]


def test_queue_order_priority_then_age() -> None:
    m, _ = make()
    m.safety_known = True
    m.armed = False  # nothing starts while locked
    a = m.create_order("GOTO_POINT", 5, goal=(1.0, 0.0, 0.0))[1]
    b = m.create_order("GOTO_POINT", 7, goal=(2.0, 0.0, 0.0))[1]
    c = m.create_order("GOTO_POINT", 5, goal=(3.0, 0.0, 0.0))[1]
    assert [o.id for o in m.store.orders(("QUEUED",))] == [b, a, c]
    m.set_safety(True)
    cmds(m)
    assert m.active_id == b


def test_stale_result_is_ignored_and_timeout() -> None:
    m, _ = make()
    oid = m.create_order("GOTO_POINT", 5, goal=(1.0, 0.0, 0.0))[1]
    nav = cmds(m, 0.0, Navigate)[0]
    m.nav_result(nav.seq - 1, Result(True))
    assert m.store.order(oid).state == "ACTIVE"
    out = cmds(m, 181.0)
    assert any(isinstance(c, CancelNav) for c in out)
    o = m.store.order(oid)
    assert o.state == "DEFERRED" and o.error_code == "E206" and o.attempts == 1
    assert any(isinstance(c, Alert) and c.code == "E206" for c in out)
    m.nav_result(nav.seq, Result(True))  # late success of the cancelled goal changes nothing
    assert m.store.order(oid).state == "DEFERRED"
    assert m.order_action(oid, "RETRY")[0]
    o = m.store.order(oid)
    assert o.state == "QUEUED" and o.attempts == 0 and o.error_code == ""


def test_nav_failure_without_cart_defers_with_cart_holds() -> None:
    m, ids = make()
    oid = m.create_order("TRANSPORT", 5, ids["A"], ids["B"])[1]
    nav = cmds(m, 0.0, Navigate)[0]
    m.nav_result(nav.seq, Result(False, "E204", "blocked"))
    cmds(m, 1.0)
    assert m.store.order(oid).state == "DEFERRED" and m.state == "IDLE"
    # second run: fail after pickup -> HELD_WITH_CART
    m.order_action(oid, "RETRY")
    nav = cmds(m, 2.0, Navigate)[0]
    m.nav_result(nav.seq, Result(True))
    eng = cmds(m, 3.0, Engage)[0]
    m.docking_result(eng.seq, Result(True))
    nav = cmds(m, 4.0, Navigate)[0]
    m.nav_result(nav.seq, Result(False, "E204", "blocked"))
    out = cmds(m, 5.0)
    assert m.state == "HELD_WITH_CART" and m.store.order(oid).state == "ACTIVE"
    alert = [c for c in out if isinstance(c, Alert)][0]
    assert alert.needs_action and alert.options == ["RETRY", "RETURN_TO_SOURCE"]
    # new orders are accepted but not started
    other = m.create_order("GOTO_POINT", 9, goal=(1.0, 1.0, 0.0))
    assert other[2] == "E309"  # GOTO_POINT not with a cart on board
    # operator: bring the cart back
    assert m.resolve_alert(alert.id, "RETURN_TO_SOURCE")[0]
    nav = cmds(m, 6.0, Navigate)[0]
    assert m.state == "GOTO_DEST" and (nav.x, nav.y) == pytest.approx((1.7, 0.0))
    m.nav_result(nav.seq, Result(True))
    rel = cmds(m, 7.0, Release)[0]
    assert rel.station_id == ids["A"]
    m.docking_result(rel.seq, Result(True))
    cmds(m, 8.0)
    assert m.store.order(oid).state == "CANCELLED"
    assert m.store.station(ids["A"]).cart_name == "cart1" and not m.lifted


def test_cancel_cases() -> None:
    m, ids = make(cart_at=("A", "C"))
    m.armed = False
    q = m.create_order("TRANSPORT", 5, ids["C"], ids["B"])[1]
    assert m.order_action(q, "CANCEL")[0]
    assert m.store.order(q).state == "CANCELLED"
    assert m.order_action(q, "CANCEL")[1] == "E309"
    assert m.order_action(12345, "CANCEL")[1] == "E308"
    m.set_safety(True)
    a = m.create_order("GOTO_POINT", 5, goal=(1.0, 0.0, 0.0))[1]
    cmds(m)
    assert m.order_action(a, "CANCEL")[0]
    out = cmds(m)
    assert m.store.order(a).state == "CANCELLED" and m.state == "IDLE"
    # cancelling while the cart is handled is refused
    t = m.create_order("TRANSPORT", 5, ids["A"], ids["B"])[1]
    nav = cmds(m, 1.0, Navigate)[0]
    m.nav_result(nav.seq, Result(True))
    cmds(m, 2.0)
    assert m.order_action(t, "CANCEL")[1] == "E309"
    del out


def test_cancel_with_cart_returns_to_source() -> None:
    m, ids = make()
    t = m.create_order("TRANSPORT", 5, ids["A"], ids["B"])[1]
    m.nav_result(cmds(m, 0.0, Navigate)[0].seq, Result(True))
    m.docking_result(cmds(m, 1.0, Engage)[0].seq, Result(True))
    cmds(m, 2.0)  # driving to B
    assert m.order_action(t, "CANCEL")[0]
    out = cmds(m, 3.0)
    assert any(isinstance(c, CancelNav) for c in out)
    nav = [c for c in out if isinstance(c, Navigate)][0]
    assert (nav.x, nav.y) == pytest.approx((1.7, 0.0))
    m.nav_result(nav.seq, Result(True))
    m.docking_result(cmds(m, 4.0, Release)[0].seq, Result(True))
    cmds(m, 5.0)
    assert m.store.order(t).state == "CANCELLED" and m.store.station(ids["A"]).cart_name == "cart1"


def test_safety_pause_and_ack() -> None:
    m, _ = make()
    oid = m.create_order("GOTO_POINT", 5, goal=(1.0, 0.0, 0.0))[1]
    cmds(m)
    m.set_safety(False)
    out = cmds(m)
    assert any(isinstance(c, CancelNav) for c in out) and m.state == "PAUSED_SAFETY"
    # re-armed without the dialog (e.g. by hand): the order waits, nothing moves
    m.set_safety(True)
    assert cmds(m, kind=Navigate) == [] and m.state == "AWAIT_RESUME"
    assert m.order_action(oid, "RESUME_AT_STEP", "ENGAGE")[1] == "E309"  # no such step for GOTO_POINT
    assert m.order_action(oid, "RESUME_AT_STEP")[0]
    assert len(cmds(m, kind=Navigate)) == 1 and m.state == "GOTO_POINT"
    # ack variants
    m.set_safety(False)
    cmds(m)
    m.set_safety(True)
    m.ack("RESUME")
    assert len(cmds(m, kind=Navigate)) == 1
    m.set_safety(False)
    cmds(m)
    m.set_safety(True)
    m.ack("CANCEL_ORDER")
    cmds(m)
    assert m.store.order(oid).state == "CANCELLED" and m.state == "IDLE"


def test_software_estop() -> None:
    m, _ = make()
    m.create_order("GOTO_POINT", 5, goal=(1.0, 0.0, 0.0))
    cmds(m)
    m.software_estop()
    out = cmds(m)
    assert any(isinstance(c, CancelNav) for c in out) and any(isinstance(c, Disarm) for c in out)
    m.set_safety(False)
    cmds(m)
    assert m.state == "PAUSED_SAFETY"


def test_battery_gate_and_map_gate() -> None:
    m, _ = make()
    m.v_batt = 11.5
    oid = m.create_order("GOTO_POINT", 5, goal=(1.0, 0.0, 0.0))[1]
    out = cmds(m)
    assert m.state == "IDLE" and m.status()["waiting_reason"] == "BATTERY"
    assert [c.code for c in out if isinstance(c, Alert)] == ["E307"]
    assert [c for c in cmds(m) if isinstance(c, Alert)] == []  # reported once
    m.v_batt = 12.4
    cmds(m)
    assert m.active_id == oid


def test_recheck_at_start_defers() -> None:
    m, ids = make()
    m.armed = False
    oid = m.create_order("TRANSPORT", 5, ids["A"], ids["B"])[1]
    m.store.set_cart_location("cart9", ids["B"])  # someone put a cart on B meanwhile
    m.set_safety(True)
    cmds(m)
    o = m.store.order(oid)
    assert o.state == "DEFERRED" and o.error_code == "E302"


def test_docking_unavailable_and_failed() -> None:
    m, ids = make()
    oid = m.create_order("TRANSPORT", 5, ids["A"], ids["B"])[1]
    m.nav_result(cmds(m, 0.0, Navigate)[0].seq, Result(True))
    eng = cmds(m, 1.0, Engage)[0]
    m.docking_result(eng.seq, Result(False, "", "server missing"), started=False)
    cmds(m)
    assert m.store.order(oid).state == "DEFERRED" and m.store.order(oid).error_code == "E305"
    m.order_action(oid, "RETRY")
    m.nav_result(cmds(m, 2.0, Navigate)[0].seq, Result(True))
    eng = cmds(m, 3.0, Engage)[0]
    m.docking_result(eng.seq, Result(False, "E305", "lift timeout"))
    out = cmds(m)
    assert m.state == "NEEDS_OPERATOR"
    alert = [c for c in out if isinstance(c, Alert)][0]
    assert alert.options == ["RETRY", "CANCEL_ORDER"]
    assert m.resolve_alert(alert.id, "RETRY")[0]
    assert len(cmds(m, 4.0, Engage)) == 1
    m.software_estop()
    assert any(isinstance(c, CancelDocking) for c in cmds(m))


def test_restart_resumes_or_needs_operator(tmp_path: Path) -> None:
    m, ids = make(tmp_path)
    oid = m.create_order("TRANSPORT", 5, ids["A"], ids["B"])[1]
    cmds(m)
    # restart during PICKUP_NAV -> AWAIT_RESUME
    m2 = Mission(Store(str(tmp_path / "m.db")))
    m2.set_map(True)
    m2.set_safety(True)
    m2.localized = True
    assert cmds(m2, kind=Navigate) == [] and m2.state == "AWAIT_RESUME" and m2.active_id == oid
    m2.ack("RESUME")
    nav = cmds(m2, kind=Navigate)[0]
    m2.nav_result(nav.seq, Result(True))
    cmds(m2)  # ENGAGE running
    # restart during ENGAGE -> NEEDS_OPERATOR
    m3 = Mission(Store(str(tmp_path / "m.db")))
    m3.set_safety(True)
    m3.localized = True
    out = cmds(m3)
    assert m3.state == "NEEDS_OPERATOR"
    assert [c.code for c in out if isinstance(c, Alert)] == ["E305"]


def test_set_priority_and_station_helpers() -> None:
    m, ids = make()
    m.armed = False
    oid = m.create_order("GOTO_POINT", 5, goal=(1.0, 0.0, 0.0))[1]
    assert m.order_action(oid, "SET_PRIORITY", "8")[0]
    assert m.store.order(oid).priority == 8
    assert m.order_action(oid, "SET_PRIORITY", "x")[1] == "E309"
    assert m.order_action(oid, "FOO")[1] == "E309"
    t = m.create_order("TRANSPORT", 5, ids["A"], ids["B"])[1]
    assert m.store.station_in_use(ids["A"]) and not m.store.station_in_use(ids["C"])
    m.order_action(t, "CANCEL")
    m.store.delete_station(ids["A"])
    assert m.store.station(ids["A"]) is None
    assert m.store.db.execute("SELECT state FROM carts WHERE name='cart1'").fetchone()[0] == "UNKNOWN"


def test_store_usable_from_another_thread(tmp_path: Path) -> None:
    # mission_manager runs a MultiThreadedExecutor; the store is created in the main thread.
    import threading

    store = Store(str(tmp_path / "t.db"))
    errors: list[Exception] = []

    def work() -> None:
        try:
            store.activate_map("m", "/m.yaml")
            store.add_order("GOTO_POINT", 5, goal=(1.0, 2.0, 0.0))
        except Exception as e:  # noqa: BLE001
            errors.append(e)

    th = threading.Thread(target=work)
    th.start()
    th.join()
    assert errors == [] and len(store.orders(("QUEUED",))) == 1
