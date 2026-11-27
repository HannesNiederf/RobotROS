"""Mission state machine and order queue (softwarespezifikation.md section 9, ADR 0007).

Pure Python, no ROS: the node feeds in safety, localization, battery and map state and the results of navigation
and docking, calls tick() periodically and executes the returned commands (Navigate, CancelNav, Engage, Release,
Disarm). Every command that starts something carries a sequence number; a result with an older number is ignored,
so a late result of a cancelled goal cannot advance the order. Time is the ROS clock passed in by the node.

Implemented here (part 1): TRANSPORT and GOTO_POINT orders with queue, reservation, checks E301-E304/E309/E310,
cancel (with the cart on board back to the source), retry, priority, resume, pause on safety, software E-stop,
HELD_WITH_CART and NEEDS_OPERATOR with operator options. Not yet: docking and charging (GOTO_DOCK, IDLE -> dock),
modes MANUAL via SetMode, keepout zones (E205), contact handling (6.4), localization quality (E201/E202).
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass, field

from .store import Order, Station, Store

# robotros_interfaces/OperatorAlert severities
INFO, WARN, ERROR = "INFO", "WARN", "ERROR"

TRANSPORT_STEPS = ("PICKUP_NAV", "ENGAGE", "DEST_NAV", "RELEASE")
STEP_STATE = {"PICKUP_NAV": "GOTO_PICKUP", "ENGAGE": "ENGAGE_CART", "DEST_NAV": "GOTO_DEST",
              "RELEASE": "RELEASE_CART", "NAV": "GOTO_POINT"}
NEXT_STEP = {"PICKUP_NAV": "ENGAGE", "ENGAGE": "DEST_NAV", "DEST_NAV": "RELEASE", "RELEASE": "DONE"}


@dataclass
class MissionConfig:
    """mission.yaml (spez_parameter_tests.md 2.3)."""

    min_order_v: float = 11.8      # V: an order starts only above this (spec 9.3), E307 otherwise
    nav_timeout_s: float = 180.0   # E206
    turn_distance: float = 0.70    # m: P_pre in front of a STORAGE station (ÄM-01 D-33)


# ---- commands for the node ----------------------------------------------------------------------
@dataclass
class Navigate:
    seq: int
    x: float
    y: float
    yaw: float


@dataclass
class CancelNav:
    pass


@dataclass
class Engage:
    seq: int
    station_id: int


@dataclass
class Release:
    seq: int
    station_id: int


@dataclass
class CancelDocking:
    pass


@dataclass
class Disarm:
    pass


@dataclass
class Alert:
    id: int
    severity: str
    code: str
    detail: str
    needs_action: bool = False
    options: list[str] = field(default_factory=list)


Command = Navigate | CancelNav | Engage | Release | CancelDocking | Disarm | Alert


@dataclass
class Result:
    ok: bool
    code: str = ""
    detail: str = ""


def approach_pose(st: Station, turn_distance: float) -> tuple[float, float, float]:
    """Navigation goal in front of a station (ÄM-01 D-33).

    STORAGE: the taught pose S is the robot under the cart, front pointing out of the box; P_pre = S + turn_distance
    along that direction, looking back at the station. DOCK: approach pose behind the dock pose, same heading.
    """
    c, s = math.cos(st.yaw), math.sin(st.yaw)
    if st.kind == "STORAGE":
        yaw = math.atan2(math.sin(st.yaw + math.pi), math.cos(st.yaw + math.pi))
        return st.x + turn_distance * c, st.y + turn_distance * s, yaw
    return st.x - st.approach_distance * c, st.y - st.approach_distance * s, st.yaw


class Mission:
    def __init__(self, store: Store, cfg: MissionConfig | None = None, mode: str = "NORMAL") -> None:
        self.store = store
        self.cfg = cfg or MissionConfig()
        self.mode = mode
        self.state = "BOOTING"
        self.safety_known = False
        self.armed = False
        self.localized = False
        self.v_batt = math.nan
        self.map_ok = False
        self.free_cell: Callable[[float, float], bool] = lambda x, y: True
        self.lifted, active = store.robot_state()
        self.active_id = 0
        self.running = ""            # step whose command is in progress ("" = nothing running)
        self.running_since = 0.0
        self.seq = 0
        self.paused = False          # active order waits for the operator (AWAIT_RESUME)
        self.held = False            # HELD_WITH_CART
        self.needs_operator = False  # NEEDS_OPERATOR
        self.waiting_reason = ""
        self.last_error = ""
        self.alert_seq = 0
        self.pending_alert: Alert | None = None
        self.out: list[Command] = []
        self.orders_changed = True
        self.stations_changed = True
        order = store.order(active) if active else None
        if order is None:
            # robot_state may lag behind: take any order still marked ACTIVE
            open_active = store.orders(("ACTIVE",))
            order = open_active[0] if open_active else None
        if order is not None and order.state == "ACTIVE":
            self.active_id = order.id
            self.paused = True  # spec 9.2 AWAIT_RESUME after a restart
            if order.step in ("ENGAGE", "RELEASE"):
                self.paused = False
                self.needs_operator = True
                self._alert(ERROR, "E305" if order.step == "ENGAGE" else "E306",
                            "Robot restarted during cart handling. Check position, then choose a step.",
                            ["RETRY", "CANCEL_ORDER"])

    # ---- inputs ---------------------------------------------------------------------------------
    def set_safety(self, armed: bool) -> None:
        self.safety_known = True
        if self.armed and not armed:
            self._stop_running()
            if self.active_id:
                self.paused = True  # spec 9.2: PAUSED_SAFETY, step stays stored; resume only after the operator
        self.armed = armed

    def set_map(self, ok: bool, free_cell: Callable[[float, float], bool] | None = None) -> None:
        self.map_ok = ok
        if free_cell is not None:
            self.free_cell = free_cell

    # ---- services -------------------------------------------------------------------------------
    def create_order(self, type_: str, priority: int, source: int = 0, dest: int = 0,
                     goal: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> tuple[bool, int, str, str]:
        if not 0 <= priority <= 9:
            return False, 0, "E309", "priority must be 0-9"
        if self.mode == "MAPPING" or not self.map_ok:
            return False, 0, "E310", "no active map"
        if type_ == "TRANSPORT":
            src, dst = self.store.station(source), self.store.station(dest)
            if src is None or dst is None:
                return False, 0, "E304", f"unknown station {source if src is None else dest}"
            if source == dest:
                return False, 0, "E303", "source and destination are the same"
            reserved = self.store.reserved_stations()
            if src.kind != "STORAGE" or not src.enabled or not src.occupied or source in reserved:
                return False, 0, "E301", self._why(src, reserved, need_cart=True)
            if dst.kind != "STORAGE" or not dst.enabled or dst.occupied or dest in reserved:
                return False, 0, "E302", self._why(dst, reserved, need_cart=False)
            order_id = self.store.add_order("TRANSPORT", priority, source, dest)
        elif type_ == "GOTO_POINT":
            if self.lifted:
                return False, 0, "E309", "GOTO_POINT is only possible without a cart on board"
            if not all(math.isfinite(v) for v in goal):
                return False, 0, "E309", "goal is not a number"
            if not self.free_cell(goal[0], goal[1]):
                return False, 0, "E203", "goal is not in a free cell of the map"
            order_id = self.store.add_order("GOTO_POINT", priority, goal=goal)
        elif type_ == "GOTO_DOCK":
            return False, 0, "E309", "docking is not implemented yet"
        else:
            return False, 0, "E309", f"unknown order type {type_!r}"
        self.orders_changed = True
        return True, order_id, "", ""

    @staticmethod
    def _why(st: Station, reserved: set[int], need_cart: bool) -> str:
        if st.kind != "STORAGE":
            return f"station {st.name} is not a storage place"
        if not st.enabled:
            return f"station {st.name} is disabled"
        if st.id in reserved:
            return f"station {st.name} is reserved by another order"
        return f"station {st.name} has no cart" if need_cart else f"station {st.name} is occupied"

    def order_action(self, order_id: int, action: str, value: str = "") -> tuple[bool, str, str]:
        order = self.store.order(order_id)
        if order is None:
            return False, "E308", f"order {order_id} not found"
        if action == "CANCEL":
            return self._cancel(order)
        if action == "RETRY":
            if order.state != "DEFERRED":
                return False, "E309", "only a deferred order can be retried"
            self.store.update_order(order.id, state="QUEUED", attempts=0, error_code=None, error_detail=None,
                                    step="PICKUP_NAV" if order.type == "TRANSPORT" else "NAV")
            self.store.event(order.id, "RETRY", "")
            self.orders_changed = True
            return True, "", ""
        if action == "SET_PRIORITY":
            if order.state not in ("QUEUED", "DEFERRED"):
                return False, "E309", "priority can only change while the order waits"
            try:
                prio = int(value)
            except ValueError:
                return False, "E309", "priority must be 0-9"
            if not 0 <= prio <= 9:
                return False, "E309", "priority must be 0-9"
            self.store.update_order(order.id, priority=prio)
            self.orders_changed = True
            return True, "", ""
        if action == "RESUME_AT_STEP":
            if order.id != self.active_id or not (self.paused or self.needs_operator):
                return False, "E309", "the order is not waiting to be resumed"
            steps = TRANSPORT_STEPS if order.type == "TRANSPORT" else ("NAV",)
            step = value or order.step
            if step not in steps:
                return False, "E309", f"unknown step {step!r}"
            self.store.update_order(order.id, step=step)
            self.store.event(order.id, "RESUME", step)
            self.paused = self.needs_operator = False
            self._clear_alert()
            self.orders_changed = True
            return True, "", ""
        return False, "E309", f"unknown action {action!r}"

    def _cancel(self, order: Order) -> tuple[bool, str, str]:
        if order.state in ("QUEUED", "DEFERRED"):
            self._finish(order, "CANCELLED", "", "")
            return True, "", ""
        if order.state != "ACTIVE":
            return False, "E309", "order is already finished"
        if self.running in ("ENGAGE", "RELEASE"):
            return False, "E309", "the cart is being handled, cancel after this step"
        if not self.lifted:
            self._stop_running()
            self._finish(order, "CANCELLED", "", "")
            return True, "", ""
        # Cart on board: bring it back to its source, then CANCELLED (spec 9.3).
        self._stop_running()
        self.store.update_order(order.id, cancel_requested=1, step="DEST_NAV")
        self.store.event(order.id, "CANCEL_RETURN", f"back to station {order.source_station_id}")
        self.held = False
        self._clear_alert()
        self.orders_changed = True
        return True, "", ""

    def resolve_alert(self, alert_id: int, option: str) -> tuple[bool, str]:
        a = self.pending_alert
        if a is None or a.id != alert_id:
            return False, "E309"
        if option not in a.options:
            return False, "E309"
        order = self.store.order(self.active_id) if self.active_id else None
        if option == "RETRY":
            self.held = self.needs_operator = False
        elif option == "RETURN_TO_SOURCE" and order is not None:
            self.held = False
            self.store.update_order(order.id, cancel_requested=1, step="DEST_NAV")
            self.store.event(order.id, "CANCEL_RETURN", f"back to station {order.source_station_id}")
        elif option == "CANCEL_ORDER" and order is not None:
            # NEEDS_OPERATOR after a failed cart step: the operator has checked that no cart is on board.
            self.needs_operator = False
            self._set_lifted(False)
            self._finish(order, "CANCELLED", order.error_code, "cancelled by the operator")
        self._clear_alert()
        self.orders_changed = True
        return True, ""

    def ack(self, resume_mode: str) -> None:
        """Called by the node after the robot was armed through the acknowledge dialog (spec 6.3)."""
        if not self.active_id:
            return
        if resume_mode == "RESUME":
            self.paused = False
        elif resume_mode == "CANCEL_ORDER":
            order = self.store.order(self.active_id)
            if order is not None:
                self._cancel(order)

    def software_estop(self) -> None:
        self._stop_running()
        if self.active_id:
            self.paused = True
        self.out.append(Disarm())

    # ---- results --------------------------------------------------------------------------------
    def nav_result(self, seq: int, res: Result) -> None:
        if seq != self.seq or self.running not in ("NAV", "PICKUP_NAV", "DEST_NAV"):
            return
        self.running = ""
        order = self.store.order(self.active_id)
        if order is None:
            return
        if res.ok:
            self._advance(order)
        else:
            self._fail(order, res.code or "E203", res.detail)

    def docking_result(self, seq: int, res: Result, started: bool = True) -> None:
        """Result of EngageCart/ReleaseCart. started=False: the action server was not reachable."""
        if seq != self.seq or self.running not in ("ENGAGE", "RELEASE"):
            return
        step, self.running = self.running, ""
        order = self.store.order(self.active_id)
        if order is None:
            return
        if res.ok:
            if step == "ENGAGE":
                cart_id = self.store.carry_cart_from(order.source_station_id)
                self.store.update_order(order.id, cart_id=cart_id or None)
                self._set_lifted(True)
            else:
                target = order.source_station_id if order.cancel_requested else order.dest_station_id
                if order.cart_id:
                    self.store.store_cart_at(order.cart_id, target)
                self._set_lifted(False)
                self.stations_changed = True
            self._advance(self.store.order(order.id) or order)
            return
        code = res.code or ("E305" if step == "ENGAGE" else "E306")
        if code in ("E301", "E302"):
            started = False  # slot check at P_pre (D-37): refused before anything moved
        if not started and step == "ENGAGE":
            self._fail(order, code, res.detail or "cart handling is not available")
        elif not started:
            self._hold(order, code, res.detail or "cart handling is not available")
        else:
            self.needs_operator = True
            self.store.update_order(order.id, error_code=code, error_detail=res.detail)
            self.store.event(order.id, "NEEDS_OPERATOR", f"{code} {res.detail}")
            self._alert(ERROR, code, res.detail or "cart handling failed. Check the robot, then choose.",
                        ["RETRY", "CANCEL_ORDER"])

    # ---- cycle ----------------------------------------------------------------------------------
    def tick(self, now: float) -> list[Command]:
        if self.running in ("NAV", "PICKUP_NAV", "DEST_NAV") and now - self.running_since > self.cfg.nav_timeout_s:
            self._stop_running()
            order = self.store.order(self.active_id)
            if order is not None:
                self._fail(order, "E206", f"navigation took longer than {self.cfg.nav_timeout_s:.0f} s")
        self.waiting_reason = ""
        if self.mode == "MAPPING":
            self.state = "MAPPING"
        elif not self.safety_known:
            self.state = "BOOTING"
        elif not self.armed:
            self.state = "PAUSED_SAFETY" if self.active_id else "LOCKED"
        elif self.needs_operator:
            self.state = "NEEDS_OPERATOR"
        elif self.held:
            self.state = "HELD_WITH_CART"
        elif self.active_id and self.paused:
            self.state = "AWAIT_RESUME"
        elif self.running:
            self.state = STEP_STATE.get(self.running, self.state)
        elif not self.localized:
            self.state = "LOCALIZING"
        elif self.active_id:
            order = self.store.order(self.active_id)
            if order is not None:
                self._start_step(order, now)
        else:
            self._pick_next(now)
        out, self.out = self.out, []
        return out

    def _pick_next(self, now: float) -> None:
        queued = self.store.orders(("QUEUED",))
        if not queued:
            self.state = "IDLE"
            return
        if not self.map_ok:
            self.state, self.waiting_reason = "IDLE", "MAP"
            return
        if math.isfinite(self.v_batt) and self.v_batt < self.cfg.min_order_v:
            self.state, self.waiting_reason = "IDLE", "BATTERY"
            if self.last_error != "E307":
                self.last_error = "E307"
                self._alert(INFO, "E307", f"battery {self.v_batt:.2f} V below {self.cfg.min_order_v:.2f} V")
            return
        if self.last_error == "E307":
            self.last_error = ""
        order = queued[0]
        if order.type == "TRANSPORT":
            # Recheck against the actual state (spec 9.3).
            src, dst = self.store.station(order.source_station_id), self.store.station(order.dest_station_id)
            others = self.store.reserved_stations(exclude_order=order.id)
            if src is None or dst is None:
                self._fail(order, "E304", "station was deleted")
                return
            if not src.enabled or not src.occupied or src.id in others:
                self._fail(order, "E301", self._why(src, others, need_cart=True))
                return
            if not dst.enabled or dst.occupied or dst.id in others:
                self._fail(order, "E302", self._why(dst, others, need_cart=False))
                return
        self.active_id = order.id
        self.store.update_order(order.id, state="ACTIVE")
        self.store.event(order.id, "STARTED", "")
        self.orders_changed = True
        self._start_step(self.store.order(order.id) or order, now)

    def _start_step(self, order: Order, now: float) -> None:
        self.seq += 1
        self.running, self.running_since = order.step, now
        self.state = STEP_STATE.get(order.step, "IDLE")
        self.store.set_robot_state(self.lifted, order.id)
        if order.step == "NAV":
            self.out.append(Navigate(self.seq, order.goal_x, order.goal_y, order.goal_yaw))
            return
        station_id = order.source_station_id
        if order.step in ("DEST_NAV", "RELEASE") and not order.cancel_requested:
            station_id = order.dest_station_id
        st = self.store.station(station_id)
        if st is None:
            self.running = ""
            self._fail(order, "E304", f"station {station_id} was deleted")
            return
        if order.step in ("PICKUP_NAV", "DEST_NAV"):
            self.out.append(Navigate(self.seq, *approach_pose(st, self.cfg.turn_distance)))
        elif order.step == "ENGAGE":
            self.out.append(Engage(self.seq, station_id))
        else:
            self.out.append(Release(self.seq, station_id))

    def _advance(self, order: Order) -> None:
        nxt = "DONE" if order.type != "TRANSPORT" else NEXT_STEP[order.step]
        if nxt == "DONE":
            self._finish(order, "CANCELLED" if order.cancel_requested else "COMPLETED", "", "")
            return
        self.store.update_order(order.id, step=nxt)
        self.store.event(order.id, "STEP", nxt)
        self.store.set_robot_state(self.lifted, order.id)
        self.orders_changed = True

    def _fail(self, order: Order, code: str, detail: str) -> None:
        if self.lifted and order.state == "ACTIVE":
            self._hold(order, code, detail)
            return
        self.store.update_order(order.id, state="DEFERRED", error_code=code, error_detail=detail,
                                attempts=order.attempts + 1)
        self.store.event(order.id, "DEFERRED", f"{code} {detail}")
        self.last_error = code
        if order.id == self.active_id:
            self.active_id = 0
            self.paused = False
            self.store.set_robot_state(self.lifted, 0)
        self.orders_changed = True
        self._alert(WARN, code, f"order {order.id}: {detail}")

    def _hold(self, order: Order, code: str, detail: str) -> None:
        self.held = True
        self.last_error = code
        self.store.update_order(order.id, error_code=code, error_detail=detail)
        self.store.event(order.id, "HELD_WITH_CART", f"{code} {detail}")
        self.orders_changed = True
        options = ["RETRY"] if order.cancel_requested else ["RETRY", "RETURN_TO_SOURCE"]
        self._alert(ERROR, code, f"order {order.id}: {detail} (cart on board)", options)

    def _finish(self, order: Order, state: str, code: str, detail: str) -> None:
        self.store.update_order(order.id, state=state, step="DONE" if state == "COMPLETED" else order.step,
                                error_code=code or None, error_detail=detail or None)
        self.store.event(order.id, state, detail)
        if order.id == self.active_id:
            self.active_id = 0
            self.paused = self.held = False
            self.store.set_robot_state(self.lifted, 0)
        self.orders_changed = True

    def _stop_running(self) -> None:
        if self.running in ("NAV", "PICKUP_NAV", "DEST_NAV"):
            self.out.append(CancelNav())
        elif self.running in ("ENGAGE", "RELEASE"):
            self.out.append(CancelDocking())
        self.running = ""
        self.seq += 1  # results of the stopped command are ignored

    def _set_lifted(self, lifted: bool) -> None:
        self.lifted = lifted
        self.store.set_robot_state(lifted, self.active_id)

    def _alert(self, severity: str, code: str, detail: str, options: list[str] | None = None) -> None:
        self.alert_seq += 1
        alert = Alert(self.alert_seq, severity, code, detail, bool(options), list(options or []))
        if options:
            self.pending_alert = alert
        self.out.append(alert)

    def _clear_alert(self) -> None:
        self.pending_alert = None

    # ---- outputs --------------------------------------------------------------------------------
    def status(self) -> dict:
        order = self.store.order(self.active_id) if self.active_id else None
        step = order.step if order else ""
        progress = 0.0
        if order is not None and order.type == "TRANSPORT" and step in TRANSPORT_STEPS:
            progress = TRANSPORT_STEPS.index(step) / len(TRANSPORT_STEPS)
        return {
            "mode": self.mode,
            "state": self.state,
            "active_order_id": self.active_id,
            "step": step,
            "progress": progress,
            "error_code": self.last_error,
            "lifted": self.lifted,
            "waiting_reason": self.waiting_reason,
            "queue_length": len(self.store.orders(("QUEUED",))),
        }
