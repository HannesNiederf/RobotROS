"""SQLite store of the mission (softwarespezifikation.md section 12, D-28: only the mission node writes).

Pure Python (sqlite3), no ROS. Migrations are the numbered files in robotros_mission/sql/; the applied version is
kept in settings(key='schema_version'). Timestamps are informative text (spec 15.1); no logic depends on them.
"""

from __future__ import annotations

import math
import sqlite3
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path

SQL_DIR = Path(__file__).resolve().parent.parent / "sql"

OPEN_STATES = ("QUEUED", "ACTIVE", "DEFERRED")


def now_text() -> str:
    """Informative timestamp for the database (never used for logic)."""
    return datetime.now(UTC).isoformat(timespec="seconds")


@dataclass
class Station:
    id: int
    map_id: int
    name: str
    kind: str
    x: float
    y: float
    yaw: float
    approach_distance: float
    enabled: bool
    cart_name: str = ""  # cart stored here, "" = free

    @property
    def occupied(self) -> bool:
        return self.cart_name != ""


@dataclass
class Order:
    id: int
    type: str
    origin: str
    priority: int
    source_station_id: int
    dest_station_id: int
    goal_x: float
    goal_y: float
    goal_yaw: float
    cart_id: int
    state: str
    step: str
    error_code: str
    error_detail: str
    attempts: int
    created_at: str
    cancel_requested: bool


class StoreError(Exception):
    """Database problem (E401)."""


class Store:
    def __init__(self, path: str) -> None:
        if path != ":memory:":
            Path(path).expanduser().parent.mkdir(parents=True, exist_ok=True)
            path = str(Path(path).expanduser())
        # The node uses the store from several executor threads, always under its own lock.
        self.db = sqlite3.connect(path, isolation_level=None, check_same_thread=False)
        self.db.row_factory = sqlite3.Row
        self.db.execute("PRAGMA foreign_keys=ON")
        if path != ":memory:":
            self.db.execute("PRAGMA journal_mode=WAL")
        self.db.execute("PRAGMA synchronous=FULL")
        self.migrate()

    @contextmanager
    def tx(self) -> Iterator[None]:
        """One transaction; rolled back and raised as StoreError (E401) on any SQLite error."""
        try:
            self.db.execute("BEGIN")
            yield
            self.db.execute("COMMIT")
        except sqlite3.Error as e:
            if self.db.in_transaction:
                self.db.execute("ROLLBACK")
            raise StoreError(str(e)) from e

    # ---- schema ---------------------------------------------------------------------------------
    def schema_version(self) -> int:
        exists = self.db.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='settings'").fetchone()
        if not exists:
            return 0
        row = self.db.execute("SELECT value FROM settings WHERE key='schema_version'").fetchone()
        return int(row["value"]) if row else 0

    def migrate(self) -> None:
        current = self.schema_version()
        for f in sorted(SQL_DIR.glob("[0-9][0-9][0-9]_*.sql")):
            number = int(f.name[:3])
            if number <= current:
                continue
            try:
                self.db.execute("BEGIN")
                for statement in f.read_text(encoding="utf-8").split(";"):
                    lines = [ln for ln in statement.splitlines() if not ln.strip().startswith("--")]
                    if "".join(lines).strip():
                        self.db.execute("\n".join(lines))
                self.db.execute("INSERT OR REPLACE INTO settings(key, value) VALUES('schema_version', ?)",
                                (str(number),))
                self.db.execute("COMMIT")
            except sqlite3.Error as e:
                self.db.execute("ROLLBACK")
                raise StoreError(f"migration {f.name}: {e}") from e

    # ---- maps -----------------------------------------------------------------------------------
    def activate_map(self, name: str, yaml_path: str, resolution: float = 0.0) -> int:
        """Make the map with this name the active one (created if missing). Returns its id."""
        with self.tx():
            self.db.execute("UPDATE maps SET is_active=0")
            row = self.db.execute("SELECT id FROM maps WHERE name=?", (name,)).fetchone()
            if row:
                map_id = int(row["id"])
                self.db.execute("UPDATE maps SET is_active=1, yaml_path=? WHERE id=?", (yaml_path, map_id))
            else:
                cur = self.db.execute(
                    "INSERT INTO maps(name, yaml_path, resolution, created_at, is_active) VALUES(?,?,?,?,1)",
                    (name, yaml_path, resolution, now_text()))
                map_id = int(cur.lastrowid)
        return map_id

    def active_map_id(self) -> int:
        row = self.db.execute("SELECT id FROM maps WHERE is_active=1").fetchone()
        return int(row["id"]) if row else 0

    # ---- stations and carts ---------------------------------------------------------------------
    def stations(self, map_id: int | None = None) -> list[Station]:
        map_id = self.active_map_id() if map_id is None else map_id
        rows = self.db.execute(
            "SELECT s.*, (SELECT name FROM carts c WHERE c.station_id=s.id) AS cart_name "
            "FROM stations s WHERE s.map_id=? ORDER BY s.id", (map_id,)).fetchall()
        return [self._station(r) for r in rows]

    def station(self, station_id: int) -> Station | None:
        r = self.db.execute(
            "SELECT s.*, (SELECT name FROM carts c WHERE c.station_id=s.id) AS cart_name "
            "FROM stations s WHERE s.id=?", (station_id,)).fetchone()
        return self._station(r) if r else None

    @staticmethod
    def _station(r: sqlite3.Row) -> Station:
        return Station(int(r["id"]), int(r["map_id"]), r["name"], r["kind"], float(r["x"]), float(r["y"]),
                       float(r["yaw"]), float(r["approach_distance"]), bool(r["enabled"]), r["cart_name"] or "")

    def upsert_station(self, s: Station) -> int:
        """Create (s.id == 0, on the active map) or update a station. Cart occupancy is not touched here."""
        try:
            if s.id == 0:
                cur = self.db.execute(
                    "INSERT INTO stations(map_id, name, kind, x, y, yaw, approach_distance, enabled) "
                    "VALUES(?,?,?,?,?,?,?,?)",
                    (s.map_id, s.name, s.kind, s.x, s.y, s.yaw, s.approach_distance, int(s.enabled)))
                return int(cur.lastrowid)
            self.db.execute(
                "UPDATE stations SET name=?, kind=?, x=?, y=?, yaw=?, approach_distance=?, enabled=? WHERE id=?",
                (s.name, s.kind, s.x, s.y, s.yaw, s.approach_distance, int(s.enabled), s.id))
            return s.id
        except sqlite3.Error as e:
            raise StoreError(str(e)) from e

    def delete_station(self, station_id: int) -> None:
        """Delete a station that no open order uses (check station_in_use first). A cart there becomes UNKNOWN;
        finished orders lose the reference (their events keep the history)."""
        with self.tx():
            self.db.execute("UPDATE carts SET station_id=NULL, state='UNKNOWN' WHERE station_id=?", (station_id,))
            self.db.execute("UPDATE orders SET source_station_id=NULL WHERE source_station_id=?", (station_id,))
            self.db.execute("UPDATE orders SET dest_station_id=NULL WHERE dest_station_id=?", (station_id,))
            self.db.execute("DELETE FROM stations WHERE id=?", (station_id,))

    def station_in_use(self, station_id: int) -> bool:
        q = f"SELECT 1 FROM orders WHERE state IN {OPEN_STATES} AND (source_station_id=? OR dest_station_id=?)"
        return self.db.execute(q, (station_id, station_id)).fetchone() is not None

    def set_cart_location(self, cart_name: str, station_id: int) -> None:
        """Put a cart at a station (0 = nowhere, state UNKNOWN). Any other cart at that station is removed."""
        state = "STORED" if station_id else "UNKNOWN"
        with self.tx():
            if station_id:
                self.db.execute("UPDATE carts SET station_id=NULL, state='UNKNOWN' WHERE station_id=? AND name<>?",
                                (station_id, cart_name))
            self.db.execute(
                "INSERT INTO carts(name, station_id, state) VALUES(?,?,?) "
                "ON CONFLICT(name) DO UPDATE SET station_id=excluded.station_id, state=excluded.state",
                (cart_name, station_id or None, state))

    def carry_cart_from(self, station_id: int) -> int:
        """The cart at this station is now on board (CARRIED). Returns its id (0 if the station had none)."""
        row = self.db.execute("SELECT id FROM carts WHERE station_id=?", (station_id,)).fetchone()
        if not row:
            return 0
        self.db.execute("UPDATE carts SET station_id=NULL, state='CARRIED' WHERE id=?", (row["id"],))
        return int(row["id"])

    def store_cart_at(self, cart_id: int, station_id: int) -> None:
        self.db.execute("UPDATE carts SET station_id=?, state='STORED' WHERE id=?", (station_id, cart_id))

    # ---- orders ---------------------------------------------------------------------------------
    def add_order(self, type_: str, priority: int, source: int = 0, dest: int = 0,
                  goal: tuple[float, float, float] = (math.nan, math.nan, math.nan), origin: str = "USER") -> int:
        cur = self.db.execute(
            "INSERT INTO orders(type, origin, priority, source_station_id, dest_station_id, goal_x, goal_y, goal_yaw,"
            " state, step, created_at) VALUES(?,?,?,?,?,?,?,?,?,?,?)",
            (type_, origin, priority, source or None, dest or None, *[None if math.isnan(v) else v for v in goal],
             "QUEUED", "PICKUP_NAV" if type_ == "TRANSPORT" else "NAV", now_text()))
        order_id = int(cur.lastrowid)
        self.event(order_id, "CREATED", "")
        return order_id

    def order(self, order_id: int) -> Order | None:
        r = self.db.execute("SELECT * FROM orders WHERE id=?", (order_id,)).fetchone()
        return self._order(r) if r else None

    def orders(self, states: tuple[str, ...] | None = None, limit: int = 50) -> list[Order]:
        """Open orders first in queue order, then the most recent finished ones."""
        if states:
            marks = ",".join("?" * len(states))
            rows = self.db.execute(f"SELECT * FROM orders WHERE state IN ({marks}) ORDER BY priority DESC, id ASC",
                                   states).fetchall()
            return [self._order(r) for r in rows]
        open_rows = self.db.execute(
            f"SELECT * FROM orders WHERE state IN {OPEN_STATES} ORDER BY priority DESC, id ASC").fetchall()
        done_rows = self.db.execute(
            f"SELECT * FROM orders WHERE state NOT IN {OPEN_STATES} ORDER BY id DESC LIMIT ?", (limit,)).fetchall()
        return [self._order(r) for r in list(open_rows) + list(done_rows)]

    @staticmethod
    def _order(r: sqlite3.Row) -> Order:
        def f(v: object) -> float:
            return math.nan if v is None else float(v)  # type: ignore[arg-type]

        return Order(int(r["id"]), r["type"], r["origin"] or "USER", int(r["priority"]),
                     int(r["source_station_id"] or 0), int(r["dest_station_id"] or 0), f(r["goal_x"]),
                     f(r["goal_y"]), f(r["goal_yaw"]), int(r["cart_id"] or 0), r["state"], r["step"] or "",
                     r["error_code"] or "", r["error_detail"] or "", int(r["attempts"] or 0), r["created_at"] or "",
                     bool(r["cancel_requested"]))

    def update_order(self, order_id: int, **fields: object) -> None:
        if not fields:
            return
        if fields.get("state") == "ACTIVE":
            fields.setdefault("started_at", now_text())
        if fields.get("state") in ("COMPLETED", "CANCELLED"):
            fields.setdefault("finished_at", now_text())
        cols = ", ".join(f"{k}=?" for k in fields)
        self.db.execute(f"UPDATE orders SET {cols} WHERE id=?", (*fields.values(), order_id))

    def reserved_stations(self, exclude_order: int = 0) -> set[int]:
        """Stations of QUEUED or ACTIVE orders (spec 9.3 reservation)."""
        rows = self.db.execute(
            "SELECT source_station_id, dest_station_id FROM orders WHERE state IN ('QUEUED','ACTIVE') AND id<>?",
            (exclude_order,)).fetchall()
        return {int(v) for r in rows for v in (r[0], r[1]) if v}

    def event(self, order_id: int, event: str, detail: str) -> None:
        self.db.execute("INSERT INTO order_events(order_id, ts, event, detail) VALUES(?,?,?,?)",
                        (order_id, now_text(), event, detail))

    def events(self, order_id: int) -> list[tuple[str, str]]:
        rows = self.db.execute("SELECT event, detail FROM order_events WHERE order_id=? ORDER BY id",
                               (order_id,)).fetchall()
        return [(r["event"], r["detail"]) for r in rows]

    # ---- robot state ----------------------------------------------------------------------------
    def robot_state(self) -> tuple[bool, int]:
        r = self.db.execute("SELECT lifted, active_order_id FROM robot_state WHERE id=1").fetchone()
        return (bool(r["lifted"]), int(r["active_order_id"] or 0)) if r else (False, 0)

    def set_robot_state(self, lifted: bool, active_order_id: int) -> None:
        self.db.execute(
            "INSERT INTO robot_state(id, lifted, active_order_id, updated_at) VALUES(1,?,?,?) "
            "ON CONFLICT(id) DO UPDATE SET lifted=excluded.lifted, active_order_id=excluded.active_order_id, "
            "updated_at=excluded.updated_at", (int(lifted), active_order_id or None, now_text()))
