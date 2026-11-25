"""Plain-language activity of the robot: what it does now and what it intends (ADR 0004).

Pure Python (no ROS), so it can be unit-tested. The node feeds it with Nav2 and hardware state; the output is an
i18n key plus arguments for "now" and for "plan". Texts live in web_ui/i18n/{en,de,it}.json (spec 13.4), keys
`activity.*` and `plan.*`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

# action_msgs/GoalStatus
STATUS_EXECUTING, STATUS_SUCCEEDED, STATUS_CANCELED, STATUS_ABORTED = 2, 4, 5, 6
# robotros_interfaces/SafetyState
SAFETY_BOOT_LOCKED, SAFETY_ESTOP, SAFETY_WAIT_ACK, SAFETY_ARMED = 0, 1, 2, 3
# nav2_msgs/CollisionMonitorState
CM_DO_NOTHING, CM_STOP, CM_SLOWDOWN, CM_APPROACH, CM_LIMIT = 0, 1, 2, 3, 4

# Behavior tree node names (navigate_wait_replan.xml) -> activity key while the node is RUNNING, by priority.
BT_ACTIVITIES = (
    ("RecoveryBackUp", "activity.backing_up"),
    ("Wait", "activity.waiting"),
    ("ClearLocalCostmap", "activity.clearing_map"),
    ("ClearGlobalCostmap", "activity.clearing_map"),
)


@dataclass
class ActivityConfig:
    """Thresholds (robotros_bringup/config/activity.yaml)."""

    moving_speed: float = 0.02        # m/s: below this the robot counts as standing
    turning_rate: float = 0.05        # rad/s: above this (while standing) the robot turns on the spot
    slow_fraction: float = 0.6        # below this fraction of the profile speed the robot "drives slowly"
    nominal_speed: float = 0.15       # m/s: profile speed (speed_profiles.yaml NORMAL)
    result_hold_s: float = 10.0       # how long "goal reached / failed" stays shown
    wait_s: float = 4.0               # person_wait_s, shown in the plan while waiting
    retries: int = 3                  # recovery retries of the behavior tree
    min_hold_s: float = 0.8           # a motion-derived activity must last this long before it is shown


# Shown at once; all other keys (derived from speed, short planning steps) only after min_hold_s.
IMMEDIATE = frozenset({
    "activity.estop", "activity.not_armed", "activity.stopped_obstacle", "activity.backing_up",
    "activity.waiting", "activity.clearing_map", "activity.goal_reached", "activity.goal_canceled",
    "activity.goal_failed",
})


@dataclass
class Activity:
    key: str
    args: dict[str, str] = field(default_factory=dict)


def _num(value: float, digits: int = 2) -> str:
    return f"{value:.{digits}f}"


class ActivityTracker:
    """Collects the inputs; now() and plan() give the current text keys."""

    def __init__(self, cfg: ActivityConfig | None = None) -> None:
        self.cfg = cfg or ActivityConfig()
        self.safety: int | None = None
        self.estop_pressed = False
        self.nav_status: int | None = None
        self.result_time: float | None = None
        self.running: set[str] = set()
        self.v = 0.0
        self.w = 0.0
        self.cm_action = CM_DO_NOTHING
        self.goal: tuple[float, float] | None = None
        self.distance_remaining: float | None = None
        self.eta_s: float | None = None
        self.recoveries = 0
        self.planning = False
        self.shown: Activity | None = None
        self.candidate: str | None = None
        self.candidate_since = 0.0

    # ---- inputs -------------------------------------------------------------------------------------------
    def on_safety(self, state: int, estop_pressed: bool) -> None:
        self.safety, self.estop_pressed = state, estop_pressed

    def on_nav_status(self, status: int, t: float) -> None:
        if status != self.nav_status and status in (STATUS_SUCCEEDED, STATUS_CANCELED, STATUS_ABORTED):
            self.result_time = t
            self.running.clear()
            self.distance_remaining = self.eta_s = None
        if status == STATUS_EXECUTING and self.nav_status != STATUS_EXECUTING:
            self.recoveries = 0
            self.result_time = None
        self.nav_status = status

    def on_feedback(self, distance_remaining: float, eta_s: float, recoveries: int) -> None:
        self.distance_remaining, self.eta_s, self.recoveries = distance_remaining, eta_s, recoveries

    def on_bt(self, node_name: str, status: str) -> None:
        if status == "RUNNING":
            self.running.add(node_name)
        else:
            self.running.discard(node_name)
        if node_name == "ComputePathToPose":
            self.planning = status == "RUNNING"

    def on_motion(self, v: float, w: float) -> None:
        self.v, self.w = v, w

    def on_collision_monitor(self, action: int) -> None:
        self.cm_action = action

    def on_plan_goal(self, x: float, y: float) -> None:
        self.goal = (x, y)

    # ---- outputs ------------------------------------------------------------------------------------------
    @property
    def navigating(self) -> bool:
        return self.nav_status == STATUS_EXECUTING

    def _result(self, t: float) -> Activity | None:
        if self.result_time is None or t - self.result_time > self.cfg.result_hold_s:
            return None
        return Activity({STATUS_SUCCEEDED: "activity.goal_reached", STATUS_CANCELED: "activity.goal_canceled",
                         STATUS_ABORTED: "activity.goal_failed"}.get(self.nav_status, "activity.idle"))

    def _safety_block(self) -> Activity | None:
        if self.safety is None or self.safety == SAFETY_ARMED:
            return None
        if self.estop_pressed or self.safety == SAFETY_ESTOP:
            return Activity("activity.estop")
        return Activity("activity.not_armed")

    def _motion(self, base: str) -> Activity:
        c = self.cfg
        if abs(self.v) < c.moving_speed:
            if abs(self.w) > c.turning_rate:
                return Activity("activity.turning", {"deg_s": _num(math.degrees(abs(self.w)), 0)})
            return Activity(f"{base}.standing")
        if self.v < 0:
            return Activity("activity.reversing", {"speed": _num(-self.v)})
        speed = {"speed": _num(self.v)}
        if self.cm_action in (CM_SLOWDOWN, CM_APPROACH, CM_LIMIT):
            return Activity(f"{base}.slow_obstacle", speed)
        if base == "activity.driving" and self.v < c.slow_fraction * c.nominal_speed:
            return Activity(f"{base}.slow", speed)
        return Activity(base, speed)

    def now(self, t: float) -> Activity:
        """Debounced activity: a key that is not IMMEDIATE must last min_hold_s before it replaces the shown one,
        so the display does not flicker between e.g. "driving" and "driving slowly"."""
        raw = self.raw_now(t)
        if self.shown is None or raw.key == self.shown.key or raw.key in IMMEDIATE:
            self.shown, self.candidate = raw, None
            return raw
        if raw.key != self.candidate:
            self.candidate, self.candidate_since = raw.key, t
        if t - self.candidate_since >= self.cfg.min_hold_s:
            self.shown, self.candidate = raw, None
        return self.shown

    def raw_now(self, t: float) -> Activity:
        blocked = self._safety_block()
        if blocked:
            return blocked
        if self.cm_action == CM_STOP and (self.navigating or abs(self.v) > 0 or abs(self.w) > 0):
            return Activity("activity.stopped_obstacle")
        if self.navigating:
            for name, key in BT_ACTIVITIES:
                if name in self.running:
                    if key == "activity.waiting":
                        return Activity(key, {"seconds": _num(self.cfg.wait_s, 0)})
                    return Activity(key)
            if self.planning and abs(self.v) < self.cfg.moving_speed and abs(self.w) < self.cfg.turning_rate:
                return Activity("activity.planning")
            return self._motion("activity.driving")
        result = self._result(t)
        if result:
            return result
        if abs(self.v) >= self.cfg.moving_speed or abs(self.w) > self.cfg.turning_rate:
            return self._motion("activity.manual")
        return Activity("activity.idle")

    def plan(self, t: float) -> Activity:
        if self._safety_block():
            return Activity("plan.blocked")
        if not self.navigating:
            return Activity("plan.none")
        args: dict[str, str] = {}
        if self.goal is not None:
            args["x"], args["y"] = _num(self.goal[0], 1), _num(self.goal[1], 1)
        if self.distance_remaining is not None:
            args["dist"] = _num(self.distance_remaining, 1)
        if self.eta_s is not None and self.eta_s > 0:
            args["eta"] = _num(self.eta_s, 0)
        if "Wait" in self.running or "RecoveryBackUp" in self.running or "ClearLocalCostmap" in self.running:
            return Activity("plan.retry", {**args, "attempt": str(self.recoveries + 1),
                                           "of": str(self.cfg.retries + 1)})
        if "dist" in args and "eta" in args:
            return Activity("plan.goto_eta", args)
        return Activity("plan.goto", args)


def render(template: str, args: dict[str, str]) -> str:
    """Fill {name} placeholders; unknown names stay visible (no crash, spec 13.4)."""
    out = template
    for name, value in args.items():
        out = out.replace("{" + name + "}", value)
    return out


def args_to_list(args: dict[str, str]) -> list[str]:
    return [f"{k}={v}" for k, v in sorted(args.items())]


def args_from_list(items: list[str]) -> dict[str, str]:
    return dict(item.split("=", 1) for item in items if "=" in item)
