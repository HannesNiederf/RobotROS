"""Activity display logic (robotros_mission/core/activity.py, ADR 0004). Needs no ROS."""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from robotros_mission.core.activity import (  # noqa: E402
    CM_SLOWDOWN,
    CM_STOP,
    SAFETY_ARMED,
    SAFETY_ESTOP,
    SAFETY_WAIT_ACK,
    STATUS_ABORTED,
    STATUS_EXECUTING,
    STATUS_SUCCEEDED,
    ActivityConfig,
    ActivityTracker,
    args_from_list,
    args_to_list,
    render,
)

I18N = PKG.parents[2] / "web_ui" / "i18n"


def armed() -> ActivityTracker:
    t = ActivityTracker(ActivityConfig(nominal_speed=0.15, min_hold_s=0.0))
    t.on_safety(SAFETY_ARMED, False)
    return t


def navigating() -> ActivityTracker:
    t = armed()
    t.on_nav_status(STATUS_EXECUTING, 0.0)
    t.on_plan_goal(2.5, -0.775)
    return t


def test_idle_and_safety() -> None:
    t = armed()
    assert t.now(0).key == "activity.idle" and t.plan(0).key == "plan.none"
    t.on_safety(SAFETY_ESTOP, True)
    assert t.now(0).key == "activity.estop" and t.plan(0).key == "plan.blocked"
    t.on_safety(SAFETY_WAIT_ACK, False)
    assert t.now(0).key == "activity.not_armed"


def test_driving_speeds_and_turning() -> None:
    t = navigating()
    t.on_motion(0.15, 0.0)
    assert t.now(1).key == "activity.driving" and t.now(1).args == {"speed": "0.15"}
    t.on_motion(0.06, 0.1)
    assert t.now(1).key == "activity.driving.slow"
    t.on_collision_monitor(CM_SLOWDOWN)
    assert t.now(1).key == "activity.driving.slow_obstacle"
    t.on_collision_monitor(0)
    t.on_motion(0.0, 0.5)
    a = t.now(1)
    assert a.key == "activity.turning" and a.args == {"deg_s": "29"}
    t.on_motion(0.0, 0.0)
    assert t.now(1).key == "activity.driving.standing"
    t.on_bt("ComputePathToPose", "RUNNING")
    assert t.now(1).key == "activity.planning"


def test_recovery_sequence_and_plan() -> None:
    t = navigating()
    t.on_feedback(1.94, 16.2, 0)
    p = t.plan(1)
    assert p.key == "plan.goto_eta" and p.args == {"x": "2.5", "y": "-0.8", "dist": "1.9", "eta": "16"}
    t.on_bt("FollowPath", "FAILURE")
    t.on_bt("Wait", "RUNNING")
    assert t.now(2).key == "activity.waiting" and t.now(2).args == {"seconds": "4"}
    assert t.plan(2).key == "plan.retry" and t.plan(2).args["attempt"] == "1" and t.plan(2).args["of"] == "4"
    t.on_bt("Wait", "SUCCESS")
    t.on_bt("ClearLocalCostmap", "RUNNING")
    assert t.now(3).key == "activity.clearing_map"
    t.on_bt("ClearLocalCostmap", "SUCCESS")
    t.on_bt("RecoveryBackUp", "RUNNING")
    assert t.now(4).key == "activity.backing_up"
    t.on_bt("RecoveryBackUp", "SUCCESS")
    t.on_motion(-0.05, 0.0)
    assert t.now(5).key == "activity.reversing"


def test_results_are_held_then_idle() -> None:
    t = navigating()
    t.on_nav_status(STATUS_SUCCEEDED, 10.0)
    assert t.now(12).key == "activity.goal_reached" and t.plan(12).key == "plan.none"
    assert t.now(25).key == "activity.idle"
    t.on_nav_status(STATUS_EXECUTING, 30.0)
    t.on_nav_status(STATUS_ABORTED, 40.0)
    assert t.now(41).key == "activity.goal_failed"


def test_obstacle_stop_and_manual() -> None:
    t = armed()
    t.on_motion(0.2, 0.0)
    assert t.now(0).key == "activity.manual"
    t.on_collision_monitor(CM_STOP)
    assert t.now(0).key == "activity.stopped_obstacle"


def test_render_and_args() -> None:
    assert render("noch {dist} m, {x}", {"dist": "1,9"}) == "noch 1,9 m, {x}"
    assert args_from_list(args_to_list({"b": "2", "a": "1"})) == {"a": "1", "b": "2"}


def test_every_key_has_a_text_with_its_placeholders() -> None:
    """Every key the tracker can produce exists in en/de/it and the texts use exactly its arguments."""
    texts = {lang: json.loads((I18N / f"{lang}.json").read_text(encoding="utf-8")) for lang in ("en", "de", "it")}
    src = (PKG / "robotros_mission" / "core" / "activity.py").read_text(encoding="utf-8")
    keys = set(re.findall(r'"((?:activity|plan)\.[a-z_.]+)"', src))
    keys |= {f"activity.{b}.{s}" for b in ("driving", "manual") for s in ("slow", "slow_obstacle", "standing")}
    for lang, data in texts.items():
        missing = keys - data.keys()
        assert not missing, (lang, missing)


def test_debounce_keeps_short_flicker_out() -> None:
    t = ActivityTracker(ActivityConfig(nominal_speed=0.15, min_hold_s=0.8))
    t.on_safety(SAFETY_ARMED, False)
    t.on_nav_status(STATUS_EXECUTING, 0.0)
    t.on_motion(0.15, 0.0)
    assert t.now(0.0).key == "activity.driving"
    t.on_motion(0.06, 0.0)
    assert t.now(0.3).key == "activity.driving"          # slow for 0.3 s: not shown yet
    t.on_motion(0.15, 0.0)
    assert t.now(0.5).key == "activity.driving"
    t.on_motion(0.06, 0.0)
    assert t.now(0.6).key == "activity.driving"
    assert t.now(1.5).key == "activity.driving.slow"     # slow for 0.9 s: shown
    t.on_bt("Wait", "RUNNING")
    assert t.now(1.6).key == "activity.waiting"          # recovery steps are shown at once


def test_manual_has_no_slow_variant() -> None:
    t = armed()
    t.on_motion(0.05, 0.0)
    assert t.now(0).key == "activity.manual"
