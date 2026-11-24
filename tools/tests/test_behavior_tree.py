"""Structure of the navigation behavior tree (softwarespezifikation.md 7.4, ADR 0002). Needs no ROS."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
BT = ROOT / "ros2_ws/src/robotros_bringup/behavior_trees/navigate_wait_replan.xml"
PARAMS = ROOT / "docs/spec/spez_parameter_tests.md"


def tree() -> ET.Element:
    return ET.parse(BT).getroot()


def test_recovery_waits_clears_then_backs_up() -> None:
    recovery = tree().find("BehaviorTree/RecoveryNode")
    assert recovery is not None and recovery.get("number_of_retries") == "3"
    seq = recovery[1]
    assert [c.tag for c in seq] == ["Wait", "ClearEntireCostmap", "ForceSuccess"]
    assert float(seq[0].get("wait_duration")) == 4.0
    assert seq[1].get("service_name") == "local_costmap/clear_entirely_local_costmap"
    backup = seq[2].find("BackUp")
    assert backup is not None
    # D-29: reversing in the recovery only at 0.05 m/s; distance = recovery_backup_m start value 0.10 m.
    assert float(backup.get("backup_speed")) == 0.05
    assert float(backup.get("backup_dist")) == 0.10
    assert "0,10" in next(line for line in PARAMS.read_text(encoding="utf-8").splitlines()
                          if "`recovery_backup_m`" in line)


def test_backup_behavior_is_loaded() -> None:
    nav2 = yaml.safe_load((ROOT / "ros2_ws/src/robotros_bringup/config/nav2.yaml").read_text(encoding="utf-8"))
    params = nav2["behavior_server"]["ros__parameters"]
    assert "backup" in params["behavior_plugins"]
    assert params["backup"]["plugin"] == "nav2_behaviors::BackUp"
