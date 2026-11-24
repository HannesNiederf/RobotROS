"""Checks for the Claude Code project permissions in .claude/settings.json.

These tests only check the file itself (structure and required rules). Whether
Claude Code matches the rules as intended is checked with `claude doctor` and by
the owner on the Surface (see docs/claude_code_permissions.md).
"""

from __future__ import annotations

import json
import re
from pathlib import Path

SETTINGS = Path(__file__).resolve().parents[2] / ".claude" / "settings.json"

RULE_RE = re.compile(r"^(Bash|Read|Edit)\((.+)\)$")

# Rules that must stay in place (CLAUDE.md: no motors, firmware or relays without the owner's OK).
REQUIRED_DENY = {
    "Bash(sudo *)",
    "Bash(git push --force *)",
    "Bash(git push * main*)",
    "Edit(//dev/tty*)",
    "Read(//dev/tty*)",
    "Edit(//dev/robotros_*)",
}
REQUIRED_ASK = {
    "Bash(pio run *upload*)",
    "Bash(pio test *-e drive*)",
    "Bash(avrdude *)",
    "Bash(ros2 topic pub *)",
    "Bash(ros2 service call *)",
    "Bash(ros2 action send_goal *)",
    "Bash(ros2 run robotros_bridge *)",
    "Bash(*hil_smoke*)",
    "Bash(ssh *)",
}


def load() -> dict:
    return json.loads(SETTINGS.read_text(encoding="utf-8"))


def test_file_is_valid_json() -> None:
    assert isinstance(load(), dict)


def test_bypass_mode_is_disabled() -> None:
    assert load()["permissions"]["disableBypassPermissionsMode"] == "disable"


def test_no_allow_rules() -> None:
    # Allow rules belong in each user's settings.local.json, not in the shared file.
    assert "allow" not in load()["permissions"]


def test_required_rules_present() -> None:
    perms = load()["permissions"]
    assert REQUIRED_DENY <= set(perms["deny"])
    assert REQUIRED_ASK <= set(perms["ask"])


def test_rules_are_well_formed() -> None:
    perms = load()["permissions"]
    for rule in perms["deny"] + perms["ask"]:
        m = RULE_RE.match(rule)
        assert m, f"unexpected rule shape: {rule}"
        spec = m.group(2)
        # ':*' is only valid at the very end; mixing it with '*' is never matched.
        assert ":*" not in spec[:-2], f"':*' not at the end: {rule}"
        if m.group(1) in ("Read", "Edit") and spec.startswith("/dev"):
            raise AssertionError(f"absolute paths need '//': {rule}")


def test_dev_null_not_blocked() -> None:
    # Redirects like '2>/dev/null' are checked against Edit rules and must keep working.
    perms = load()["permissions"]
    for rule in perms["deny"] + perms["ask"]:
        assert rule not in ("Edit(//dev/**)", "Edit(//dev/*)", "Read(//dev/**)")


def test_no_duplicates() -> None:
    perms = load()["permissions"]
    for key in ("deny", "ask"):
        assert len(perms[key]) == len(set(perms[key])), key
