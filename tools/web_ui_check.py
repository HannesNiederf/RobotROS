#!/usr/bin/env python3
"""End-to-end check of the browser UI against a running simulation (ADR 0005), with headless Chromium.

Needs Playwright (`pip install playwright` and `python3 -m playwright install --with-deps chromium`).
Usage: web_ui_check.py [--url http://localhost:8080] [--goal X Y] [--shot FILE]
Checks: no JavaScript error; rosbridge connected; map shown; robot pose and safety state "Ready"; the activity
line is filled; tapping the map and confirming sends a goal and the robot drives towards it; language switch.
Exit code 0 = PASS, 1 = FAIL, 2 = Playwright missing.
"""

from __future__ import annotations

import argparse
import math
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default="http://localhost:8080/")
    ap.add_argument("--goal", nargs=2, type=float, default=[0.3, 0.8])
    ap.add_argument("--shot", default="")
    args = ap.parse_args()
    try:
        from playwright.sync_api import sync_playwright
    except ImportError:
        print("SKIP  playwright not installed")
        return 2

    failures = 0

    def report(ok: bool, text: str) -> None:
        nonlocal failures
        print(("ok    " if ok else "FAIL  ") + text, flush=True)
        failures += 0 if ok else 1

    with sync_playwright() as p:
        browser = p.chromium.launch()
        page = browser.new_page(viewport={"width": 1368, "height": 912})  # spec 13.1 design size
        errors: list[str] = []
        page.on("pageerror", lambda e: errors.append(str(e)))
        page.on("console", lambda m: errors.append(m.text) if m.type == "error" else None)
        page.goto(args.url)

        def wait_js(expr: str, timeout_s: float) -> bool:
            end = time.monotonic() + timeout_s
            while time.monotonic() < end:
                try:
                    if page.evaluate(expr):
                        return True
                except Exception:  # noqa: BLE001 - page may still be loading
                    pass
                page.wait_for_timeout(250)
            return False

        report(wait_js("document.getElementById('conn').dataset.state === 'connected'", 30),
               "rosbridge connected (header)")
        report(wait_js("window.robotros && !!window.robotros.state().map", 60), "map image shown")
        report(wait_js("window.robotros && !!window.robotros.state().pose", 30), "robot pose (/robot_pose) received")
        report(wait_js("document.getElementById('safety').dataset.state === '3'", 30),
               f"safety state shown as armed ('{page.inner_text('#safety')}')")
        report(wait_js("document.getElementById('activity-now').textContent.length > 0", 30),
               f"activity line filled ('{page.inner_text('#activity-now')}')")

        gx, gy = args.goal
        pt = page.evaluate(f"window.robotros.worldToClient({gx}, {gy})")
        if pt is None:
            report(False, "map point for the goal")
        else:
            page.mouse.click(pt["x"], pt["y"])
            text = page.inner_text("#goal-dialog-text")
            report(page.is_visible("#goal-dialog"), f"tap on the map opens the confirmation ({text})")
            page.click("#goal-ok")
            report(wait_js("!!window.robotros.state().lastGoal", 5), "goal sent (/goal_pose)")
            start = page.evaluate("window.robotros.state().pose")
            d0 = math.hypot(start["x"] - gx, start["y"] - gy)
            reached = wait_js(f"(() => {{ const p = window.robotros.state().pose; "
                              f"return Math.hypot(p.x - {gx}, p.y - {gy}) < 0.25; }})()", 240)
            end = page.evaluate("window.robotros.state().pose")
            d1 = math.hypot(end["x"] - gx, end["y"] - gy)
            report(reached, f"robot drove to the tapped goal ({d0:.2f} m -> {d1:.2f} m)")
            report(wait_js("window.robotros.state().planLength > 0", 5) or reached, "planned path received (/plan)")

        page.click("button[data-lang='de']")
        report(wait_js("document.getElementById('conn').textContent === 'Verbunden'", 5),
               f"language switch to German ('{page.inner_text('#conn')}')")
        if args.shot:
            page.screenshot(path=args.shot)
        report(not errors, "no JavaScript errors" + (f": {errors[:3]}" if errors else ""))
        browser.close()
    print("WEB UI:", "PASS" if failures == 0 else f"FAIL ({failures})")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
