#!/usr/bin/env python3
"""End-to-end check of the browser UI against a running simulation (ADR 0005), with headless Chromium.

Needs Playwright (`pip install playwright` and `python3 -m playwright install --with-deps chromium`).
Usage: web_ui_check.py [--url http://localhost:8080] [--goal X Y] [--shot FILE]
Checks: no JavaScript error; rosbridge connected; map shown; robot pose and safety state "Ready"; the activity
line is filled; tapping the map and confirming creates a GOTO_POINT order and the robot drives there; the order
appears as COMPLETED on the Orders page; STOP disarms, the acknowledge dialog arms again; language switch.
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
            report(wait_js("!!(window.robotros.state().lastGoal || {}).orderId", 10),
                   "goal sent as GOTO_POINT order (/mission/create_order)")
            start = page.evaluate("window.robotros.state().pose")
            d0 = math.hypot(start["x"] - gx, start["y"] - gy)
            reached = wait_js(f"(() => {{ const p = window.robotros.state().pose; "
                              f"return Math.hypot(p.x - {gx}, p.y - {gy}) < 0.25; }})()", 240)
            end = page.evaluate("window.robotros.state().pose")
            d1 = math.hypot(end["x"] - gx, end["y"] - gy)
            report(reached, f"robot drove to the tapped goal ({d0:.2f} m -> {d1:.2f} m)")
            report(wait_js("window.robotros.state().planLength > 0", 5) or reached, "planned path received (/plan)")
            oid = page.evaluate("window.robotros.state().lastGoal.orderId")
            done_js = f"(window.robotros.state().orders || []).some(o => o.id === {oid} && o.state === 'COMPLETED')"
            report(wait_js(done_js, 20), f"order {oid} COMPLETED on /mission/orders")
            page.evaluate("window.robotros.showPage('orders')")
            report(wait_js(f"!!document.querySelector('#order-list [data-order-id=\"{oid}\"]')", 5),
                   "Orders page lists the order")
            page.evaluate("window.robotros.showPage('map')")

        # Software E-stop button (spec 13.2) and the acknowledge dialog (spec 6.3).
        page.click("#estop")
        report(wait_js("document.getElementById('safety').dataset.state !== '3'", 10),
               f"STOP button disarms the robot ('{page.inner_text('#safety')}')")
        report(wait_js("!document.getElementById('ack-dialog').hidden", 10), "acknowledge dialog appears")
        page.click("#ack-buttons button[data-mode='RESUME']")
        report(wait_js("document.getElementById('safety').dataset.state === '3'", 10)
               and wait_js("document.getElementById('ack-dialog').hidden", 5),
               "arming in the dialog: robot armed again, dialog closed")
        report(wait_js("document.getElementById('mission').dataset.state === 'IDLE'", 15),
               f"mission state shown ('{page.inner_text('#mission')}')")

        # Drive panel (ADR 0010): hold the joystick up -> the robot moves; teach a station with its heading.
        page.click("#drive-toggle")
        report(wait_js("!document.getElementById('drive-panel').hidden", 5), "drive panel opens")
        box = page.locator("#joystick").bounding_box()
        before = page.evaluate("window.robotros.state().pose")
        cx, cy = box["x"] + box["width"] / 2, box["y"] + box["height"] / 2
        page.mouse.move(cx, cy)
        page.mouse.down()
        page.mouse.move(cx, cy - box["height"] * 0.45, steps=5)
        moved = wait_js(f"(() => {{ const p = window.robotros.state().pose; "
                        f"return Math.hypot(p.x - {before['x']}, p.y - {before['y']}) > 0.05; }})()", 20)
        page.mouse.up()
        after = page.evaluate("window.robotros.state().pose")
        dist = math.hypot(after["x"] - before["x"], after["y"] - before["y"])
        report(moved, f"joystick drives the robot ({dist:.2f} m)")
        page.wait_for_timeout(1500)
        still = page.evaluate("window.robotros.state().pose")
        page.wait_for_timeout(1500)
        later = page.evaluate("window.robotros.state().pose")
        drift = math.hypot(later["x"] - still["x"], later["y"] - still["y"])
        report(drift < 0.02, f"robot stops after releasing the joystick ({drift:.3f} m in 1.5 s)")
        page.click("#drive-teach")
        page.fill("#teach-name", "ui_check")
        page.click("#teach-ok")
        yaw = later["yaw"]
        heading_ok = (f"Math.abs(Math.atan2(Math.sin(s.pose.theta - ({yaw})), "
                      f"Math.cos(s.pose.theta - ({yaw})))) < 0.1")
        report(wait_js(f"(window.robotros.state().stations || []).some(s => s.name === 'ui_check' && {heading_ok})",
                       10), "station taught from the drive panel with the robot's heading")
        page.click("#drive-toggle")

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
