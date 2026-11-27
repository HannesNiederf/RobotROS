import test from "node:test";
import assert from "node:assert/strict";
import { PROFILES, headingDegrees, joystickTwist } from "../js/joystick.js";

const P = PROFILES.MANUAL;

test("centre and dead zone give no motion", () => {
  assert.deepEqual(joystickTwist(0, 0, 100, P), { v: 0, w: 0 });
  assert.deepEqual(joystickTwist(8, -5, 100, P), { v: 0, w: 0 });
});

test("full deflection reaches the profile limits with the right signs", () => {
  const up = joystickTwist(0, -100, 100, P);
  assert.equal(up.v, P.v);
  assert.equal(Math.abs(up.w), 0);
  assert.equal(joystickTwist(0, 100, 100, P).v, -P.v); // down = backwards
  assert.equal(joystickTwist(100, 0, 100, P).w, -P.w); // right = clockwise
  assert.equal(joystickTwist(-100, 0, 100, P).w, P.w);
});

test("outside the circle is clamped, half deflection is a quarter speed", () => {
  const far = joystickTwist(0, -300, 100, P);
  assert.equal(far.v, P.v);
  assert.ok(Math.abs(joystickTwist(0, -50, 100, P).v - P.v / 4) < 1e-9);
  const slow = joystickTwist(0, -100, 100, PROFILES.MANUAL_SLOW);
  assert.equal(slow.v, PROFILES.MANUAL_SLOW.v);
});

test("profiles match speed_profiles.yaml", async () => {
  const { readFileSync } = await import("node:fs");
  const yaml = readFileSync(new URL("../../ros2_ws/src/robotros_bringup/config/speed_profiles.yaml", import.meta.url), "utf8");
  for (const [name, p] of Object.entries(PROFILES)) {
    const m = yaml.match(new RegExp(`^${name}: \\{v: ([0-9.]+), w: ([0-9.]+)\\}`, "m"));
    assert.ok(m, name);
    assert.equal(Number(m[1]), p.v);
    assert.equal(Number(m[2]), p.w);
  }
});

test("heading in degrees", () => {
  assert.equal(headingDegrees(0), 0);
  assert.equal(headingDegrees(Math.PI / 2), 90);
  assert.equal(headingDegrees(-Math.PI / 2), 270);
  assert.equal(headingDegrees(Math.PI), 180);
});
