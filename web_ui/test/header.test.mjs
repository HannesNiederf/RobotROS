import test from "node:test";
import assert from "node:assert/strict";
import { headerView } from "../js/header.js";

const safety = { state: 3 };
const activity = { now_key: "activity.idle", now_args: [], plan_key: "plan.none", plan_args: [] };

test("connected: current safety state and activity", () => {
  const v = headerView("connected", safety, activity);
  assert.equal(v.safetyKey, "safety.armed");
  assert.equal(v.safetyState, "3");
  assert.equal(v.now.key, "activity.idle");
  assert.equal(v.stale, false);
});

test("lost: last values are not shown as current", () => {
  const v = headerView("lost", safety, activity);
  assert.equal(v.connKey, "ui.connection.lost");
  assert.equal(v.safetyKey, "safety.unknown");
  assert.equal(v.safetyState, "");
  assert.equal(v.now, null);
  assert.equal(v.plan, null);
  assert.equal(v.stale, true);
});

test("connecting without data", () => {
  const v = headerView("connecting", undefined, undefined);
  assert.equal(v.safetyKey, "safety.unknown");
  assert.equal(v.stale, true);
});
