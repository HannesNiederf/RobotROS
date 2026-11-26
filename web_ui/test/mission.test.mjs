import test from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import {
  ackDialog,
  errorKey,
  orderActions,
  orderRows,
  orderText,
  stationName,
  transportChoices,
} from "../js/mission.js";

const stations = [
  { id: 1, name: "A", kind: "STORAGE", enabled: true, occupied: true, cart_name: "c1" },
  { id: 2, name: "B", kind: "STORAGE", enabled: true, occupied: false, cart_name: "" },
  { id: 3, name: "C", kind: "STORAGE", enabled: false, occupied: false, cart_name: "" },
  { id: 4, name: "Dock", kind: "DOCK", enabled: true, occupied: false, cart_name: "" },
];

test("ack dialog hidden while armed or disconnected", () => {
  assert.equal(ackDialog("connected", { state: 3 }, null).show, false);
  assert.equal(ackDialog("lost", { state: 2 }, null).show, false);
  assert.equal(ackDialog("connected", undefined, null).show, false);
});

test("ack dialog: one button without order, three with an order, disabled while E-stop pressed", () => {
  const plain = ackDialog("connected", { state: 2, estop_pressed: false }, { active_order_id: 0 });
  assert.equal(plain.show, true);
  assert.equal(plain.enabled, true);
  assert.deepEqual(plain.buttons.map((b) => b.mode), ["RESUME"]);
  const withOrder = ackDialog("connected", { state: 0, estop_pressed: false }, { active_order_id: 7 });
  assert.deepEqual(withOrder.buttons.map((b) => b.mode), ["RESUME", "KEEP_PAUSED", "CANCEL_ORDER"]);
  const pressed = ackDialog("connected", { state: 1, estop_pressed: true }, null);
  assert.equal(pressed.enabled, false);
  assert.equal(pressed.messageKey, "ack.release_estop");
});

test("orders: open first, finished limited; actions per state", () => {
  const orders = [
    { id: 5, state: "ACTIVE" },
    { id: 6, state: "QUEUED" },
    { id: 3, state: "DEFERRED" },
    { id: 2, state: "COMPLETED" },
    { id: 1, state: "CANCELLED" },
  ];
  assert.deepEqual(orderRows(orders, 1).map((o) => o.id), [5, 6, 3, 2]);
  assert.deepEqual(orderActions({ id: 6, state: "QUEUED" }, null), ["CANCEL"]);
  assert.deepEqual(orderActions({ id: 3, state: "DEFERRED" }, null), ["CANCEL", "RETRY"]);
  assert.deepEqual(orderActions({ id: 5, state: "ACTIVE" }, { active_order_id: 5, state: "AWAIT_RESUME" }),
    ["CANCEL", "RESUME_AT_STEP"]);
  assert.deepEqual(orderActions({ id: 2, state: "COMPLETED" }, null), []);
});

test("order text and station names", () => {
  assert.equal(stationName(stations, 2), "B");
  assert.equal(stationName(stations, 99), "#99");
  assert.equal(stationName(stations, 0), "");
  const tr = orderText({ type: "TRANSPORT", source_station_id: 1, dest_station_id: 2 }, stations);
  assert.deepEqual(tr, { key: "orders.transport", args: { from: "A", to: "B" } });
  const gp = orderText({ type: "GOTO_POINT", goal: { x: 1, y: -0.5 } }, stations);
  assert.deepEqual(gp.args, { x: "1.00", y: "-0.50" });
});

test("transport choices: enabled storage places, sources with cart, destinations free", () => {
  const c = transportChoices(stations);
  assert.deepEqual(c.sources.map((s) => s.name), ["A"]);
  assert.deepEqual(c.dests.map((s) => s.name), ["B"]);
});

test("every error code the mission sends has a text in en.json", () => {
  const en = JSON.parse(readFileSync(new URL("../i18n/en.json", import.meta.url)));
  for (const code of ["E101", "E102", "E203", "E206", "E301", "E302", "E303", "E304", "E305", "E306", "E307",
    "E308", "E309", "E310", "E401"]) {
    assert.ok(en[errorKey(code)], code);
  }
  assert.equal(errorKey(""), "");
  for (const s of ["BOOTING", "LOCKED", "IDLE", "AWAIT_RESUME", "GOTO_POINT", "PAUSED_SAFETY", "HELD_WITH_CART"]) {
    assert.ok(en[`mission.state.${s}`], s);
  }
});
