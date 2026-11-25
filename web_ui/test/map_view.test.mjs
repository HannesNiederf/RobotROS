// node --test web_ui/test : pure helpers of the UI (no browser needed).
import test from "node:test";
import assert from "node:assert/strict";
import { canvasToWorld, fitTransform, insideMap, quatOf, worldToCanvas, yawOf } from "../js/map_view.js";
import { argsFromList, render } from "../js/i18n.js";

const meta = { resolution: 0.05, origin: { x: -4, y: -4 }, width: 160, height: 80 }; // 8 m x 4 m

test("fit keeps the aspect ratio and centres the map", () => {
  const tf = fitTransform(meta, 820, 820, 10);
  assert.equal(tf.scale, 100); // 800 px / 8 m
  assert.equal(tf.offX, 10);
  assert.equal(tf.offY, 210); // (820 - 400) / 2
});

test("world <-> canvas round trip, top of the image is the highest y", () => {
  const tf = fitTransform(meta, 820, 820, 10);
  const top = worldToCanvas(tf, -4, 0); // upper-left corner of the map (x min, y max)
  assert.deepEqual(top, { px: 10, py: 210 });
  const low = worldToCanvas(tf, 4, -4); // lower-right corner
  assert.deepEqual(low, { px: 810, py: 610 });
  const w = canvasToWorld(tf, 410, 410);
  assert.ok(Math.abs(w.x - 0) < 1e-9 && Math.abs(w.y + 2) < 1e-9);
  assert.ok(insideMap(tf, 410, 410) && !insideMap(tf, 410, 100));
});

test("yaw <-> quaternion", () => {
  for (const yaw of [0, 1, -2.5, Math.PI / 2]) assert.ok(Math.abs(yawOf(quatOf(yaw)) - yaw) < 1e-12);
});

test("i18n render: placeholders, decimal comma, unknown stays", () => {
  assert.equal(render("noch {dist} m, {x}", { dist: "1.9" }, "de"), "noch 1,9 m, {x}");
  assert.equal(render("{dist} m", { dist: "1.9" }, "en"), "1.9 m");
  assert.equal(render("{n} s", { n: "16" }, "it"), "16 s");
  assert.deepEqual(argsFromList(["a=1", "b=x=y", "bad"]), { a: "1", b: "x=y" });
});
