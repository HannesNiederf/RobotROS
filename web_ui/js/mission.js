// Pure helpers for the mission parts of the UI (ADR 0008): acknowledge dialog, orders and stations.
// No DOM, no ROS: tested with node --test (web_ui/test/mission.test.mjs).

export const SAFETY_ARMED = 3;
const SAFETY_ESTOP = 1;
const OPEN = ["ACTIVE", "QUEUED", "DEFERRED"];

/**
 * Buttons of the acknowledge dialog (spec 6.3). The dialog shows while the robot is not armed (BOOT_LOCKED,
 * WAIT_ACK, ESTOP). Arming is only possible with the E-stop button released. With a stored active order the
 * operator chooses resume / keep paused / cancel, otherwise there is one "arm" button.
 * @returns {{show: boolean, enabled: boolean, buttons: {key: string, mode: string}[], messageKey: string}}
 */
export function ackDialog(conn, safety, status) {
  const live = conn === "connected";
  if (!live || !safety || safety.state === SAFETY_ARMED) {
    return { show: false, enabled: false, buttons: [], messageKey: "" };
  }
  const pressed = safety.state === SAFETY_ESTOP || safety.estop_pressed;
  const hasOrder = !!(status && status.active_order_id);
  const buttons = hasOrder
    ? [
        { key: "ack.arm_resume", mode: "RESUME" },
        { key: "ack.arm_keep", mode: "KEEP_PAUSED" },
        { key: "ack.arm_cancel", mode: "CANCEL_ORDER" },
      ]
    : [{ key: "ack.arm", mode: "RESUME" }];
  return {
    show: true,
    enabled: !pressed,
    buttons,
    messageKey: pressed ? "ack.release_estop" : hasOrder ? "ack.with_order" : "ack.text",
  };
}

/** Open orders first (as sent by the mission: queue order), then finished ones, at most `limit` finished. */
export function orderRows(orders = [], limit = 10) {
  const open = orders.filter((o) => OPEN.includes(o.state));
  const done = orders.filter((o) => !OPEN.includes(o.state)).slice(0, limit);
  return [...open, ...done];
}

/** Actions the operator may take on an order (the mission checks again). */
export function orderActions(order, status) {
  const out = [];
  if (OPEN.includes(order.state)) out.push("CANCEL");
  if (order.state === "DEFERRED") out.push("RETRY");
  const waiting = status && status.active_order_id === order.id && status.state === "AWAIT_RESUME";
  if (waiting) out.push("RESUME_AT_STEP");
  return out;
}

/** "Station name" for an id, "#id" if unknown, "" for 0. */
export function stationName(stations, id) {
  if (!id) return "";
  const s = (stations || []).find((st) => st.id === id);
  return s ? s.name : `#${id}`;
}

/** i18n key and arguments describing an order. */
export function orderText(order, stations) {
  if (order.type === "TRANSPORT") {
    return {
      key: "orders.transport",
      args: { from: stationName(stations, order.source_station_id), to: stationName(stations, order.dest_station_id) },
    };
  }
  if (order.type === "GOTO_POINT") {
    return { key: "orders.goto_point", args: { x: order.goal.x.toFixed(2), y: order.goal.y.toFixed(2) } };
  }
  return { key: "orders.goto_dock", args: {} };
}

/** Station choices for a new transport: sources have a cart, destinations are free; both storage and enabled. */
export function transportChoices(stations = []) {
  const storage = stations.filter((s) => s.kind === "STORAGE" && s.enabled);
  return { sources: storage.filter((s) => s.occupied), dests: storage.filter((s) => !s.occupied) };
}

/** Error text key for a mission error code ("" -> ""). */
export function errorKey(code) {
  return code ? `error.${code}` : "";
}
