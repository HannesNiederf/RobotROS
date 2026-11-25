// Small observable store for what the UI shows. Pages subscribe to keys they need.
const values = {};
const listeners = {};

export function get(key) {
  return values[key];
}

export function set(key, value) {
  values[key] = value;
  for (const fn of listeners[key] || []) fn(value);
}

export function on(key, fn) {
  (listeners[key] ||= []).push(fn);
  if (key in values) fn(values[key]);
}

// For the end-to-end test (tools/web_ui_check.py): read-only view of the state.
export function snapshot() {
  return { ...values };
}
