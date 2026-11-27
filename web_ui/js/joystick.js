// On-screen joystick for driving by hand (spec 13.3 "Manual", ADR 0010).
// Publishes geometry_msgs/Twist on /cmd_vel_manual while the knob is held (twist_mux: manual beats Nav2, timeout
// 0.5 s, so a lost browser stops the robot). Speeds follow the MANUAL / MANUAL_SLOW profile of speed_profiles.yaml.
import * as ros from "./ros.js";

export const PROFILES = { MANUAL: { v: 0.2, w: 0.8 }, MANUAL_SLOW: { v: 0.1, w: 0.4 } };
const DEAD_ZONE = 0.12; // fraction of the radius without motion

/**
 * Knob offset (dx right, dy down, in pixels) -> {v, w}. Up = forward, right = turn clockwise (negative w).
 * The square of the deflection gives fine control near the middle. Pure function, tested in joystick.test.mjs.
 */
export function joystickTwist(dx, dy, radius, profile) {
  let nx = dx / radius;
  let ny = -dy / radius;
  const len = Math.hypot(nx, ny);
  if (len > 1) {
    nx /= len;
    ny /= len;
  }
  if (Math.hypot(nx, ny) < DEAD_ZONE) return { v: 0, w: 0 };
  const shape = (u) => Math.sign(u) * u * u;
  return { v: shape(ny) * profile.v, w: -shape(nx) * profile.w };
}

/** Build a joystick inside `root` (an element with class "joystick"). Returns {stop}. */
export function createJoystick(root, { getProfile = () => PROFILES.MANUAL_SLOW, onMove = () => {} } = {}) {
  root.replaceChildren();
  const knob = document.createElement("div");
  knob.className = "knob";
  root.appendChild(knob);
  let active = null; // pointer id
  let twist = { v: 0, w: 0 };
  let timer = null;

  const send = () =>
    ros.publish("/cmd_vel_manual", "geometry_msgs/msg/Twist", {
      linear: { x: twist.v, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: twist.w },
    });

  const place = (dx, dy) => {
    knob.style.transform = `translate(${dx}px, ${dy}px)`;
  };

  const update = (ev) => {
    const r = root.getBoundingClientRect();
    const radius = r.width / 2;
    let dx = ev.clientX - (r.left + radius);
    let dy = ev.clientY - (r.top + radius);
    const len = Math.hypot(dx, dy);
    if (len > radius) {
      dx *= radius / len;
      dy *= radius / len;
    }
    place(dx, dy);
    twist = joystickTwist(dx, dy, radius, getProfile());
    onMove(twist);
  };

  const stop = () => {
    active = null;
    twist = { v: 0, w: 0 };
    place(0, 0);
    clearInterval(timer);
    timer = null;
    send(); // one zero command, then silence (twist_mux falls back to Nav2 after its timeout)
    onMove(twist);
    root.classList.remove("held");
  };

  root.addEventListener("pointerdown", (ev) => {
    if (active !== null) return;
    active = ev.pointerId;
    root.setPointerCapture(ev.pointerId);
    root.classList.add("held");
    update(ev);
    send();
    timer = setInterval(send, 100); // 10 Hz, well inside the 0.5 s timeout
  });
  root.addEventListener("pointermove", (ev) => {
    if (ev.pointerId === active) update(ev);
  });
  for (const type of ["pointerup", "pointercancel", "lostpointercapture"]) {
    root.addEventListener(type, (ev) => {
      if (ev.pointerId === active) stop();
    });
  }
  window.addEventListener("blur", () => active !== null && stop());
  document.addEventListener("visibilitychange", () => document.hidden && active !== null && stop());
  return { stop };
}

/** Heading of a map yaw for display: degrees 0..359, 0 = map right (+x), counter-clockwise. */
export function headingDegrees(yaw) {
  const d = Math.round((yaw * 180) / Math.PI);
  return ((d % 360) + 360) % 360;
}
