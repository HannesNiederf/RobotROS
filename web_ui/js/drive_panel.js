// Drive panel on the map page (ADR 0010): joystick, slow/normal speed, pose and heading of the robot, teach a
// station right here (position and heading are saved), and the "order paused" bar with resume / cancel.
import * as ros from "./ros.js";
import * as state from "./state.js";
import { t } from "./i18n.js";
import { errorKey } from "./mission.js";
import { PROFILES, createJoystick, headingDegrees } from "./joystick.js";
import { teachStation } from "./pages/stations.js";

export function initDrivePanel(root) {
  const toggle = root.querySelector("#drive-toggle");
  const panel = root.querySelector("#drive-panel");
  const speed = root.querySelector("#drive-speed");
  const poseLine = root.querySelector("#drive-pose");
  const arrow = root.querySelector("#drive-arrow");
  let slow = true;

  const joystick = createJoystick(root.querySelector("#joystick"), {
    getProfile: () => (slow ? PROFILES.MANUAL_SLOW : PROFILES.MANUAL),
  });
  const renderSpeed = () => {
    speed.textContent = t(slow ? "drive.slow" : "drive.normal");
    speed.dataset.slow = String(slow);
  };
  speed.addEventListener("click", () => {
    slow = !slow;
    renderSpeed();
  });
  toggle.addEventListener("click", () => {
    panel.hidden = !panel.hidden;
    toggle.classList.toggle("active", !panel.hidden);
    if (panel.hidden) joystick.stop();
  });

  const renderPose = () => {
    const p = state.get("pose");
    if (!p) {
      poseLine.textContent = t("stations.no_pose");
      return;
    }
    const deg = headingDegrees(p.yaw);
    poseLine.textContent = t("drive.pose", { x: p.x.toFixed(2), y: p.y.toFixed(2), deg });
    arrow.style.transform = `rotate(${-deg}deg)`; // screen y points down: counter-clockwise on the map
  };
  state.on("pose", renderPose);

  // Teach a station at the current pose.
  const dlg = root.querySelector("#teach-dialog");
  const name = root.querySelector("#teach-name");
  const kind = root.querySelector("#teach-kind");
  const occupied = root.querySelector("#teach-occupied");
  const result = root.querySelector("#teach-result");
  root.querySelector("#drive-teach").addEventListener("click", () => {
    joystick.stop();
    result.textContent = "";
    dlg.hidden = false;
  });
  root.querySelector("#teach-cancel").addEventListener("click", () => (dlg.hidden = true));
  root.querySelector("#teach-ok").addEventListener("click", () => {
    const res = teachStation(name.value, kind.value, occupied.checked);
    if (!res) {
      result.textContent = t("stations.no_pose");
      return;
    }
    res
      .then((r) => {
        if (r.success) {
          dlg.hidden = true;
          name.value = "";
        } else {
          result.textContent = t(errorKey(r.error_code));
        }
      })
      .catch(() => (result.textContent = t("ui.connection.lost")));
  });

  // Paused order: say so on the map and offer resume / cancel (spec 9.2 AWAIT_RESUME).
  const bar = root.querySelector("#resume-bar");
  const act = (action) => {
    const m = state.get("mission");
    if (!m || !m.active_order_id) return;
    ros.callService("/mission/order_action", "robotros_interfaces/srv/OrderAction", {
      order_id: m.active_order_id,
      action,
      value: "",
    }).catch(() => {});
  };
  root.querySelector("#resume-go").addEventListener("click", () => act("RESUME_AT_STEP"));
  root.querySelector("#resume-cancel").addEventListener("click", () => act("CANCEL"));
  const renderBar = () => {
    const m = state.get("mission");
    bar.hidden = !(state.get("connection") === "connected" && m && m.state === "AWAIT_RESUME");
  };
  state.on("mission", renderBar);
  state.on("connection", renderBar);

  window.addEventListener("robotros:lang", () => {
    renderSpeed();
    renderPose();
  });
  renderSpeed();
  renderPose();
  renderBar();
}
