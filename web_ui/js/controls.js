// Global controls (spec 13.2, ADR 0008): software E-stop button, acknowledge dialog, operator alerts.
import * as ros from "./ros.js";
import * as state from "./state.js";
import { t } from "./i18n.js";
import { ackDialog, errorKey } from "./mission.js";

const SRV = "robotros_interfaces/srv/";

/** Text of an alert: translated error code if known, then the detail from the robot. */
export function alertText(a) {
  const key = errorKey(a.code);
  const name = key && t(key) !== key ? t(key) : a.code;
  return a.detail ? `${name}: ${a.detail}` : name;
}

export function initControls() {
  const estop = document.getElementById("estop");
  estop.addEventListener("click", () => {
    // Works in every state; the result shows in the safety pill.
    ros.callService("/mission/software_estop", "std_srvs/srv/Trigger", {})
      .then((res) => state.set("estopResult", res.success ? "ok" : res.message))
      .catch(() => state.set("estopResult", "error"));
  });

  const dlg = document.getElementById("ack-dialog");
  const msg = document.getElementById("ack-text");
  const box = document.getElementById("ack-buttons");
  const err = document.getElementById("ack-error");
  let shownKey = "";
  const renderAck = () => {
    const v = ackDialog(state.get("connection"), state.get("safety"), state.get("mission"));
    dlg.hidden = !v.show;
    if (!v.show) {
      shownKey = "";
      return;
    }
    msg.textContent = t(v.messageKey);
    const key = v.buttons.map((b) => b.mode).join(",") + (v.enabled ? "+" : "-");
    if (key !== shownKey) {
      shownKey = key;
      box.replaceChildren();
      for (const b of v.buttons) {
        const btn = document.createElement("button");
        btn.className = "primary";
        btn.textContent = t(b.key);
        btn.dataset.mode = b.mode;
        btn.disabled = !v.enabled;
        btn.addEventListener("click", () => {
          err.textContent = "";
          ros.callService("/mission/ack_safety", SRV + "AckSafety", { resume_mode: b.mode })
            .then((res) => {
              if (!res.success) err.textContent = t(errorKey(res.error_code));
            })
            .catch(() => {
              err.textContent = t("ui.connection.lost");
            });
        });
        box.appendChild(btn);
      }
    }
  };
  for (const k of ["connection", "safety", "mission"]) state.on(k, renderAck);
  window.addEventListener("robotros:lang", () => {
    shownKey = "";
    renderAck();
  });

  // Alerts: the latest one that needs a decision stays until it is resolved; others show for 10 s.
  const bar = document.getElementById("alert-bar");
  const text = document.getElementById("alert-text");
  const opts = document.getElementById("alert-options");
  let hideTimer = null;
  ros.subscribe("/mission/alerts", "robotros_interfaces/msg/OperatorAlert", (a) => {
    state.set("alert", a);
    bar.hidden = false;
    bar.dataset.severity = a.severity;
    text.textContent = alertText(a);
    opts.replaceChildren();
    clearTimeout(hideTimer);
    if (a.needs_action) {
      for (const option of a.options) {
        const btn = document.createElement("button");
        btn.textContent = t(`alert.option.${option}`);
        btn.addEventListener("click", () => {
          ros.callService("/mission/resolve_alert", SRV + "ResolveAlert", { alert_id: a.id, option })
            .then((res) => {
              if (res.success) bar.hidden = true;
            })
            .catch(() => {});
        });
        opts.appendChild(btn);
      }
    } else {
      const close = document.createElement("button");
      close.textContent = "✕";
      close.addEventListener("click", () => (bar.hidden = true));
      opts.appendChild(close);
      hideTimer = setTimeout(() => (bar.hidden = true), 10000);
    }
  });
}
