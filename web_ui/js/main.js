// Entry point: header (connection, safety state, activity) and the map page.
import * as ros from "./ros.js";
import * as state from "./state.js";
import { LANGS, argsFromList, currentLang, initLang, setLang, t } from "./i18n.js";
import { initMapPage } from "./pages/map.js";
import { headerView } from "./header.js";

function renderHeader() {
  const conn = state.get("connection") || "connecting";
  const v = headerView(conn, state.get("safety"), state.get("activity"));
  const connEl = document.getElementById("conn");
  connEl.textContent = t(v.connKey);
  connEl.dataset.state = conn;
  document.getElementById("overlay-lost").hidden = conn !== "lost";

  const safetyEl = document.getElementById("safety");
  safetyEl.textContent = t(v.safetyKey);
  safetyEl.dataset.state = v.safetyState;

  const line = (part) => (part ? t(part.key, argsFromList(part.args)) : v.stale ? t("ui.no_data") : "");
  document.getElementById("activity-now").textContent = line(v.now);
  document.getElementById("activity-plan").textContent = line(v.plan);
  for (const b of document.querySelectorAll("[data-lang]")) b.classList.toggle("active", b.dataset.lang === currentLang());
}

async function main() {
  await initLang();
  for (const key of ["connection", "safety", "activity"]) state.on(key, renderHeader);
  window.addEventListener("robotros:lang", renderHeader);
  const langBox = document.getElementById("langs");
  for (const l of LANGS) {
    const b = document.createElement("button");
    b.dataset.lang = l;
    b.textContent = l.toUpperCase();
    b.addEventListener("click", () => setLang(l));
    langBox.appendChild(b);
  }
  ros.subscribe("/hw/safety", "robotros_interfaces/msg/SafetyState", (m) => state.set("safety", m));
  ros.subscribe("/activity", "robotros_interfaces/msg/ActivityState", (m) => state.set("activity", m));
  const mapPage = initMapPage(document.getElementById("page-map"));
  renderHeader();
  ros.connect();
  // Read-only hooks for tools/web_ui_check.py (end-to-end test in CI).
  window.robotros = { state: state.snapshot, worldToClient: mapPage.worldToClient, ready: true };
}

main();
