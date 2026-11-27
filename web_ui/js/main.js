// Entry point: header (connection, safety, mission state, activity, E-stop), tabs and pages (ADR 0005, 0008).
import * as ros from "./ros.js";
import * as state from "./state.js";
import { LANGS, argsFromList, currentLang, initLang, setLang, t } from "./i18n.js";
import { initMapPage } from "./pages/map.js";
import { initOrdersPage } from "./pages/orders.js";
import { initStationsPage } from "./pages/stations.js";
import { initControls } from "./controls.js";
import { initDrivePanel } from "./drive_panel.js";
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

  const mission = state.get("mission");
  const missionEl = document.getElementById("mission");
  const live = conn === "connected" && mission;
  missionEl.textContent = live ? t(`mission.state.${mission.state}`) : t("mission.unknown");
  missionEl.dataset.state = live ? mission.state : "";

  const line = (part) => (part ? t(part.key, argsFromList(part.args)) : v.stale ? t("ui.no_data") : "");
  document.getElementById("activity-now").textContent = line(v.now);
  document.getElementById("activity-plan").textContent = line(v.plan);
  for (const b of document.querySelectorAll("[data-lang]")) b.classList.toggle("active", b.dataset.lang === currentLang());
}

async function main() {
  await initLang();
  for (const key of ["connection", "safety", "activity", "mission"]) state.on(key, renderHeader);
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
  ros.subscribe("/mission/status", "robotros_interfaces/msg/MissionStatus", (m) => state.set("mission", m));
  ros.subscribe("/mission/orders", "robotros_interfaces/msg/OrderList", (m) => state.set("orders", m.orders));
  ros.subscribe("/web/stations", "robotros_interfaces/msg/StationList", (m) => state.set("stations", m.stations));
  const mapPage = initMapPage(document.getElementById("page-map"));
  initOrdersPage(document.getElementById("page-orders"));
  initStationsPage(document.getElementById("page-stations"));
  initControls();
  initDrivePanel(document.getElementById("page-map"));
  const showPage = (name) => {
    for (const b of document.querySelectorAll("#tabs [data-page]")) b.classList.toggle("active", b.dataset.page === name);
    for (const m of document.querySelectorAll("main")) m.hidden = m.id !== `page-${name}`;
    if (name === "map") window.dispatchEvent(new Event("resize"));
  };
  for (const b of document.querySelectorAll("#tabs [data-page]")) b.addEventListener("click", () => showPage(b.dataset.page));
  renderHeader();
  ros.connect();
  // Read-only hooks for tools/web_ui_check.py (end-to-end test in CI).
  window.robotros = { state: state.snapshot, worldToClient: mapPage.worldToClient, showPage, ready: true };
}

main();
