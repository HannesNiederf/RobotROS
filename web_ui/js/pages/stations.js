// Stations page (spec 8.4 / 13.3 "Stations", ADR 0008): teach a station at the robot's pose, set the cart
// occupancy, delete. The mission_manager owns the database (/map/upsert_station, /map/delete_station,
// /mission/set_cart_location) and publishes /web/stations.
import * as ros from "../ros.js";
import * as state from "../state.js";
import { t } from "../i18n.js";
import { errorKey } from "../mission.js";

const SRV = "robotros_interfaces/srv/";

export function initStationsPage(root) {
  const name = root.querySelector("#station-name");
  const kind = root.querySelector("#station-kind");
  const occupied = root.querySelector("#station-occupied");
  const result = root.querySelector("#station-result");
  const list = root.querySelector("#station-list");
  const show = (res, okKey, args = {}) => {
    result.textContent = res.success ? t(okKey, args) : t(errorKey(res.error_code));
    result.dataset.ok = String(res.success);
  };
  const lost = () => (result.textContent = t("ui.connection.lost"));

  root.querySelector("#station-teach").addEventListener("click", () => {
    const pose = state.get("pose");
    if (!pose) {
      result.textContent = t("stations.no_pose");
      return;
    }
    const station = {
      id: 0,
      name: name.value.trim(),
      kind: kind.value,
      pose: { x: pose.x, y: pose.y, theta: pose.yaw },
      approach_distance: 0.4,
      enabled: true,
      occupied: kind.value === "STORAGE" && occupied.checked,
      cart_name: "",
    };
    ros.callService("/map/upsert_station", SRV + "UpsertStation", { station })
      .then((res) => {
        show(res, "stations.saved", { name: station.name });
        if (res.success) name.value = "";
      })
      .catch(lost);
  });

  const setCart = (s, present) =>
    ros.callService("/mission/set_cart_location", SRV + "SetCartLocation", {
      cart_name: present ? `cart@${s.name}` : s.cart_name,
      station_id: present ? s.id : 0,
    }).then((res) => show(res, "stations.saved", { name: s.name })).catch(lost);

  const remove = (s) =>
    ros.callService("/map/delete_station", SRV + "DeleteStation", { station_id: s.id })
      .then((res) => show(res, "stations.deleted", { name: s.name }))
      .catch(lost);

  const render = () => {
    list.replaceChildren();
    for (const s of state.get("stations") || []) {
      const row = document.createElement("div");
      row.className = "row";
      row.dataset.stationId = String(s.id);
      const main = document.createElement("div");
      main.className = "grow";
      main.textContent = `${s.name}  (${t(`stations.kind.${s.kind}`)})`;
      const sub = document.createElement("div");
      sub.className = "sub";
      const cart = s.kind === "STORAGE" ? (s.occupied ? t("stations.cart", { cart: s.cart_name }) : t("stations.free")) : "";
      sub.textContent = [cart, `x ${s.pose.x.toFixed(2)}, y ${s.pose.y.toFixed(2)}`].filter(Boolean).join(" · ");
      main.appendChild(sub);
      row.appendChild(main);
      if (s.kind === "STORAGE") {
        const b = document.createElement("button");
        b.textContent = t(s.occupied ? "stations.mark_free" : "stations.mark_cart");
        b.addEventListener("click", () => setCart(s, !s.occupied));
        row.appendChild(b);
      }
      const del = document.createElement("button");
      del.textContent = t("stations.delete");
      del.addEventListener("click", () => remove(s));
      row.appendChild(del);
      list.appendChild(row);
    }
    if (!list.children.length) {
      const empty = document.createElement("div");
      empty.className = "empty";
      empty.textContent = t("stations.none");
      list.appendChild(empty);
    }
  };
  state.on("stations", render);
  window.addEventListener("robotros:lang", render);
  render();
}
