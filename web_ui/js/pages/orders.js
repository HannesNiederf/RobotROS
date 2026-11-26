// Orders page (spec 13.3 "Orders", ADR 0008): new transport order, queue and history with cancel/retry/resume.
import * as ros from "../ros.js";
import * as state from "../state.js";
import { t } from "../i18n.js";
import { errorKey, orderActions, orderRows, orderText, transportChoices } from "../mission.js";

const SRV = "robotros_interfaces/srv/";

export function initOrdersPage(root) {
  const src = root.querySelector("#order-source");
  const dst = root.querySelector("#order-dest");
  const prio = root.querySelector("#order-priority");
  const result = root.querySelector("#order-result");
  const list = root.querySelector("#order-list");

  const fill = (select, items) => {
    const keep = select.value;
    select.replaceChildren();
    for (const s of items) {
      const o = document.createElement("option");
      o.value = String(s.id);
      o.textContent = s.name;
      select.appendChild(o);
    }
    if ([...select.options].some((o) => o.value === keep)) select.value = keep;
  };
  const renderForm = () => {
    const c = transportChoices(state.get("stations") || []);
    fill(src, c.sources);
    fill(dst, c.dests);
  };

  root.querySelector("#order-create").addEventListener("click", () => {
    result.textContent = "";
    const req = {
      type: "TRANSPORT",
      priority: Number(prio.value),
      source_station_id: Number(src.value || 0),
      dest_station_id: Number(dst.value || 0),
      goal: { x: 0, y: 0, theta: 0 },
    };
    ros.callService("/mission/create_order", SRV + "CreateOrder", req)
      .then((res) => {
        result.textContent = res.success ? t("orders.created", { id: res.order_id }) : t(errorKey(res.error_code));
        result.dataset.ok = String(res.success);
      })
      .catch(() => (result.textContent = t("ui.connection.lost")));
  });

  const act = (order, action) =>
    ros.callService("/mission/order_action", SRV + "OrderAction", { order_id: order.id, action, value: "" })
      .then((res) => {
        if (!res.success) result.textContent = t(errorKey(res.error_code));
      })
      .catch(() => {});

  const renderList = () => {
    const stations = state.get("stations") || [];
    const status = state.get("mission");
    list.replaceChildren();
    for (const o of orderRows(state.get("orders") || [])) {
      const row = document.createElement("div");
      row.className = "row";
      row.dataset.state = o.state;
      row.dataset.orderId = String(o.id);
      const desc = orderText(o, stations);
      const main = document.createElement("div");
      main.className = "grow";
      main.textContent = `#${o.id}  ${t(desc.key, desc.args)}`;
      const sub = document.createElement("div");
      sub.className = "sub";
      const parts = [t(`order.state.${o.state}`), `P${o.priority}`];
      if (o.state === "ACTIVE") parts.push(t(`order.step.${o.step}`));
      if (o.error_code) parts.push(t(errorKey(o.error_code)));
      sub.textContent = parts.join(" · ");
      main.appendChild(sub);
      row.appendChild(main);
      for (const a of orderActions(o, status)) {
        const b = document.createElement("button");
        b.textContent = t(`orders.action.${a}`);
        b.dataset.action = a;
        b.addEventListener("click", () => act(o, a));
        row.appendChild(b);
      }
      list.appendChild(row);
    }
    if (!list.children.length) {
      const empty = document.createElement("div");
      empty.className = "empty";
      empty.textContent = t("orders.none");
      list.appendChild(empty);
    }
  };

  state.on("stations", () => {
    renderForm();
    renderList();
  });
  state.on("orders", renderList);
  state.on("mission", renderList);
  window.addEventListener("robotros:lang", () => {
    renderForm();
    renderList();
  });
  renderForm();
  renderList();
}
