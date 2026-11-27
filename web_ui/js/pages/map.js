// Map page (spec 13.3): map, stations, robot pose, plan, tap -> confirm -> GOTO_POINT order (ADR 0008).
import * as ros from "../ros.js";
import * as state from "../state.js";
import { t } from "../i18n.js";
import { errorKey } from "../mission.js";
import { MapView, worldToCanvas, yawOf } from "../map_view.js";

export function initMapPage(root) {
  const canvas = root.querySelector("#map-canvas");
  const view = new MapView(canvas);
  const dialog = root.querySelector("#goal-dialog");
  const dialogText = root.querySelector("#goal-dialog-text");
  let pending = null;

  const showMap = (image, meta) => {
    if (!image || !meta || !image.data) return;
    view.setMap(`data:image/png;base64,${image.data}`, meta);
    state.set("map", { width: meta.width, height: meta.height, resolution: meta.resolution });
  };
  let lastMeta = null;
  ros.subscribe("/web/map_meta", "robotros_interfaces/msg/MapMeta", (m) => {
    lastMeta = m;
  });
  ros.subscribe("/web/map_image", "sensor_msgs/msg/CompressedImage", (img) => showMap(img, lastMeta));
  ros.onConnect(() => {
    ros.callService("/map/get_snapshot", "robotros_interfaces/srv/GetSnapshot")
      .then((res) => {
        if (res.success) showMap(res.map_image, res.map_meta);
        if (res.stations) state.set("stations", res.stations);
      })
      .catch(() => {});
  });
  ros.subscribe("/robot_pose", "geometry_msgs/msg/PoseStamped", (msg) => {
    const p = { x: msg.pose.position.x, y: msg.pose.position.y, yaw: yawOf(msg.pose.orientation) };
    state.set("pose", p);
    view.setPose(p);
  }, 200);
  ros.subscribe("/plan", "nav_msgs/msg/Path", (msg) => {
    const pts = msg.poses.map((ps) => ({ x: ps.pose.position.x, y: ps.pose.position.y }));
    state.set("planLength", pts.length);
    view.setPlan(pts);
  }, 1000);

  state.on("stations", (list) => view.setStations(list));
  const goalError = root.querySelector("#goal-error");

  view.onTap = (w) => {
    pending = w;
    dialogText.textContent = t("map.goal.where", { x: w.x.toFixed(2), y: w.y.toFixed(2) });
    dialog.hidden = false;
  };
  root.querySelector("#goal-cancel").addEventListener("click", () => {
    pending = null;
    dialog.hidden = true;
  });
  root.querySelector("#goal-ok").addEventListener("click", () => {
    dialog.hidden = true;
    if (!pending) return;
    const pose = state.get("pose");
    // Default heading: from the robot towards the goal (chosen here).
    const yaw = pose ? Math.atan2(pending.y - pose.y, pending.x - pose.x) : 0;
    const goal = { x: pending.x, y: pending.y, yaw };
    goalError.textContent = "";
    ros.callService("/mission/create_order", "robotros_interfaces/srv/CreateOrder", {
      type: "GOTO_POINT",
      priority: 5,
      source_station_id: 0,
      dest_station_id: 0,
      goal: { x: goal.x, y: goal.y, theta: yaw },
    })
      .then((res) => {
        if (res.success) {
          view.setGoal(goal);
          state.set("lastGoal", { ...goal, orderId: res.order_id });
        } else {
          goalError.textContent = t(errorKey(res.error_code));
        }
      })
      .catch(() => (goalError.textContent = t("ui.connection.lost")));
    pending = null;
  });

  // For the end-to-end test: page coordinates of a map point, or null while no map is shown.
  return {
    worldToClient(x, y) {
      if (!view.tf) return null;
      const r = canvas.getBoundingClientRect();
      const c = worldToCanvas(view.tf, x, y);
      return { x: r.left + c.px, y: r.top + c.py };
    },
  };
}
