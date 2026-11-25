// Map page (spec 13.3, first part): map, robot pose, plan, tap -> confirm -> goal.
// The goal goes to /goal_pose (Nav2 bt_navigator) until the mission_manager with GOTO_POINT exists (ADR 0005).
import * as ros from "../ros.js";
import * as state from "../state.js";
import { t } from "../i18n.js";
import { MapView, quatOf, worldToCanvas, yawOf } from "../map_view.js";

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
    const sent = ros.publish("/goal_pose", "geometry_msgs/msg/PoseStamped", {
      header: { frame_id: "map", stamp: { sec: 0, nanosec: 0 } },
      pose: { position: { x: pending.x, y: pending.y, z: 0 }, orientation: quatOf(yaw) },
    });
    if (sent) {
      view.setGoal(pending);
      state.set("lastGoal", { x: pending.x, y: pending.y, yaw });
    }
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
