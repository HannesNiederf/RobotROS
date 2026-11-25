// What the header shows (pure, tested with node --test). Without a live connection the UI cannot know the robot's
// state, so the last received safety state and activity are not shown as if they were current.
const SAFETY_KEYS = ["safety.boot_locked", "safety.estop", "safety.wait_ack", "safety.armed"];

/**
 * @param {string} conn "connecting" | "connected" | "lost"
 * @param {object|undefined} safety last SafetyState
 * @param {object|undefined} activity last ActivityState
 * @returns {{connKey, safetyKey, safetyState, now: {key, args}|null, plan: {key, args}|null, stale}}
 */
export function headerView(conn, safety, activity) {
  const live = conn === "connected";
  return {
    connKey: `ui.connection.${conn || "connecting"}`,
    safetyKey: live && safety ? SAFETY_KEYS[safety.state] || "safety.unknown" : "safety.unknown",
    safetyState: live && safety ? String(safety.state) : "",
    now: live && activity ? { key: activity.now_key, args: activity.now_args } : null,
    plan: live && activity ? { key: activity.plan_key, args: activity.plan_args } : null,
    stale: !live,
  };
}
