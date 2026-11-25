#!/usr/bin/env bash
# PROBE ONLY: mapping once, then M1-03 acceptance and a passage reliability run on the same map.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="$ROOT/build/sim_nav"; mkdir -p "$LOG_DIR"
export ROS_DOMAIN_ID=77 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_IP=127.0.0.1
source "$ROOT/ros2_ws/install/setup.bash"
CHECK="$ROOT/ros2_ws/install/robotros_sim/lib/robotros_sim/sim_nav_check.py"
MAP="$LOG_DIR/generic_map"
LAUNCH_PID=""
stop_sim() { [ -n "$LAUNCH_PID" ] || return 0; kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true
  for _ in $(seq 1 30); do kill -0 -- "-$LAUNCH_PID" 2>/dev/null || break; sleep 0.5; done
  kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true; LAUNCH_PID=""; sleep 3; }
trap stop_sim EXIT
phase() { local name="$1"; shift; local la=(); while [ "$1" != "--" ]; do la+=("$1"); shift; done; shift
  export GZ_PARTITION="probe_${name}_$$"
  setsid ros2 launch robotros_sim sim.launch.py headless:=true "${la[@]}" > "$LOG_DIR/$name.log" 2>&1 &
  LAUNCH_PID=$!; local rc=0; timeout 1500 python3 "$CHECK" "$@" || rc=$?; stop_sim
  echo "collision-ahead lines: $(grep -ac 'collision ahead' "$LOG_DIR/$name.log")"
  echo "== $name rc=$rc"; }
NAV2="$ROOT/ros2_ws/install/robotros_bringup/share/robotros_bringup/config/nav2.yaml"
G="d['planner_server']['ros__parameters']['GridBased']"
case "$1" in
  c1) EXPR="$G['cost_travel_multiplier']=3.0" ;;
  c2) EXPR="$G['smoother']={'max_iterations': 1000, 'w_smooth': 0.1, 'w_data': 0.4, 'tolerance': 1e-10}" ;;
  c3) EXPR="$G['cost_travel_multiplier']=3.0; $G['smoother']={'max_iterations': 1000, 'w_smooth': 0.1, 'w_data': 0.4, 'tolerance': 1e-10}" ;;
  c0) EXPR="pass" ;;
esac
echo "######## variant $1: $EXPR"
python3 - "$NAV2" "$EXPR" <<'PY'
import sys, yaml
path, expr = sys.argv[1:]
d = yaml.safe_load(open(path))
amcl = d["amcl"]["ros__parameters"]; rpp = d["controller_server"]["ros__parameters"]["FollowPath"]
lc = d["local_costmap"]["local_costmap"]["ros__parameters"]; gc = d["global_costmap"]["global_costmap"]["ros__parameters"]
exec(expr)
yaml.safe_dump(d, open(path, "w"))
PY
python3 "$ROOT/ros2_ws/install/robotros_sim/lib/robotros_sim/world_map.py" "$ROOT/ros2_ws/install/robotros_sim/share/robotros_sim/worlds/generic.sdf" "$LOG_DIR/world_map"
phase accept mode:=nav "map:=$LOG_DIR/world_map.yaml" -- nav --goal -2.3 0.0 3.14159 --goal 0.7 -1.5 0.0 --runs 10 --acceptance || true
