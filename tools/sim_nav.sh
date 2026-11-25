#!/usr/bin/env bash
# Headless mapping + navigation check (softwarespezifikation.md sections 7.1, 7.2, 14):
#   1. simulation with mode:=mapping: drive a pattern, check the slam_toolbox map, save it
#   2. simulation with mode:=nav: AMCL, Nav2 goals, check the true end pose. The map for this phase is the
#      ground-truth lidar map of the world (world_map.py), so the end errors measure navigation and AMCL, not
#      the varying quality of one SLAM run; the SLAM map of phase 1 is checked there and used by sim_start --check.
# Usage (after `colcon build` in ros2_ws): tools/sim_nav.sh [--acceptance] [log_dir]
#   default       goals A (-2.3, 0, pi), B (0.7, -1.5, 0), P (2.5, -0.775, 0) behind the 0.45 m passage (west to
#                 east); one run each, gross limits per run. East to west with the sharp turn towards B is not in
#                 CI yet: it failed in 3 of 4 probe runs on the exact map (ADR 0003, open).
#   --acceptance  M1-03 in simulation: 10 runs A <-> B, >= 9 within 0.10 m / 10 deg
# Isolation as tools/sim_smoke.sh: own ROS domain (not 42), localhost only, own Gazebo partition.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAV_ARGS=(--goal -2.3 0.0 3.14159 --goal 0.7 -1.5 0.0 --goal 2.5 -0.775 0.0)
if [ "${1:-}" = "--acceptance" ]; then
  NAV_ARGS=(--goal -2.3 0.0 3.14159 --goal 0.7 -1.5 0.0 --runs 10 --acceptance)
  shift
fi
LOG_DIR="${1:-$ROOT/build/sim_nav}"
mkdir -p "$LOG_DIR"

export ROS_DOMAIN_ID="${SIM_SMOKE_DOMAIN_ID:-77}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_IP=127.0.0.1
if [ "$ROS_DOMAIN_ID" = "42" ]; then
  echo "refusing to run on ROS_DOMAIN_ID 42 (the robot's domain)" >&2
  exit 2
fi

# shellcheck disable=SC1091
source "$ROOT/ros2_ws/install/setup.bash"
CHECK="$ROOT/ros2_ws/install/robotros_sim/lib/robotros_sim/sim_nav_check.py"
MAP="$LOG_DIR/generic_map"

LAUNCH_PID=""
stop_sim() {
  [ -n "$LAUNCH_PID" ] || return 0
  kill -INT -- "-$LAUNCH_PID" 2> /dev/null || true
  for _ in $(seq 1 30); do
    kill -0 -- "-$LAUNCH_PID" 2> /dev/null || break
    sleep 0.5
  done
  kill -KILL -- "-$LAUNCH_PID" 2> /dev/null || true
  LAUNCH_PID=""
}
trap stop_sim EXIT

run_phase() {  # name, launch args..., -- , check args...
  local name="$1"
  shift
  local launch_args=()
  while [ "$1" != "--" ]; do
    launch_args+=("$1")
    shift
  done
  shift
  export GZ_PARTITION="robotros_nav_${name}_$$"
  echo "== phase $name: ros2 launch robotros_sim sim.launch.py headless:=true ${launch_args[*]}"
  setsid ros2 launch robotros_sim sim.launch.py headless:=true "${launch_args[@]}" > "$LOG_DIR/$name.log" 2>&1 &
  LAUNCH_PID=$!
  local rc=0
  python3 "$CHECK" "$@" || rc=$?
  stop_sim
  if [ "$rc" -ne 0 ]; then
    echo "---- $name.log: errors, warnings, lifecycle (first 80) ----"
    grep -aiE "error|warn|fail|reject|abort|exception|Managed nodes|lifecycle_manager" "$LOG_DIR/$name.log" | head -n 80 || true
    echo "---- last 30 lines of $name.log ----"
    tail -n 30 "$LOG_DIR/$name.log" || true
  fi
  return "$rc"
}

status=0
run_phase mapping mode:=mapping -- mapping --map-out "$MAP" || status=$?
if [ "$status" -eq 0 ]; then
  WORLD="$ROOT/ros2_ws/install/robotros_sim/share/robotros_sim/worlds/generic.sdf"
  python3 "$ROOT/ros2_ws/install/robotros_sim/lib/robotros_sim/world_map.py" "$WORLD" "$LOG_DIR/world_map" || status=$?
fi
if [ "$status" -eq 0 ]; then
  run_phase nav mode:=nav "map:=$LOG_DIR/world_map.yaml" -- nav "${NAV_ARGS[@]}" || status=$?
fi
echo "SIM NAV: $([ "$status" -eq 0 ] && echo PASS || echo FAIL)"
exit "$status"
