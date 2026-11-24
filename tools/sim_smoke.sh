#!/usr/bin/env bash
# Headless simulation smoke test: start robotros_sim (generic world, no display), run
# sim_smoke_check.py against it, stop everything. Exit code 0 = pass.
# Usage (after `colcon build` in ros2_ws): tools/sim_smoke.sh [log_dir]
#
# Isolation: a separate ROS domain, localhost-only discovery and an own Gazebo partition,
# so this never reaches the robot or another simulation on the network.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${1:-$ROOT/build/sim_smoke}"
mkdir -p "$LOG_DIR"

export ROS_DOMAIN_ID="${SIM_SMOKE_DOMAIN_ID:-77}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION="robotros_smoke_$$"
export GZ_IP=127.0.0.1

if [ "$ROS_DOMAIN_ID" = "42" ]; then
  echo "refusing to run on ROS_DOMAIN_ID 42 (the robot's domain)" >&2
  exit 2
fi

# shellcheck disable=SC1091
source "$ROOT/ros2_ws/install/setup.bash"

echo "sim smoke: domain $ROS_DOMAIN_ID, partition $GZ_PARTITION, logs in $LOG_DIR"
# Own process group (setsid), so cleanup stops Gazebo and every node it started, and nothing else.
setsid ros2 launch robotros_sim sim.launch.py headless:=true > "$LOG_DIR/sim_launch.log" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill -INT -- "-$LAUNCH_PID" 2> /dev/null || true
  for _ in $(seq 1 20); do
    kill -0 -- "-$LAUNCH_PID" 2> /dev/null || break
    sleep 0.5
  done
  kill -KILL -- "-$LAUNCH_PID" 2> /dev/null || true
}
trap cleanup EXIT

status=0
python3 "$ROOT/ros2_ws/install/robotros_sim/lib/robotros_sim/sim_smoke_check.py" || status=$?

if [ "$status" -ne 0 ]; then
  echo "---- last 60 lines of sim_launch.log ----"
  tail -n 60 "$LOG_DIR/sim_launch.log" || true
fi
exit "$status"
