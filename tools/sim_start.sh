#!/usr/bin/env bash
# Start the robotROS simulation for interactive use (Surface), see claude.ai project doc anleitung_simulation.md.
#
#   tools/sim_start.sh mapping [--gazebo-gui]          map the room: keyboard control in this terminal, RViz
#   tools/sim_start.sh save [NAME]                     save the map while "mapping" runs (second terminal),
#                                                      default ~/robotros_karte
#   tools/sim_start.sh nav [MAP.yaml] [--gazebo-gui]   navigate with a saved map (default ~/robotros_karte.yaml):
#                                                      set goals in RViz with "2D Goal Pose"
#   --check            test mode (CI): no keyboard, runs sim_start_check.py and exits with its result
#
# The robot is armed once automatically and, in mode nav, gets its initial pose automatically
# (simulation only, sim_helper). After an E-stop it must be armed by hand, as on the robot.
# Isolation: ROS domain 77 and localhost only, so the real robot (domain 42) is never reached.
set -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-}"
shift || true
MAP_DEFAULT="$HOME/robotros_karte"
GZ_GUI=false
CHECK=false
ARG=""
for a in "$@"; do
  case "$a" in
    --gazebo-gui) GZ_GUI=true ;;
    --check) CHECK=true ;;
    *) ARG="$a" ;;
  esac
done

export ROS_DOMAIN_ID="${SIM_DOMAIN_ID:-77}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_IP=127.0.0.1
export GZ_PARTITION="robotros_sim_$USER"
if [ "$ROS_DOMAIN_ID" = "42" ]; then
  echo "refusing to run on ROS_DOMAIN_ID 42 (the robot's domain)" >&2
  exit 2
fi
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
if [ ! -f "$ROOT/ros2_ws/install/setup.bash" ]; then
  echo "Not built yet: cd $ROOT/ros2_ws && colcon build" >&2
  exit 1
fi
# shellcheck disable=SC1091
source "$ROOT/ros2_ws/install/setup.bash"

case "$MODE" in
  save)
    NAME="${ARG:-$MAP_DEFAULT}"
    exec ros2 run nav2_map_server map_saver_cli -f "$NAME" --ros-args -p save_map_timeout:=20.0
    ;;
  mapping) LAUNCH_ARGS=(mode:=mapping) ;;
  nav)
    MAP="${ARG:-$MAP_DEFAULT.yaml}"
    if [ ! -f "$MAP" ]; then
      echo "Map $MAP not found. Map first: tools/sim_start.sh mapping, then tools/sim_start.sh save" >&2
      exit 1
    fi
    LAUNCH_ARGS=(mode:=nav "map:=$(realpath "$MAP")")
    ;;
  *)
    sed -n '2,13p' "$0"
    exit 1
    ;;
esac

HEADLESS=true
[ "$GZ_GUI" = true ] && HEADLESS=false
LOG_DIR="$ROOT/build/sim_start"
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/$MODE.log"
echo "Starting simulation ($MODE). Log: $LOG"
setsid ros2 launch robotros_sim sim.launch.py "${LAUNCH_ARGS[@]}" headless:=$HEADLESS auto:=true rviz:=true \
  > "$LOG" 2>&1 &
LAUNCH_PID=$!

stop() {
  echo "Stopping simulation ..."
  kill -INT -- "-$LAUNCH_PID" 2> /dev/null || true
  for _ in $(seq 1 30); do
    kill -0 -- "-$LAUNCH_PID" 2> /dev/null || break
    sleep 0.5
  done
  kill -KILL -- "-$LAUNCH_PID" 2> /dev/null || true
}
trap stop EXIT
trap 'exit 130' INT TERM

if [ "$CHECK" = true ]; then
  rc=0
  python3 "$ROOT/ros2_ws/install/robotros_sim/lib/robotros_sim/sim_start_check.py" "$MODE" || rc=$?
  if pgrep -g "$LAUNCH_PID" -x rviz2 > /dev/null; then
    echo "ok    RViz is running"
  else
    echo "FAIL  RViz is not running"
    rc=1
  fi
  # The glsl120 shader message comes from software OpenGL under Xvfb (CI) when RViz draws a map; on a machine
  # with a GPU driver it should not appear (unverified). Everything else from RViz counts as an error.
  if grep -aiE "\[rviz2.*(error|fatal)|rviz2-[0-9]+\]: process has died" "$LOG" | grep -av "glsl120" | head -n 20 | grep .; then
    echo "FAIL  RViz reported errors (lines above)"
    rc=1
  fi
  if [ "$rc" -ne 0 ]; then
    echo "---- $LOG: errors and warnings (first 60) ----"
    grep -aiE "error|fail|exception|died" "$LOG" | head -n 60 || true
  fi
  exit "$rc"
fi

if [ "$MODE" = mapping ]; then
  echo "RViz opens in a moment. The robot is armed automatically."
  echo "Save the map from a second terminal: tools/sim_start.sh save"
  if ros2 pkg prefix teleop_twist_keyboard > /dev/null 2>&1; then
    echo "Keyboard control (keep this window active). Ctrl+C ends everything."
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_manual
  else
    echo "teleop_twist_keyboard is not installed: no keyboard control. Ctrl+C ends everything."
    wait "$LAUNCH_PID"
  fi
else
  echo "RViz opens in a moment. The robot is armed and localized automatically."
  echo "Set goals in RViz: '2D Goal Pose', click on the target and drag the heading. Ctrl+C ends everything."
  wait "$LAUNCH_PID"
fi
