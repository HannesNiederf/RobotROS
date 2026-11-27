#!/usr/bin/env bash
# Start the robotROS simulation for interactive use (Surface), see claude.ai project doc anleitung_simulation.md.
#
#   tools/sim_start.sh mapping [--gazebo-gui]          map the room: keyboard control in this terminal, RViz
#   tools/sim_start.sh save [NAME]                     save the map while "mapping" runs (second terminal),
#                                                      default ~/robotros_karte
#   tools/sim_start.sh nav [MAP.yaml] [--gazebo-gui]   navigate with a saved map (default ~/robotros_karte.yaml):
#                                                      set goals in RViz with "2D Goal Pose"
#   tools/sim_start.sh now                            what the robot does now, running log (another terminal)
#   tools/sim_start.sh plan                           what the robot intends (another terminal)
#   tools/sim_start.sh rtf [SECONDS]                  real time factor of the running simulation (another terminal)
#   tools/sim_start.sh rates [SECONDS]                messages per second and KiB/s of every topic (another terminal)
#                                                      language from $LANG (de/it/en), or add --lang de|en|it
#   --check            test mode (CI): no keyboard, runs sim_start_check.py and exits with its result
#   --pi[=HOST]        distributed simulation (ADR 0006): Gazebo and RViz here, the robot software
#                      (tools/pi_robot_software.sh) on the Pi over SSH, default HOST robot-ros@robotros.local.
#                      Needs an SSH key (ssh-copy-id) and the same workspace built on the Pi. Prints the Pi's
#                      CPU load (PILOAD) at the end. --pi=local runs the "Pi" part on this computer (CI).
#                      now/plan/rtf/save in other terminals follow automatically while a --pi session runs.
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
VIEW_ARGS=()
PI=""
for a in "$@"; do
  case "$a" in
    --pi) PI="robot-ros@robotros.local" ;;
    --pi=*) PI="${a#--pi=}" ;;
    --gazebo-gui) GZ_GUI=true ;;
    --lang) VIEW_ARGS+=(--lang) ;;
    de|en|it) VIEW_ARGS+=("$a") ;;
    --check) CHECK=true ;;
    *) ARG="$a" ;;
  esac
done

export ROS_DOMAIN_ID="${SIM_DOMAIN_ID:-77}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
LOG_DIR="$ROOT/build/sim_start"
PI_MARK="$LOG_DIR/.pi_session"
# Distributed simulation: DDS must reach the Pi over the WLAN (still only domain 77, never the robot's 42).
if [ -n "$PI" ] && [ "$PI" != local ]; then
  export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
elif [ -z "$PI" ] && [ -f "$PI_MARK" ] && [[ "$MODE" =~ ^(now|plan|rtf|rates|save)$ ]]; then
  export ROS_AUTOMATIC_DISCOVERY_RANGE="$(cat "$PI_MARK")"
fi
# Middleware as on the robot (spec 15.1), on both sides of a --pi session: Ubuntu's ~/.bashrc stops early in
# non-interactive shells, so an SSH command would otherwise run with the default (Fast DDS) and mix vendors.
export RMW_IMPLEMENTATION="${SIM_RMW:-rmw_cyclonedds_cpp}"
# Distributed simulation over WLAN: multicast only for discovery, data unicast (ADR 0006).
if [ -n "$PI" ] || [ "$ROS_AUTOMATIC_DISCOVERY_RANGE" = SUBNET ]; then
  export CYCLONEDDS_URI="file://$ROOT/ros2_ws/src/robotros_sim/config/cyclonedds_wifi.xml"
fi
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
  rtf)
    exec python3 "$ROOT/ros2_ws/install/robotros_sim/lib/robotros_sim/sim_rtf.py" ${ARG:+"$ARG"}
    ;;
  rates)
    exec python3 "$ROOT/ros2_ws/install/robotros_sim/lib/robotros_sim/topic_rates.py" ${ARG:+"$ARG"}
    ;;
  now|plan)
    exec ros2 run robotros_mission activity_view "$MODE" --i18n "$ROOT/web_ui/i18n" "${VIEW_ARGS[@]}"
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
    sed -n '2,23p' "$0"
    exit 1
    ;;
esac

HEADLESS=true
[ "$GZ_GUI" = true ] && HEADLESS=false
mkdir -p "$LOG_DIR"
LOG="$LOG_DIR/$MODE.log"
PI_PID=""
PING_PID=""
if [ "$CHECK" = true ]; then
  # Fresh mission database per check run (ADR 0007), so earlier sessions cannot leave orders behind.
  export SIM_DB="$LOG_DIR/$MODE.db"
  rm -f "$SIM_DB" "$SIM_DB-wal" "$SIM_DB-shm"
fi
WEB_URL="http://localhost:8080"
if [ -n "$PI" ]; then
  PI_LOG="$LOG_DIR/$MODE.pi.log"
  REMOTE_MAP=""
  if [ "$PI" = local ]; then
    REMOTE_MAP="${LAUNCH_ARGS[1]#map:=}"
    echo "Robot software runs locally (--pi=local). Log: $PI_LOG"
    PI_DISCOVERY=LOCALHOST setsid "$ROOT/tools/pi_robot_software.sh" "$MODE" ${REMOTE_MAP:+"$REMOTE_MAP"} \
      > "$PI_LOG" 2>&1 &
    PI_PID=$!
  else
    if ! ssh -o BatchMode=yes -o ConnectTimeout=10 "$PI" true 2> /dev/null; then
      echo "Cannot log in to $PI without a password. Once: ssh-copy-id $PI   (then try again)" >&2
      exit 1
    fi
    if [ "$MODE" = nav ]; then
      # Copy the map (YAML and its image) to the Pi, same file names, into ~/robotros_sim_maps/.
      MAP_ABS="${LAUNCH_ARGS[1]#map:=}"
      IMG="$(sed -n 's/^image:[[:space:]]*//p' "$MAP_ABS" | tr -d "\"'")"
      [ "${IMG:0:1}" = / ] || IMG="$(dirname "$MAP_ABS")/$IMG"
      ssh "$PI" mkdir -p robotros_sim_maps && scp -q "$MAP_ABS" "$IMG" "$PI:robotros_sim_maps/" || exit 1
      REMOTE_MAP="robotros_sim_maps/$(basename "$MAP_ABS")"
    fi
    WEB_URL="http://${PI#*@}:8080"
    # Round trip time to the Pi during the whole session, summarized at the end (network_summary).
    PING_LOG="$LOG_DIR/$MODE.ping"
    ping -i 1 "${PI#*@}" > "$PING_LOG" 2>&1 &
    PING_PID=$!
    echo "Robot software runs on $PI. Its output: $PI_LOG"
    # -tt: closing the connection (Ctrl+C here) hangs up the remote side, which then stops cleanly.
    ssh -tt "$PI" "cd RobotROS && tools/pi_robot_software.sh $MODE ${REMOTE_MAP:+\$HOME/$REMOTE_MAP}" \
      > "$PI_LOG" 2>&1 < /dev/null &
    PI_PID=$!
  fi
  echo "$ROS_AUTOMATIC_DISCOVERY_RANGE" > "$PI_MARK"
  SIM_ARGS=(robot_software:=false)
else
  SIM_ARGS=("${LAUNCH_ARGS[@]}" auto:=true web:=true ${SIM_DB:+"db:=$SIM_DB"})
fi
echo "Starting simulation ($MODE). Log: $LOG"
echo "Browser UI (ADR 0005): $WEB_URL  (open it in Firefox on this computer)"
setsid ros2 launch robotros_sim sim.launch.py "${SIM_ARGS[@]}" headless:=$HEADLESS rviz:=true > "$LOG" 2>&1 &
LAUNCH_PID=$!

pi_load_summary() {
  [ -n "$PI_PID" ] && [ -f "$PI_LOG" ] || return 0
  grep -aE "^(robot software|WLAN) " "$PI_LOG" | tr -d '\r'
  python3 - "$PI_LOG" <<'PY'
import re, sys
vals = [(int(b), int(m)) for b, m in re.findall(r"PILOAD cpu_busy=(\d+)% mem_used=(\d+)MiB", open(sys.argv[1], errors="replace").read())]
if vals:
    busy = [b for b, _ in vals]
    print(f"Pi load: CPU busy mean {sum(busy) / len(busy):.0f} %, max {max(busy)} % ({len(busy)} samples of 5 s); "
          f"memory used max {max(m for _, m in vals)} MiB")
else:
    print("Pi load: no samples")
PY
}

network_summary() {
  [ -n "$PING_PID" ] || return 0
  kill -INT "$PING_PID" 2> /dev/null || true
  wait "$PING_PID" 2> /dev/null || true
  echo "Network to the Pi (ping every second during the session):"
  grep -E "packet loss|^rtt" "$PING_LOG" | sed 's/^/  /' || echo "  no ping summary, see $PING_LOG"
}

stop() {
  if [ -n "$PI_PID" ]; then
    pi_load_summary
    network_summary
    kill -TERM -- "-$PI_PID" 2> /dev/null || kill -TERM "$PI_PID" 2> /dev/null || true
    for _ in $(seq 1 30); do kill -0 "$PI_PID" 2> /dev/null || break; sleep 0.5; done
    kill -KILL -- "-$PI_PID" 2> /dev/null || kill -KILL "$PI_PID" 2> /dev/null || true
    rm -f "$PI_MARK"
  fi
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
  # The two activity viewers (ADR 0004) run along, in German, and must show what the check provokes.
  VIEW_LOG="$LOG_DIR/$MODE.view"
  VIEW="$ROOT/ros2_ws/install/robotros_mission/lib/robotros_mission/activity_view"
  "$VIEW" now --lang de --i18n "$ROOT/web_ui/i18n" > "$VIEW_LOG.now" 2>&1 &
  VIEW_NOW=$!
  "$VIEW" plan --lang de --i18n "$ROOT/web_ui/i18n" > "$VIEW_LOG.plan" 2>&1 &
  VIEW_PLAN=$!
  python3 "$ROOT/ros2_ws/install/robotros_sim/lib/robotros_sim/sim_start_check.py" "$MODE" || rc=$?
  kill -INT "$VIEW_NOW" "$VIEW_PLAN" 2> /dev/null || true
  for _ in $(seq 1 20); do kill -0 "$VIEW_NOW" 2> /dev/null || kill -0 "$VIEW_PLAN" 2> /dev/null || break; sleep 0.25; done
  kill -KILL "$VIEW_NOW" "$VIEW_PLAN" 2> /dev/null || true
  # Diagnosis (ADR 0006): message rates of all topics, i.e. what would cross the WLAN in a --pi session.
  python3 "$ROOT/ros2_ws/install/robotros_sim/lib/robotros_sim/topic_rates.py" 10 | head -n 40 || true
  if python3 "$ROOT/ros2_ws/install/robotros_sim/lib/robotros_sim/sim_rtf.py" 5; then
    echo "ok    sim_rtf measured the real time factor"
  else
    echo "FAIL  sim_rtf could not measure the real time factor"
    rc=1
  fi
  if [ "$MODE" = nav ]; then
    # Browser UI end-to-end (ADR 0005). Required when WEB_UI_CHECK=1 (CI); otherwise skipped without Playwright.
    wrc=0
    python3 "$ROOT/tools/web_ui_check.py" --url "$WEB_URL/" --shot "$LOG_DIR/web_ui.png" || wrc=$?
    if [ "$wrc" -eq 1 ] || { [ "$wrc" -eq 2 ] && [ "${WEB_UI_CHECK:-0}" = 1 ]; }; then
      rc=1
    fi
  fi
  if [ "$MODE" = nav ]; then
    # Scan, odometry and AMCL while turning on the spot, against the ground truth (measurement only).
    python3 "$ROOT/ros2_ws/install/robotros_sim/lib/robotros_sim/rotation_check.py" || rc=1
    # Orders, stations, software E-stop (mission_manager part 1, ADR 0007).
    python3 "$ROOT/ros2_ws/install/robotros_sim/lib/robotros_sim/mission_check.py" || rc=1
  fi
  echo "---- activity_view now (German) ----"
  tr '\r' '\n' < "$VIEW_LOG.now" | grep -a . | tail -n 25 || true
  if [ "$MODE" = nav ]; then
    want_now="Ziel erreicht"; want_plan="Fahrt zu"
  else
    want_now="Wird von Hand gefahren"; want_plan="Nichts geplant"
  fi
  if grep -aq "$want_now" "$VIEW_LOG.now" && grep -aq "$want_plan" "$VIEW_LOG.plan"; then
    echo "ok    activity_view shows '$want_now' (now) and '$want_plan' (plan)"
  else
    echo "FAIL  activity_view did not show '$want_now' / '$want_plan'"
    tail -n 20 "$VIEW_LOG.plan"
    rc=1
  fi
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
    if [ -n "$PI_PID" ]; then
      echo "---- $PI_LOG: errors and warnings (first 60) ----"
      grep -aiE "error|fail|exception|died" "$PI_LOG" | head -n 60 || true
    fi
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
