#!/usr/bin/env bash
# Robot software of the distributed simulation (ADR 0006), started by tools/sim_start.sh --pi on the Pi over SSH
# (or locally with --pi=local in CI). Gazebo and RViz run on the Surface.
#
#   tools/pi_robot_software.sh MODE [MAP.yaml]     MODE = mapping | nav
#
# Starts robotros_sim/launch/robot_software.launch.py (sim_hw_adapter, navigation, sim_helper, web UI) in domain 77
# and prints the CPU and memory load of this computer every 5 s as "PILOAD ..." lines. Stops cleanly on HUP/INT/TERM
# (the SSH connection closing). Simulation only: no serial port, no hardware is touched.
set -o pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-}"
MAP="${2:-}"
case "$MODE" in mapping|nav) ;; *) echo "usage: $0 mapping|nav [MAP.yaml]" >&2; exit 2 ;; esac

export ROS_DOMAIN_ID="${SIM_DOMAIN_ID:-77}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${PI_DISCOVERY:-SUBNET}"
# Same middleware and WLAN settings as tools/sim_start.sh (ADR 0006); ~/.bashrc is not read over SSH.
export RMW_IMPLEMENTATION="${SIM_RMW:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="file://$ROOT/ros2_ws/src/robotros_sim/config/cyclonedds_wifi.xml"
if [ "$ROS_DOMAIN_ID" = "42" ]; then
  echo "refusing to run on ROS_DOMAIN_ID 42 (the robot's domain)" >&2
  exit 2
fi
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$ROOT/ros2_ws/install/setup.bash" || { echo "not built: cd $ROOT/ros2_ws && colcon build" >&2; exit 1; }

ARGS=(mode:="$MODE" auto:=true web:=true)
[ "$MODE" = nav ] && ARGS+=("map:=$MAP")
[ -n "${SIM_DB:-}" ] && ARGS+=("db:=$SIM_DB")  # fresh mission database of a --check run (ADR 0007)
echo "robot software ($MODE) on $(hostname), domain $ROS_DOMAIN_ID, discovery $ROS_AUTOMATIC_DISCOVERY_RANGE," \
  "middleware $RMW_IMPLEMENTATION"
# WLAN power saving delays incoming packets by up to a beacon interval (assumption to be checked on the Pi).
if command -v iw > /dev/null; then
  for dev in /sys/class/net/*/wireless; do
    [ -e "$dev" ] || continue
    dev="$(basename "$(dirname "$dev")")"
    echo "WLAN $dev: $(iw dev "$dev" get power_save 2>&1 | head -n 1)"
  done
fi
setsid ros2 launch robotros_sim robot_software.launch.py "${ARGS[@]}" &
LAUNCH=$!

python3 -u - <<'PY' &
import time
def cpu():
    with open("/proc/stat") as f:
        v = [int(x) for x in f.readline().split()[1:]]
    return sum(v), v[3] + v[4]
def mem():
    info = {}
    with open("/proc/meminfo") as f:
        for line in f:
            k, val = line.split(":")
            info[k] = int(val.split()[0])
    return (info["MemTotal"] - info["MemAvailable"]) // 1024
t0, i0 = cpu()
while True:
    time.sleep(5)
    t1, i1 = cpu()
    busy = 100.0 * (1 - (i1 - i0) / max(1, t1 - t0))
    print(f"PILOAD cpu_busy={busy:.0f}% mem_used={mem()}MiB", flush=True)
    t0, i0 = t1, i1
PY
SAMPLER=$!

stop() {
  kill -INT -- "-$LAUNCH" 2> /dev/null || true
  for _ in $(seq 1 30); do kill -0 "$LAUNCH" 2> /dev/null || break; sleep 0.5; done
  kill -KILL -- "-$LAUNCH" 2> /dev/null || true
  kill "$SAMPLER" 2> /dev/null || true
  exit 0
}
trap stop HUP INT TERM
wait "$LAUNCH"
kill "$SAMPLER" 2> /dev/null || true
