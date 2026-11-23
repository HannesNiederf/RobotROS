#!/usr/bin/env bash
# Post-build smoke test for robotros_interfaces: every interface file in the
# package must be installed and visible to `ros2 interface show`, and a few
# generated Python types must behave as the spec says.
# Usage (after `colcon build` in ros2_ws): tools/smoke_interfaces.sh
set -eo pipefail

PKG=robotros_interfaces
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$ROOT/ros2_ws/src/$PKG"

# ROS setup scripts read unset variables, so `set -u` is deliberately not used.
# shellcheck disable=SC1091
source "$ROOT/ros2_ws/install/setup.bash"

status=0
count=0
for kind in msg srv action; do
  for file in "$SRC/$kind"/*."$kind"; do
    [ -e "$file" ] || continue
    name="$(basename "$file" ".$kind")"
    count=$((count + 1))
    if ros2 interface show "$PKG/$kind/$name" > /dev/null 2>&1; then
      echo "ok    $PKG/$kind/$name"
    else
      echo "FAIL  $PKG/$kind/$name"
      status=1
    fi
  done
done
echo "checked $count interface(s) with ros2 interface show"

python3 - <<'PY'
from robotros_interfaces.msg import (HubState, MissionStatus, Order, OrderList,
                                     RangeState, SafetyState, Station)
from robotros_interfaces.srv import CreateOrder, GetSnapshot, SetArm
from robotros_interfaces.action import EngageCart, Localize

# Values fixed by softwarespezifikation.md section 5.1 / spez_firmware_protokoll.md 2.3.
assert (SafetyState.BOOT_LOCKED, SafetyState.ESTOP, SafetyState.WAIT_ACK, SafetyState.ARMED) == (0, 1, 2, 3)
assert (RangeState.CLASS_UNKNOWN, RangeState.CLASS_FLOOR, RangeState.CLASS_CLIFF, RangeState.CLASS_OBSTACLE) == (0, 1, 2, 3)
assert (HubState.IDLE, HubState.MOVING_UP, HubState.MOVING_DOWN, HubState.TOP, HubState.BOTTOM, HubState.FAULT) == (0, 1, 2, 3, 4, 5)

# Default construction, field assignment and nesting work.
order = Order(id=7, type=Order.TYPE_TRANSPORT, priority=5, state=Order.STATE_QUEUED)
orders = OrderList(orders=[order])
assert orders.orders[0].priority == 5
station = Station(id=1, name="S1", kind=Station.KIND_STORAGE)
station.pose.theta = 1.5
assert station.pose.theta == 1.5
assert MissionStatus().queue_length == 0

# Services and actions expose request/response/goal/result/feedback types.
assert CreateOrder.Request().priority == 0 and CreateOrder.Response().success is False
assert SetArm.Request().arm is False
assert hasattr(GetSnapshot.Response(), "map_image")
assert EngageCart.Goal().station_id == 0
assert Localize.Goal.MODE_RELOCALIZE == 1
print("python type checks ok")
PY

exit "$status"
