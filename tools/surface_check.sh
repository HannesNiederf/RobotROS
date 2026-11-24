#!/usr/bin/env bash
# Full check on the Surface (development machine) with a report for the developers.
#
#   tools/surface_check.sh                 test the branch that is checked out (normally main)
#   tools/surface_check.sh feature/xyz     fetch and test that branch first
#   tools/surface_check.sh --no-post ...   only write the report locally, do not post it
#
# Steps: environment, colcon build, colcon test, interface smoke test, Python tests, ruff,
# firmware host tests, headless simulation smoke test. The report is posted as a comment on
# GitHub issue #9 ("Surface test reports") with the gh CLI and also kept in build/surface_check/.
#
# Safety: nothing here talks to hardware. All ROS traffic uses an isolated domain (77) with
# localhost-only discovery, so the robot (domain 42) never sees it.
set -o pipefail

# Everything runs inside main(): bash parses the whole function before running it, so a git checkout
# that replaces this file while it runs cannot break the running copy.
main() {

REPO="HannesNiederf/RobotROS"
ISSUE=9
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT" || exit 1

POST=1
if [ "${1:-}" = "--no-post" ]; then
  POST=0
  shift
fi
BRANCH="${1:-}"

export ROS_DOMAIN_ID=77
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export SIM_SMOKE_DOMAIN_ID=77

STAMP="$(date +%Y%m%d-%H%M%S)"
OUT="$ROOT/build/surface_check/$STAMP"
mkdir -p "$OUT"
REPORT="$OUT/report.md"
SUMMARY="$OUT/summary.txt"
: > "$SUMMARY"
FAILED=0

# ---- branch ---------------------------------------------------------------------------------
if [ -n "$(git status --porcelain --untracked-files=no)" ]; then
  echo "The repository has local changes; please tell Claude instead of running the check." >&2
  git status --short >&2
  exit 1
fi
ORIG_BRANCH="${SURFACE_CHECK_RETURN_TO:-}"
if [ -n "$BRANCH" ]; then
  echo "== fetching and checking out $BRANCH"
  ORIG_BRANCH="$(git rev-parse --abbrev-ref HEAD)"
  git fetch origin "$BRANCH" && git checkout "$BRANCH" && git pull --ff-only origin "$BRANCH" || {
    echo "Could not check out $BRANCH." >&2
    exit 1
  }
  # Continue with the branch's own version of this script.
  if [ "$POST" -eq 1 ]; then
    exec env SURFACE_CHECK_RETURN_TO="$ORIG_BRANCH" "$ROOT/tools/surface_check.sh"
  else
    exec env SURFACE_CHECK_RETURN_TO="$ORIG_BRANCH" "$ROOT/tools/surface_check.sh" --no-post
  fi
elif [ -z "$ORIG_BRANCH" ]; then
  git pull --ff-only || echo "(git pull failed, testing the local state)"
fi

# ---- helpers --------------------------------------------------------------------------------
# step NAME COMMAND...   runs COMMAND, logs to $OUT/NAME.log, records PASS/FAIL and the duration.
step() {
  local name="$1"
  shift
  local log="$OUT/$name.log"
  local t0 t1 rc
  echo "== $name"
  t0=$(date +%s)
  "$@" > "$log" 2>&1
  rc=$?
  t1=$(date +%s)
  if [ "$rc" -eq 0 ]; then
    echo "PASS|$name|$((t1 - t0))" >> "$SUMMARY"
    echo "   PASS ($((t1 - t0)) s)"
  else
    echo "FAIL|$name|$((t1 - t0))" >> "$SUMMARY"
    echo "   FAIL ($((t1 - t0)) s), log: $log"
    FAILED=1
  fi
  return 0
}

skip() {
  echo "SKIP|$1|$2" >> "$SUMMARY"
  echo "== $1: SKIP ($2)"
}

have() { command -v "$1" > /dev/null 2>&1; }

ros_env() {
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
}

do_env() (
  echo "commit: $(git rev-parse HEAD) ($(git rev-parse --abbrev-ref HEAD))"
  echo "host: $(hostname), kernel $(uname -r)"
  grep PRETTY_NAME /etc/os-release
  echo "ROS_DISTRO (after setup): $(ros_env && echo "$ROS_DISTRO")"
  echo "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-unset}"
  (ros_env && gz sim --versions) 2>&1 | head -1 | sed 's/^/gz sim: /'
  python3 --version
  python3 -m pytest --version 2>&1 | head -1
  echo "ROS environment inherited from the calling shell: ${ROS_DISTRO:-no}${PYTHONPATH:+ (PYTHONPATH set)}"
  python3 -c "import serial; print('pyserial', serial.__version__)" 2>&1 | tail -1
  for tool in cmake g++ clang-format ruff pio; do
    if have "$tool"; then echo "$tool: $("$tool" --version 2>&1 | head -1)"; else echo "$tool: not installed"; fi
  done
  return 0
)

do_build() ( ros_env && cd ros2_ws && colcon build --event-handlers console_direct+ )
do_test() ( ros_env && cd ros2_ws && colcon test --event-handlers console_direct+ && colcon test-result --verbose )
# These tests need no ROS. Run them without a ROS environment that may be inherited from the calling shell
# (PYTHONPATH, pytest plugins of ROS), and print the header and a summary of everything not passed.
do_pytest() {
  env -u PYTHONPATH -u AMENT_PREFIX_PATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
    python3 -m pytest tools/tests ros2_ws/src/robotros_bridge/test ros2_ws/src/robotros_sim/test -ra -p no:cacheprovider
}
do_fw() {
  cmake -S firmware/test/host -B "$OUT/fw_build" -DWITH_SANITIZERS=ON && cmake --build "$OUT/fw_build" &&
    ctest --test-dir "$OUT/fw_build" --output-on-failure
}
do_sim() ( ros_env && tools/sim_smoke.sh "$OUT/sim" )

# ---- steps ----------------------------------------------------------------------------------
step environment do_env
step colcon_build do_build
if grep -q "^FAIL|colcon_build" "$SUMMARY"; then
  skip colcon_test "build failed"
  skip interfaces "build failed"
  skip sim_smoke "build failed"
else
  step colcon_test do_test
  step interfaces tools/smoke_interfaces.sh
fi
if python3 -c "import pytest, yaml" 2> /dev/null; then step pytest do_pytest; else skip pytest "python3-pytest or yaml missing"; fi
if have ruff; then step ruff ruff check .; else skip ruff "ruff not installed"; fi
if have cmake && have g++; then step firmware_host do_fw; else skip firmware_host "cmake or g++ missing"; fi
if ! grep -q "^FAIL|colcon_build" "$SUMMARY"; then step sim_smoke do_sim; fi

# ---- report ---------------------------------------------------------------------------------
{
  if [ "$FAILED" -eq 0 ]; then echo "## Surface check: PASS"; else echo "## Surface check: FAIL"; fi
  echo
  echo "Commit \`$(git rev-parse --short HEAD)\` on \`$(git rev-parse --abbrev-ref HEAD)\`, run $STAMP (local time)."
  echo
  echo "| Step | Result | Time / reason |"
  echo "|---|---|---|"
  while IFS='|' read -r res name info; do
    if [ "$res" = "SKIP" ]; then echo "| $name | $res | $info |"; else echo "| $name | $res | $info s |"; fi
  done < "$SUMMARY"
  echo
  echo "<details><summary>environment</summary>"
  echo
  echo '```'
  cat "$OUT/environment.log"
  echo '```'
  echo "</details>"
  for key in colcon_test sim_smoke; do
    if [ -f "$OUT/$key.log" ] && ! grep -q "^FAIL|$key" "$SUMMARY"; then
      echo
      echo "<details><summary>$key (last 25 lines)</summary>"
      echo
      echo '```'
      tail -n 25 "$OUT/$key.log"
      echo '```'
      echo "</details>"
    fi
  done
  while IFS='|' read -r res name _; do
    if [ "$res" = "FAIL" ]; then
      echo
      echo "<details open><summary>$name (failed, last 120 lines)</summary>"
      echo
      echo '```'
      tail -n 120 "$OUT/$name.log" | cut -c1-300
      echo '```'
      echo "</details>"
    fi
  done < "$SUMMARY"
} > "$REPORT"

# GitHub comments are limited to 65536 characters.
if [ "$(wc -c < "$REPORT")" -gt 60000 ]; then
  head -c 60000 "$REPORT" > "$REPORT.cut"
  printf '\n```\n\n(report cut, full logs in %s)\n' "$OUT" >> "$REPORT.cut"
  mv "$REPORT.cut" "$REPORT"
fi

if [ -n "$ORIG_BRANCH" ] && [ "$ORIG_BRANCH" != "$(git rev-parse --abbrev-ref HEAD)" ]; then
  git checkout -q "$ORIG_BRANCH" && echo "(back on $ORIG_BRANCH)"
fi

echo
tr '|' ' ' < "$SUMMARY"
echo "Report: $REPORT"
if [ "$POST" -eq 1 ]; then
  if gh issue comment "$ISSUE" --repo "$REPO" --body-file "$REPORT" > /dev/null; then
    echo "Report posted to https://github.com/$REPO/issues/$ISSUE - Claude can read it now."
  else
    echo "Posting the report failed; please send the file above to Claude."
  fi
fi
if [ "$FAILED" -eq 0 ]; then echo "RESULT: PASS"; else echo "RESULT: FAIL"; fi
return "$FAILED"
}

main "$@"
exit $?
