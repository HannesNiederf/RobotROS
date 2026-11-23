# CLAUDE.md - RobotROS (project "Patrizio")

ROS 2 Jazzy transport robot: maps a room, shows map and robot pose on a Surface touch UI,
and carries grid carts from a chosen station to another. Raspberry Pi 5 runs the robot,
two Arduino Nano Every handle real-time and safety, the Surface shows a web UI.

## Rules (set by the project owner, they apply to every session)
- Only state things you are sure of. Mark everything else as unverified or as an assumption.
- Before software is handed over it must have been tested by you. Say exactly what was run
  and what could not be run (for example no ROS, no hardware).
- Never run anything that moves motors, flashes firmware or switches relays on the real robot
  without the owner's explicit OK for that command. Hardware tests are done by the owner.
- Never disable or bypass safety logic (E-stop, watchdog, cliff and bumper handling) to make a test pass.

## Language
- Code, identifiers, comments, commit messages and PR text: English.
- Documents and talking to the owner (Hannes): German. Final documentation also Italian and English.

## Where the specification lives
The binding specification is currently in the claude.ai project "RobotROS", not yet in this repository:
`softwarespezifikation.md` (D-01..D-28), `spez_aenderung_01_rueckwaerts.md` (ÄM-01, takes precedence),
`spez_firmware_protokoll.md`, `spez_parameter_tests.md`, `pinplan_nano.md`.
Copies are meant to go to `docs/spec/`. If `docs/spec/` is empty, ask the owner instead of guessing.
Conflict rule: the newer change wins (D-39 and round 26 > ÄM-01 > the three specs > projektwissen).
Decisions that change the spec are recorded as ADRs in `docs/adr/`.

## Layout
```
ros2_ws/src/   ROS packages (spec section 3.2). Implemented so far: patrizio_interfaces
firmware/      Nano firmware (PlatformIO) - not started
web_ui/        Browser UI (vanilla JS, no build step) - not started
deploy/        systemd, udev, sudoers, chrony, cyclonedds.xml - not started
tools/         helper scripts and their tests
docs/          documentation
```

## Build and test
```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws && colcon build && colcon test && colcon test-result --verbose
cd .. && tools/smoke_interfaces.sh           # after the build
python3 tools/check_interfaces.py ros2_ws/src  # interface syntax check, needs no ROS
python3 -m pytest tools/tests -q               # tests of the tools, need no ROS
```
CI (`.github/workflows/ci.yml`) runs the same in a `ros:jazzy` container. Add a CI job whenever
a new kind of code appears (ruff, PlatformIO native tests, `tools/check_i18n.py`).

## Conventions (spec section 16)
- Python with type annotations, `ruff`; C++ with `clang-format`; ROS parameters always declared with defaults.
- Keep logic out of node classes (`core/*.py`) so unit tests run without ROS.
- No values in code that belong in YAML (`patrizio_bringup/config/`). Never use wall-clock time for logic.
- Branches: `main` always works; work on `feature/<topic>`; open a PR; changes to safety logic need a second pair of eyes.

## Environment
- Surface Pro 7: Ubuntu 24.04.5, linux-surface kernel, ROS 2 Jazzy desktop, Gazebo Harmonic (for development).
- Pi 5: Ubuntu Server 24.04 arm64, ROS 2 Jazzy ros-base.
- Middleware: `rmw_cyclonedds_cpp`, `ROS_DOMAIN_ID=42`.

## Status
See the open tasks in the project index (`00_index.md` in the claude.ai project).
Interface fields marked "chosen here" in the .msg/.srv/.action comments are not fixed by the spec and may change.
