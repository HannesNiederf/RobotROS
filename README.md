# RobotROS

ROS 2 robot ("Patrizio") that maps a room, shows the map and its position on a Surface touch
screen, and autonomously carries grid carts from a chosen station to another.

## Status
Early development. Implemented so far: the interface package `patrizio_interfaces`
(messages, services and actions). Everything else follows the order in the specification.

## Layout
| Directory | Content |
|---|---|
| `ros2_ws/src/` | ROS 2 packages |
| `firmware/` | Arduino Nano Every firmware (PlatformIO) |
| `web_ui/` | Browser UI for the Surface |
| `deploy/` | systemd, udev and network configuration for the Pi |
| `tools/` | helper scripts and their tests |
| `docs/` | documentation |

## Quickstart (Ubuntu 24.04, ROS 2 Jazzy)
```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build
colcon test && colcon test-result --verbose
```
