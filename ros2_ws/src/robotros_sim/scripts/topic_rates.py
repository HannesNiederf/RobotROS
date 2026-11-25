#!/usr/bin/env python3
"""Message rate and bandwidth of every topic for a while: topic_rates.py [SECONDS] (default 10).

Diagnosis for the distributed simulation (ADR 0006): what has to cross the WLAN. Wall clock is used only for the
measurement. Prints one line per topic, sorted by messages per second.
"""

from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message


def main() -> int:
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    rclpy.init()
    node = Node("topic_rates")
    end = time.monotonic() + 3.0
    while time.monotonic() < end:  # discovery
        rclpy.spin_once(node, timeout_sec=0.1)
    counts: dict[str, list[int]] = {}
    for name, types in node.get_topic_names_and_types():
        try:
            cls = get_message(types[0])
        except (AttributeError, ModuleNotFoundError, ValueError):
            continue
        counts[name] = [0, 0]

        def cb(msg: object, name: str = name) -> None:
            counts[name][0] += 1
            counts[name][1] += len(serialize_message(msg))

        node.create_subscription(cls, name, cb, qos_profile_sensor_data)
    start = time.monotonic()
    while time.monotonic() - start < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
    wall = time.monotonic() - start
    total = 0.0
    for name, (n, size) in sorted(counts.items(), key=lambda kv: -kv[1][0]):
        if n:
            total += size / wall
            print(f"RATE {name:40s} {n / wall:8.1f} msg/s {size / wall / 1024:9.1f} KiB/s")
    print(f"RATE total {total / 1024:.1f} KiB/s over {wall:.1f} s")
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
