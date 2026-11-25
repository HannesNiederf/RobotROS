#!/usr/bin/env python3
"""Write the ground-truth lidar map of a world: world_map.py WORLD.sdf OUT_BASENAME [--lidar-z Z].

The lidar height defaults to wheel_radius_m * wheel_radius_scale + lidar_z from robotros_bringup/config/robot.yaml.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from robotros_sim.world_map import boxes_from_sdf, rasterize, write_map


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("world")
    ap.add_argument("out")
    ap.add_argument("--lidar-z", type=float)
    ap.add_argument("--resolution", type=float, default=0.05)  # spec: map resolution 0.05 m
    args = ap.parse_args()
    z = args.lidar_z
    if z is None:
        cfg_path = Path(get_package_share_directory("robotros_bringup")) / "config" / "robot.yaml"
        cfg = yaml.safe_load(cfg_path.read_text(encoding="utf-8"))
        z = cfg["wheel_radius_m"] * cfg["wheel_radius_scale"] + cfg["lidar_z"]
    boxes = boxes_from_sdf(Path(args.world).read_text(encoding="utf-8"))
    origin, size = (-4.0, -4.0), (8.0, 8.0)
    grid = rasterize(boxes, z, origin, size, args.resolution)
    print(write_map(grid, origin, args.resolution, Path(args.out)))


if __name__ == "__main__":
    main()
