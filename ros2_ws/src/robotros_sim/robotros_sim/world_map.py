"""Occupancy map of a simulation world as the lidar sees it (ground truth for navigation tests).

Every box collision of the world SDF whose height range contains the lidar plane becomes occupied; collisions
whose name starts with "glass" are left out (invisible to the GPU lidar, softwarespezifikation.md section 14).
Output: map_server PGM (P5) + YAML, free = 254, occupied = 0. Pure Python, no ROS needed.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Box:
    """Box in the world frame, footprint only: centre, yaw, size x/y, z range."""

    x: float
    y: float
    yaw: float
    sx: float
    sy: float
    z0: float
    z1: float

    def contains(self, px: float, py: float) -> bool:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = px - self.x, py - self.y
        u, v = c * dx + s * dy, -s * dx + c * dy
        return abs(u) <= self.sx / 2 and abs(v) <= self.sy / 2


def _pose(elem: ET.Element | None) -> tuple[float, float, float, float]:
    """x, y, z, yaw of an SDF <pose> child (roll and pitch must be 0 for this flat map)."""
    text = elem.findtext("pose", "0 0 0 0 0 0") if elem is not None else "0 0 0 0 0 0"
    x, y, z, _, _, yaw = (float(v) for v in text.split())
    return x, y, z, yaw


def _compose(a: tuple[float, float, float, float], b: tuple[float, float, float, float]):  # noqa: ANN202
    c, s = math.cos(a[3]), math.sin(a[3])
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2], a[3] + b[3]


def boxes_from_sdf(sdf_text: str, skip_prefix: str = "glass") -> list[Box]:
    world = ET.fromstring(sdf_text).find("world")
    out: list[Box] = []
    for model in world.findall("model") if world is not None else []:
        mp = _pose(model)
        for link in model.findall("link"):
            lp = _compose(mp, _pose(link))
            for col in link.findall("collision"):
                size = col.findtext("geometry/box/size")
                if size is None or str(col.get("name", "")).startswith(skip_prefix):
                    continue
                sx, sy, sz = (float(v) for v in size.split())
                cx, cy, cz, yaw = _compose(lp, _pose(col))
                out.append(Box(cx, cy, yaw, sx, sy, cz - sz / 2, cz + sz / 2))
    return out


def rasterize(boxes: list[Box], lidar_z: float, origin: tuple[float, float], size_m: tuple[float, float],
              resolution: float) -> list[list[bool]]:
    """Occupied cells (row 0 = lowest y). A cell is occupied if a box cut by the lidar plane covers part of it
    (4 x 4 sample points per cell, so thin parts such as 2 cm cart feet are kept and box faces on a cell
    boundary do not spill into the neighbour cell)."""
    cut = [b for b in boxes if b.z0 <= lidar_z <= b.z1]
    w, h = round(size_m[0] / resolution), round(size_m[1] / resolution)
    offs = [(k + 0.5) / 4 * resolution for k in range(4)]
    grid = []
    for r in range(h):
        y0 = origin[1] + r * resolution
        row = []
        for c in range(w):
            x0 = origin[0] + c * resolution
            reach = [(b, b.sx + b.sy + resolution) for b in cut]
            near = [b for b, d in reach if abs(b.x - x0) < d and abs(b.y - y0) < d]
            row.append(any(b.contains(x0 + u, y0 + v) for b in near for u in offs for v in offs))
        grid.append(row)
    return grid


def write_map(grid: list[list[bool]], origin: tuple[float, float], resolution: float, out: Path) -> Path:
    """Write out.pgm and out.yaml for nav2_map_server; return the YAML path."""
    h, w = len(grid), len(grid[0])
    pixels = bytearray()
    for r in range(h - 1, -1, -1):  # PGM rows run from the top (highest y) down
        pixels.extend(0 if occ else 254 for occ in grid[r])
    pgm, yml = out.with_suffix(".pgm"), out.with_suffix(".yaml")
    pgm.write_bytes(f"P5\n{w} {h}\n255\n".encode() + bytes(pixels))
    yml.write_text(
        f"image: {pgm.name}\nmode: trinary\nresolution: {resolution}\n"
        f"origin: [{origin[0]}, {origin[1]}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n",
        encoding="utf-8",
    )
    return yml
