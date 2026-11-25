"""Ground-truth map of worlds/generic.sdf (robotros_sim/world_map.py). Needs no ROS."""

from __future__ import annotations

import sys
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))

from robotros_sim.world_map import boxes_from_sdf, rasterize, write_map  # noqa: E402

WORLD = (PKG / "worlds" / "generic.sdf").read_text(encoding="utf-8")
LIDAR_Z = 0.1525
ORIGIN, SIZE, RES = (-4.0, -4.0), (8.0, 8.0), 0.05


def grid() -> list[list[bool]]:
    return rasterize(boxes_from_sdf(WORLD), LIDAR_Z, ORIGIN, SIZE, RES)


def occ(g: list[list[bool]], x: float, y: float) -> bool:
    return g[int((y - ORIGIN[1]) / RES)][int((x - ORIGIN[0]) / RES)]


def test_walls_obstacle_passage_glass() -> None:
    g = grid()
    assert occ(g, 0.0, 3.55) and occ(g, 0.0, -3.55) and occ(g, 3.55, 0.0) and occ(g, -3.55, 0.0)
    assert occ(g, -1.5, 1.0)                      # static obstacle
    assert occ(g, 1.5, -2.0) and occ(g, 1.5, 0.0)  # inner walls a and b
    assert not occ(g, 1.5, -0.775)                # centre of the 0.45 m passage
    free = sum(1 for y in (-0.975, -0.925, -0.875, -0.825, -0.775, -0.725, -0.675, -0.625, -0.575)
               if not occ(g, 1.5, y))
    assert free == 9                              # 0.45 m = 9 cells free
    assert not occ(g, -1.0, -2.0)                 # glass wall: invisible to the lidar
    assert not occ(g, 0.0, 0.0)


def test_cart_feet_below_plate_and_plate_ignored() -> None:
    g = grid()
    assert occ(g, -3.25 + 0.19, 0.19)             # foot of cart C1 (cuts the lidar plane)
    assert not occ(g, -3.25, 0.0)                 # plate is above the lidar plane


def test_write_map(tmp_path: Path) -> None:
    g = grid()
    yml = write_map(g, ORIGIN, RES, tmp_path / "m")
    data = (tmp_path / "m.pgm").read_bytes()
    assert data.startswith(b"P5\n160 160\n255\n") and len(data) == len(b"P5\n160 160\n255\n") + 160 * 160
    text = yml.read_text(encoding="utf-8")
    assert "image: m.pgm" in text and "resolution: 0.05" in text and "origin: [-4.0, -4.0, 0.0]" in text
