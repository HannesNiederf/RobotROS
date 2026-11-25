"""Web map image (robotros_map/core/map_image.py). Needs no ROS."""

from __future__ import annotations

import sys
import zlib
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from robotros_map.core.map_image import FREE, OCCUPIED, UNKNOWN, cell_grey, grid_to_png, png_size  # noqa: E402


def decode_rows(png: bytes, width: int, height: int) -> list[bytes]:
    """Minimal decoder for the PNGs written here (single IDAT, filter 0)."""
    pos, idat = 8, b""
    while pos < len(png):
        length = int.from_bytes(png[pos:pos + 4], "big")
        kind = png[pos + 4:pos + 8]
        if kind == b"IDAT":
            idat += png[pos + 8:pos + 8 + length]
        pos += 12 + length
    raw = zlib.decompress(idat)
    rows = [raw[r * (width + 1):(r + 1) * (width + 1)] for r in range(height)]
    assert all(row[0] == 0 for row in rows)
    return [row[1:] for row in rows]


def test_cell_grey() -> None:
    assert cell_grey(-1) == UNKNOWN and cell_grey(0) == FREE and cell_grey(100) == OCCUPIED
    assert cell_grey(65) == OCCUPIED and cell_grey(25) == FREE and cell_grey(50) == UNKNOWN


def test_png_rows_are_flipped_and_values_mapped() -> None:
    # 3 x 2 grid: row 0 (lowest y) = free, unknown, occupied; row 1 (top) = occupied, free, free
    grid = [0, -1, 100, 100, 0, 0]
    png = grid_to_png(grid, 3, 2)
    assert png_size(png) == (3, 2)
    rows = decode_rows(png, 3, 2)
    assert rows[0] == bytes([OCCUPIED, FREE, FREE])        # top of the image = highest y
    assert rows[1] == bytes([FREE, UNKNOWN, OCCUPIED])


def test_int8_wraparound_values() -> None:
    png = grid_to_png([255, 0], 2, 1)  # 255 as unsigned byte = -1 (unknown)
    assert decode_rows(png, 2, 1)[0] == bytes([UNKNOWN, FREE])


def test_size_mismatch() -> None:
    with pytest.raises(ValueError):
        grid_to_png([0, 0, 0], 2, 2)
