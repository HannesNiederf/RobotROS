"""Occupancy grid -> PNG for the web UI (/web/map_image, softwarespezifikation.md 5.3). Pure Python, no ROS.

The PNG is 8-bit grey, one pixel per cell, top row = highest y (row 0 of the grid is the lowest y, as in
nav_msgs/OccupancyGrid). The UI places it with /web/map_meta (resolution, origin, width, height).
"""

from __future__ import annotations

import struct
import zlib
from collections.abc import Sequence

# Grey values of the web map: free light, occupied dark, unknown middle grey.
FREE, OCCUPIED, UNKNOWN = 245, 40, 160


def cell_grey(value: int, occupied_thresh: int = 65, free_thresh: int = 25) -> int:
    """OccupancyGrid value (-1 unknown, 0..100) -> grey level (thresholds as map_server's trinary mode)."""
    if value < 0:
        return UNKNOWN
    if value >= occupied_thresh:
        return OCCUPIED
    if value <= free_thresh:
        return FREE
    return UNKNOWN


def _chunk(kind: bytes, data: bytes) -> bytes:
    return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)


def grid_to_png(data: Sequence[int], width: int, height: int) -> bytes:
    """Encode an occupancy grid (row-major, row 0 = lowest y) as a greyscale PNG with the top row = highest y."""
    if width <= 0 or height <= 0 or len(data) != width * height:
        raise ValueError(f"grid size {width}x{height} does not match {len(data)} cells")
    lut = bytes(cell_grey(v - 256 if v > 127 else v) for v in range(256))  # int8 values arrive as -1..100
    raw = bytearray()
    for row in range(height - 1, -1, -1):
        raw.append(0)  # PNG filter type 0 (none)
        start = row * width
        raw.extend(lut[v & 0xFF] for v in data[start:start + width])
    header = struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)  # 8-bit greyscale
    return b"\x89PNG\r\n\x1a\n" + _chunk(b"IHDR", header) + _chunk(b"IDAT", zlib.compress(bytes(raw), 6)) + \
        _chunk(b"IEND", b"")


def png_size(png: bytes) -> tuple[int, int]:
    """Width and height from a PNG header (for tests)."""
    if png[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("not a PNG")
    return struct.unpack(">II", png[16:24])
