"""Lidar masking of scan_masker (softwarespezifikation.md section 8.1, ÄM-01 D-31). No ROS imports.

- Self mask: rectangles in base_link (``self_mask_boxes`` in robot.yaml, each ``[x_min, y_min, x_max, y_max]``).
- Cart mask: while the hub is up, the square ``±(cart_mask_half + cart_mask_margin)`` around base_link
  (the cart's feet) is masked.
- Range limits ``range_min`` .. ``range_max``.
Masked or out-of-range beams become ``+inf``, never NaN (D-31: AMCL skips beams >= range_max, NaN would
break its weights; the costmap obstacle layers use ``inf_is_valid: false`` so they neither mark nor clear).
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

Box = tuple[float, float, float, float]  # x_min, y_min, x_max, y_max in base_link


@dataclass(frozen=True)
class MaskConfig:
    range_min: float
    range_max: float
    self_boxes: tuple[Box, ...]
    cart_half: float  # half size of the cart mask square incl. margin
    laser_x: float = 0.0  # laser pose in base_link
    laser_y: float = 0.0
    laser_yaw: float = 0.0

    @classmethod
    def from_yaml(cls, robot_cfg: dict, obstacles_cfg: dict) -> MaskConfig:
        sm = obstacles_cfg["scan_masker"]
        boxes = []
        for box in robot_cfg.get("self_mask_boxes") or []:
            x0, y0, x1, y1 = (float(v) for v in box)
            boxes.append((min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)))
        return cls(
            range_min=float(sm["range_min"]),
            range_max=float(sm["range_max"]),
            self_boxes=tuple(boxes),
            cart_half=float(sm["cart_mask_half"]) + float(sm["cart_mask_margin"]),
            laser_x=float(robot_cfg.get("lidar_x", 0.0)),
            laser_y=0.0,
            laser_yaw=float(robot_cfg.get("lidar_yaw_rad", 0.0)),
        )


def _in_boxes(x: np.ndarray, y: np.ndarray, boxes: Sequence[Box]) -> np.ndarray:
    hit = np.zeros(x.shape, dtype=bool)
    for x0, y0, x1, y1 in boxes:
        hit |= (x >= x0) & (x <= x1) & (y >= y0) & (y <= y1)
    return hit


def mask_scan(
    ranges: Sequence[float], angle_min: float, angle_increment: float, cfg: MaskConfig, cart_mask: bool
) -> np.ndarray:
    """Return the filtered ranges (float32) for one scan."""
    r = np.asarray(ranges, dtype=np.float64)
    angles = angle_min + angle_increment * np.arange(r.size) + cfg.laser_yaw
    bad = ~np.isfinite(r) | (r < cfg.range_min) | (r > cfg.range_max)
    safe = np.where(bad, 0.0, r)
    x = cfg.laser_x + safe * np.cos(angles)
    y = cfg.laser_y + safe * np.sin(angles)
    boxes = list(cfg.self_boxes)
    if cart_mask:
        h = cfg.cart_half
        boxes.append((-h, -h, h, h))
    bad |= _in_boxes(x, y, boxes)
    out = r.astype(np.float32)
    out[bad] = np.inf
    return out
