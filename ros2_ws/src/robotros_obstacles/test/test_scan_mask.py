"""scan_mask core without ROS."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG))
from robotros_obstacles.core.scan_mask import MaskConfig, mask_scan  # noqa: E402

CONFIG = PKG.parent / "robotros_bringup" / "config"


def cfg(**kw: object) -> MaskConfig:
    base = dict(range_min=0.15, range_max=8.0, self_boxes=(), cart_half=0.20)
    base.update(kw)
    return MaskConfig(**base)  # type: ignore[arg-type]


def beams(values: list[float]) -> tuple[list[float], float, float]:
    n = len(values)
    return values, -math.pi, 2 * math.pi / n


def test_range_limits_and_nan_become_inf() -> None:
    r, a0, da = beams([0.1, 0.15, 1.0, 8.0, 9.0, float("nan"), float("inf"), -1.0])
    out = mask_scan(r, a0, da, cfg(), cart_mask=False)
    assert np.isinf(out[0]) and out[1] == pytest.approx(0.15) and out[2] == pytest.approx(1.0)
    assert out[3] == pytest.approx(8.0)
    assert all(np.isinf(out[4:])) and not np.isnan(out).any()
    assert all(out[4:] > 0)  # +inf, never -inf


def test_cart_mask_only_when_lifted() -> None:
    # 8 beams around; feet at 0.19 m diagonal distance about 0.27 m -> inside the +-0.20 square on the diagonals.
    n = 8
    r = [0.27] * n
    a0, da = -math.pi, 2 * math.pi / n
    off = mask_scan(r, a0, da, cfg(), cart_mask=False)
    on = mask_scan(r, a0, da, cfg(), cart_mask=True)
    assert np.isfinite(off).all()
    diag = [i for i in range(n) if abs(math.sin(2 * (a0 + i * da))) > 0.9]  # +-45 deg, +-135 deg
    straight = [i for i in range(n) if i not in diag]
    assert all(np.isinf(on[i]) for i in diag)
    assert all(np.isfinite(on[i]) for i in straight)  # 0.27 m straight ahead is outside 0.20


def test_self_mask_box_with_laser_offset() -> None:
    c = cfg(self_boxes=((0.25, -0.05, 0.35, 0.05),), laser_x=0.1)
    # beam straight ahead (angle 0) at 0.2 m -> point x = 0.3 in base_link -> masked
    r, a0, da = [0.2, 0.2], 0.0, math.pi
    out = mask_scan(r, a0, da, c, cart_mask=False)
    assert np.isinf(out[0]) and out[1] == pytest.approx(0.2)


def test_from_yaml_uses_bringup_config() -> None:
    robot = yaml.safe_load((CONFIG / "robot.yaml").read_text())
    obstacles = yaml.safe_load((CONFIG / "obstacles.yaml").read_text())
    c = MaskConfig.from_yaml(robot, obstacles)
    assert c.range_min == pytest.approx(0.15) and c.range_max == pytest.approx(8.0)
    assert c.cart_half == pytest.approx(0.17 + 0.03)
    assert c.self_boxes == ()
