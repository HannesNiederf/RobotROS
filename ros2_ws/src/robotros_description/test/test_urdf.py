"""The xacro model expands and matches robot.yaml (needs ROS: xacro Python module)."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

xacro = pytest.importorskip("xacro")

PKG = Path(__file__).resolve().parents[1]
XACRO = PKG / "urdf" / "robotros.urdf.xacro"
ROBOT_YAML = PKG.parent / "robotros_bringup" / "config" / "robot.yaml"

FRAMES = {"base_footprint", "base_link", "laser", "us_front", "us_rear", "bumper", "left_wheel", "right_wheel"}


def expand(sim: bool) -> ET.Element:
    doc = xacro.process_file(str(XACRO), mappings={"robot_yaml": str(ROBOT_YAML), "sim": str(sim).lower()})
    return ET.fromstring(doc.toxml())


@pytest.fixture(scope="module")
def cfg() -> dict:
    return yaml.safe_load(ROBOT_YAML.read_text())


def joint(root: ET.Element, name: str) -> ET.Element:
    found = root.find(f"joint[@name='{name}']")
    assert found is not None, name
    return found


def origin(j: ET.Element) -> tuple[list[float], list[float]]:
    o = j.find("origin")
    xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
    rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
    return xyz, rpy


def test_frames_and_tree() -> None:
    root = expand(sim=False)
    links = {link.get("name") for link in root.findall("link")}
    assert FRAMES <= links
    parents = {j.find("child").get("link"): j.find("parent").get("link") for j in root.findall("joint")}
    # Every link except the root has exactly one parent, and the root is base_footprint (spec section 4).
    assert links - set(parents) == {"base_footprint"}
    for frame in ("laser", "us_front", "us_rear", "bumper", "left_wheel", "right_wheel"):
        assert parents[frame] == "base_link", frame


def test_geometry_from_yaml(cfg: dict) -> None:
    root = expand(sim=False)
    wheel_r = cfg["wheel_radius_m"] * cfg["wheel_radius_scale"]
    track = cfg["track_width_m"] * cfg["track_scale"]

    xyz, _ = origin(joint(root, "base_footprint_joint"))
    assert xyz == pytest.approx([0, 0, wheel_r])

    left, _ = origin(joint(root, "left_wheel_joint"))
    right, _ = origin(joint(root, "right_wheel_joint"))
    assert left == pytest.approx([0, track / 2, 0])
    assert right == pytest.approx([0, -track / 2, 0])
    assert joint(root, "left_wheel_joint").get("type") == "continuous"

    xyz, rpy = origin(joint(root, "laser_joint"))
    assert xyz == pytest.approx([cfg["lidar_x"], 0, cfg["lidar_z"]])
    assert rpy == pytest.approx([0, 0, cfg["lidar_yaw_rad"]])

    pitch = math.radians(cfg["us_pitch_deg"])
    xyz, rpy = origin(joint(root, "us_front_joint"))
    assert xyz[0] == pytest.approx(cfg["us_front_x"])
    assert rpy == pytest.approx([0, pitch, 0])
    xyz, rpy = origin(joint(root, "us_rear_joint"))
    assert xyz[0] == pytest.approx(cfg["us_rear_x"])
    assert rpy == pytest.approx([0, pitch, math.pi])

    xyz, _ = origin(joint(root, "bumper_joint"))
    assert xyz[0] == pytest.approx(cfg["bumper_front_x"])

    body = root.find("link[@name='base_link']/collision/geometry/box")
    assert [float(v) for v in body.get("size").split()][:2] == pytest.approx([cfg["robot_length"], cfg["robot_width"]])


def test_casters_touch_ground(cfg: dict) -> None:
    root = expand(sim=False)
    wheel_r = cfg["wheel_radius_m"] * cfg["wheel_radius_scale"]
    for name in ("caster_front", "caster_rear"):
        xyz, _ = origin(joint(root, f"{name}_joint"))
        r = float(root.find(f"link[@name='{name}']/collision/geometry/sphere").get("radius"))
        assert wheel_r + xyz[2] - r == pytest.approx(0.0, abs=1e-9), name


def test_no_gazebo_without_sim() -> None:
    assert expand(sim=False).find("gazebo") is None


def test_sim_plugins(cfg: dict) -> None:
    root = expand(sim=True)
    plugins = {p.get("name"): p for p in root.iter("plugin")}
    dd = plugins["gz::sim::systems::DiffDrive"]
    assert float(dd.findtext("wheel_separation")) == pytest.approx(cfg["track_width_m"] * cfg["track_scale"])
    assert float(dd.findtext("wheel_radius")) == pytest.approx(cfg["wheel_radius_m"] * cfg["wheel_radius_scale"])
    assert dd.findtext("frame_id") == "odom"
    assert dd.findtext("child_frame_id") == "base_footprint"
    assert "gz::sim::systems::JointStatePublisher" in plugins

    sensor = root.find("gazebo[@reference='laser']/sensor")
    assert sensor.get("type") == "gpu_lidar"
    assert sensor.findtext("gz_frame_id") == "laser"
    assert int(sensor.findtext("lidar/scan/horizontal/samples")) == 720
    assert float(sensor.findtext("lidar/range/min")) == pytest.approx(0.15)
    assert float(sensor.findtext("lidar/range/max")) == pytest.approx(12.0)
    assert float(sensor.findtext("update_rate")) == pytest.approx(10.0)
