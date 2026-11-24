"""Static checks of the simulation world and the bridge config (no Gazebo or ROS needed)."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
WORLD = PKG / "worlds" / "generic.sdf"
ROBOT_YAML = PKG.parent / "robotros_bringup" / "config" / "robot.yaml"


@pytest.fixture(scope="module")
def world() -> ET.Element:
    return ET.parse(WORLD).getroot().find("world")


def boxes(model: ET.Element, kind: str) -> dict[str, tuple[list[float], list[float]]]:
    out = {}
    for el in model.iter(kind):
        pose = [float(v) for v in el.findtext("pose", "0 0 0 0 0 0").split()]
        size = [float(v) for v in el.findtext("geometry/box/size").split()]
        out[el.get("name")[:-2]] = (pose, size)
    return out


def model(world: ET.Element, name: str) -> ET.Element:
    m = world.find(f"model[@name='{name}']")
    assert m is not None, name
    return m


def test_world_name_and_systems(world: ET.Element) -> None:
    assert world.get("name") == "generic"
    plugins = {p.get("name") for p in world.findall("plugin")}
    assert {"gz::sim::systems::Physics", "gz::sim::systems::Sensors", "gz::sim::systems::SceneBroadcaster"} <= plugins


def test_room_is_7_by_7_with_010_walls(world: ET.Element) -> None:
    walls = boxes(model(world, "room"), "collision")
    (pn, sn), (pe, se) = walls["wall_n"], walls["wall_e"]
    assert pn[1] - sn[1] / 2 == pytest.approx(3.5)  # inner face of the north wall
    assert pe[0] - se[0] / 2 == pytest.approx(3.5)
    assert sn[1] == pytest.approx(0.10) and se[0] == pytest.approx(0.10)
    assert walls["wall_s"][0][1] + walls["wall_s"][1][1] / 2 == pytest.approx(-3.5)
    assert walls["wall_w"][0][0] + walls["wall_w"][1][0] / 2 == pytest.approx(-3.5)


def test_narrow_passage_is_045(world: ET.Element) -> None:
    walls = boxes(model(world, "room"), "collision")
    (pa, sa), (pb, sb) = walls["inner_a"], walls["inner_b"]
    gap = (pb[1] - sb[1] / 2) - (pa[1] + sa[1] / 2)
    assert gap == pytest.approx(0.45)


def test_glass_wall_has_no_visual(world: ET.Element) -> None:
    room = model(world, "room")
    assert "glass" in boxes(room, "collision")
    assert "glass" not in boxes(room, "visual")


@pytest.mark.parametrize("name", ["cart_c1", "cart_c2"])
def test_cart_fits_the_robot(world: ET.Element, name: str) -> None:
    cfg = yaml.safe_load(ROBOT_YAML.read_text())
    parts = boxes(model(world, name), "collision")
    feet = [v for k, v in parts.items() if k.startswith("foot")]
    assert len(feet) == 4
    xs = sorted({round(p[0], 4) for p, _ in feet})
    foot = feet[0][1][0]
    clear = (xs[1] - xs[0]) - foot
    assert foot == pytest.approx(0.02)  # spec section 14: supports 0.02 m
    assert clear >= 0.36 - 1e-9  # round 26: at least 360 mm between the feet
    assert clear > cfg["robot_width"]
    outer = (xs[1] - xs[0]) + foot
    assert outer == pytest.approx(cfg["cart_size"])
    # Plate above the lidar (lidar_z above the axle, axle at wheel radius).
    plate_p, plate_s = parts["plate"]
    underside = plate_p[2] - plate_s[2] / 2
    lidar_height = cfg["wheel_radius_m"] + cfg["lidar_z"]
    assert underside >= lidar_height + 0.05


def test_walled_cart_open_towards_room(world: ET.Element) -> None:
    parts = boxes(model(world, "cart_c2"), "collision")
    walls = {k for k in parts if k.startswith("wall_")}
    assert walls == {"wall_back", "wall_left", "wall_right"}
    assert parts["wall_back"][0][1] > 0  # back wall towards the north wall, open side towards -y


def test_bridge_topics() -> None:
    entries = yaml.safe_load((PKG / "config" / "gz_bridge.yaml").read_text())
    ros = {e["ros_topic_name"]: e for e in entries}
    assert set(ros) == {"/clock", "/scan", "/odom", "/tf", "/joint_states", "/sim/cmd_vel", "/sim/ground_truth"}
    # Only sim_hw_adapter drives Gazebo: the ROS /cmd_vel is not bridged (it goes through the simulated Nano).
    assert "/cmd_vel" not in ros
    assert ros["/sim/cmd_vel"]["direction"] == "ROS_TO_GZ"
    assert all(e["direction"] == "GZ_TO_ROS" for k, e in ros.items() if k != "/sim/cmd_vel")


def test_bridge_matches_gazebo_topics() -> None:
    gazebo = ET.parse(PKG.parent / "robotros_description" / "urdf" / "robotros.gazebo.xacro").getroot()
    used = {el.text for tag in ("topic", "odom_topic", "tf_topic") for el in gazebo.iter(tag)}
    entries = yaml.safe_load((PKG / "config" / "gz_bridge.yaml").read_text())
    bridged = {e["gz_topic_name"] for e in entries} - {"/clock"}
    assert used - {"/sim/ground_truth_tf"} == bridged  # the ground-truth TF stays inside Gazebo
