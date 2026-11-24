"""scan_masker node: /scan -> /scan_filtered with self and cart mask (softwarespezifikation.md section 8.1)."""

from __future__ import annotations

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robotros_interfaces.msg import HubState
from sensor_msgs.msg import LaserScan

from .core.scan_mask import MaskConfig, mask_scan


def _load(path: str) -> dict:
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


class ScanMasker(Node):
    def __init__(self) -> None:
        super().__init__("scan_masker")
        share = get_package_share_directory("robotros_bringup")
        self.declare_parameter("robot_yaml", f"{share}/config/robot.yaml")
        self.declare_parameter("obstacles_yaml", f"{share}/config/obstacles.yaml")
        self.cfg = MaskConfig.from_yaml(
            _load(self.get_parameter("robot_yaml").value), _load(self.get_parameter("obstacles_yaml").value)
        )
        self.cart_mask = False
        self.pub = self.create_publisher(LaserScan, "scan_filtered", qos_profile_sensor_data)
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(HubState, "hw/hub_state", self.on_hub, 10)

    def on_hub(self, msg: HubState) -> None:
        self.cart_mask = bool(msg.limit_top) or msg.state == HubState.TOP

    def on_scan(self, msg: LaserScan) -> None:
        out = LaserScan()
        out.header = msg.header
        out.angle_min, out.angle_max, out.angle_increment = msg.angle_min, msg.angle_max, msg.angle_increment
        out.time_increment, out.scan_time = msg.time_increment, msg.scan_time
        out.range_min, out.range_max = self.cfg.range_min, self.cfg.range_max
        out.ranges = mask_scan(msg.ranges, msg.angle_min, msg.angle_increment, self.cfg, self.cart_mask).tolist()
        out.intensities = list(msg.intensities)
        self.pub.publish(out)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ScanMasker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
