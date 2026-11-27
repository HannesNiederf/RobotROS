"""scan_masker node: /scan -> /scan_filtered with self and cart mask (softwarespezifikation.md section 8.1)."""

from __future__ import annotations

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from robotros_interfaces.msg import HubState
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

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
        self.hub_top = False
        self.mask_request = False
        self.cart_mask = False
        self.pub = self.create_publisher(LaserScan, "scan_filtered", qos_profile_sensor_data)
        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(HubState, "hw/hub_state", self.on_hub, 10)
        # D-30: the docking server switches the mask from the start of the reverse drive into the box until the
        # forward exit after setting the cart down. The hub at the top keeps it on as well (cart lifted).
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "mission/scan_mask", self.on_mask_request, latched)

    def on_hub(self, msg: HubState) -> None:
        self.hub_top = bool(msg.limit_top) or msg.state == HubState.TOP
        self.cart_mask = self.hub_top or self.mask_request

    def on_mask_request(self, msg: Bool) -> None:
        self.mask_request = bool(msg.data)
        self.cart_mask = self.hub_top or self.mask_request

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
