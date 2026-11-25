"""map_manager, first part (ADR 0005): web map image, map meta, robot pose and the snapshot service.

Spec (softwarespezifikation.md 5.2/5.3): /web/map_image (PNG) and /web/map_meta on change and every
republish_period_s; /robot_pose (map frame) at pose_rate_hz; /map/get_snapshot for UI start and reconnect.
Map files, mode switching, stations, zones and contact points follow later (empty lists in the snapshot).
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from robotros_interfaces.msg import MapMeta
from robotros_interfaces.srv import GetSnapshot
from sensor_msgs.msg import CompressedImage
from tf2_ros import Buffer, TransformException, TransformListener

from .core.map_image import grid_to_png


class MapManager(Node):
    def __init__(self) -> None:
        super().__init__("map_manager")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("republish_period_s", 5.0)
        self.declare_parameter("pose_rate_hz", 5.0)
        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.image: CompressedImage | None = None
        self.meta: MapMeta | None = None
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, "map", self.on_map, latched)
        self.pub_image = self.create_publisher(CompressedImage, "web/map_image", latched)
        self.pub_meta = self.create_publisher(MapMeta, "web/map_meta", latched)
        self.pub_pose = self.create_publisher(PoseStamped, "robot_pose", 10)
        self.create_service(GetSnapshot, "map/get_snapshot", self.on_snapshot)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(self.get_parameter("republish_period_s").value, self.republish)
        self.create_timer(1.0 / self.get_parameter("pose_rate_hz").value, self.publish_pose)

    def on_map(self, msg: OccupancyGrid) -> None:
        info = msg.info
        image = CompressedImage()
        image.header.stamp = msg.header.stamp
        image.header.frame_id = msg.header.frame_id or self.map_frame
        image.format = "png"
        image.data = grid_to_png(msg.data, info.width, info.height)
        meta = MapMeta()
        meta.resolution = info.resolution
        meta.origin.x, meta.origin.y = info.origin.position.x, info.origin.position.y
        q = info.origin.orientation
        meta.origin.theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        meta.width, meta.height = info.width, info.height
        self.image, self.meta = image, meta
        self.republish()

    def republish(self) -> None:
        if self.image is not None and self.meta is not None:
            self.pub_meta.publish(self.meta)
            self.pub_image.publish(self.image)

    def publish_pose(self) -> None:
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time(),
                                                 timeout=Duration(seconds=0.0))
        except TransformException:
            return
        pose = PoseStamped()
        pose.header.stamp = tf.header.stamp
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.orientation = tf.transform.rotation
        self.pub_pose.publish(pose)

    def on_snapshot(self, _req: GetSnapshot.Request, res: GetSnapshot.Response) -> GetSnapshot.Response:
        res.success = self.image is not None
        if self.image is not None and self.meta is not None:
            res.map_image, res.map_meta = self.image, self.meta
        return res


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MapManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
