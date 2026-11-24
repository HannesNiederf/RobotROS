"""sim_hw_adapter: stands in for nano_bridge + both Nanos in the simulation (softwarespezifikation.md section 14).

ROS side (same names as nano_bridge): subscribes /cmd_vel, /hw/cmd/hub; publishes /hw/safety, /hw/range,
/hw/bumper, /hw/power, /hw/hub_state; services /hw/set_arm, /hw/set_relay.
Simulation side: publishes /sim/cmd_vel (to Gazebo's diff drive, only what the simulated Nano lets
through), reads the true robot pose /sim/ground_truth (for cliff areas and the dock) and the test
inputs /sim/estop and /sim/bumper (std_msgs).
Odometry, /joint_states and TF come from Gazebo, not from here. All timing is simulation time.
"""

from __future__ import annotations

import math
import sys

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robotros_interfaces.msg import BumperState, HubCommand, HubState, PowerState, RangeState, SafetyState
from robotros_interfaces.srv import SetArm, SetRelay
from std_msgs.msg import Bool, Header, UInt8

from .hw_model import SimHardware, load_sim_config


def _load(path: str) -> dict:
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


class SimHwAdapter(Node):
    def __init__(self) -> None:
        super().__init__("sim_hw_adapter")
        bringup = get_package_share_directory("robotros_bringup")
        sim = get_package_share_directory("robotros_sim")
        self.declare_parameter("robot_yaml", f"{bringup}/config/robot.yaml")
        self.declare_parameter("firmware_yaml", f"{bringup}/config/firmware.yaml")
        self.declare_parameter("bridge_yaml", f"{bringup}/config/bridge.yaml")
        self.declare_parameter("sim_hw_yaml", f"{sim}/config/sim_hw.yaml")
        self.declare_parameter("world_sdf", f"{sim}/worlds/generic.sdf")
        gp = self.get_parameter
        sim_cfg = _load(gp("sim_hw_yaml").value)
        with open(gp("world_sdf").value, encoding="utf-8") as f:
            world = f.read()
        robot, firmware, bridge = (_load(gp(name).value) for name in ("robot_yaml", "firmware_yaml", "bridge_yaml"))
        self.cfg = load_sim_config(robot, firmware, bridge, sim_cfg, world)
        n_cliff = len(self.cfg.cliff_polygons)
        self.get_logger().info(f"{n_cliff} cliff area(s) from the world, dock at {self.cfg.dock_xy}")
        self.hw: SimHardware | None = None  # created at the first valid simulation time

        self.pub_cmd = self.create_publisher(Twist, "sim/cmd_vel", 10)
        self.pub_safety = self.create_publisher(SafetyState, "hw/safety", 10)
        self.pub_bumper = self.create_publisher(BumperState, "hw/bumper", 10)
        self.pub_range = self.create_publisher(RangeState, "hw/range", qos_profile_sensor_data)
        self.pub_power = self.create_publisher(PowerState, "hw/power", 10)
        self.pub_hub = self.create_publisher(HubState, "hw/hub_state", 10)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(HubCommand, "hw/cmd/hub", self.on_hub, 10)
        self.create_subscription(Odometry, "sim/ground_truth", self.on_pose, qos_profile_sensor_data)
        self.create_subscription(Bool, "sim/estop", self.on_estop, 10)
        self.create_subscription(UInt8, "sim/bumper", self.on_bumper, 10)
        self.create_service(SetArm, "hw/set_arm", self.on_set_arm)
        self.create_service(SetRelay, "hw/set_relay", self.on_set_relay)
        self.create_timer(float(sim_cfg["step_period_s"]), self.step)

    def now_ms(self) -> int:
        return self.get_clock().now().nanoseconds // 1_000_000

    def ready(self) -> bool:
        if self.hw is None:
            t = self.now_ms()
            if t <= 0:
                return False  # no /clock yet
            self.hw = SimHardware(self.cfg, t)
        return True

    # ------------------------------------------------------------------ inputs
    def on_cmd_vel(self, msg: Twist) -> None:
        if self.ready():
            self.hw.set_cmd_vel(self.now_ms(), msg.linear.x, msg.angular.z)

    def on_hub(self, msg: HubCommand) -> None:
        if self.ready():
            self.hw.set_hub(self.now_ms(), int(msg.command), int(msg.pwm))

    def on_pose(self, msg: Odometry) -> None:
        if self.ready():
            q = msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.hw.set_pose_world(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def on_estop(self, msg: Bool) -> None:
        if self.ready():
            self.hw.set_estop(self.now_ms(), bool(msg.data))

    def on_bumper(self, msg: UInt8) -> None:
        if self.ready():
            self.hw.set_bumper(int(msg.data))

    def on_set_arm(self, request: SetArm.Request, response: SetArm.Response) -> SetArm.Response:
        if not self.ready():
            response.success, response.state = False, SafetyState.BOOT_LOCKED
            return response
        response.success, response.state = self.hw.arm(self.now_ms(), bool(request.arm))
        return response

    def on_set_relay(self, request: SetRelay.Request, response: SetRelay.Response) -> SetRelay.Response:
        if not self.ready():
            response.success, response.message = False, "simulation clock not running"
            return response
        response.success, response.message = self.hw.set_relay(self.now_ms(), int(request.channel), bool(request.on))
        return response

    # ------------------------------------------------------------------ step
    def step(self) -> None:
        if not self.ready():
            return
        self.hw.step(self.now_ms())
        v, w = self.hw.wheel_twist()
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, w
        self.pub_cmd.publish(cmd)
        out = self.hw.take_outputs()
        header = Header(stamp=self.get_clock().now().to_msg(), frame_id="base_link")
        for sv in out.safety:
            self.pub_safety.publish(
                SafetyState(
                    header=header,
                    state=sv.state,
                    estop_pressed=sv.estop_pressed,
                    relay_on=sv.relay_on,
                    block_forward=sv.block_forward,
                    block_backward=sv.block_backward,
                    pi_link_lost=sv.pi_link_lost,
                    software_stop=sv.software_stop,
                    flags=sv.flags,
                )
            )
        for sen in out.sensors:
            mask = int(sen["bump"])
            self.pub_bumper.publish(
                BumperState(header=header, left=bool(mask & 1), middle=bool(mask & 2), right=bool(mask & 4))
            )
            self.pub_range.publish(
                RangeState(
                    header=header,
                    front_mm=int(sen["us_front_mm"]),
                    rear_mm=int(sen["us_rear_mm"]),
                    front_class=int(sen["cls_front"]),
                    rear_class=int(sen["cls_rear"]),
                )
            )
        for pv in out.power:
            self.pub_power.publish(
                PowerState(
                    header=header,
                    v_batt=pv.v_batt,
                    v_charger=pv.v_charger,
                    v_rail=pv.v_rail,
                    i_charge=pv.i_charge,
                    charger_present=pv.charger_present,
                    charge_relay_on=bool(self.hw.nano.relays.get(1)),
                    soc_estimate=-1.0,  # as on the real robot: the bridge does not know the charge state
                )
            )
        for state, top, bottom, fault in out.hub:
            self.pub_hub.publish(HubState(header=header, state=state, limit_top=top, limit_bottom=bottom, fault=fault))
        for evt in out.events:
            self.get_logger().info(f"EVT {evt['type']} arg {evt['arg']}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SimHwAdapter()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()
