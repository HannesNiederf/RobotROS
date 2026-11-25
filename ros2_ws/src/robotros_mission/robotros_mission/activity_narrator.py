"""activity_narrator: publishes what the robot does now and intends (/activity, ADR 0004)."""

from __future__ import annotations

import math

import rclpy
import yaml
from action_msgs.msg import GoalStatusArray
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import BehaviorTreeLog, CollisionMonitorState
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from robotros_interfaces.msg import ActivityState, SafetyState

from .core.activity import ActivityConfig, ActivityTracker, args_to_list


class ActivityNarrator(Node):
    def __init__(self) -> None:
        super().__init__("activity_narrator")
        share = get_package_share_directory("robotros_bringup")
        self.declare_parameter("speed_profiles_yaml", f"{share}/config/speed_profiles.yaml")
        self.declare_parameter("moving_speed", 0.02)
        self.declare_parameter("turning_rate", 0.05)
        self.declare_parameter("slow_fraction", 0.6)
        self.declare_parameter("result_hold_s", 10.0)
        self.declare_parameter("wait_s", 4.0)
        self.declare_parameter("retries", 3)
        self.declare_parameter("publish_period_s", 0.5)
        with open(self.get_parameter("speed_profiles_yaml").value, encoding="utf-8") as f:
            nominal = float(yaml.safe_load(f)["NORMAL"]["v"])
        p = self.get_parameter
        self.tracker = ActivityTracker(ActivityConfig(
            moving_speed=p("moving_speed").value, turning_rate=p("turning_rate").value,
            slow_fraction=p("slow_fraction").value, nominal_speed=nominal,
            result_hold_s=p("result_hold_s").value, wait_s=p("wait_s").value, retries=p("retries").value))
        self.pub = self.create_publisher(ActivityState, "activity", 10)
        self.create_subscription(SafetyState, "hw/safety", self.on_safety, 10)
        self.create_subscription(GoalStatusArray, "navigate_to_pose/_action/status", self.on_status, 10)
        self.create_subscription(NavigateToPose.Impl.FeedbackMessage, "navigate_to_pose/_action/feedback",
                                 self.on_feedback, 10)
        self.create_subscription(BehaviorTreeLog, "behavior_tree_log", self.on_bt, 50)
        self.create_subscription(Odometry, "odom", self.on_odom, 10)
        self.create_subscription(CollisionMonitorState, "collision_monitor_state", self.on_cm, 10)
        self.create_subscription(Path, "plan", self.on_plan, 10)
        self.create_timer(p("publish_period_s").value, self.publish)

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def on_safety(self, msg: SafetyState) -> None:
        self.tracker.on_safety(msg.state, msg.estop_pressed)

    def on_status(self, msg: GoalStatusArray) -> None:
        if msg.status_list:
            latest = max(msg.status_list, key=lambda s: (s.goal_info.stamp.sec, s.goal_info.stamp.nanosec))
            self.tracker.on_nav_status(latest.status, self.now_s())
            self.publish()

    def on_feedback(self, msg: NavigateToPose.Impl.FeedbackMessage) -> None:
        fb = msg.feedback
        eta = fb.estimated_time_remaining.sec + fb.estimated_time_remaining.nanosec / 1e9
        self.tracker.on_feedback(fb.distance_remaining, eta, fb.number_of_recoveries)

    def on_bt(self, msg: BehaviorTreeLog) -> None:
        for ev in msg.event_log:
            self.tracker.on_bt(ev.node_name, ev.current_status)
        self.publish()

    def on_odom(self, msg: Odometry) -> None:
        self.tracker.on_motion(msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def on_cm(self, msg: CollisionMonitorState) -> None:
        self.tracker.on_collision_monitor(msg.action_type)

    def on_plan(self, msg: Path) -> None:
        if msg.poses:
            p = msg.poses[-1].pose.position
            if math.isfinite(p.x) and math.isfinite(p.y):
                self.tracker.on_plan_goal(p.x, p.y)

    def publish(self) -> None:
        t = self.now_s()
        now, plan = self.tracker.now(t), self.tracker.plan(t)
        out = ActivityState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.now_key, out.now_args = now.key, args_to_list(now.args)
        out.plan_key, out.plan_args = plan.key, args_to_list(plan.args)
        self.pub.publish(out)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ActivityNarrator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
