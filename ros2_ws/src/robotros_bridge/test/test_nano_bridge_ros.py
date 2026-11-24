"""End-to-end test: nano_bridge (real ROS node, real pyserial) against two fake_nano processes on ptys.

Needs ROS 2 (rclpy, robotros_interfaces) and pyserial; skipped otherwise. Runs in CI via colcon test.
"""

from __future__ import annotations

import os
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest
import yaml

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("serial")
pytest.importorskip("robotros_interfaces")

from geometry_msgs.msg import Twist  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from robotros_interfaces.msg import PowerState, SafetyState  # noqa: E402
from robotros_interfaces.srv import SetArm, SetRelay  # noqa: E402

PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_ROOT))
from robotros_bridge.nano_bridge import NanoBridge  # noqa: E402

CONFIG = PKG_ROOT.parent / "robotros_bringup" / "config"


def wait_until(predicate, timeout: float) -> bool:  # noqa: ANN001
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return predicate()


@pytest.fixture()
def system(tmp_path: Path):  # noqa: ANN201
    env = dict(os.environ, PYTHONPATH=str(PKG_ROOT) + os.pathsep + os.environ.get("PYTHONPATH", ""))
    procs = []
    links = {}
    for role in ("drive", "aux"):
        link = tmp_path / f"fake_{role}"
        proc = subprocess.Popen(
            [sys.executable, "-m", "robotros_bridge.fake_nano_main", "--role", role, "--link", str(link)],
            stdout=subprocess.PIPE,
            text=True,
            env=env,
        )
        procs.append(proc)
        proc.stdout.readline()  # pty path printed once the link exists
        links[role] = str(link)
    bridge_cfg = yaml.safe_load((CONFIG / "bridge.yaml").read_text())
    for dev in bridge_cfg["devices"]:
        dev["port"] = links[dev["name"]]
    bridge_yaml = tmp_path / "bridge.yaml"
    bridge_yaml.write_text(yaml.safe_dump(bridge_cfg))

    rclpy.init()
    node = NanoBridge(
        parameter_overrides=[
            Parameter("bridge_yaml", value=str(bridge_yaml)),
            Parameter("robot_yaml", value=str(CONFIG / "robot.yaml")),
            Parameter("firmware_yaml", value=str(CONFIG / "firmware.yaml")),
        ]
    )
    probe = rclpy.create_node("bridge_probe")
    seen: dict[str, list] = {"safety": [], "odom": [], "power": []}
    probe.create_subscription(SafetyState, "hw/safety", seen["safety"].append, 10)
    probe.create_subscription(Odometry, "odom", seen["odom"].append, rclpy.qos.qos_profile_sensor_data)
    probe.create_subscription(PowerState, "hw/power", seen["power"].append, 10)
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    executor.add_node(probe)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        yield probe, seen
    finally:
        print(f"nano_bridge serial loop: longest gap {node.io_gap_max_ms} ms", flush=True)
        executor.shutdown()
        node.destroy_node()
        probe.destroy_node()
        rclpy.try_shutdown()
        for proc in procs:
            proc.terminate()
            proc.wait(timeout=5)


def call(probe, srv_type, name: str, request, timeout: float = 5.0):  # noqa: ANN001, ANN201
    client = probe.create_client(srv_type, name)
    assert client.wait_for_service(timeout_sec=timeout)
    future = client.call_async(request)
    assert wait_until(future.done, timeout)
    return future.result()


def test_bridge_against_fake_nanos(system) -> None:  # noqa: ANN001
    probe, seen = system
    assert wait_until(lambda: any(m.state == SafetyState.BOOT_LOCKED for m in seen["safety"]), 10.0)
    assert wait_until(lambda: len(seen["power"]) >= 1, 5.0)

    res = call(probe, SetArm, "hw/set_arm", SetArm.Request(arm=True))
    assert res.success and res.state == SafetyState.ARMED

    pub = probe.create_publisher(Twist, "cmd_vel", 10)
    cmd = Twist()
    cmd.linear.x = 0.2  # above the D-24 limit: the bridge must cap it at 0.05 m/s
    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        pub.publish(cmd)
        time.sleep(0.05)
    last = seen["odom"][-1]
    assert last.pose.pose.position.x > 0.04
    recent = [m.twist.twist.linear.x for m in seen["odom"][-25:]]
    mean = sum(recent) / len(recent)
    assert 0.03 < mean <= 0.052, recent  # D-24 cap 0.05 m/s; one tick/s is about 0.00035 m/s
    assert max(recent) <= 0.052, recent

    time.sleep(1.0)  # cmd_vel timeout 0.5 s -> zero setpoints
    assert abs(seen["odom"][-1].twist.twist.linear.x) < 0.005

    res = call(probe, SetRelay, "hw/set_relay", SetRelay.Request(channel=1, on=True))
    assert res.success, res.message

    res = call(probe, SetArm, "hw/set_arm", SetArm.Request(arm=False))
    assert res.success and res.state == SafetyState.WAIT_ACK
    assert wait_until(lambda: seen["safety"][-1].software_stop, 2.0)
