"""End-to-end test: nano_bridge (real ROS node, real pyserial) against two fake_nano processes on ptys.

Needs ROS 2 (rclpy, robotros_interfaces) and pyserial; skipped otherwise. Runs in CI via colcon test.
"""

from __future__ import annotations

import os
import signal
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
from robotros_interfaces.msg import PowerState, SafetyState  # noqa: E402
from robotros_interfaces.srv import SetArm, SetRelay  # noqa: E402

PKG_ROOT = Path(__file__).resolve().parents[1]

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

    # The bridge runs in its own process, as on the robot: test code and probe node do not share its
    # interpreter (GIL), so loop stalls seen here are the bridge's own.
    bridge_log = tmp_path / "nano_bridge.log"
    log_file = bridge_log.open("w")
    bridge = subprocess.Popen(
        [
            sys.executable, "-m", "robotros_bridge.nano_bridge", "--ros-args",
            "-p", f"bridge_yaml:={bridge_yaml}",
            "-p", f"robot_yaml:={CONFIG / 'robot.yaml'}",
            "-p", f"firmware_yaml:={CONFIG / 'firmware.yaml'}",
        ],
        stdout=log_file,
        stderr=subprocess.STDOUT,
        env=env,
    )
    procs.insert(0, bridge)

    rclpy.init()
    probe = rclpy.create_node("bridge_probe")
    seen: dict[str, list] = {"safety": [], "odom": [], "power": []}
    probe.create_subscription(SafetyState, "hw/safety", seen["safety"].append, 10)
    probe.create_subscription(Odometry, "odom", seen["odom"].append, rclpy.qos.qos_profile_sensor_data)
    probe.create_subscription(PowerState, "hw/power", seen["power"].append, 10)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(probe)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    try:
        yield probe, seen
    finally:
        executor.shutdown()
        probe.destroy_node()
        rclpy.try_shutdown()
        bridge.send_signal(signal.SIGINT)
        try:
            bridge.wait(timeout=10)
        except subprocess.TimeoutExpired:
            bridge.kill()
        for proc in procs[1:]:
            proc.terminate()
            proc.wait(timeout=5)
        log_file.close()
        lines = bridge_log.read_text().splitlines()
        notes = [ln for ln in lines if any(k in ln for k in ("stalled", "longest gap", "link lost", "EVT 5", "Error"))]
        print("\n".join(["---- nano_bridge (selected lines) ----", *notes, "---- nano_bridge (last 15 lines) ----",
                         *lines[-15:]]), flush=True)


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
    # At least 2 s of commands, longer (up to 6 s) until 0.04 m are covered: on a loaded CI runner the robot covered
    # only 0.033 m in 2 s once (26.11.2026). The speed checks below stay strict.
    start = time.monotonic()
    while time.monotonic() - start < 6.0:
        pub.publish(cmd)
        time.sleep(0.05)
        moved = seen["odom"] and seen["odom"][-1].pose.pose.position.x > 0.04
        if time.monotonic() - start >= 2.0 and moved:
            break
    last = seen["odom"][-1]
    assert last.pose.pose.position.x > 0.04
    recent = [m.twist.twist.linear.x for m in seen["odom"][-25:]]
    mean = sum(recent) / len(recent)
    assert 0.03 < mean <= 0.052, recent  # D-24 cap 0.05 m/s; one tick/s is about 0.00035 m/s
    assert max(recent) <= 0.052, recent

    # cmd_vel timeout 0.5 s -> zero setpoints; then the wheel lag and the measured-rate window. A fixed 1 s wait
    # failed once on a loaded CI runner (0.0064 m/s left, 25.11.2026), so wait up to 2 s for the standstill.
    assert wait_until(lambda: abs(seen["odom"][-1].twist.twist.linear.x) < 0.005, 2.0), seen["odom"][-1].twist

    res = call(probe, SetRelay, "hw/set_relay", SetRelay.Request(channel=1, on=True))
    assert res.success, res.message

    res = call(probe, SetArm, "hw/set_arm", SetArm.Request(arm=False))
    assert res.success and res.state == SafetyState.WAIT_ACK
    assert wait_until(lambda: seen["safety"][-1].software_stop, 2.0)
