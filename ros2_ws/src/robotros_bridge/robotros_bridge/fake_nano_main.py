"""Run a FakeNano behind a pseudo-terminal (pty) so a client can open it like a serial port.

Usage: ``ros2 run robotros_bridge fake_nano --role drive [--link /tmp/robotros_fake_drive]``
or without ROS: ``python3 -m robotros_bridge.fake_nano_main --role drive``.
Prints the pty path on stdout. Uses only the monotonic clock.
"""

from __future__ import annotations

import argparse
import os
import select
import sys
import termios
import time
import tty

from .core import protocol as p
from .core.fake_nano import FakeNano

ROLES = {"drive": p.Role.DRIVE, "aux": p.Role.AUX, "all": p.Role.ALL}


def now_ms() -> int:
    return time.monotonic_ns() // 1_000_000


def open_pty() -> tuple[int, str]:
    master, slave = os.openpty()
    tty.setraw(slave)  # no echo, no line editing: bytes pass unchanged
    attrs = termios.tcgetattr(slave)
    attrs[4] = attrs[5] = termios.B115200  # ispeed, ospeed (informational on a pty)
    termios.tcsetattr(slave, termios.TCSANOW, attrs)
    os.set_blocking(master, False)  # never block when no client reads: drop data like a real UART
    return master, os.ttyname(slave)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--role", choices=sorted(ROLES), default="drive")
    parser.add_argument("--link", help="create a symlink to the pty at this path")
    parser.add_argument("--estop", action="store_true", help="start with the E-stop pressed")
    parser.add_argument("--period-ms", type=int, default=5, help="simulation step")
    args = parser.parse_args(argv)

    master, path = open_pty()
    if args.link:
        if os.path.lexists(args.link):
            os.unlink(args.link)
        os.symlink(path, args.link)
    print(path, flush=True)

    nano = FakeNano(ROLES[args.role], log=lambda msg: print(msg, file=sys.stderr, flush=True))
    nano.power_on(now_ms(), estop_pressed=args.estop)
    last = now_ms()
    try:
        while True:
            readable, _, _ = select.select([master], [], [], args.period_ms / 1000.0)
            t = now_ms()
            if t - last > 100:  # diagnostics: the simulated Nano itself was not scheduled
                print(f"fake_nano {args.role}: loop stalled for {t - last} ms", file=sys.stderr, flush=True)
            last = t
            if readable:
                try:
                    data = os.read(master, 1024)
                except (BlockingIOError, OSError):
                    data = b""
                if data:
                    nano.feed(t, data)
            nano.step(t)
            out = nano.read_output()
            if out:
                try:
                    os.write(master, out)
                except (BlockingIOError, OSError):
                    pass  # buffer full or no reader
    except KeyboardInterrupt:
        return 0
    finally:
        if args.link and os.path.islink(args.link):
            os.unlink(args.link)


if __name__ == "__main__":
    sys.exit(main())
