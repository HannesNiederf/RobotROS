"""activity_view: shows /activity in a terminal (ADR 0004).

  activity_view now  [--lang de|en|it] [--i18n DIR]   running log: one line per change of what the robot does
  activity_view plan [--lang de|en|it] [--i18n DIR]   what the robot intends, redrawn in place
Language: --lang, else from $LANG (de/it), else English (spec 13.4). Texts: web_ui/i18n/<lang>.json.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from robotros_interfaces.msg import ActivityState

from .core.activity import args_from_list, render

NUMBER = re.compile(r"^-?\d+\.\d+$")


def pick_lang(arg: str | None) -> str:
    if arg:
        return arg
    env = os.environ.get("LANG", "")[:2].lower()
    return env if env in ("de", "it") else "en"


def load_texts(i18n_dir: str | None, lang: str) -> dict[str, str]:
    if not i18n_dir:
        return {}
    path = Path(i18n_dir) / f"{lang}.json"
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def text(texts: dict[str, str], key: str, items: list[str], lang: str) -> str:
    args = args_from_list(items)
    if lang in ("de", "it"):  # decimal comma
        args = {k: v.replace(".", ",") if NUMBER.match(v) else v for k, v in args.items()}
    return render(texts.get(key, key), args)  # unknown key: show the key itself (spec 13.4)


class ActivityView(Node):
    def __init__(self, mode: str, lang: str, texts: dict[str, str]) -> None:
        super().__init__("activity_view_" + mode)
        self.mode, self.lang, self.texts = mode, lang, texts
        self.last = ""
        self.create_subscription(ActivityState, "activity", self.on_activity, 10)
        title = texts.get("activity.title" if mode == "now" else "plan.title", mode)
        print(f"== {title} ==", flush=True)

    def on_activity(self, msg: ActivityState) -> None:
        sim_t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if self.mode == "now":
            line = text(self.texts, msg.now_key, list(msg.now_args), self.lang)
            # A new line only when the activity itself changes, not for every speed digit.
            if msg.now_key != self.last:
                self.last = msg.now_key
                print(f"[{sim_t:8.1f} s] {line}", flush=True)
            else:
                print(f"\r[{sim_t:8.1f} s] {line}\033[K", end="", flush=True)
        else:
            line = text(self.texts, msg.plan_key, list(msg.plan_args), self.lang)
            if line != self.last:
                self.last = line
                print(f"\r{line}\033[K", end="", flush=True)


def main(argv: list[str] | None = None) -> None:
    argv = sys.argv[1:] if argv is None else argv
    ap = argparse.ArgumentParser(prog="activity_view")
    ap.add_argument("mode", choices=["now", "plan"])
    ap.add_argument("--lang", choices=["de", "en", "it"])
    ap.add_argument("--i18n", default=os.environ.get("ROBOTROS_I18N"))
    args, ros_args = ap.parse_known_args(argv)
    lang = pick_lang(args.lang)
    rclpy.init(args=ros_args)
    node = ActivityView(args.mode, lang, load_texts(args.i18n, lang))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        print(flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
