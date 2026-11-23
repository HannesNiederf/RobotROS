#!/usr/bin/env python3
"""Lightweight syntax check for ROS 2 interface files (.msg, .srv, .action).

This is NOT a replacement for rosidl: the authoritative check is `colcon build`
in the CI. The script catches typos early (names, separators, unknown types)
without needing a ROS installation.

Usage: check_interfaces.py [ros2_ws/src]
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

PRIMITIVES = {
    "bool", "byte", "char", "float32", "float64",
    "int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64",
    "string", "wstring",
}
# Packages that interface files in this repository may reference.
ALLOWED_PACKAGES = {"std_msgs", "geometry_msgs", "sensor_msgs", "builtin_interfaces"}
# Number of '---' separators per file kind.
SEPARATORS = {".msg": 0, ".srv": 1, ".action": 2}

FIELD_NAME = re.compile(r"^[a-z](?:_?[a-z0-9])*$")
CONSTANT_NAME = re.compile(r"^[A-Z](?:_?[A-Z0-9])*$")
TYPE_FILE_NAME = re.compile(r"^[A-Z][A-Za-z0-9]*$")
TYPE_REF = re.compile(
    r"^(?:(?P<pkg>[a-z][a-z0-9_]*)/)?(?P<name>[A-Za-z][A-Za-z0-9]*)"
    r"(?P<array>\[(?:<=)?\d*\])?$"
)
CONSTANT_LINE = re.compile(r"^(\S+)\s+([A-Za-z0-9_]+)\s*=\s*(.+)$")
FIELD_LINE = re.compile(r"^(\S+)\s+(\S+)(?:\s+(.+))?$")


def strip_comment(line: str) -> str:
    """Remove a trailing '#' comment, ignoring '#' inside quotes."""
    quote = ""
    for i, ch in enumerate(line):
        if quote:
            if ch == quote:
                quote = ""
        elif ch in "\"'":
            quote = ch
        elif ch == "#":
            return line[:i].strip()
    return line.strip()


def check_file(path: Path, package: str, local_messages: set[str]) -> list[str]:
    errors: list[str] = []

    def err(lineno: int, text: str) -> None:
        errors.append(f"{path}:{lineno}: {text}")

    if not TYPE_FILE_NAME.match(path.stem):
        err(0, f"file name '{path.stem}' is not CamelCase")

    separators = 0
    names_in_section: set[str] = set()
    for lineno, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = strip_comment(raw)
        if not line:
            continue
        if line == "---":
            separators += 1
            names_in_section = set()
            continue

        constant = CONSTANT_LINE.match(line)
        if constant:
            type_name, name = constant.group(1), constant.group(2)
            if not CONSTANT_NAME.match(name):
                err(lineno, f"constant '{name}' must be UPPER_CASE")
            if type_name not in PRIMITIVES:
                err(lineno, f"constant '{name}' has non-primitive type '{type_name}'")
        else:
            field = FIELD_LINE.match(line)
            if not field:
                err(lineno, f"cannot parse line: {line!r}")
                continue
            type_name, name = field.group(1), field.group(2)
            if not FIELD_NAME.match(name):
                err(lineno, f"field '{name}' must be lower_snake_case "
                            "(no leading digit, no trailing or double underscore)")
            ref = TYPE_REF.match(type_name)
            if not ref:
                err(lineno, f"cannot parse type '{type_name}'")
            else:
                base, pkg = ref.group("name"), ref.group("pkg")
                if pkg is None and base not in PRIMITIVES and base not in local_messages:
                    err(lineno, f"unknown type '{base}' (not a primitive, no such message "
                                f"in {package}); use 'package/Type' for other packages")
                elif pkg is not None and pkg != package and pkg not in ALLOWED_PACKAGES:
                    err(lineno, f"package '{pkg}' is not in ALLOWED_PACKAGES "
                                "(add it here and to package.xml/CMakeLists.txt if intended)")
                elif pkg == package and base not in local_messages:
                    err(lineno, f"unknown message '{pkg}/{base}'")
        if name in names_in_section:
            err(lineno, f"duplicate name '{name}' in this section")
        names_in_section.add(name)

    expected = SEPARATORS[path.suffix]
    if separators != expected:
        err(0, f"expected {expected} '---' separator(s) in a {path.suffix} file, found {separators}")
    return errors


def check_package(package_dir: Path) -> tuple[list[str], int]:
    package = package_dir.name
    local_messages = {p.stem for p in (package_dir / "msg").glob("*.msg")}
    errors: list[str] = []
    count = 0
    for kind, suffix in (("msg", ".msg"), ("srv", ".srv"), ("action", ".action")):
        for path in sorted((package_dir / kind).glob(f"*{suffix}")):
            count += 1
            errors.extend(check_file(path, package, local_messages))

    # Every interface file must be listed in CMakeLists.txt, and vice versa.
    cmake = package_dir / "CMakeLists.txt"
    if cmake.exists():
        text = cmake.read_text(encoding="utf-8")
        listed = set(re.findall(r'"((?:msg|srv|action)/[A-Za-z0-9]+\.(?:msg|srv|action))"', text))
        on_disk = {
            f"{kind}/{p.name}"
            for kind, suffix in (("msg", ".msg"), ("srv", ".srv"), ("action", ".action"))
            for p in (package_dir / kind).glob(f"*{suffix}")
        }
        for missing in sorted(on_disk - listed):
            errors.append(f"{cmake}: '{missing}' exists but is not listed in CMakeLists.txt")
        for stale in sorted(listed - on_disk):
            errors.append(f"{cmake}: '{stale}' is listed but the file does not exist")
    return errors, count


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("src", nargs="?", default="ros2_ws/src", type=Path)
    args = parser.parse_args()

    all_errors: list[str] = []
    total = 0
    for package_dir in sorted(p for p in args.src.iterdir() if (p / "package.xml").exists()):
        if not any((package_dir / k).is_dir() for k in ("msg", "srv", "action")):
            continue
        errors, count = check_package(package_dir)
        total += count
        all_errors.extend(errors)

    for message in all_errors:
        print(message)
    print(f"checked {total} interface file(s), {len(all_errors)} problem(s)")
    return 1 if all_errors else 0


if __name__ == "__main__":
    sys.exit(main())
