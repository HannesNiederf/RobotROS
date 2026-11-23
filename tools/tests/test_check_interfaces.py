"""Tests for tools/check_interfaces.py (runs without ROS)."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import check_interfaces as ci  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[2]


def make_package(tmp_path: Path, files: dict[str, str], listed: list[str] | None = None) -> Path:
    """Create a fake interface package and return its directory."""
    package = tmp_path / "demo_interfaces"
    package.mkdir()
    (package / "package.xml").write_text("<package/>")
    for relative, content in files.items():
        target = package / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(content)
    names = listed if listed is not None else list(files)
    cmake_items = "\n".join(f'  "{n}"' for n in names)
    (package / "CMakeLists.txt").write_text(f"rosidl_generate_interfaces(demo\n{cmake_items}\n)\n")
    return package


def problems(package: Path) -> list[str]:
    errors, _ = ci.check_package(package)
    return errors


def test_valid_package_has_no_problems(tmp_path):
    package = make_package(tmp_path, {
        "msg/Thing.msg": "uint8 A=1\nstd_msgs/Header header\nuint8 state  # comment\nstring[] options\n",
        "srv/Do.srv": "Thing thing\n---\nbool success\n",
        "action/Run.action": "uint32 id\n---\nbool ok\n---\nfloat32 progress\n",
    })
    assert problems(package) == []


def test_string_constant_with_hash_inside_quotes_is_not_a_comment(tmp_path):
    package = make_package(tmp_path, {"msg/Thing.msg": 'string TAG="a#b"\nuint8 x\n'})
    assert problems(package) == []


@pytest.mark.parametrize("line", [
    "uint8 BadName",     # field in CamelCase
    "uint8 trailing_",   # trailing underscore
    "uint8 double__us",  # double underscore
    "uint8 1abc",        # leading digit
])
def test_bad_field_names_are_reported(tmp_path, line):
    package = make_package(tmp_path, {"msg/Thing.msg": line + "\n"})
    assert any("lower_snake_case" in e for e in problems(package))


def test_lower_case_constant_is_reported(tmp_path):
    package = make_package(tmp_path, {"msg/Thing.msg": "uint8 lower=1\n"})
    assert any("UPPER_CASE" in e for e in problems(package))


def test_unknown_local_type_is_reported(tmp_path):
    package = make_package(tmp_path, {"msg/Thing.msg": "Missing thing\n"})
    assert any("unknown type 'Missing'" in e for e in problems(package))


def test_unknown_package_is_reported(tmp_path):
    package = make_package(tmp_path, {"msg/Thing.msg": "geometry_msg/Pose2D pose\n"})
    assert any("not in ALLOWED_PACKAGES" in e for e in problems(package))


def test_wrong_separator_count_is_reported(tmp_path):
    package = make_package(tmp_path, {
        "srv/Do.srv": "bool a\n",                      # needs one separator
        "action/Run.action": "bool a\n---\nbool b\n",  # needs two
        "msg/Thing.msg": "bool a\n---\n",              # needs none
    })
    errors = problems(package)
    assert sum("separator" in e for e in errors) == 3


def test_duplicate_name_in_one_section_is_reported(tmp_path):
    package = make_package(tmp_path, {"msg/Thing.msg": "uint8 a\nuint8 a\n"})
    assert any("duplicate name 'a'" in e for e in problems(package))


def test_same_name_in_request_and_response_is_fine(tmp_path):
    package = make_package(tmp_path, {"srv/Do.srv": "uint8 a\n---\nuint8 a\n"})
    assert problems(package) == []


def test_file_missing_from_cmake_is_reported(tmp_path):
    package = make_package(tmp_path, {"msg/Thing.msg": "uint8 a\n"}, listed=[])
    assert any("not listed in CMakeLists.txt" in e for e in problems(package))


def test_stale_cmake_entry_is_reported(tmp_path):
    package = make_package(tmp_path, {"msg/Thing.msg": "uint8 a\n"},
                           listed=["msg/Thing.msg", "msg/Gone.msg"])
    assert any("does not exist" in e for e in problems(package))


def test_repository_interfaces_are_clean():
    errors, count = ci.check_package(REPO_ROOT / "ros2_ws" / "src" / "robotros_interfaces")
    assert errors == []
    # 19 messages + 18 services + 5 actions (softwarespezifikation.md section 5, ÄM-01)
    assert count == 42
