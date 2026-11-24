"""Conversion of Nano telegrams to plain values for the /hw/* topics, and config loading (no ROS).

Field meanings: spez_firmware_protokoll.md section 2.3; topic contents: softwarespezifikation.md 5.1.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

from . import protocol as p
from .safety import Flag


@dataclass(frozen=True)
class SafetyValues:
    state: int
    estop_pressed: bool
    relay_on: bool
    block_forward: bool
    block_backward: bool
    pi_link_lost: bool
    software_stop: bool
    flags: int


def safety_from_saf(msg: p.Message, link_lost: bool = False) -> SafetyValues:
    """``$SAF`` to SafetyState values. ``link_lost`` adds the bridge's own view of the serial link."""
    flags = int(msg["flags"])
    return SafetyValues(
        state=int(msg["state"]),
        estop_pressed=bool(flags & Flag.ESTOP_PRESSED),
        relay_on=bool(flags & Flag.RELAY_ON),
        block_forward=bool(flags & Flag.BLOCK_FORWARD),
        block_backward=bool(flags & Flag.BLOCK_BACKWARD),
        pi_link_lost=bool(flags & Flag.PI_LINK_LOST) or link_lost,
        software_stop=bool(flags & Flag.SOFTWARE_STOP),
        flags=flags,
    )


def bumper_from_mask(mask: int) -> tuple[bool, bool, bool]:
    """``bump`` bits (1 left, 2 middle, 4 right) to (left, middle, right)."""
    return bool(mask & 1), bool(mask & 2), bool(mask & 4)


@dataclass(frozen=True)
class PowerValues:
    v_batt: float
    v_charger: float
    v_rail: float
    i_charge: float  # A, positive = charging; NaN if no INA219
    charger_present: bool


def power_from_pwr(msg: p.Message, bridge_cfg: dict) -> PowerValues:
    """``$PWR`` to volts/amps: V = adc_mv/1000 * divider_ratio * gain + offset (spez_firmware_protokoll 9)."""
    ratio = float(bridge_cfg["divider_ratio"])
    gain = bridge_cfg["voltage_gain"]
    offset = bridge_cfg["voltage_offset"]

    def volts(field: str, key: str) -> float:
        return int(msg[field]) / 1000.0 * ratio * float(gain[key]) + float(offset[key])

    v_charger = volts("adc_chg_mv", "chg")
    i_ma = int(msg["i_ma"])
    return PowerValues(
        v_batt=volts("adc_batt_mv", "batt"),
        v_charger=v_charger,
        v_rail=volts("adc_rail_mv", "rail"),
        i_charge=float("nan") if i_ma == -32768 else i_ma / 1000.0,
        charger_present=v_charger >= float(bridge_cfg["charger_present_v"]),
    )


def hub_from_hubst(msg: p.Message) -> tuple[int, bool, bool, int]:
    """``$HUBST`` to (state, limit_top, limit_bottom, fault). State numbers equal HubState.msg."""
    limits = int(msg["limits"])
    return int(msg["state"]), bool(limits & 1), bool(limits & 2), int(msg["fault"])


ROLE_NAMES = {"drive": p.Role.DRIVE, "aux": p.Role.AUX, "all": p.Role.ALL}


@dataclass(frozen=True)
class DeviceConfig:
    name: str
    port: str
    role: p.Role
    baud: int


def load_yaml(path: str | Path) -> dict:
    with open(path, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"{path}: expected a mapping at top level")
    return data


def devices_from_bridge_cfg(cfg: dict) -> list[DeviceConfig]:
    devices = []
    for entry in cfg["devices"]:
        role = ROLE_NAMES[str(entry["role"]).lower()]
        devices.append(DeviceConfig(str(entry["name"]), str(entry["port"]), role, int(entry.get("baud", 115200))))
    names = [d.name for d in devices]
    if len(names) != len(set(names)):
        raise ValueError("device names in bridge.yaml must be unique")
    return devices


def cfg_for_role(firmware_cfg: dict, role: p.Role) -> dict[str, int]:
    """CFG keys for a device: DRIVE gets ``drive``, AUX gets ``aux``, ALL gets both (drive first)."""
    sections = {p.Role.DRIVE: ["drive"], p.Role.AUX: ["aux"], p.Role.ALL: ["drive", "aux"]}[role]
    out: dict[str, int] = {}
    for section in sections:
        for key, value in (firmware_cfg.get(section) or {}).items():
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(f"firmware.yaml {section}.{key}: integer expected, got {value!r}")
            out[str(key)] = value
    return out
