"""Generate firmware/test/vectors/protocol_vectors.json (protocol version 1).

Uses its own XOR checksum (functools.reduce), independent of the protocol
libraries under test (Python bridge, fake_nano, firmware core).

Run: python3 tools/gen_protocol_vectors.py > firmware/test/vectors/protocol_vectors.json
     python3 tools/gen_protocol_vectors.py --text > firmware/test/vectors/protocol_vectors.txt
"""

from __future__ import annotations

import functools
import json
import operator

MAX_FRAME_LEN = 96


def frame(payload: str) -> str:
    """Frame text without line end: ``$payload*HH``."""
    return f"${payload}*{functools.reduce(operator.xor, payload.encode(), 0):02X}"


VALID: list[tuple[str, str, dict]] = [
    ("WHO", "WHO", {}),
    ("PING,7", "PING", {"seq": 7}),
    ("VEL,12,300,-300", "VEL", {"seq": 12, "l_tps": 300, "r_tps": -300}),
    (
        "VEL,4294967295,-2147483648,2147483647",
        "VEL",
        {"seq": 4294967295, "l_tps": -2147483648, "r_tps": 2147483647},
    ),
    ("ARM,1", "ARM", {"arm": 1}),
    ("ARM,0", "ARM", {"arm": 0}),
    ("CLR", "CLR", {}),
    ("CFG,KP,1500", "CFG", {"key": "KP", "value": 1500}),
    ("CFG,US_FLOOR_F,120", "CFG", {"key": "US_FLOOR_F", "value": 120}),
    ("HUB,1,200", "HUB", {"command": 1, "pwm": 200}),
    ("LED,2,3,255,110,0", "LED", {"strip": 2, "pattern": 3, "r": 255, "g": 110, "b": 0}),
    ("BUZ,2", "BUZ", {"pattern": 2}),
    ("REL,1,1", "REL", {"channel": 1, "on": 1}),
    ("HELLO,1,1.0.0,1,7", "HELLO", {"proto": 1, "fw": "1.0.0", "role": 1, "caps": 7}),
    ("PONG,7", "PONG", {"seq": 7}),
    (
        "ODO,123456,-1000,1002,-250,251",
        "ODO",
        {"t_ms": 123456, "l_ticks": -1000, "r_ticks": 1002, "l_tps": -250, "r_tps": 251},
    ),
    (
        "SEN,5000,65535,118,2,1,5",
        "SEN",
        {"t_ms": 5000, "us_front_mm": 65535, "us_rear_mm": 118, "cls_front": 2, "cls_rear": 1, "bump": 5},
    ),
    ("SAF,4294967295,3,258", "SAF", {"t_ms": 4294967295, "state": 3, "flags": 258}),
    (
        "PWR,100,2512,0,2490,-32768",
        "PWR",
        {"t_ms": 100, "adc_batt_mv": 2512, "adc_chg_mv": 0, "adc_rail_mv": 2490, "i_ma": -32768},
    ),
    ("HUBST,900,3,1,0", "HUBST", {"t_ms": 900, "state": 3, "limits": 1, "fault": 0}),
    ("EVT,42,30,5", "EVT", {"t_ms": 42, "type": 30, "arg": 5}),
    ("ACK,ARM", "ACK", {"command": "ARM"}),
    ("ACK,CFG,KP", "ACK", {"command": "CFG", "arg": "KP"}),
    ("ERR,1,VEL", "ERR", {"code": 1, "detail": "VEL"}),
    ("ERR,6", "ERR", {"code": 6}),
]


def build() -> dict:
    out: dict = {
        "_comment": (
            "Golden vectors for protocol version 1 (spez_firmware_protokoll.md section 2). "
            "Shared by the Python bridge, fake_nano and the firmware host tests. Checksums were "
            "computed with an implementation independent of the libraries under test."
        ),
        "proto_ver": 1,
        "max_frame_len": MAX_FRAME_LEN,
        "valid": [{"frame": frame(p), "name": n, "fields": f} for p, n, f in VALID],
        "extra_fields": [
            {"frame": frame("PING,7,99,abc"), "name": "PING", "fields": {"seq": 7}, "extra": ["99", "abc"]},
            {
                "frame": frame("HELLO,1,1.0.0,2,255,9"),
                "name": "HELLO",
                "fields": {"proto": 1, "fw": "1.0.0", "role": 2, "caps": 255},
                "extra": ["9"],
            },
        ],
        "invalid": [],
    }
    good = frame("VEL,12,300,-300")  # checksum 5D
    invalid = [
        (good[:-2] + "00", 1, "wrong checksum"),
        (good[:-2] + good[-2:].lower(), 1, "lower-case checksum"),
        ("$VEL,12,300,-300", 1, "no checksum"),
        (frame("FOO,1"), 2, "unknown command"),
        (frame("VEL,12,300"), 3, "missing field"),
        (frame("VEL,12,3.5,0"), 3, "float is not allowed"),
        (frame("VEL,12,+3,0"), 3, "plus sign is not allowed"),
        (frame("ARM,2"), 3, "value out of range"),
        (frame("VEL,1,2147483648,0"), 3, "int32 overflow"),
        (frame("CFG,,5"), 3, "empty string field"),
        (frame("PING,1 "), 3, "space in payload"),
        ("$PING," + "1" * 88 + "*00", 6, "frame longer than 96 bytes including line end"),
    ]
    out["invalid"] = [{"frame": fr, "error_code": c, "why": why} for fr, c, why in invalid]

    # Boundary: a frame of exactly MAX_FRAME_LEN bytes including '\n' is valid, one byte more is not.
    key = "K"
    while len(frame(f"CFG,{key},1")) + 1 < MAX_FRAME_LEN:
        key += "K"
    assert len(frame(f"CFG,{key},1")) + 1 == MAX_FRAME_LEN
    out["valid"].append(
        {
            "frame": frame(f"CFG,{key},1"),
            "name": "CFG",
            "fields": {"key": key, "value": 1},
            "why": "exactly 96 bytes including line end",
        }
    )
    too_long = frame(f"CFG,{key}K,1")
    assert len(too_long) + 1 == MAX_FRAME_LEN + 1
    out["invalid"].append({"frame": too_long, "error_code": 6, "why": "97 bytes including line end, correct checksum"})
    return out


def to_text(data: dict) -> str:
    """Line format for the C++ host tests (no JSON parser on the firmware side).

    V<TAB>frame<TAB>NAME<TAB>field...   valid frame, fields in schema order
    X<TAB>frame<TAB>NAME<TAB>field...   valid frame with extra fields; the listed fields must be decoded
    I<TAB>frame<TAB>error_code          invalid frame
    """
    lines = [f"# Generated by tools/gen_protocol_vectors.py --text, protocol version {data['proto_ver']}. Do not edit."]
    for kind, key in (("V", "valid"), ("X", "extra_fields")):
        for vec in data[key]:
            lines.append("\t".join([kind, vec["frame"], vec["name"], *(str(v) for v in vec["fields"].values())]))
    for vec in data["invalid"]:
        lines.append("\t".join(["I", vec["frame"], str(vec["error_code"])]))
    return "\n".join(lines) + "\n"


if __name__ == "__main__":
    import sys

    if "--text" in sys.argv[1:]:
        sys.stdout.write(to_text(build()))
    else:
        print(json.dumps(build(), indent=2))
