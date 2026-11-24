"""Tests of robotros_bridge.core.protocol against the shared golden vectors and edge cases."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_ROOT))

from robotros_bridge.core import protocol as p  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[4]
VECTORS = json.loads((REPO_ROOT / "firmware" / "test" / "vectors" / "protocol_vectors.json").read_text())


def test_vector_header() -> None:
    assert VECTORS["proto_ver"] == p.PROTO_VER
    assert VECTORS["max_frame_len"] == p.MAX_FRAME_LEN


@pytest.mark.parametrize("vec", VECTORS["valid"], ids=lambda v: v["frame"][:30])
def test_valid_vectors_decode(vec: dict) -> None:
    msg = p.decode_frame(vec["frame"])
    assert msg.name == vec["name"]
    assert msg.fields == vec["fields"]
    assert msg.extra == ()


@pytest.mark.parametrize("vec", VECTORS["valid"], ids=lambda v: v["frame"][:30])
def test_valid_vectors_encode(vec: dict) -> None:
    assert p.encode(vec["name"], **vec["fields"]) == (vec["frame"] + "\n").encode()


@pytest.mark.parametrize("vec", VECTORS["extra_fields"], ids=lambda v: v["frame"])
def test_extra_fields_are_ignored(vec: dict) -> None:
    msg = p.decode_frame(vec["frame"])
    assert msg.name == vec["name"]
    assert msg.fields == vec["fields"]
    assert list(msg.extra) == vec["extra"]


@pytest.mark.parametrize("vec", VECTORS["invalid"], ids=lambda v: v["why"])
def test_invalid_vectors(vec: dict) -> None:
    with pytest.raises(p.ProtocolError) as info:
        p.decode_frame(vec["frame"])
    assert info.value.code == vec["error_code"]


@pytest.mark.parametrize("vec", VECTORS["invalid"], ids=lambda v: v["why"])
def test_invalid_vectors_through_stream(vec: dict) -> None:
    results = list(p.StreamParser().feed((vec["frame"] + "\n").encode()))
    assert len(results) == 1
    assert isinstance(results[0], p.ProtocolError)
    assert results[0].code == vec["error_code"]


def test_stream_all_valid_vectors_in_one_chunk_with_crlf_and_noise() -> None:
    data = b"garbage before first frame"
    for vec in VECTORS["valid"]:
        data += vec["frame"].encode() + b"\r\n"
    results = list(p.StreamParser().feed(data))
    assert [r.name for r in results] == [v["name"] for v in VECTORS["valid"]]


def test_stream_byte_by_byte() -> None:
    parser = p.StreamParser()
    out: list[p.ParseResult] = []
    for byte in b"$PING,7*0B\n$WHO*50\n":
        out.extend(parser.feed(bytes([byte])))
    assert [m.name for m in out] == ["PING", "WHO"]


def test_dollar_restarts_frame() -> None:
    out = list(p.StreamParser().feed(b"$VEL,1,2$PING,7*0B\n"))
    assert len(out) == 1 and out[0].name == "PING"


def test_overflow_then_recovery() -> None:
    data = b"$" + b"A" * 200 + b"\n$PING,7*0B\n"
    out = list(p.StreamParser().feed(data))
    assert isinstance(out[0], p.ProtocolError) and out[0].code == p.ErrCode.LINE_TOO_LONG
    assert out[1].name == "PING"


def test_frame_of_exactly_96_bytes_passes_stream() -> None:
    vec = next(v for v in VECTORS["valid"] if v.get("why", "").startswith("exactly 96"))
    line = (vec["frame"] + "\n").encode()
    assert len(line) == 96
    out = list(p.StreamParser().feed(line))
    assert out[0].name == "CFG"


def test_encode_rejects_bad_values() -> None:
    with pytest.raises(p.ProtocolError):
        p.encode("ARM", arm=2)
    with pytest.raises(p.ProtocolError):
        p.encode("VEL", seq=1, l_tps=1.5, r_tps=0)  # type: ignore[arg-type]
    with pytest.raises(p.ProtocolError):
        p.encode("ARM", arm=True)  # bool is not accepted as int
    with pytest.raises(p.ProtocolError):
        p.encode("CFG", key="A,B", value=1)
    with pytest.raises(p.ProtocolError):
        p.encode("VEL", seq=1, l_tps=0)
    with pytest.raises(p.ProtocolError):
        p.encode("NOPE")
    with pytest.raises(p.ProtocolError):
        p.encode("PING", seq=1, bogus=2)
    with pytest.raises(p.ProtocolError) as info:
        p.encode("CFG", key="K" * 100, value=1)
    assert info.value.code == p.ErrCode.LINE_TOO_LONG


def test_err_reply_for_checksum_names_command() -> None:
    err = next(r for r in p.StreamParser().feed(b"$VEL,12,300,-300*00\n"))
    assert isinstance(err, p.ProtocolError)
    assert p.err_reply(err) == p.encode("ERR", code=1, detail="VEL")


def test_every_schema_has_a_valid_vector() -> None:
    covered = {v["name"] for v in VECTORS["valid"]}
    assert covered == set(p.SCHEMAS)
