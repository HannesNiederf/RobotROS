"""Serial protocol version 1 between the Pi and the Nano boards.

Source: docs/spec/spez_firmware_protokoll.md section 2. This module has no ROS
dependency so it can be unit-tested anywhere; nano_bridge and fake_nano use it.

Frame: ``$`` payload ``*`` checksum ``\\n``.
- payload: ``NAME,field1,field2,...``, printable ASCII, no spaces.
- checksum: XOR of all bytes between ``$`` and ``*``, two upper-case hex digits.
- ``\\r`` is ignored. At most ``MAX_FRAME_LEN`` characters per frame including
  ``$``, ``*``, checksum and ``\\n``.
- Numbers are decimal integers with an optional minus sign, never floats.
- Receivers ignore extra trailing fields (section 2.5).

Choices made here (not fixed by the spec, see the vectors file):
- MAX_FRAME_LEN counts the terminating ``\\n`` (96 bytes in total).
- A ``$`` in the middle of a line starts a new frame; bytes before it are dropped.
- Bytes outside a frame (before the first ``$``) are ignored silently.
"""

from __future__ import annotations

from collections.abc import Iterable, Iterator
from dataclasses import dataclass, field
from enum import IntEnum

PROTO_VER = 1
MAX_FRAME_LEN = 96

INT32_MIN = -(2**31)
INT32_MAX = 2**31 - 1
UINT32_MAX = 2**32 - 1


class ErrCode(IntEnum):
    """Codes of the ``$ERR`` telegram (section 2.3)."""

    CHECKSUM = 1
    UNKNOWN_COMMAND = 2
    BAD_ARGUMENT = 3
    ROLE_NOT_SUPPORTED = 4
    NOT_ALLOWED_IN_STATE = 5
    LINE_TOO_LONG = 6
    BUSY = 7


class Role(IntEnum):
    DRIVE = 1
    AUX = 2
    ALL = 3


class ProtocolError(ValueError):
    """A frame or field violates the protocol. ``code`` maps to ``$ERR``."""

    def __init__(self, code: ErrCode, message: str, command: str = "") -> None:
        super().__init__(message)
        self.code = code
        self.command = command


# Field kinds: (name, kind, min, max). kind "int" or "str".
_Field = tuple[str, str, int, int]


def _i(name: str, lo: int, hi: int) -> _Field:
    return (name, "int", lo, hi)


def _s(name: str) -> _Field:
    return (name, "str", 0, 0)


_U8 = (0, 255)
_U16 = (0, 65535)
_I16 = (-32768, 32767)
_U32 = (0, UINT32_MAX)
_I32 = (INT32_MIN, INT32_MAX)
_SEQ = _U32

# Pi -> Nano (section 2.2). Optional trailing fields are listed in _OPTIONAL.
PI_TO_NANO: dict[str, list[_Field]] = {
    "WHO": [],
    "PING": [_i("seq", *_SEQ)],
    "VEL": [_i("seq", *_SEQ), _i("l_tps", *_I32), _i("r_tps", *_I32)],
    "ARM": [_i("arm", 0, 1)],
    "CLR": [],
    "CFG": [_s("key"), _i("value", *_I32)],
    "HUB": [_i("command", 0, 2), _i("pwm", *_U8)],
    "LED": [_i("strip", 0, 2), _i("pattern", 0, 3), _i("r", *_U8), _i("g", *_U8), _i("b", *_U8)],
    "BUZ": [_i("pattern", 0, 3)],
    "REL": [_i("channel", 1, 3), _i("on", 0, 1)],
}

# Nano -> Pi (section 2.3).
NANO_TO_PI: dict[str, list[_Field]] = {
    "HELLO": [_i("proto", *_U16), _s("fw"), _i("role", 1, 3), _i("caps", *_U8)],
    "PONG": [_i("seq", *_SEQ)],
    "ODO": [_i("t_ms", *_U32), _i("l_ticks", *_I32), _i("r_ticks", *_I32), _i("l_tps", *_I32), _i("r_tps", *_I32)],
    "SEN": [
        _i("t_ms", *_U32),
        _i("us_front_mm", *_U16),
        _i("us_rear_mm", *_U16),
        _i("cls_front", 0, 3),
        _i("cls_rear", 0, 3),
        _i("bump", 0, 7),
    ],
    "SAF": [_i("t_ms", *_U32), _i("state", 0, 3), _i("flags", *_U16)],
    "PWR": [
        _i("t_ms", *_U32),
        _i("adc_batt_mv", *_U16),
        _i("adc_chg_mv", *_U16),
        _i("adc_rail_mv", *_U16),
        _i("i_ma", *_I16),
    ],
    "HUBST": [_i("t_ms", *_U32), _i("state", 0, 5), _i("limits", 0, 3), _i("fault", 0, 2)],
    "EVT": [_i("t_ms", *_U32), _i("type", *_U8), _i("arg", *_I32)],
    "ACK": [_s("command"), _s("arg")],
    "ERR": [_i("code", 1, 7), _s("detail")],
}

# Number of trailing fields that may be missing (ACK and ERR have an optional argument).
_OPTIONAL: dict[str, int] = {"ACK": 1, "ERR": 1}

SCHEMAS: dict[str, list[_Field]] = {**PI_TO_NANO, **NANO_TO_PI}


def checksum(payload: str | bytes) -> int:
    """XOR of all payload bytes (the bytes between ``$`` and ``*``)."""
    data = payload.encode("ascii") if isinstance(payload, str) else payload
    value = 0
    for byte in data:
        value ^= byte
    return value


def _check_text(text: str, what: str) -> None:
    for ch in text:
        if not (0x21 <= ord(ch) <= 0x7E) or ch in "$*,":
            raise ProtocolError(ErrCode.BAD_ARGUMENT, f"invalid character {ch!r} in {what}")


def encode_payload(payload: str) -> bytes:
    """Wrap an already formatted payload into a full frame."""
    _check_text(payload.replace(",", ""), "payload")
    frame = f"${payload}*{checksum(payload):02X}\n"
    if len(frame) > MAX_FRAME_LEN:
        raise ProtocolError(ErrCode.LINE_TOO_LONG, f"frame has {len(frame)} > {MAX_FRAME_LEN} bytes")
    return frame.encode("ascii")


@dataclass(frozen=True)
class Message:
    """A decoded telegram. ``fields`` holds the known fields by name, ``extra`` ignored trailing ones."""

    name: str
    fields: dict[str, int | str]
    extra: tuple[str, ...] = field(default_factory=tuple)

    def __getitem__(self, key: str) -> int | str:
        return self.fields[key]


def _format_value(spec: _Field, value: int | str) -> str:
    name, kind, lo, hi = spec
    if kind == "int":
        if isinstance(value, bool) or not isinstance(value, int):
            raise ProtocolError(ErrCode.BAD_ARGUMENT, f"{name} must be int, got {type(value).__name__}")
        if not lo <= value <= hi:
            raise ProtocolError(ErrCode.BAD_ARGUMENT, f"{name}={value} outside [{lo}, {hi}]")
        return str(value)
    text = str(value)
    if not text:
        raise ProtocolError(ErrCode.BAD_ARGUMENT, f"{name} must not be empty")
    _check_text(text, name)
    return text


def encode(name: str, **values: int | str) -> bytes:
    """Build a frame for message ``name`` from keyword fields (validated against the schema)."""
    if name not in SCHEMAS:
        raise ProtocolError(ErrCode.UNKNOWN_COMMAND, f"unknown message {name}", name)
    schema = SCHEMAS[name]
    optional = _OPTIONAL.get(name, 0)
    parts = [name]
    unknown = set(values) - {spec[0] for spec in schema}
    if unknown:
        raise ProtocolError(ErrCode.BAD_ARGUMENT, f"unknown field(s) {sorted(unknown)} for {name}", name)
    for index, spec in enumerate(schema):
        if spec[0] not in values:
            if index >= len(schema) - optional:
                break
            raise ProtocolError(ErrCode.BAD_ARGUMENT, f"missing field {spec[0]} for {name}", name)
        parts.append(_format_value(spec, values[spec[0]]))
    return encode_payload(",".join(parts))


def _parse_int(text: str) -> int:
    body = text[1:] if text.startswith("-") else text
    if not body or not body.isdigit() or not body.isascii():
        raise ValueError(text)
    return int(text)


def decode_payload(payload: str) -> Message:
    """Decode a checked payload (without ``$``, ``*`` and checksum) into a Message."""
    parts = payload.split(",")
    name = parts[0]
    if name not in SCHEMAS:
        raise ProtocolError(ErrCode.UNKNOWN_COMMAND, f"unknown message {name}", name)
    schema = SCHEMAS[name]
    args = parts[1:]
    required = len(schema) - _OPTIONAL.get(name, 0)
    if len(args) < required:
        raise ProtocolError(ErrCode.BAD_ARGUMENT, f"{name} needs {required} fields, got {len(args)}", name)
    fields: dict[str, int | str] = {}
    for spec, raw in zip(schema, args, strict=False):  # optional fields may be missing
        fname, kind, lo, hi = spec
        if kind == "int":
            try:
                value = _parse_int(raw)
            except ValueError:
                raise ProtocolError(ErrCode.BAD_ARGUMENT, f"{fname}={raw!r} is not an integer", name) from None
            if not lo <= value <= hi:
                raise ProtocolError(ErrCode.BAD_ARGUMENT, f"{fname}={value} outside [{lo}, {hi}]", name)
            fields[fname] = value
        else:
            if not raw:
                raise ProtocolError(ErrCode.BAD_ARGUMENT, f"{fname} is empty", name)
            fields[fname] = raw
    return Message(name, fields, tuple(args[len(schema) :]))


def split_frame(frame: str) -> str:
    """Check one frame text (``$...*HH``, without line end) and return its payload."""
    if not frame.startswith("$"):
        raise ProtocolError(ErrCode.BAD_ARGUMENT, "frame does not start with '$'")
    star = frame.rfind("*")
    if star < 0 or len(frame) - star != 3:
        raise ProtocolError(ErrCode.CHECKSUM, "missing or malformed checksum")
    payload = frame[1:star]
    command = payload.split(",", 1)[0]
    digits = frame[star + 1 :]
    if any(c not in "0123456789ABCDEF" for c in digits):
        raise ProtocolError(ErrCode.CHECKSUM, f"checksum {digits!r} is not upper-case hex", command)
    if int(digits, 16) != checksum(payload):
        raise ProtocolError(ErrCode.CHECKSUM, f"checksum {digits} != {checksum(payload):02X}", command)
    _check_text(payload.replace(",", ""), "payload")
    if not command:
        raise ProtocolError(ErrCode.UNKNOWN_COMMAND, "empty command")
    return payload


def decode_frame(frame: str) -> Message:
    """Decode one complete frame text (``$...*HH``, line end optional)."""
    text = frame.replace("\r", "").rstrip("\n")
    if len(text) + 1 > MAX_FRAME_LEN:
        raise ProtocolError(ErrCode.LINE_TOO_LONG, f"frame longer than {MAX_FRAME_LEN} bytes")
    return decode_payload(split_frame(text))


ParseResult = Message | ProtocolError


class StreamParser:
    """Incremental parser for a byte stream. ``feed`` yields Messages and ProtocolErrors in order."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self._in_frame = False
        self._overflow = False

    def feed(self, data: bytes) -> Iterator[ParseResult]:
        for byte in data:
            yield from self._byte(byte)

    def _byte(self, byte: int) -> Iterable[ParseResult]:
        if byte == 0x0D:  # '\r' is ignored everywhere
            return ()
        if byte == 0x24:  # '$' (re)starts a frame
            self._buf = bytearray(b"$")
            self._in_frame = True
            self._overflow = False
            return ()
        if not self._in_frame:
            return ()
        if byte == 0x0A:  # '\n' ends a frame
            self._in_frame = False
            if self._overflow:
                return (ProtocolError(ErrCode.LINE_TOO_LONG, f"frame longer than {MAX_FRAME_LEN} bytes"),)
            text = self._buf.decode("ascii", errors="replace")
            try:
                return (decode_frame(text),)
            except ProtocolError as err:
                return (err,)
        if len(self._buf) + 1 >= MAX_FRAME_LEN:  # +1 for the '\n' still to come
            self._overflow = True
            return ()
        self._buf.append(byte)
        return ()


def err_reply(error: ProtocolError) -> bytes:
    """``$ERR`` frame a Nano sends for ``error`` (checksum errors carry the command name)."""
    if error.code == ErrCode.CHECKSUM and error.command:
        return encode("ERR", code=int(error.code), detail=error.command)
    return encode("ERR", code=int(error.code))
