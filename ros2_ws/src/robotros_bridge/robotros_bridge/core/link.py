"""Connection state of one Nano for nano_bridge (spec FW section 2.6, steps 2, 5, 6, 8).

Pure logic: the caller opens the port, passes received bytes and monotonic time in ms, and writes
whatever ``take_output`` returns. No serial or ROS imports here.

States: DISCONNECTED -> HANDSHAKE ($WHO sent, waiting for $HELLO) -> CONFIGURING ($CFG one by one,
each acknowledged) -> READY. ``LOST`` after no byte for ``link_timeout_ms``; the caller then closes the
port and calls ``connect`` again after ``reconnect_period_ms``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from . import protocol as p


class LinkState(Enum):
    DISCONNECTED = "disconnected"
    HANDSHAKE = "handshake"
    CONFIGURING = "configuring"
    READY = "ready"
    LOST = "lost"
    INCOMPATIBLE = "incompatible"


@dataclass
class LinkTimings:
    hello_timeout_ms: int = 1000
    cfg_ack_timeout_ms: int = 500
    link_timeout_ms: int = 300
    cfg_retries: int = 3


@dataclass
class LinkEvents:
    """What happened during one call; the caller turns it into topics, logs and alerts."""

    messages: list[p.Message] = field(default_factory=list)
    became_ready: bool = False
    nano_reset: bool = False  # unexpected $HELLO while READY (spec 2.6 step 6)
    lost: bool = False
    incompatible: str = ""  # E104 reason
    warnings: list[str] = field(default_factory=list)


class DeviceLink:
    def __init__(self, name: str, role: p.Role, cfg: dict[str, int], timings: LinkTimings | None = None) -> None:
        self.name = name
        self.role = role
        self.cfg = list(cfg.items())
        self.t = timings or LinkTimings()
        self.state = LinkState.DISCONNECTED
        self._parser = p.StreamParser()
        self._out = bytearray()
        self._last_rx_ms = 0
        self._since_ms = 0
        self._cfg_index = 0
        self._cfg_tries = 0
        self.fw_version = ""
        self.caps = 0
        self.frames_ok = 0
        self.frames_bad = 0

    # --------------------------------------------------------------- helpers
    def _send(self, name: str, **fields: int | str) -> None:
        self._out += p.encode(name, **fields)

    def take_output(self) -> bytes:
        data = bytes(self._out)
        self._out.clear()
        return data

    @property
    def error_ratio(self) -> float:
        total = self.frames_ok + self.frames_bad
        return self.frames_bad / total if total else 0.0

    def send_command(self, name: str, **fields: int | str) -> bool:
        """Queue a Pi->Nano command; only allowed once READY. Returns False if not sent."""
        if self.state != LinkState.READY:
            return False
        self._send(name, **fields)
        return True

    # ------------------------------------------------------------ lifecycle
    def connect(self, now_ms: int) -> None:
        """Port was (re)opened: start the handshake."""
        self._parser = p.StreamParser()
        self._out.clear()
        self.state = LinkState.HANDSHAKE
        self._last_rx_ms = now_ms
        self._since_ms = now_ms
        self._send("WHO")

    def disconnect(self) -> None:
        self.state = LinkState.DISCONNECTED

    def _start_config(self, now_ms: int, ev: LinkEvents) -> None:
        self._cfg_index = 0
        self._cfg_tries = 0
        self.state = LinkState.CONFIGURING
        self._send_cfg(now_ms, ev)

    def _send_cfg(self, now_ms: int, ev: LinkEvents) -> None:
        if self._cfg_index >= len(self.cfg):
            self.state = LinkState.READY
            ev.became_ready = True
            return
        key, value = self.cfg[self._cfg_index]
        self._since_ms = now_ms
        self._cfg_tries += 1
        self._send("CFG", key=key, value=int(value))

    def _on_hello(self, now_ms: int, msg: p.Message, ev: LinkEvents) -> None:
        if msg["proto"] != p.PROTO_VER:
            self.state = LinkState.INCOMPATIBLE
            ev.incompatible = f"{self.name}: protocol {msg['proto']} != {p.PROTO_VER}"
            return
        if msg["role"] != int(self.role):
            self.state = LinkState.INCOMPATIBLE
            ev.incompatible = f"{self.name}: role {msg['role']} != {int(self.role)}"
            return
        self.fw_version = str(msg["fw"])
        self.caps = int(msg["caps"])
        if self.state in (LinkState.READY, LinkState.CONFIGURING):
            ev.nano_reset = True
        self._start_config(now_ms, ev)

    # -------------------------------------------------------------- receive
    def feed(self, now_ms: int, data: bytes) -> LinkEvents:
        ev = LinkEvents()
        if not data or self.state in (LinkState.DISCONNECTED, LinkState.INCOMPATIBLE):
            return ev
        self._last_rx_ms = now_ms
        if self.state == LinkState.LOST:
            self.state = LinkState.HANDSHAKE  # bytes again: ask who is there
            self._since_ms = now_ms
            self._send("WHO")
        for result in self._parser.feed(data):
            if isinstance(result, p.ProtocolError):
                self.frames_bad += 1
                ev.warnings.append(f"{self.name}: {result}")
                continue
            self.frames_ok += 1
            msg = result
            if msg.name == "HELLO":
                self._on_hello(now_ms, msg, ev)
                ev.messages.append(msg)
                continue
            if self.state == LinkState.CONFIGURING and msg.name in ("ACK", "ERR"):
                key = self.cfg[self._cfg_index][0] if self._cfg_index < len(self.cfg) else ""
                if msg.name == "ACK" and msg["command"] == "CFG" and msg.fields.get("arg") == key:
                    self._cfg_index += 1
                    self._cfg_tries = 0
                    self._send_cfg(now_ms, ev)
                    continue
                if msg.name == "ERR" and msg.fields.get("detail") == key:
                    ev.warnings.append(f"{self.name}: CFG {key} refused (ERR {msg['code']})")
                    self._cfg_index += 1
                    self._cfg_tries = 0
                    self._send_cfg(now_ms, ev)
                    continue
            if self.state in (LinkState.CONFIGURING, LinkState.READY):
                ev.messages.append(msg)
        return ev

    # ----------------------------------------------------------------- time
    def tick(self, now_ms: int) -> LinkEvents:
        ev = LinkEvents()
        if self.state in (LinkState.DISCONNECTED, LinkState.INCOMPATIBLE, LinkState.LOST):
            return ev
        live = self.state in (LinkState.CONFIGURING, LinkState.READY)
        if live and now_ms - self._last_rx_ms > self.t.link_timeout_ms:
            self.state = LinkState.LOST
            ev.lost = True
            return ev
        if self.state == LinkState.HANDSHAKE and now_ms - self._since_ms >= self.t.hello_timeout_ms:
            self._since_ms = now_ms
            self._send("WHO")
            ev.warnings.append(f"{self.name}: no HELLO within {self.t.hello_timeout_ms} ms, asking again")
        elif self.state == LinkState.CONFIGURING and now_ms - self._since_ms >= self.t.cfg_ack_timeout_ms:
            key = self.cfg[self._cfg_index][0]
            if self._cfg_tries >= self.t.cfg_retries:
                ev.warnings.append(f"{self.name}: CFG {key} not acknowledged, skipped")
                self._cfg_index += 1
                self._cfg_tries = 0
            self._send_cfg(now_ms, ev)
        return ev
