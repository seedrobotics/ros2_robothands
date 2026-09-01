"""Shared RGMP v2 client helpers.

This module implements the wire-level pieces of the Rokoko General Motion
Protocol v2 (see `docs/rgmp-v2.md` for the full spec) so that example
scripts can focus on what to do with decoded frames rather than on framing
bytes.

Every message on the wire is an 8-byte outer header (frame type +
payload length) followed by a payload. There are three frame types:

    1. Stream Definition  -- one UTF-8 JSON blob per connected device.
    2. Data Frame         -- a 16-byte inner header (device, group,
                             timestamp) plus tightly packed stream values.
    3. Device Disconnect  -- 4 bytes naming the device that went away.

All integers are little-endian.
"""

from __future__ import annotations

import enum
import json
import re
import socket
import struct
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any


DEFAULT_PORT = 12276

OUTER_HEADER_SIZE = 8  # msg_prefix (u32) + msg_len (u32)
DATA_INNER_HEADER_SIZE = 16  # device_id (u32) + group_id (u32) + ts_us (u64)


class FrameType(enum.IntEnum):
    """`msg_prefix` values from the RGMP v2 outer header."""

    DEFINITION = 1
    DATA = 2
    DISCONNECT = 3


# ── Data-type grammar ─────────────────────────────────────────────────────────
# Mirrors DataTypeInfo in csharp/Rgmp/Protocol/DataType.cs. The grammar is
# documented in docs/rgmp-v2.md under "Data Types".

# (struct format char, byte size) keyed by RGMP base-type name.
_BASE_TYPES: dict[str, tuple[str, int]] = {
    "INT32": ("i", 4),
    "UINT32": ("I", 4),
    "INT64": ("q", 8),
    "UINT64": ("Q", 8),
    "FLOAT": ("f", 4),
    "DOUBLE": ("d", 8),
}

_DATA_TYPE_RE = re.compile(
    r"^(INT32|UINT32|INT64|UINT64|FLOAT|DOUBLE)(?:\[(\d+)(?:,\s*(\d+))?\])?$"
)


def parse_data_type(raw: str) -> tuple[str, int, int]:
    """Return `(base_type, element_count, total_byte_size)` for an RGMP type.

    Examples: ``FLOAT`` -> ``("FLOAT", 1, 4)``; ``FLOAT[7]`` -> ``("FLOAT", 7,
    28)``; ``DOUBLE[3,3]`` -> ``("DOUBLE", 9, 72)``.
    """
    m = _DATA_TYPE_RE.match(raw.strip())
    if not m:
        raise ValueError(f"Unknown data_type: {raw!r}")
    base = m.group(1)
    cols = int(m.group(2)) if m.group(2) else 1
    rows = int(m.group(3)) if m.group(3) else 1
    count = rows * cols
    return base, count, _BASE_TYPES[base][1] * count


def unpack_values(
    base_type: str, count: int, buf: bytes, offset: int = 0
) -> tuple[Any, ...]:
    """Unpack `count` values of `base_type` from `buf` starting at `offset`."""
    fmt = "<" + _BASE_TYPES[base_type][0] * count
    return struct.unpack_from(fmt, buf, offset)


# ── Stream layout ─────────────────────────────────────────────────────────────
# Mirrors StreamLayout in csharp/Rgmp/Protocol/StreamDefinition.cs.


@dataclass
class StreamSlot:
    """One stream's position and shape within a group's packed payload."""

    offset: int
    base_type: str
    count: int
    byte_size: int
    entry: dict[str, Any]  # raw stream JSON, useful for labels/measure_type


@dataclass
class StreamLayout:
    """Pre-computed decoding plan for a single device's stream definition."""

    definition: dict[str, Any]
    groups: dict[int, list[StreamSlot]] = field(default_factory=dict)
    group_names: dict[int, str] = field(default_factory=dict)

    @classmethod
    def from_definition(cls, definition: dict[str, Any]) -> "StreamLayout":
        layout = cls(definition=definition)
        for gid, group in enumerate(definition.get("groups", [])):
            slots: list[StreamSlot] = []
            offset = 0
            for entry in group.get("streams", []):
                base, count, size = parse_data_type(entry["data_type"])
                slots.append(StreamSlot(offset, base, count, size, entry))
                offset += size
            layout.groups[gid] = slots
            layout.group_names[gid] = group.get("name", "")
        return layout


# ── Client ────────────────────────────────────────────────────────────────────


DefinitionCallback = Callable[[dict[str, Any], StreamLayout], None]
DataCallback = Callable[[int, int, int, bytes, StreamLayout], None]
DisconnectCallback = Callable[[int], None]
ErrorCallback = Callable[[Exception], None]


class RgmpClient:
    """RGMP v2 TCP client with multi-device support.

    Mirrors `RgmpClient` in `csharp/Rgmp/Client/RgmpClient.cs`. Definition
    frames build per-device `StreamLayout`s that are then used to decode
    subsequent data frames for the same device.

    Example usage::

        with RgmpClient(host, port) as client:
            client.on_definition = lambda d, layout: print(d["device_id"])
            client.on_data = lambda dev, gid, ts, payload, layout: ...
            client.run()
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = DEFAULT_PORT,
        timeout: float | None = None,
    ) -> None:
        self.host = host
        self.port = port
        self.timeout = timeout
        self._sock: socket.socket | None = None
        self._layouts: dict[int, StreamLayout] = {}

        # Per-frame callbacks. Assign these before calling `run()`.
        self.on_definition: DefinitionCallback | None = None
        self.on_data: DataCallback | None = None
        self.on_disconnect: DisconnectCallback | None = None
        self.on_error: ErrorCallback | None = None

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def __enter__(self) -> "RgmpClient":
        self._sock = socket.create_connection(
            (self.host, self.port), timeout=self.timeout
        )
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    # ── Socket helpers ────────────────────────────────────────────────────────

    @property
    def _socket(self) -> socket.socket:
        if self._sock is None:
            raise RuntimeError(
                "RgmpClient is not connected; use it as a context manager."
            )
        return self._sock

    def _recv_exact(self, n: int) -> bytes:
        buf = bytearray()
        while len(buf) < n:
            chunk = self._socket.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("Server closed the connection mid-frame")
            buf.extend(chunk)
        return bytes(buf)

    def _read_frame(self) -> tuple[FrameType, bytes]:
        header = self._recv_exact(OUTER_HEADER_SIZE)
        msg_prefix, msg_len = struct.unpack("<II", header)
        payload = self._recv_exact(msg_len) if msg_len else b""
        return FrameType(msg_prefix), payload

    # ── Main loop ─────────────────────────────────────────────────────────────

    def run(self) -> None:
        """Read frames forever, dispatching to callbacks until the peer closes."""
        try:
            while True:
                try:
                    frame_type, payload = self._read_frame()
                except ValueError:
                    # Unknown frame type — per spec, ignore and resync is
                    # impossible without re-reading the header, so we stop.
                    return

                if frame_type is FrameType.DEFINITION:
                    definition = json.loads(payload.decode("utf-8"))
                    layout = StreamLayout.from_definition(definition)
                    device_id = int(definition["device_id"])
                    self._layouts[device_id] = layout
                    if self.on_definition is not None:
                        self.on_definition(definition, layout)

                elif frame_type is FrameType.DATA:
                    device_id, group_id, ts_us = struct.unpack_from("<IIQ", payload, 0)
                    data_layout = self._layouts.get(device_id)
                    if data_layout is None:
                        # Data frame for a device we never saw a definition for —
                        # the spec disallows this, but be defensive.
                        continue
                    if self.on_data is not None:
                        self.on_data(
                            device_id,
                            group_id,
                            ts_us,
                            payload[DATA_INNER_HEADER_SIZE:],
                            data_layout,
                        )

                elif frame_type is FrameType.DISCONNECT:
                    (device_id,) = struct.unpack("<I", payload[:4])
                    self._layouts.pop(device_id, None)
                    if self.on_disconnect is not None:
                        self.on_disconnect(device_id)

        except ConnectionError:
            # Clean peer close — not an error worth surfacing.
            return
        except OSError as e:
            if self.on_error is not None:
                self.on_error(e)
