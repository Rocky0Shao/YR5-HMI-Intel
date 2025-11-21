"""Utility for sending perception CAN detections over TCP using Protobuf."""

from __future__ import annotations

import logging
import socket
from datetime import datetime, timezone
from typing import Dict, Tuple

try:
    import percept_message_pb2 as message_pb2  # type: ignore
except ImportError:  # pragma: no cover - optional dependency
    message_pb2 = None  # type: ignore

LOGGER = logging.getLogger(__name__)

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 5002
DEFAULT_TIMEOUT = 2.0

_SENDER_CACHE: Dict[Tuple[str, int, float], "PerceptStreamSender"] = {}


class PerceptSenderError(RuntimeError):
    """Raised when a percept message cannot be serialized or transmitted."""


class PerceptStreamSender:
    """Maintains a persistent TCP connection for streaming percept messages."""

    def __init__(
        self,
        *,
        host: str = DEFAULT_HOST,
        port: int = DEFAULT_PORT,
        timeout: float = DEFAULT_TIMEOUT,
    ) -> None:
        self.host = host
        self.port = port
        self.timeout = timeout
        self._sock: socket.socket | None = None

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            finally:
                self._sock = None

    def _ensure_connected(self) -> None:
        if self._sock is not None:
            return
        try:
            sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self._sock = sock
        except OSError as exc:
            raise PerceptSenderError(
                f"Unable to connect to {self.host}:{self.port}: {exc}"
            ) from exc

    def _send_frame(self, frame: bytes) -> None:
        last_error: OSError | None = None
        for _ in range(2):
            self._ensure_connected()
            try:
                assert self._sock is not None
                self._sock.sendall(frame)
                return
            except OSError as exc:
                last_error = exc
                self.close()
        if last_error is not None:
            raise PerceptSenderError(f"Failed to send percept payload: {last_error}")

    def send_payload(self, payload: bytes, *, add_length_prefix: bool = True) -> None:
        frame = payload
        if add_length_prefix:
            frame = len(payload).to_bytes(4, byteorder="big") + payload
        self._send_frame(frame)

    def send_percept(
        self,
        *,
        category: str,
        message_name: str,
        can_id: int,
        dlc: int,
        raw_data: bytes,
        decoded_signals: Dict[str, object],
    ) -> None:
        payload = build_protobuf_payload(
            category=category,
            message_name=message_name,
            can_id=can_id,
            dlc=dlc,
            raw_data=raw_data,
            decoded_signals=decoded_signals,
        )
        self.send_payload(payload)


def _ensure_message_module() -> None:
    if message_pb2 is None:
        raise PerceptSenderError(
            "message_pb2 module not found. Compile percept_message.proto before sending."
        )


def build_protobuf_payload(
    *,
    category: str,
    message_name: str,
    can_id: int,
    dlc: int,
    raw_data: bytes,
    decoded_signals: Dict[str, object],
    timestamp_iso8601: str | None = None,
) -> bytes:
    """Serialize the supplied values into the PerceptMessage protobuf."""

    _ensure_message_module()
    percept = message_pb2.PerceptMessage()
    percept.category = category
    percept.message_name = message_name
    percept.can_id = can_id
    percept.dlc = dlc
    percept.raw_data = raw_data
    if timestamp_iso8601 is None:
        timestamp_iso8601 = datetime.now(timezone.utc).isoformat()
    percept.timestamp_iso8601 = timestamp_iso8601

    for name, value in sorted(decoded_signals.items()):
        signal = percept.decoded_signals.add()
        signal.name = name
        signal.value = str(value)

    return percept.SerializeToString()


def _get_shared_sender(host: str, port: int, timeout: float) -> PerceptStreamSender:
    key = (host, port, timeout)
    sender = _SENDER_CACHE.get(key)
    if sender is None:
        sender = PerceptStreamSender(host=host, port=port, timeout=timeout)
        _SENDER_CACHE[key] = sender
    return sender


def send_payload(
    payload: bytes,
    *,
    host: str = DEFAULT_HOST,
    port: int = DEFAULT_PORT,
    timeout: float = DEFAULT_TIMEOUT,
    add_length_prefix: bool = True,
) -> None:
    """Send a serialized payload over TCP, optionally prefixing with length."""

    sender = _get_shared_sender(host, port, timeout)
    sender.send_payload(payload, add_length_prefix=add_length_prefix)


def send_percept(
    *,
    category: str,
    message_name: str,
    can_id: int,
    dlc: int,
    raw_data: bytes,
    decoded_signals: Dict[str, object],
    host: str = DEFAULT_HOST,
    port: int = DEFAULT_PORT,
    timeout: float = DEFAULT_TIMEOUT,
) -> None:
    """High-level helper that builds and sends a perception protobuf."""

    sender = _get_shared_sender(host, port, timeout)
    try:
        sender.send_percept(
            category=category,
            message_name=message_name,
            can_id=can_id,
            dlc=dlc,
            raw_data=raw_data,
            decoded_signals=decoded_signals,
        )
        LOGGER.debug(
            "Sent percept message '%s' (0x%03X) to %s:%d",
            message_name,
            can_id,
            host,
            port,
        )
    except OSError as exc:
        raise PerceptSenderError(f"Failed to send percept message: {exc}") from exc
