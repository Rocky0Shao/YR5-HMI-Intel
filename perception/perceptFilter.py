"""Perception CAN filter.

Given a raw CAN frame expressed as a string, validate the message against the
subset of CAN frames defined in ``ADC2_SC_2024.2.dbc`` that correspond to
objects, traffic signals, traffic signs, or observed speed limits. Messages
that match the expected format are passed to a transmission stub so that the
integration point with the downstream service is explicit.
"""

from __future__ import annotations

import argparse
import logging
import re
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, Optional

try:
	import cantools  # type: ignore
except ImportError:  # pragma: no cover - optional dependency
	cantools = None  # type: ignore

import percept_sender


LOGGER = logging.getLogger(__name__)


STREAM_MESSAGE_PATTERN = re.compile(
    r"Sent CAN message: ID=(?P<identifier>0x[0-9A-Fa-f]+|\d+), Data=(?P<data>[0-9A-Fa-f]+)"
)


@dataclass(frozen=True)
class DBCSignal:
	"""Minimal signal representation extracted from the DBC."""

	name: str
	start_bit: int
	length: int
	byte_order: str  # "little" or "big"
	is_signed: bool
	factor: float
	offset: float
	minimum: Optional[float]
	maximum: Optional[float]
	unit: Optional[str]


@dataclass
class DBCMessage:
	"""CAN message definition derived from the DBC file."""

	frame_id: int
	name: str
	dlc: int
	signals: Dict[str, DBCSignal] = field(default_factory=dict)

	def category(self) -> Optional[str]:
		"""Return the logical category for perception-relevant messages."""

		if self.name in PerceptionFilter.OBJECT_MESSAGES:
			return "object"
		if self.name in PerceptionFilter.TRAFFIC_LIGHT_MESSAGES:
			return "traffic_light"
		if self.name in PerceptionFilter.TRAFFIC_SIGN_MESSAGES:
			return "traffic_sign"
		if self.name in PerceptionFilter.SPEED_LIMIT_MESSAGES:
			return "speed_limit"
		return None


@dataclass(frozen=True)
class CANFrame:
	"""Parsed CAN frame."""

	identifier: int
	data: bytes
	is_extended: bool = False

	@property
	def dlc(self) -> int:
		return len(self.data)


@dataclass(frozen=True)
class FilterOutcome:
    """Result produced when a frame matches the perception criteria."""

    category: str
    message: DBCMessage
    frame: CANFrame
    decoded: Optional[Dict[str, Any]]


class DBCParser:
	"""Tiny DBC parser tailored to the structure of ADC2_SC_2024.2.dbc."""

	BO_PATTERN = re.compile(r"^BO_\s+(\d+)\s+(\w+):\s+(\d+)\s+\w+")
	SG_PATTERN = re.compile(
		r"^SG_\s+(\w+)\s*:\s*(\d+)\|(\d+)@(\d+)([+-])\s*"
		r"\(([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?),\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\)\s*"
		r"\[([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\|([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)\]\s*"
		r"\"([^\"]*)\""
	)

	@classmethod
	def parse(cls, dbc_path: Path) -> Dict[int, DBCMessage]:
		messages: Dict[int, DBCMessage] = {}
		current_message: Optional[DBCMessage] = None

		with dbc_path.open("r", encoding="ascii", errors="ignore") as dbc_file:
			for raw_line in dbc_file:
				line = raw_line.strip()
				if not line or line.startswith("CM_") or line.startswith("VAL_"):
					continue

				bo_match = cls.BO_PATTERN.match(line)
				if bo_match:
					frame_id = int(bo_match.group(1))
					name = bo_match.group(2)
					dlc = int(bo_match.group(3))
					current_message = DBCMessage(frame_id=frame_id, name=name, dlc=dlc)
					messages[frame_id] = current_message
					continue

				if line.startswith("SG_") and current_message is not None:
					sg_match = cls.SG_PATTERN.match(line)
					if not sg_match:
						LOGGER.debug("Skipping unparsed SG_ line: %s", line)
						continue
					signal = DBCSignal(
						name=sg_match.group(1),
						start_bit=int(sg_match.group(2)),
						length=int(sg_match.group(3)),
						byte_order="little" if sg_match.group(4) == "1" else "big",
						is_signed=sg_match.group(5) == "-",
						factor=float(sg_match.group(6)),
						offset=float(sg_match.group(7)),
						minimum=float(sg_match.group(8)),
						maximum=float(sg_match.group(9)),
						unit=sg_match.group(10) or None,
					)
					current_message.signals[signal.name] = signal

		return messages


class PerceptionFilter:
	"""Filter object that validates perception-relevant CAN frames."""

	OBJECT_MESSAGES = {
		"Objects",
		"Object_TrackA",
		"Object_TrackB",
		"Object_TrackC",
		"Object_TrackD",
	}
	TRAFFIC_LIGHT_MESSAGES = {
		"TrafficSignalHeads",
		"TrafficSignalHead_TrackA",
	}
	TRAFFIC_SIGN_MESSAGES = {
		"TrafficSigns",
		"TrafficSign_TrackA",
	}
	SPEED_LIMIT_MESSAGES = {
		"RoadState",
		"TrafficSign_TrackA",  # Sign type 1/2 indicates speed limits.
	}

	def __init__(
		self,
		dbc_path: Optional[Path] = None,
		sink_host: Optional[str] = None,
		sink_port: Optional[int] = None,
	) -> None:
		if dbc_path is None:
			dbc_path = Path(__file__).with_name("ADC2_SC_2024.2.dbc")
		if not dbc_path.exists():
			raise FileNotFoundError(f"DBC file not found: {dbc_path}")
		self.dbc_path = dbc_path
		self.messages_by_id = DBCParser.parse(dbc_path)
		self._cantools_db: Optional[Any] = None
		if cantools is not None:
			try:
				self._cantools_db = cantools.database.load_file(str(dbc_path))
			except Exception as exc:  # pragma: no cover - defensive
				LOGGER.warning("cantools decoding disabled: %s", exc)
				self._cantools_db = None
		self.sink_host = sink_host or percept_sender.DEFAULT_HOST
		self.sink_port = sink_port or percept_sender.DEFAULT_PORT
		self.sender = percept_sender.PerceptStreamSender(
			host=self.sink_host,
			port=self.sink_port,
		)
		self._min_transmit_interval = 0.2  # seconds (5 Hz)
		self._last_transmit_time = 0.0

	def filter_message(self, raw_frame: str) -> Optional[FilterOutcome]:
		"""Return a FilterOutcome when the frame matches the perception filters."""

		frame = self._parse_can_frame(raw_frame)
		message_def = self.messages_by_id.get(frame.identifier)
		if message_def is None:
			LOGGER.debug("CAN ID 0x%03X not found in DBC", frame.identifier)
			return None

		category = message_def.category()
		if category is None:
			LOGGER.debug(
				"CAN ID 0x%03X (%s) not in perception set", frame.identifier, message_def.name
			)
			return None

		if frame.dlc != message_def.dlc:
			LOGGER.warning(
				"CAN ID 0x%03X (%s) has DLC %s but expected %s",
				frame.identifier,
				message_def.name,
				frame.dlc,
				message_def.dlc,
			)
			return None

		decoded = self._decode_frame(frame, message_def)
		outcome = FilterOutcome(
			category=category,
			message=message_def,
			frame=frame,
			decoded=decoded,
		)
		self._transmit_stub(outcome)
		return outcome

	def _parse_can_frame(self, raw_frame: str) -> CANFrame:
		"""Parse a string representation of a CAN frame."""

		candidate = raw_frame.strip()
		if not candidate:
			raise ValueError("Empty CAN frame string")

		if "#" in candidate:
			identifier_str, payload_str = candidate.split("#", maxsplit=1)
			data_bytes = self._parse_payload(payload_str)
			identifier = self._parse_identifier(identifier_str)
			return CANFrame(identifier=identifier, data=data_bytes)

		parts = candidate.replace("\t", " ").split()
		if len(parts) < 2:
			raise ValueError(f"Unrecognised CAN frame format: {raw_frame!r}")

		identifier = self._parse_identifier(parts[0])
		# Some log formats include DLC explicitly as the second token.
		possible_dlc = parts[1]
		data_tokens: Iterable[str]
		if possible_dlc.isdigit() and len(parts) > 2:
			data_tokens = parts[2:]
		else:
			data_tokens = parts[1:]

		data_bytes = bytes(int(token, 16) for token in data_tokens)
		return CANFrame(identifier=identifier, data=data_bytes)

	@staticmethod
	def _parse_identifier(identifier: str) -> int:
		identifier = identifier.strip().lower()
		base = 16 if identifier.startswith("0x") else 10
		try:
			return int(identifier, base)
		except ValueError as exc:
			raise ValueError(f"Invalid CAN identifier: {identifier!r}") from exc

	@staticmethod
	def _parse_payload(payload: str) -> bytes:
		cleaned = payload.strip().replace(" ", "")
		if len(cleaned) % 2 != 0:
			raise ValueError(f"Hex payload must contain an even number of digits: {payload!r}")
		try:
			return bytes.fromhex(cleaned)
		except ValueError as exc:
			raise ValueError(f"Invalid hex payload: {payload!r}") from exc

	def close(self) -> None:
		if hasattr(self, "sender") and self.sender is not None:
			self.sender.close()

	def _enforce_rate_limit(self) -> None:
		"""Ensure transmissions do not exceed the 5 Hz requirement."""

		if self._min_transmit_interval <= 0:
			return
		now = time.monotonic()
		elapsed = now - self._last_transmit_time
		if elapsed < self._min_transmit_interval:
			time.sleep(self._min_transmit_interval - elapsed)
			now = time.monotonic()
		self._last_transmit_time = now

	def _decode_frame(self, frame: CANFrame, message_def: DBCMessage) -> Optional[Dict[str, Any]]:
		"""Attempt to decode the CAN frame using cantools, if available."""

		if self._cantools_db is None:
			return None
		try:
			db_message = self._cantools_db.get_message_by_frame_id(frame.identifier)
		except (KeyError, AttributeError):
			return None
		try:
			return db_message.decode(frame.data)
		except Exception as exc:  # pragma: no cover - defensive
			if LOGGER.isEnabledFor(logging.DEBUG):
				LOGGER.debug("Failed to decode CAN frame 0x%03X: %s", frame.identifier, exc)
			return None

	def _transmit_stub(self, outcome: FilterOutcome) -> None:
		"""Serialize and forward the frame via the TCP sender."""

		self._enforce_rate_limit()
		payload_decoded = outcome.decoded or {}
		try:
			self.sender.send_percept(
				category=outcome.category,
				message_name=outcome.message.name,
				can_id=outcome.frame.identifier,
				dlc=outcome.frame.dlc,
				raw_data=outcome.frame.data,
				decoded_signals=payload_decoded,
			)
		except percept_sender.PerceptSenderError as exc:
			LOGGER.error("Failed to send percept message: %s", exc)
		else:
			LOGGER.info(
				"Forwarded %s CAN frame 0x%03X (%s) to %s:%d",
				outcome.category,
				outcome.frame.identifier,
				outcome.message.name,
				self.sink_host,
				self.sink_port,
			)
			if outcome.decoded:
				LOGGER.debug("Decoded payload: %s", outcome.decoded)


def _frame_string_from_stream_line(raw_line: str) -> Optional[str]:
    """Convert a generator log line into a canonical CAN frame string."""

    match = STREAM_MESSAGE_PATTERN.search(raw_line)
    if match is None:
        return None
    identifier = match.group("identifier")
    data = match.group("data")
    return f"{identifier}#{data}"


def _iter_stream_lines(path: Path, *, follow: bool) -> Iterable[str]:
    """Yield lines from *path*, optionally waiting for new data like tail -f."""

    with path.open("r", encoding="ascii", errors="ignore") as stream:
        while True:
            line = stream.readline()
            if line:
                yield line
                continue
            if not follow:
                break
            time.sleep(0.1)


def _print_decoded(outcome: FilterOutcome) -> None:
    """Display decoded CAN signal values when available."""

    print(f"  category: {outcome.category}")
    print(f"  message: {outcome.message.name} (0x{outcome.frame.identifier:03X})")

    if not outcome.decoded:
        print("  decoded signals: <unavailable>")
        return

    print("  decoded signals:")
    for signal, value in sorted(outcome.decoded.items()):
        print(f"    {signal}: {value}")


def _process_stream(filter_: PerceptionFilter, *, stream_path: Path, follow: bool) -> None:
    """Feed lines from a generator output file into the perception filter."""

    for raw_line in _iter_stream_lines(stream_path, follow=follow):
        frame_candidate = _frame_string_from_stream_line(raw_line)
        if frame_candidate is None:
            LOGGER.debug("Skipping unrelated stream line: %s", raw_line.rstrip())
            continue
        try:
            outcome = filter_.filter_message(frame_candidate)
        except ValueError as exc:
            LOGGER.warning("Skipping unparsable CAN frame '%s': %s", raw_line.rstrip(), exc)
            continue
        status = "MATCH" if outcome else "NO MATCH"
        print(f"{status}: {raw_line.rstrip()}")
        if outcome:
            _print_decoded(outcome)


def _build_argument_parser() -> argparse.ArgumentParser:
	parser = argparse.ArgumentParser(description=__doc__)
	parser.add_argument(
		"frame",
		nargs="?",
		help=(
			"Raw CAN message string, e.g. '0x21#0011223344556677' or '33 8 01 02 03 04 05 06 07 08'."
		),
	)
	parser.add_argument(
		"--dbc",
		type=Path,
		default=None,
		help="Path to the ADC2_SC_2024.2.dbc file (defaults to sibling of this script).",
	)
	parser.add_argument(
		"--stream",
		type=Path,
		default=None,
		help="Path to a text stream produced by SCTestGenerator.py (processed line by line).",
	)
	parser.add_argument(
		"--follow",
		action="store_true",
		help="Continue watching the stream file for newly appended messages.",
	)
	parser.add_argument(
		"--sink-host",
		default=percept_sender.DEFAULT_HOST,
		help="Destination TCP host for serialized percept messages.",
	)
	parser.add_argument(
		"--sink-port",
		type=int,
		default=percept_sender.DEFAULT_PORT,
		help="Destination TCP port for serialized percept messages.",
	)
	parser.add_argument(
		"--verbose",
		action="store_true",
		help="Emit debug output while parsing and validating frames.",
	)
	return parser


def main() -> None:
	parser = _build_argument_parser()
	args = parser.parse_args()

	logging.basicConfig(
		level=logging.DEBUG if args.verbose else logging.INFO,
		format="%(asctime)s %(levelname)s %(name)s: %(message)s",
	)

	filter_ = PerceptionFilter(
		dbc_path=args.dbc,
		sink_host=args.sink_host,
		sink_port=args.sink_port,
	)

	if args.stream is None and args.frame is None:
		parser.error("either a raw CAN frame or --stream must be provided")

	try:
		if args.stream is not None:
			_process_stream(filter_, stream_path=args.stream, follow=args.follow)
		elif args.frame is not None:
			outcome = filter_.filter_message(args.frame)
			if outcome:
				print("MATCH: perception-relevant CAN frame")
				_print_decoded(outcome)
			else:
				print("NO MATCH: frame ignored")
	finally:
		filter_.close()


if __name__ == "__main__":
	main()

