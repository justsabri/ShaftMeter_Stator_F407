#!/usr/bin/env python3
"""Parse four values from USART3 result frames captured in a PCAPNG file."""

from __future__ import annotations

import argparse
import struct
from dataclasses import dataclass
from typing import Iterator


FRAME_SIZE = 25
SLAVE_ID = 0x01
FUNCTION_WRITE_MULTIPLE = 0x10
START_ADDRESS = 0x0000
REGISTER_COUNT = 0x0008


@dataclass(frozen=True)
class ResultFrame:
    torque: float
    power: float
    rpm: float
    thrust: float


def parse_frame(frame: bytes) -> ResultFrame:
    """Parse one complete 25-byte result frame without CRC checking."""
    if len(frame) != FRAME_SIZE:
        raise ValueError(f"invalid frame length: {len(frame)}")
    if frame[0] != SLAVE_ID:
        raise ValueError(f"unexpected slave id: 0x{frame[0]:02X}")
    if frame[1] != FUNCTION_WRITE_MULTIPLE:
        raise ValueError(f"unexpected function code: 0x{frame[1]:02X}")
    if int.from_bytes(frame[2:4], "big") != START_ADDRESS:
        raise ValueError(f"unexpected start address: 0x{int.from_bytes(frame[2:4], 'big'):04X}")
    if int.from_bytes(frame[4:6], "big") != REGISTER_COUNT:
        raise ValueError(f"unexpected register count: {int.from_bytes(frame[4:6], 'big')}")
    if frame[6] != 16:
        raise ValueError(f"unexpected payload length: {frame[6]}")

    # The STM32 sends each IEEE-754 float most-significant byte first.
    values = struct.unpack(">4f", frame[7:23])
    return ResultFrame(*values)


def extract_frames(data: bytearray) -> Iterator[bytes]:
    """Yield valid candidate frames and resynchronize after invalid bytes."""
    while len(data) >= FRAME_SIZE:
        if (
            data[0] != SLAVE_ID
            or data[1] != FUNCTION_WRITE_MULTIPLE
            or data[2:4] != START_ADDRESS.to_bytes(2, "big")
            or data[4:6] != REGISTER_COUNT.to_bytes(2, "big")
            or data[6] != 16
        ):
            del data[0]
            continue

        candidate = bytes(data[:FRAME_SIZE])
        del data[:FRAME_SIZE]
        yield candidate


def print_result(result: ResultFrame) -> None:
    print(
        f"torque={result.torque:.6f}, "
        f"power={result.power:.6f}, "
        f"rpm={result.rpm:.6f}, "
        f"thrust={result.thrust:.6f}"
    )


def read_pcapng_file(path: str) -> None:
    """Parse result frames from packet bytes in a PCAPNG file."""
    try:
        from scapy.utils import PcapNgReader
    except ImportError:
        raise SystemExit("scapy is required: python -m pip install scapy")

    buffer = bytearray()
    try:
        with PcapNgReader(path) as capture:
            for packet in capture:
                buffer.extend(bytes(packet))
                for frame in extract_frames(buffer):
                    print_result(parse_frame(frame))
    except (OSError, ValueError) as exc:
        raise SystemExit(f"pcapng read error: {exc}") from exc


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", help="PCAPNG file containing captured USART3 data")
    args = parser.parse_args()

    read_pcapng_file(args.input)


if __name__ == "__main__":
    main()
