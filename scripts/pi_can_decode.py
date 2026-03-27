#!/usr/bin/env python3
"""Decode ESP32 weight CAN frames on a Raspberry Pi via SocketCAN.

Expected payload (8 bytes):
- Byte 0: bin_id
- Bytes 1-4: float32 little-endian weight
- Byte 5: status
- Byte 6: tare flag
- Byte 7: reserved
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

import can


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Receive and decode weight frames from MCP2515 sender")
    parser.add_argument("--channel", default="can0", help="SocketCAN channel name (default: can0)")
    parser.add_argument("--base-id", type=lambda x: int(x, 0), default=0x100, help="Base CAN ID (default: 0x100)")
    parser.add_argument("--max-id", type=lambda x: int(x, 0), default=0x1FF, help="Max CAN ID in filter (default: 0x1FF)")
    return parser.parse_args()


def build_filters(base_id: int, max_id: int) -> list[dict[str, int]]:
    # Match exact standard IDs in the range [base_id, max_id].
    return [
        {"can_id": can_id, "can_mask": 0x7FF, "extended": False}
        for can_id in range(base_id, max_id + 1)
    ]


def decode_frame(msg: can.Message) -> str:
    if msg.is_extended_id:
        return f"ignore extended frame id=0x{msg.arbitration_id:X}"

    if len(msg.data) < 8:
        return f"short frame id=0x{msg.arbitration_id:03X} dlc={len(msg.data)} data={msg.data.hex()}"

    bin_id = msg.data[0]
    weight = struct.unpack("<f", bytes(msg.data[1:5]))[0]
    status = msg.data[5]
    tare_flag = msg.data[6]
    reserved = msg.data[7]

    return (
        f"id=0x{msg.arbitration_id:03X} "
        f"bin={bin_id} "
        f"weight={weight:.3f} "
        f"status=0x{status:02X} "
        f"tare=0x{tare_flag:02X} "
        f"res=0x{reserved:02X}"
    )


def main() -> int:
    args = parse_args()

    if args.base_id > args.max_id:
        print("--base-id must be <= --max-id", file=sys.stderr)
        return 2

    print(f"Listening on {args.channel} for standard IDs 0x{args.base_id:03X}-0x{args.max_id:03X}...")

    try:
        bus = can.interface.Bus(
            channel=args.channel,
            interface="socketcan",
            can_filters=build_filters(args.base_id, args.max_id),
        )
    except Exception as exc:
        print(f"Failed to open CAN interface: {exc}", file=sys.stderr)
        return 1

    try:
        while True:
            msg = bus.recv(timeout=1.0)
            if msg is None:
                continue

            # Extra software guard so only the requested ID window is printed.
            if not (args.base_id <= msg.arbitration_id <= args.max_id):
                continue

            ts = time.strftime("%H:%M:%S")
            print(f"[{ts}] {decode_frame(msg)}")
    except KeyboardInterrupt:
        print("\nStopped.")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
