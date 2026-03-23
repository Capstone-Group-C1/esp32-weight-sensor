#!/usr/bin/env python3
"""Receive and decode ESP32 counter CAN frames on Raspberry Pi SocketCAN."""

import argparse
import importlib
import struct
import sys
import time

can = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Receive counter frames from ESP32 MCP2515 sender")
    parser.add_argument("--channel", default="can0", help="SocketCAN channel (default: can0)")
    parser.add_argument("--bitrate", type=int, default=500000, help="Expected bitrate in bps (default: 500000)")
    parser.add_argument("--can-id", type=lambda x: int(x, 0), default=0x100, help="CAN ID to match (default: 0x100)")
    return parser.parse_args()


def decode_counter(data: bytes) -> int:
    if len(data) < 4:
        raise ValueError("Payload too short for counter")
    return struct.unpack("<I", data[0:4])[0]


def main() -> int:
    args = parse_args()

    global can
    try:
        can = importlib.import_module("can")
    except ImportError:
        print("Missing dependency: python-can", file=sys.stderr)
        print("Install with: python3 -m pip install --user python-can", file=sys.stderr)
        return 1

    print(f"Listening on {args.channel} for CAN ID 0x{args.can_id:03X} at {args.bitrate} bps")

    bus = can.interface.Bus(channel=args.channel, interface="socketcan")

    # Kernel-level filter keeps only the expected ID.
    bus.set_filters([
        {"can_id": args.can_id, "can_mask": 0x7FF, "extended": False},
    ])

    last_counter = None
    while True:
        msg = bus.recv(timeout=2.0)
        if msg is None:
            print("No frame in 2s, still waiting...")
            continue

        if msg.is_error_frame:
            print("Error frame received")
            continue

        counter = decode_counter(msg.data)
        now = time.strftime("%H:%M:%S")

        if last_counter is None:
            gap = "first"
        else:
            delta = (counter - last_counter) & 0xFFFFFFFF
            gap = f"delta={delta}"

        print(f"[{now}] id=0x{msg.arbitration_id:03X} dlc={msg.dlc} counter={counter} {gap}")
        last_counter = counter


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nStopped")
        raise SystemExit(0)
