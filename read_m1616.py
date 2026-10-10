#!/usr/bin/env python3
"""Read Guantuo M1616L packets (manual page 10). Values are reported in g.

Install: python -m pip install pyserial
List ports: python -m serial.tools.list_ports -v
Run: python read_m1616.py --port COM3
CSV: python read_m1616.py --port COM3 --csv readings.csv
"""
import argparse
import csv
from datetime import datetime, timezone
import struct
import sys
import time
from contextlib import ExitStack

HEADER = b"\xaa\xab\xac"
FRAME_SIZE = 516


class FrameParser:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_checksums = 0

    def feed(self, data):
        """Accept arbitrary serial chunks; return complete, checked frames."""
        self.buffer.extend(data)
        frames = []
        while True:
            start = self.buffer.find(HEADER)
            if start < 0:
                # Preserve a possible split header at the end of the chunk.
                self.buffer[:] = self.buffer[-2:]
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < FRAME_SIZE:
                break
            packet = self.buffer[:FRAME_SIZE]
            if (sum(packet[:-1]) & 0xff) != packet[-1]:
                self.bad_checksums += 1
                del self.buffer[0]
                continue
            frames.append(struct.unpack(">256H", packet[3:515]))
            del self.buffer[:FRAME_SIZE]
        return frames


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", required=True, help="COM3 or /dev/cu... or /dev/ttyUSB0")
    ap.add_argument("--csv", help="New CSV file; every valid frame is recorded")
    ap.add_argument("--print-every", type=int, default=10,
                    help="Print every N frames (default 10, about once/second; 1 = all)")
    args = ap.parse_args()
    if args.print_every < 1:
        ap.error("--print-every must be at least 1")
    try:
        import serial
    except ImportError:
        print("Install first: python -m pip install pyserial", file=sys.stderr)
        return 1
    parser = FrameParser()
    count = 0
    byte_count = 0
    last_valid = time.monotonic()
    last_notice = last_valid
    try:
        with ExitStack() as stack:
            writer = None
            if args.csv:
                # Exclusive creation protects earlier recordings from overwrite.
                output = stack.enter_context(open(args.csv, "x", newline="", encoding="utf-8"))
                writer = csv.writer(output)
                writer.writerow(["received_at_utc", "frame"] +
                                [f"sensor_{i}_g" for i in range(1, 257)])
            ser = stack.enter_context(serial.Serial(
                port=args.port, baudrate=115200, bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                timeout=0.2, xonxoff=False, rtscts=False, dsrdtr=False))
            print(f"Opened {ser.port}: 115200 8N1. Ctrl+C to stop.", file=sys.stderr)
            while True:
                data = ser.read(min(max(ser.in_waiting, 1), 4096))
                byte_count += len(data)
                for values in parser.feed(data):
                    count += 1
                    last_valid = time.monotonic()
                    stamp = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
                    # values[0] = sensor 1, ..., values[255] = sensor 256.
                    if writer:
                        writer.writerow([stamp, count, *values])
                        output.flush()
                    if count == 1 or count % args.print_every == 0:
                        lines = [f"\nFrame {count} | {stamp} | unit: g"]
                        lines.extend(f"sensor {i}: {v} g" for i, v in enumerate(values, 1))
                        print("\n".join(lines), flush=True)
                now = time.monotonic()
                if now - last_valid >= 5 and now - last_notice >= 5:
                    print(f"No valid frame for 5s; received bytes={byte_count}, "
                          f"bad checksums={parser.bad_checksums}. "
                          "Check port, connection and protocol version.", file=sys.stderr)
                    last_notice = now
    except KeyboardInterrupt:
        print(f"\nStopped: {count} valid frames, {parser.bad_checksums} bad checksums.", file=sys.stderr)
        return 0
    except (serial.SerialException, OSError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
