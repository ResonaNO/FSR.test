#!/usr/bin/env python3
"""Read and print one valid M1616L frame, then exit.
Coordinates: x increases left to right; y increases bottom to top.
Usage: python read_m1616_xy_once.py --port COM6
Dependency: python -m pip install pyserial
"""
import argparse
import struct
import sys
import time


def read_one_frame(ser, timeout):
    header = b"\xaa\xab\xac"
    buffer = bytearray()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        buffer.extend(ser.read(min(max(ser.in_waiting, 1), 4096)))
        while True:
            start = buffer.find(header)
            if start < 0:
                buffer[:] = buffer[-2:]
                break
            if start:
                del buffer[:start]
            if len(buffer) < 516:
                break
            packet = buffer[:516]
            if (sum(packet[:-1]) & 255) != packet[-1]:
                del buffer[0]
                continue
            return struct.unpack(">256H", packet[3:515])
    raise TimeoutError(f"No valid frame received within {timeout:g} seconds.")


def sensor_coordinates(index):
    """Zero-based packet index to (x, y), both 1-based.

    Sensor 1 -> (1, 1), 16 -> (1, 16),
    241 -> (16, 1), 256 -> (16, 16).
    """
    return index // 16 + 1, index % 16 + 1


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", required=True, help="For example COM6")
    ap.add_argument("--timeout", type=float, default=5, help="Maximum wait in seconds")
    args = ap.parse_args()
    if not 0 < args.timeout < float("inf"):
        ap.error("--timeout must be a finite positive number")
    try:
        import serial
    except ImportError:
        print("Install first: python -m pip install pyserial", file=sys.stderr)
        return 1
    try:
        with serial.Serial(
            port=args.port, baudrate=115200, bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
            timeout=min(0.2, args.timeout), xonxoff=False,
            rtscts=False, dsrdtr=False
        ) as ser:
            ser.reset_input_buffer()
            values = read_one_frame(ser, args.timeout)
        for index, value in enumerate(values):
            x, y = sensor_coordinates(index)
            print(f"sensor_{x}_{y}: {value} g")
        return 0
    except KeyboardInterrupt:
        print("Cancelled.", file=sys.stderr)
        return 130
    except (serial.SerialException, OSError, TimeoutError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
