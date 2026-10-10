#!/usr/bin/env python3
"""Detect an object from one valid M1616L frame, then exit.
Coordinates: x increases left to right; y increases bottom to top.
Usage: python detect_load_once.py --port COM6
Dependency: python -m pip install pyserial
"""
import argparse
import json
import math
from pathlib import Path
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


def detect(values, baseline, point_threshold=100, total_threshold=1000, min_active=3):
    excess = [max(0, v - b) for v, b in zip(values, baseline)]
    active = [(i, value) for i, value in enumerate(excess) if value >= point_threshold]
    effective_total = sum(value for _, value in active)
    present = len(active) >= min_active and effective_total >= total_threshold
    return present, excess, active, effective_total


def load_baseline(path):
    record = json.loads(Path(path).read_text(encoding="utf-8"))
    if record.get("model") != "M1616L" or record.get("unit") != "g":
        raise ValueError("Baseline model/unit mismatch")
    values = record.get("values")
    if not isinstance(values, list) or len(values) != 256:
        raise ValueError("Baseline must contain 256 values")
    if any(type(v) not in (int, float) or not math.isfinite(v) or not 0 <= v <= 65535 for v in values):
        raise ValueError("Invalid baseline values")
    return values


def render_grid(values):
    """Render physical orientation: top y=16; left x=1."""
    if len(values) != 256:
        raise ValueError("Expected 256 sensor values")
    width = max(4, max(len(str(v)) for v in values))
    prefix = " " * 7
    border = prefix + "+" + "+".join("-" * (width + 2) for _ in range(16)) + "+"
    lines = ["M1616L | one frame | unit: g | cell = sensor_x_y",
             "x ->".ljust(7) + " " + " ".join(f"{x:^{width + 2}}" for x in range(1, 17)),
             border]
    for y in range(16, 0, -1):
        cells = [values[(x - 1) * 16 + (y - 1)] for x in range(1, 17)]
        lines.append(f"y={y:2d}   |" + "|".join(f" {v:>{width}} " for v in cells) + "|")
        lines.append(border)
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", required=True)
    ap.add_argument("--timeout", type=float, default=5)
    group = ap.add_mutually_exclusive_group()
    group.add_argument("--baseline", help="Empty-pad baseline JSON to subtract")
    group.add_argument("--save-baseline", help="Record ONE empty-pad frame to a new JSON file")
    ap.add_argument("--point-threshold", type=float, default=100, help="Per-point excess threshold in g (default 100)")
    ap.add_argument("--total-threshold", type=float, default=1000, help="Active-point sum threshold in g (default 1000)")
    ap.add_argument("--min-active", type=int, default=3, help="Minimum active points (default 3)")
    ap.add_argument("--show-sensors", action="store_true", help="Also print all 256 raw and baseline-subtracted values")
    ap.add_argument("--grid", action="store_true", help="Also print the 16x16 raw-value grid")
    args = ap.parse_args()
    for name in ("timeout", "point_threshold", "total_threshold"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            ap.error(f"--{name.replace('_', '-')} must be finite and positive")
    if not 1 <= args.min_active <= 256:
        ap.error("--min-active must be between 1 and 256")
    try:
        import serial
    except ImportError:
        print("Install first: python -m pip install pyserial", file=sys.stderr)
        return 1
    try:
        baseline = load_baseline(args.baseline) if args.baseline else [0] * 256
        if args.save_baseline and Path(args.save_baseline).exists():
            raise ValueError("Baseline file already exists; use a new filename")
        with serial.Serial(
            port=args.port, baudrate=115200, bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
            timeout=min(0.2, args.timeout), xonxoff=False,
            rtscts=False, dsrdtr=False
        ) as ser:
            ser.reset_input_buffer()
            values = read_one_frame(ser, args.timeout)
        if args.save_baseline:
            with open(args.save_baseline, "x", encoding="utf-8") as f:
                json.dump({"model": "M1616L", "unit": "g", "values": list(values)}, f, indent=2)
            print(f"Saved one empty-pad frame: {args.save_baseline}")
            return 0
        present, excess, active, total = detect(values, baseline, args.point_threshold,
                                              args.total_threshold, args.min_active)
        print("object_present: " + str(present).lower())
        print("result: " + ("OBJECT DETECTED" if present else "NO OBJECT DETECTED"))
        print(f"baseline_used: {bool(args.baseline)}")
        print(f"active_sensors: {len(active)} / 256 (required >= {args.min_active})")
        print(f"effective_total_g: {total:g} (required >= {args.total_threshold:g})")
        print(f"max_excess_g: {max(excess):g}")
        print(f"mean_excess_g: {sum(excess) / 256:g}")
        print("Note: load detection only; not identification of a person or an accurate body-weight measurement.")
        if args.grid:
            print(render_grid(values))
        print(f"point_threshold_g: {args.point_threshold:g}")
        print("active_points (x increases right, y increases up):")
        for index, value in active:
            x, y = sensor_coordinates(index)
            print(f"  sensor_{x}_{y}: {value:g} g above baseline")
        if args.show_sensors:
            print("all_sensors:")
            for index, (raw, adjusted) in enumerate(zip(values, excess)):
                x, y = sensor_coordinates(index)
                print(f"sensor_{x}_{y}: raw={raw} g, excess={adjusted:g} g")
        return 0
    except KeyboardInterrupt:
        print("Cancelled.", file=sys.stderr)
        return 130
    except (serial.SerialException, OSError, ValueError, TimeoutError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
