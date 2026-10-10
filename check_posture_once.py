#!/usr/bin/env python3
"""Detect an object from one valid M1616L frame, then exit.
Coordinates: x increases left to right; y increases bottom to top.
Usage: python check_posture_once.py --pad-port COM6 --back-port COM7
Dependency: python -m pip install pyserial
"""
import argparse
import re
from contextlib import ExitStack
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



# Empty-pad reference transcribed from the user's screenshot, 2026-10-06.
# Packet order: index = (x - 1) * 16 + (y - 1).
# Recalibrate after changing mounting, backing, wiring or pad orientation.
def screenshot_baseline():
    baseline = [0] * 256
    bottom_row = [0, 6, 0, 2, 5, 2, 0, 14, 13, 32, 1, 26, 18, 27, 13, 0]
    for x, value in enumerate(bottom_row, 1):
        baseline[(x - 1) * 16] = value
    baseline[(9 - 1) * 16 + (16 - 1)] = 67
    baseline[(9 - 1) * 16 + (15 - 1)] = 3
    baseline[(10 - 1) * 16 + (15 - 1)] = 11
    return baseline


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


class BackParser:
    """Accept only complete sequential FSR1/FSR2/FSR3 groups from existing C++."""
    def __init__(self):
        self.buffer = bytearray()
        self.pending = []

    def feed(self, data):
        self.buffer.extend(data)
        groups = []
        while b"\n" in self.buffer:
            line, _, rest = self.buffer.partition(b"\n")
            self.buffer = bytearray(rest)
            match = re.fullmatch(rb"FSR([123]) Value: (\d+)", line.strip())
            if not match:
                continue
            index, value = map(int, match.groups())
            if not 0 <= value <= 4095:
                self.pending = []
                continue
            if index == 1:
                self.pending = [value]
            elif index == len(self.pending) + 1 and self.pending:
                self.pending.append(value)
                if len(self.pending) == 3:
                    groups.append(tuple(self.pending))
                    self.pending = []
            else:
                self.pending = []
        if len(self.buffer) > 4096:
            self.buffer.clear()
            self.pending = []
        return groups


class PadParser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while True:
            start = self.buffer.find(b"\xaa\xab\xac")
            if start < 0:
                self.buffer[:] = self.buffer[-2:]
                break
            del self.buffer[:start]
            if len(self.buffer) < 516:
                break
            frame = self.buffer[:516]
            if sum(frame[:-1]) & 255 != frame[-1]:
                del self.buffer[0]
                continue
            frames.append(struct.unpack(">256H", frame[3:515]))
            del self.buffer[:516]
        return frames


def posture_result(seated, back_values, min_back=1):
    states = ["pressed" if v < 3000 else "not_pressed" if v >= 4000 else "transition"
              for v in back_values]
    pressed = states.count("pressed")
    possible = pressed + states.count("transition")
    if not seated:
        result = "NO_OCCUPANT"
    elif pressed >= min_back:
        result = "POSTURE_OK"
    elif possible >= min_back:
        result = "UNCERTAIN"
    else:
        result = "POSTURE_INCORRECT"
    return result, states


def read_pair(pad, back, timeout=10, max_skew=0.3):
    pad_parser, back_parser = PadParser(), BackParser()
    latest_pad = latest_back = None
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for values in pad_parser.feed(pad.read(min(max(pad.in_waiting, 1), 4096))):
            latest_pad = (time.monotonic(), values)
        for values in back_parser.feed(back.read(min(max(back.in_waiting, 1), 4096))):
            latest_back = (time.monotonic(), values)
        if latest_pad and latest_back:
            skew = abs(latest_pad[0] - latest_back[0])
            if skew <= max_skew:
                return latest_pad[1], latest_back[1], skew
    missing = []
    if latest_pad is None:
        missing.append("pad: no valid binary packet")
    if latest_back is None:
        missing.append("ESP32: no complete FSR1/FSR2/FSR3 group")
    raise TimeoutError("No fresh data pair: " + ("; ".join(missing) or "timestamps too far apart"))


def main():
    ap = argparse.ArgumentParser(description="One posture decision using M1616L + existing ESP32 firmware")
    ap.add_argument("--pad-port", required=True)
    ap.add_argument("--back-port", required=True)
    ap.add_argument("--baseline", help="Optional empty-pad baseline JSON; default is screenshot baseline")
    ap.add_argument("--min-back", type=int, choices=(1,2,3), default=1,
                    help="Required pressed back sensors (default: any one)")
    ap.add_argument("--timeout", type=float, default=10)
    ap.add_argument("--grid", action="store_true")
    args = ap.parse_args()
    if args.pad_port.lower() == args.back_port.lower():
        ap.error("Pad and ESP32 must use two different ports")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        ap.error("--timeout must be finite and positive")
    try:
        import serial
    except ImportError:
        print("Install: python -m pip install pyserial", file=sys.stderr)
        return 1
    try:
        baseline = load_baseline(args.baseline) if args.baseline else screenshot_baseline()
        with ExitStack() as stack:
            ports = []
            for name in (args.pad_port, args.back_port):
                ser = stack.enter_context(serial.Serial(
                    port=name, baudrate=115200, bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                    timeout=0.05, xonxoff=False, rtscts=False, dsrdtr=False))
                ports.append(ser)
            for ser in ports:
                ser.reset_input_buffer()
            pad_values, back_values, skew = read_pair(*ports, timeout=args.timeout)
        seated, excess, active, total = detect(pad_values, baseline)
        result, states = posture_result(seated, back_values, args.min_back)
        print(f"seat_occupied: {str(seated).lower()}")
        print(f"seat_active_sensors: {len(active)}")
        print(f"seat_effective_total_g: {total:g}")
        for i, (value, state) in enumerate(zip(back_values, states), 1):
            print(f"back_sensor_{i}: adc={value}, status={state}")
        print(f"required_back_contacts: {args.min_back}")
        print(f"receive_time_difference_ms: {skew * 1000:.1f}")
        print(f"posture: {result}")
        messages = {
            "NO_OCCUPANT": "未检测到坐垫载荷，不判断姿势。",
            "POSTURE_OK": "姿势正确（符合坐垫有载荷且接触靠背的规则）。",
            "POSTURE_INCORRECT": "姿势不正确（坐垫有载荷，靠背接触未达到要求）。",
            "UNCERTAIN": "暂不能判断：靠背读数处于过渡区。",
        }
        print(messages[result])
        if args.grid:
            print(render_grid(pad_values))
        return 0
    except KeyboardInterrupt:
        print("Cancelled.", file=sys.stderr)
        return 130
    except (serial.SerialException, OSError, ValueError, TimeoutError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
