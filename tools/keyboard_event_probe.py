#!/usr/bin/env python3
"""Probe Linux keyboard press/release events without ROS.

This is intended to validate whether /dev/input event devices can provide
low-latency key release events for rover teleop.
"""

import argparse
import glob
import os
import re
import select
import struct
import sys
import time


EV_KEY = 0x01
KEY_CODES = {
    16: "q",
    17: "w",
    30: "a",
    31: "s",
    32: "d",
    57: "space",
}
INPUT_EVENT = struct.Struct("llHHi")


def dedupe_paths(paths):
    seen = set()
    result = []
    for path in paths:
        real = os.path.realpath(path)
        if real in seen or not os.path.exists(path):
            continue
        seen.add(real)
        result.append(path)
    return result


def discover_keyboard_devices():
    paths = []
    paths.extend(sorted(glob.glob("/dev/input/by-id/*event-kbd")))
    paths.extend(sorted(glob.glob("/dev/input/by-path/*event-kbd")))

    try:
        with open("/proc/bus/input/devices", "r", encoding="utf-8") as handle:
            blocks = handle.read().split("\n\n")
    except OSError:
        blocks = []

    for block in blocks:
        name_match = re.search(r'N: Name="([^"]+)"', block)
        handlers_match = re.search(r"H: Handlers=(.*)", block)
        if handlers_match is None:
            continue
        name = name_match.group(1) if name_match else ""
        handlers = handlers_match.group(1)
        if "kbd" not in handlers:
            continue
        lowered = name.lower()
        if any(skip in lowered for skip in ("power button", "sleep button", "video bus")):
            continue
        for event_name in re.findall(r"\bevent\d+\b", handlers):
            paths.append(f"/dev/input/{event_name}")

    return dedupe_paths(paths)


def parse_devices(devices_arg):
    if devices_arg:
        return [
            item.strip()
            for item in devices_arg.replace(";", ",").split(",")
            if item.strip()
        ]
    return discover_keyboard_devices()


def open_devices(paths):
    opened = []
    errors = []
    for path in paths:
        try:
            fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        except PermissionError as exc:
            errors.append(f"{path}: permission denied ({exc.errno})")
            continue
        except OSError as exc:
            errors.append(f"{path}: {exc}")
            continue
        opened.append((fd, path))
    return opened, errors


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--devices",
        default="",
        help="comma-separated /dev/input/event* or /dev/input/by-id/*event-kbd paths",
    )
    parser.add_argument(
        "--all-keys",
        action="store_true",
        help="print every EV_KEY code instead of only W/A/S/D/Space/Q",
    )
    args = parser.parse_args()

    paths = parse_devices(args.devices)
    if not paths:
        print("No keyboard event devices found.", file=sys.stderr)
        return 2

    opened, errors = open_devices(paths)
    if not opened:
        print("Could not open any keyboard event devices.", file=sys.stderr)
        for error in errors:
            print(f"  {error}", file=sys.stderr)
        print(
            "Try: sudo ./tools/keyboard_event_probe.py --devices <event-kbd path>",
            file=sys.stderr,
        )
        return 2

    if args.all_keys:
        print("Listening for all EV_KEY press/release/repeat events.")
        print("Use Ctrl-C to quit.")
    else:
        print("Listening for W/A/S/D/Space/Q press/release events.")
        print("Use Q press to quit.")
    print("Devices:")
    for _, path in opened:
        print(f"  {path}")

    down_since = {}
    try:
        while True:
            readable, _, _ = select.select([fd for fd, _ in opened], [], [], 0.5)
            now = time.monotonic()
            for fd in readable:
                while True:
                    try:
                        data = os.read(fd, INPUT_EVENT.size * 32)
                    except BlockingIOError:
                        break
                    if not data:
                        break
                    usable = len(data) - (len(data) % INPUT_EVENT.size)
                    for offset in range(0, usable, INPUT_EVENT.size):
                        _, _, event_type, code, value = INPUT_EVENT.unpack_from(
                            data, offset
                        )
                        if event_type != EV_KEY:
                            continue
                        if not args.all_keys and code not in KEY_CODES:
                            continue
                        key = KEY_CODES.get(code, f"code_{code}")
                        if value == 1:
                            down_since[key] = now
                            print(f"{now:.6f} {key:8s} press   code={code}")
                            if not args.all_keys and key == "q":
                                return 0
                        elif value == 0:
                            held = now - down_since.pop(key, now)
                            print(
                                f"{now:.6f} {key:8s} release code={code} "
                                f"held={held:.3f}s"
                            )
                        elif value == 2:
                            print(f"{now:.6f} {key:8s} repeat  code={code}")
    finally:
        for fd, _ in opened:
            try:
                os.close(fd)
            except OSError:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
