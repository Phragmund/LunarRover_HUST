#!/usr/bin/env python3
"""USB CDC command client for the CtrBoard-H7 M3508 controller."""

from __future__ import annotations

import argparse
import sys
import threading
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover
    raise SystemExit("Install pyserial first: python -m pip install pyserial") from exc


def build_frame(mode: str, values: list[float] | None) -> str:
    mode = mode.upper()
    if mode in {"STOP", "ZERO"}:
        return f"$M3508:{mode}!"
    if values is None or len(values) != 4:
        raise SystemExit("speed/rpm/position/current modes require exactly four values")

    if mode == "SPEED":
        cmd = "SPD"
    elif mode == "RPM":
        cmd = "RPM"
    elif mode == "POSITION":
        cmd = "POS"
    elif mode == "CURRENT":
        cmd = "CUR"
    else:
        raise SystemExit(f"unsupported mode: {mode}")

    payload = ",".join(f"{value:.6g}" for value in values)
    return f"$M3508:{cmd},{payload}!"


def reader(port: serial.Serial, stop_event: threading.Event) -> None:
    while not stop_event.is_set():
        try:
            data = port.read(port.in_waiting or 1)
        except serial.SerialException:
            return
        if data:
            sys.stdout.write(data.decode("utf-8", errors="replace"))
            sys.stdout.flush()


def list_serial_ports() -> None:
    for item in list_ports.comports():
        print(f"{item.device}\t{item.description}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Control four M3508 motors through H7 USB CDC.")
    parser.add_argument("--list", action="store_true", help="list available serial ports and exit")
    parser.add_argument("--port", help="serial port, for example COM7")
    parser.add_argument("--baud", type=int, default=115200, help="ignored by USB CDC but kept for tools")
    parser.add_argument(
        "--mode",
        choices=["speed", "rpm", "position", "current", "stop", "zero"],
        default="stop",
        help="speed: output shaft r/s; rpm: output shaft rpm; position: output shaft deg; current: raw C620 current",
    )
    parser.add_argument("values", nargs="*", type=float, help="four values for speed/rpm/position/current")
    parser.add_argument("--period", type=float, default=0.05, help="repeat period in seconds")
    parser.add_argument("--duration", type=float, default=0.0, help="0 means repeat until Ctrl-C")
    parser.add_argument("--once", action="store_true", help="send one frame and exit")
    parser.add_argument("--dry-run", action="store_true", help="print the frame and exit without opening the port")
    args = parser.parse_args()

    if args.list:
        list_serial_ports()
        return 0

    frame = build_frame(args.mode, args.values if args.values else None)
    raw = frame.encode("ascii")
    if args.dry_run:
        print(frame)
        return 0

    if not args.port:
        raise SystemExit("missing --port, use --list to find the H7 virtual COM port")

    stop_event = threading.Event()

    try:
        port_context = serial.Serial(args.port, args.baud, timeout=0.05, write_timeout=0.2)
    except serial.SerialException as exc:
        raise SystemExit(f"failed to open {args.port}: {exc}") from exc

    with port_context as port:
        rx_thread = threading.Thread(target=reader, args=(port, stop_event), daemon=True)
        rx_thread.start()
        start = time.monotonic()
        try:
            while True:
                port.write(raw)
                if args.once:
                    time.sleep(0.2)
                    break
                if args.duration > 0.0 and (time.monotonic() - start) >= args.duration:
                    break
                time.sleep(args.period)
        except KeyboardInterrupt:
            pass
        finally:
            if args.mode not in {"stop", "zero"}:
                port.write(b"$M3508:STOP!")
            stop_event.set()
            rx_thread.join(timeout=0.5)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
