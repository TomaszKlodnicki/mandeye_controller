#!/usr/bin/env python3
"""Simple UART logger for lidar sessions.

Reads newline-delimited text from a serial port and writes each line to a log
file (and stdout) prefixed with a timestamp. One input line -> one log entry.

Example:
    python3 uart_logger.py --port /dev/serial0 --baud 115200 --outdir ./uart_logs

Stop with Ctrl+C.
"""

import argparse
import datetime
import os
import sys
import time

try:
    import serial  # pyserial
except ImportError:
    sys.exit("pyserial is required. Install with: pip3 install pyserial")


def timestamp():
    """Local time with millisecond precision, e.g. 2026-07-21 14:03:07.512"""
    return datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def real_home():
    """Home of the invoking user, even under sudo (so logs don't land in /root)."""
    sudo_user = os.environ.get("SUDO_USER")
    if sudo_user:
        try:
            import pwd
            return pwd.getpwnam(sudo_user).pw_dir
        except (ImportError, KeyError):
            pass
    return os.path.expanduser("~")


DEFAULT_PORT = "/dev/ttyAMA0"  # GPIO14/15 UART on Pi 5 header pins 8/10
DEFAULT_OUTDIR = os.path.join(real_home(), "mandeye_data", "uart_logs")


def parse_args():
    p = argparse.ArgumentParser(description="Timestamped UART logger (one line = one log entry).")
    p.add_argument("--port", default=DEFAULT_PORT, help=f"Serial device (default: {DEFAULT_PORT})")
    p.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    p.add_argument("--outdir", default=DEFAULT_OUTDIR, help=f"Directory for log files (default: {DEFAULT_OUTDIR})")
    p.add_argument("--echo", action=argparse.BooleanOptionalAction, default=True,
                   help="Print each line to stdout (default: on; use --no-echo to silence)")
    p.add_argument("--reconnect", action="store_true", help="Keep retrying if the port drops or is unavailable")
    return p.parse_args()


def open_port(port, baud):
    # read_timeout=1s so readline() returns periodically and Ctrl+C stays responsive.
    return serial.Serial(port=port, baudrate=baud, timeout=1)


def main():
    args = parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    session = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    logpath = os.path.join(args.outdir, f"uart_{session}.log")

    print(f"[uart_logger] port={args.port} baud={args.baud}")
    print(f"[uart_logger] logging to {logpath}")
    print("[uart_logger] Ctrl+C to stop")

    line_count = 0
    with open(logpath, "a", encoding="utf-8") as logfile:
        while True:
            try:
                ser = open_port(args.port, args.baud)
            except serial.SerialException as e:
                msg = f"{timestamp()}\t[uart_logger] cannot open {args.port}: {e}"
                logfile.write(msg + "\n")
                logfile.flush()
                print(msg)
                if not args.reconnect:
                    return 1
                time.sleep(2)
                continue

            try:
                with ser:
                    while True:
                        raw = ser.readline()  # reads up to '\n' or until timeout
                        if not raw:
                            continue  # timeout, no data — loop and stay responsive to Ctrl+C
                        text = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                        entry = f"{timestamp()}\t{text}"
                        logfile.write(entry + "\n")
                        logfile.flush()  # persist immediately so nothing is lost on power cut
                        line_count += 1
                        if args.echo:
                            print(entry)
            except serial.SerialException as e:
                msg = f"{timestamp()}\t[uart_logger] serial error: {e}"
                logfile.write(msg + "\n")
                logfile.flush()
                print(msg)
                if not args.reconnect:
                    return 1
                time.sleep(2)
                # loop back and reopen
            except KeyboardInterrupt:
                print(f"\n[uart_logger] stopped. {line_count} lines written to {logpath}")
                return 0


if __name__ == "__main__":
    sys.exit(main())
