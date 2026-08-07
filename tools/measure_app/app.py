#!/usr/bin/env python3
"""Simple measurement control panel for the Mandeye controller + UART logger.

One small Flask app that lets you, from a browser on your phone/laptop over wifi:
  * start/stop the mandeye control_program and the UART logger together,
  * start/stop a measurement (scan) and watch live progress,
  * download any session directory as a zip.

Run:
    python3 app.py
Then open  http://<rpi-ip>:8080  in a browser.

Everything is configurable via environment variables (see CONFIG below); the
defaults match the Unitree L2 + RPi 5 setup.
"""

import json
import os
import shutil
import signal
import socket
import subprocess
import threading
import time
import urllib.request
import zipfile
from pathlib import Path

from flask import Flask, jsonify, request, send_file, Response

# --------------------------------------------------------------------------- #
# Config (override with env vars)
# --------------------------------------------------------------------------- #
HOME = os.path.expanduser("~")


def env(name, default):
    return os.environ.get(name, default)


REPO = env("MANDEYE_REPO", os.path.join(HOME, "mandeye_data"))
BUILD_DIR = env("MANDEYE_BUILD_DIR", os.path.join(HOME, "mandeye_controller", "build"))
CONTROLLER_BIN = env("MANDEYE_CONTROLLER_BIN", os.path.join(BUILD_DIR, "control_program"))
CONTROLLER_URL = env("MANDEYE_CONTROLLER_URL", "http://127.0.0.1:8003")
UART_SCRIPT = env("UART_LOGGER", os.path.join(HOME, "mandeye_controller", "tools", "uart_logger.py"))
UART_PORT = env("UART_PORT", "/dev/ttyAMA0")
UART_BAUD = env("UART_BAUD", "115200")
UART_OUTDIR = env("UART_OUTDIR", os.path.join(REPO, "uart_logs"))
APP_PORT = int(env("MEASURE_APP_PORT", "8080"))

# Environment handed to the controller subprocess.
CONTROLLER_ENV = {
    "MANDEYE_LIDAR_SDK": env("MANDEYE_LIDAR_SDK", "UNITREE"),
    "MANDEYE_LIVOX_LISTEN_IP": env("MANDEYE_LIVOX_LISTEN_IP", "192.168.123.120"),
    "MANDEYE_REPO": REPO,
    "MANDEYE_GPIO_SIM": env("MANDEYE_GPIO_SIM", "1"),
}


def laszip_lib_dir():
    """Locate the freshly-built liblaszip so the controller can be run from build/."""
    for p in Path(BUILD_DIR).rglob("liblaszip.so*"):
        return str(p.parent)
    return os.path.join(BUILD_DIR, "3rd", "LASzip")


# --------------------------------------------------------------------------- #
# Process manager
# --------------------------------------------------------------------------- #
class Managed:
    """Tracks a single child process we spawned."""

    def __init__(self, name):
        self.name = name
        self.proc = None
        self.logf = None
        self.log_path = os.path.join(REPO, f"{name}.out.log")

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self, argv, cwd=None, extra_env=None):
        if self.running():
            return False
        os.makedirs(REPO, exist_ok=True)
        e = os.environ.copy()
        if extra_env:
            e.update(extra_env)
        self._close_log()
        self.logf = open(self.log_path, "ab", buffering=0)
        self.proc = subprocess.Popen(
            argv, cwd=cwd, env=e,
            stdin=subprocess.DEVNULL, stdout=self.logf, stderr=subprocess.STDOUT,
        )
        return True

    def stop(self, sig=signal.SIGTERM, timeout=8):
        if not self.running():
            self.proc = None
            self._close_log()
            return False
        self.proc.send_signal(sig)
        try:
            self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait(timeout=3)
        self.proc = None
        self._close_log()
        return True

    def _close_log(self):
        if self.logf is not None:
            try:
                self.logf.close()
            except OSError:
                pass
            self.logf = None


controller = Managed("controller")
uart = Managed("uart_logger")
state_lock = threading.Lock()
scan_started_at = {"t": None}  # epoch seconds when a measurement began


# --------------------------------------------------------------------------- #
# Controller HTTP helpers
# --------------------------------------------------------------------------- #
def controller_reachable():
    host = CONTROLLER_URL.split("://", 1)[-1].split(":")[0]
    try:
        port = int(CONTROLLER_URL.rsplit(":", 1)[-1])
    except ValueError:
        port = 8003
    try:
        with socket.create_connection((host, port), timeout=0.5):
            return True
    except OSError:
        return False


def controller_get(path, timeout=2.0):
    with urllib.request.urlopen(CONTROLLER_URL + path, timeout=timeout) as r:
        return r.read().decode("utf-8", errors="replace")


# --------------------------------------------------------------------------- #
# Session / download helpers
# --------------------------------------------------------------------------- #
def dir_stats(path):
    total, count = 0, 0
    for root, _dirs, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(root, f))
                count += 1
            except OSError:
                pass
    return total, count


def list_sessions():
    out = []
    if not os.path.isdir(REPO):
        return out
    for name in sorted(os.listdir(REPO)):
        full = os.path.join(REPO, name)
        if not os.path.isdir(full):
            continue
        size, count = dir_stats(full)
        out.append({
            "name": name,
            "size_bytes": size,
            "file_count": count,
            "mtime": os.path.getmtime(full),
        })
    return out


def safe_session_path(name):
    """Resolve a session name to a directory that is a direct child of REPO."""
    if not name or "/" in name or "\\" in name or name in (".", ".."):
        return None
    full = os.path.realpath(os.path.join(REPO, name))
    if os.path.commonpath([full, os.path.realpath(REPO)]) != os.path.realpath(REPO):
        return None
    if not os.path.isdir(full):
        return None
    return full


class _ZipSink:
    """Non-seekable file-like sink for streaming a ZipFile out over HTTP.

    Keeps a running position (`tell`) so the central directory offsets stay
    correct, while `drain()` hands back and clears accumulated bytes to yield.
    Intentionally has no `seek`, so ZipFile uses data descriptors instead of
    seeking back to patch local headers.
    """

    def __init__(self):
        self._chunks = []
        self._pos = 0

    def write(self, b):
        self._chunks.append(bytes(b))
        self._pos += len(b)
        return len(b)

    def flush(self):
        pass

    def tell(self):
        return self._pos

    def drain(self):
        if not self._chunks:
            return b""
        out = b"".join(self._chunks)
        self._chunks = []
        return out


def zip_dir_stream(path, arc_root):
    """Yield a zip of `path` as a stream (STORED — .laz is already compressed).

    Streams each file block-by-block and drains after every block, so memory
    stays bounded (~one block) even for multi-GB files — never buffers a whole
    file or the whole archive.
    """
    sink = _ZipSink()
    zf = zipfile.ZipFile(sink, mode="w", compression=zipfile.ZIP_STORED, allowZip64=True)
    for root, _dirs, files in os.walk(path):
        for f in sorted(files):
            fp = os.path.join(root, f)
            arc = os.path.join(arc_root, os.path.relpath(fp, path))
            try:
                zinfo = zipfile.ZipInfo.from_file(fp, arc)
                zinfo.compress_type = zipfile.ZIP_STORED
                with zf.open(zinfo, mode="w") as dest, open(fp, "rb") as src:
                    while True:
                        block = src.read(262144)
                        if not block:
                            break
                        dest.write(block)
                        data = sink.drain()
                        if data:
                            yield data
            except OSError:
                # file vanished or became unreadable mid-walk (e.g. a live
                # session rotating files) — skip it and keep going.
                continue
            data = sink.drain()
            if data:
                yield data
    zf.close()
    tail = sink.drain()
    if tail:
        yield tail


# --------------------------------------------------------------------------- #
# Flask app
# --------------------------------------------------------------------------- #
app = Flask(__name__)
HERE = os.path.dirname(os.path.abspath(__file__))


@app.route("/")
def index():
    return send_file(os.path.join(HERE, "index.html"))


@app.route("/api/system/start", methods=["POST"])
def system_start():
    with state_lock:
        started = {"controller": False, "uart": False}
        if not controller_reachable() and not controller.running():
            ld = os.pathsep.join(p for p in (laszip_lib_dir(), os.environ.get("LD_LIBRARY_PATH", "")) if p)
            started["controller"] = controller.start(
                [CONTROLLER_BIN], cwd=BUILD_DIR,
                extra_env={**CONTROLLER_ENV, "LD_LIBRARY_PATH": ld},
            )
        if not uart.running():
            started["uart"] = uart.start(
                ["python3", UART_SCRIPT, "--port", UART_PORT, "--baud", UART_BAUD,
                 "--outdir", UART_OUTDIR, "--time-format", "epoch_ns", "--reconnect", "--no-echo"],
            )
    return jsonify({"ok": True, "started": started})


@app.route("/api/system/stop", methods=["POST"])
def system_stop():
    with state_lock:
        # Stop any running scan cleanly before killing the controller.
        if controller_reachable():
            try:
                controller_get("/trig/stop_bag")
            except Exception:
                pass
            time.sleep(0.5)
        controller.stop()
        uart.stop()
        scan_started_at["t"] = None
    return jsonify({"ok": True})


@app.route("/api/scan/start", methods=["POST"])
def scan_start():
    try:
        controller_get("/trig/start_bag")
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 502
    scan_started_at["t"] = time.time()
    return jsonify({"ok": True})


@app.route("/api/scan/stop", methods=["POST"])
def scan_stop():
    try:
        controller_get("/trig/stop_bag")
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 502
    scan_started_at["t"] = None
    return jsonify({"ok": True})


@app.route("/api/status")
def status():
    reachable = controller_reachable()
    ctrl = None
    if reachable:
        try:
            ctrl = json.loads(controller_get("/json/status"))
        except Exception:
            ctrl = None
    du = shutil.disk_usage(REPO) if os.path.isdir(REPO) else None
    elapsed = (time.time() - scan_started_at["t"]) if scan_started_at["t"] else 0
    return jsonify({
        "controller_running": reachable,
        "controller_managed": controller.running(),
        "uart_running": uart.running(),
        "scan_elapsed_s": round(elapsed, 1),
        "disk": {"total": du.total, "used": du.used, "free": du.free} if du else None,
        "repo": REPO,
        "controller": ctrl,
    })


@app.route("/api/sessions")
def sessions():
    return jsonify({"repo": REPO, "sessions": list_sessions()})


@app.route("/api/download")
def download():
    name = request.args.get("name", "")
    full = safe_session_path(name)
    if not full:
        return jsonify({"ok": False, "error": "invalid session"}), 400
    fname = f"{name}.zip"
    return Response(
        zip_dir_stream(full, name),
        mimetype="application/zip",
        headers={"Content-Disposition": f'attachment; filename="{fname}"'},
    )


if __name__ == "__main__":
    os.makedirs(REPO, exist_ok=True)
    print(f"[measure_app] repo={REPO}")
    print(f"[measure_app] controller={CONTROLLER_BIN}")
    print(f"[measure_app] open http://<this-rpi-ip>:{APP_PORT}")
    app.run(host="0.0.0.0", port=APP_PORT, threaded=True)
