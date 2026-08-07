#!/usr/bin/env python3
"""Simple measurement control panel for the Mandeye controller + UART logger.

One small Flask app to run a measurement from a browser over wifi:

  * Start   — wipe the (scratch) data dir, boot control_program, wait for the
              lidar to warm up + sync, then begin recording + the UART logger.
  * Stop    — stop the scan, stop controller + logger, zip ALL of the data dir
              into <test_name>.zip (kept outside the data dir), then wipe it.
  * Download the packaged tests.

Because each test owns the whole data dir, packaging is just "zip everything,
wipe everything" — robust, with no per-file bookkeeping.

Run:
    python3 app.py
Then open  http://<rpi-ip>:8080

Everything is configurable via environment variables (see CONFIG below); the
defaults match the Unitree L2 + RPi 5 setup.
"""

import glob
import json
import os
import re
import shutil
import signal
import socket
import subprocess
import threading
import time
import urllib.request
import zipfile

from flask import Flask, jsonify, request, send_file

# --------------------------------------------------------------------------- #
# Config (override with env vars)
# --------------------------------------------------------------------------- #
HOME = os.path.expanduser("~")


def env(name, default):
    return os.environ.get(name, default)


REPO = env("MANDEYE_REPO", os.path.join(HOME, "mandeye_data"))            # scratch dir, wiped per test
PACKAGES_DIR = env("MANDEYE_PACKAGES", os.path.join(HOME, "mandeye_tests"))  # zips live here (NOT wiped)
BUILD_DIR = env("MANDEYE_BUILD_DIR", os.path.join(HOME, "mandeye_controller", "build"))
CONTROLLER_BIN = env("MANDEYE_CONTROLLER_BIN", os.path.join(BUILD_DIR, "control_program"))
CONTROLLER_URL = env("MANDEYE_CONTROLLER_URL", "http://127.0.0.1:8003")
UART_SCRIPT = env("UART_LOGGER", os.path.join(HOME, "mandeye_controller", "tools", "uart_logger.py"))
UART_PORT = env("UART_PORT", "/dev/ttyAMA0")
UART_BAUD = env("UART_BAUD", "115200")
UART_OUTDIR = env("UART_OUTDIR", os.path.join(REPO, "uart_logs"))
APP_PORT = int(env("MEASURE_APP_PORT", "8080"))
ARM_TIMEOUT = float(env("MANDEYE_ARM_TIMEOUT", "120"))  # max seconds to wait for lidar-ready before recording

# Environment handed to the controller subprocess.
CONTROLLER_ENV = {
    "MANDEYE_LIDAR_SDK": env("MANDEYE_LIDAR_SDK", "UNITREE"),
    "MANDEYE_LIVOX_LISTEN_IP": env("MANDEYE_LIVOX_LISTEN_IP", "192.168.123.120"),
    "MANDEYE_REPO": REPO,
    "MANDEYE_GPIO_SIM": env("MANDEYE_GPIO_SIM", "1"),
}


def laszip_lib_dir():
    """Locate the freshly-built liblaszip so the controller can be run from build/."""
    for p in glob.glob(os.path.join(BUILD_DIR, "**", "liblaszip.so*"), recursive=True):
        return os.path.dirname(p)
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
        self.error = None
        self.log_path = os.path.join(REPO, f"{name}.out.log")

    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self, argv, cwd=None, extra_env=None):
        if self.running():
            return True
        os.makedirs(REPO, exist_ok=True)
        e = os.environ.copy()
        if extra_env:
            e.update(extra_env)
        self._close_log()
        self.error = None
        try:
            self.logf = open(self.log_path, "ab", buffering=0)
            self.proc = subprocess.Popen(
                argv, cwd=cwd, env=e,
                stdin=subprocess.DEVNULL, stdout=self.logf, stderr=subprocess.STDOUT,
            )
        except OSError as ex:
            self.error = str(ex)
            self._close_log()
            self.proc = None
            return False
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
scan_started_at = {"t": None}       # epoch seconds when recording actually began
arming = {"active": False, "error": None}   # warming-up-then-start phase
current_test = {"name": None, "uart_log": None}


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


def controller_status():
    try:
        return json.loads(controller_get("/json/status"))
    except Exception:
        return None


def wait_for_state(states, timeout):
    end = time.time() + timeout
    while time.time() < end:
        st = controller_status()
        if st and st.get("state") in states:
            return st.get("state")
        time.sleep(0.5)
    return None


# --------------------------------------------------------------------------- #
# Data / packaging helpers
# --------------------------------------------------------------------------- #
def sanitize_name(name):
    name = (name or "").strip()
    name = re.sub(r"[^A-Za-z0-9._-]+", "_", name).strip("._")
    if not name:
        name = "test_" + time.strftime("%Y%m%d_%H%M%S")
    return name


def unique_path(path):
    if not os.path.exists(path):
        return path
    base, ext = os.path.splitext(path)
    i = 1
    while os.path.exists(f"{base}_{i}{ext}"):
        i += 1
    return f"{base}_{i}{ext}"


def clear_dir(d):
    """Delete the *contents* of d (keep the dir itself)."""
    if not os.path.isdir(d):
        return
    for name in os.listdir(d):
        p = os.path.join(d, name)
        try:
            if os.path.isdir(p) and not os.path.islink(p):
                shutil.rmtree(p, ignore_errors=True)
            else:
                os.remove(p)
        except OSError:
            pass


def pack_dir_to_zip(src_dir, zip_path):
    """Zip the whole src_dir into zip_path (STORED — .laz already compressed).

    Writes to a real seekable file, so memory stays low regardless of size.
    """
    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_STORED, allowZip64=True) as zf:
        for root, _dirs, files in os.walk(src_dir):
            for f in sorted(files):
                fp = os.path.join(root, f)
                try:
                    zf.write(fp, arcname=os.path.relpath(fp, src_dir))
                except OSError:
                    pass
    return zip_path


def newest_uart_log():
    files = glob.glob(os.path.join(UART_OUTDIR, "uart_*.log"))
    return max(files, key=os.path.getmtime) if files else None


def count_lines(path):
    if not path or not os.path.isfile(path):
        return 0
    n = 0
    try:
        with open(path, "rb") as f:
            while True:
                b = f.read(1 << 20)
                if not b:
                    break
                n += b.count(b"\n")
    except OSError:
        return 0
    return n


def list_packages():
    out = []
    if not os.path.isdir(PACKAGES_DIR):
        return out
    for fp in glob.glob(os.path.join(PACKAGES_DIR, "*.zip")):
        if os.path.isfile(fp):
            out.append({
                "name": os.path.basename(fp),
                "size_bytes": os.path.getsize(fp),
                "mtime": os.path.getmtime(fp),
            })
    return out


# --------------------------------------------------------------------------- #
# Recording lifecycle
# --------------------------------------------------------------------------- #
def start_uart_logger():
    return uart.start(
        ["python3", UART_SCRIPT, "--port", UART_PORT, "--baud", UART_BAUD,
         "--outdir", UART_OUTDIR, "--time-format", "epoch_ns", "--reconnect", "--no-echo"],
    )


def _arm_and_scan():
    """Background: wait for the lidar to be ready, then begin recording.

    The controller's own isReadyToScan() gate may reject start_bag until the
    lidar has warmed up and synced, so we poll and (re)issue start_bag until the
    state actually becomes SCANNING, or we give up after ARM_TIMEOUT.
    """
    end = time.time() + ARM_TIMEOUT
    while arming["active"] and time.time() < end:
        st = controller_status()
        if st:
            state = st.get("state")
            synced = bool((st.get("lidar") or {}).get("is_synced"))
            if state in ("SCANNING", "STARTING_SCAN"):
                scan_started_at["t"] = time.time()
                arming["active"] = False
                return
            if state in ("IDLE", "STOPPED") and synced:
                try:
                    controller_get("/trig/start_bag")
                except Exception as e:
                    arming["error"] = str(e)
                    arming["active"] = False
                    return
        time.sleep(0.5)
    if arming["active"]:
        arming["error"] = "timed out waiting for the lidar to be ready"
    arming["active"] = False


# --------------------------------------------------------------------------- #
# Flask app
# --------------------------------------------------------------------------- #
app = Flask(__name__)
HERE = os.path.dirname(os.path.abspath(__file__))


@app.route("/")
def index():
    return send_file(os.path.join(HERE, "index.html"))


@app.route("/api/measure/start", methods=["POST"])
def measure_start():
    body = request.get_json(silent=True) or {}
    name = sanitize_name(body.get("name"))
    with state_lock:
        if controller.running() or controller_reachable() or arming["active"]:
            return jsonify({"ok": False, "error": "a measurement is already running"}), 409

        # Fresh scratch dir for this test.
        os.makedirs(REPO, exist_ok=True)
        clear_dir(REPO)

        ld = os.pathsep.join(p for p in (laszip_lib_dir(), os.environ.get("LD_LIBRARY_PATH", "")) if p)
        if not controller.start([CONTROLLER_BIN], cwd=BUILD_DIR,
                                 extra_env={**CONTROLLER_ENV, "LD_LIBRARY_PATH": ld}):
            return jsonify({"ok": False, "error": f"controller: {controller.error}"}), 500

        start_uart_logger()
        # capture the log file this run writes to (for the live line count)
        uart_log = None
        for _ in range(20):
            uart_log = newest_uart_log()
            if uart_log:
                break
            time.sleep(0.1)

        current_test.update({"name": name, "uart_log": uart_log})
        scan_started_at["t"] = None
        arming.update({"active": True, "error": None})
        threading.Thread(target=_arm_and_scan, daemon=True).start()
    return jsonify({"ok": True, "name": name})


@app.route("/api/measure/stop", methods=["POST"])
def measure_stop():
    with state_lock:
        arming["active"] = False  # cancel any pending arm

        # Stop the scan and let the controller flush the final chunk.
        if controller_reachable():
            try:
                controller_get("/trig/stop_bag")
            except Exception:
                pass
            wait_for_state({"IDLE", "STOPPED"}, 15)

        controller.stop()
        uart.stop()
        scan_started_at["t"] = None

        # Pack everything, then wipe the scratch dir.
        name = current_test.get("name") or sanitize_name("")
        os.makedirs(PACKAGES_DIR, exist_ok=True)
        zip_path = unique_path(os.path.join(PACKAGES_DIR, name + ".zip"))
        try:
            pack_dir_to_zip(REPO, zip_path)
        except Exception as e:
            return jsonify({"ok": False, "error": f"packing failed: {e}"}), 500
        clear_dir(REPO)
        current_test.update({"name": None, "uart_log": None})
    return jsonify({"ok": True, "zip": os.path.basename(zip_path),
                    "size_bytes": os.path.getsize(zip_path)})


@app.route("/api/status")
def status():
    reachable = controller_reachable()
    ctrl = controller_status() if reachable else None
    du = shutil.disk_usage(REPO) if os.path.isdir(REPO) else None
    elapsed = (time.time() - scan_started_at["t"]) if scan_started_at["t"] else 0
    return jsonify({
        "controller_running": reachable,
        "uart_running": uart.running(),
        "arming": arming["active"],
        "arming_error": arming["error"],
        "test_name": current_test.get("name"),
        "uart_log_lines": count_lines(current_test.get("uart_log")),
        "scan_elapsed_s": round(elapsed, 1),
        "disk": {"total": du.total, "used": du.used, "free": du.free} if du else None,
        "repo": REPO,
        "controller": ctrl,
    })


@app.route("/api/sessions")
def sessions():
    return jsonify({"dir": PACKAGES_DIR, "sessions": list_packages()})


@app.route("/api/download")
def download():
    name = request.args.get("name", "")
    if not name.endswith(".zip") or "/" in name or "\\" in name or name in (".", ".."):
        return jsonify({"ok": False, "error": "invalid name"}), 400
    full = os.path.realpath(os.path.join(PACKAGES_DIR, name))
    if os.path.commonpath([full, os.path.realpath(PACKAGES_DIR)]) != os.path.realpath(PACKAGES_DIR) \
            or not os.path.isfile(full):
        return jsonify({"ok": False, "error": "not found"}), 404
    return send_file(full, as_attachment=True, download_name=name)


if __name__ == "__main__":
    os.makedirs(REPO, exist_ok=True)
    os.makedirs(PACKAGES_DIR, exist_ok=True)
    print(f"[measure_app] data (scratch) = {REPO}")
    print(f"[measure_app] packages       = {PACKAGES_DIR}")
    print(f"[measure_app] controller     = {CONTROLLER_BIN}")
    print(f"[measure_app] open http://<this-rpi-ip>:{APP_PORT}")
    app.run(host="0.0.0.0", port=APP_PORT, threaded=True)
