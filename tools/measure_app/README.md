# Mandeye Measure — simple control panel

A tiny Flask web app to run a measurement end-to-end from a phone or laptop
browser over wifi. Each test owns the whole data dir, so the flow is simple and
robust:

1. **Start** — wipes the scratch data dir, boots `control_program`, waits for
   the lidar to warm up + sync, then begins recording **and** the UART logger.
2. **Stop & pack** — stops the scan, stops controller + logger, zips *all* of
   the data dir into `<test_name>.zip` (stored outside the data dir), then wipes
   the data dir for the next test.

Live progress (state, sync, lidar/IMU counters, UART line count, elapsed) polls
once a second. Packaged tests appear under **Recorded data — download**.

## Install + auto-start on boot (recommended)

```bash
cd ~/mandeye_controller/tools/measure_app
sudo bash install.sh
```

This installs the dependencies (into a local `venv`), applies the
`/dev/ttyAMA0` UART + permission setup, and installs a **systemd service** so
the panel starts automatically after every boot. Then open
`http://<rpi-ip>:8080` (find the IP with `hostname -I`).

- Skip the UART / boot-config changes with `sudo bash install.sh --no-uart`.
- If the installer added boot-config lines, **reboot once** so `/dev/ttyAMA0`
  and the `dialout` group take effect.

Service management:

```bash
systemctl status mandeye_measure       # is it running?
journalctl -u mandeye_measure -f       # live logs
sudo systemctl restart mandeye_measure # restart after changes
```

## Run manually (without installing the service)

```bash
pip3 install -r requirements.txt
python3 app.py
# open http://<rpi-ip>:8080 from any device on the same network
```

## What the controls do

| Control | Action |
|---------|--------|
| **Test name** | Names the resulting `<test_name>.zip` (auto `test_<timestamp>` if blank; sanitized; collisions get `_1`, `_2`, …) |
| **▶ Start** | Wipe data dir → boot `control_program` → wait for lidar-ready → `start_bag` + UART logger |
| **⏹ Stop & pack** | `stop_bag` → stop controller + logger → zip the whole data dir → wipe it |
| **Download** | Downloads a packaged `<test>.zip` |

The status panel polls `/json/status` once a second: state (incl. "WARMING UP…"),
sync, point/IMU counters, **UART log line count**, elapsed time, free disk.

## Configuration (env vars, all optional)

Defaults match the Unitree L2 + RPi 5 setup.

| Var | Default | Meaning |
|-----|---------|---------|
| `MANDEYE_REPO` | `~/mandeye_data` | scratch data dir — **wiped** at start & after packing |
| `MANDEYE_PACKAGES` | `~/mandeye_tests` | where `<test>.zip` files are kept (never wiped) |
| `MANDEYE_BUILD_DIR` | `~/mandeye_controller/build` | |
| `MANDEYE_CONTROLLER_BIN` | `<build>/control_program` | |
| `MANDEYE_CONTROLLER_URL` | `http://127.0.0.1:8003` | |
| `UART_LOGGER` | `~/mandeye_controller/tools/uart_logger.py` | |
| `UART_PORT` / `UART_BAUD` | `/dev/ttyAMA0` / `115200` | |
| `MANDEYE_ARM_TIMEOUT` | `120` | max seconds to wait for lidar-ready before giving up |
| `MEASURE_APP_PORT` | `8080` | |
| `MANDEYE_LIDAR_SDK`, `MANDEYE_LIVOX_LISTEN_IP`, `MANDEYE_GPIO_SIM` | `UNITREE`, `192.168.123.120`, `1` | |

## Notes

- **The data dir is scratch** and is wiped on Start and after packing. Anything
  you want to keep lives in the packaged zips under `MANDEYE_PACKAGES`.
- Each test is a full controller run, so the lidar warm-up + `isReadyToScan()`
  gate guarantees the first frames are already clean — no "delete first N
  batches" in post-processing.
- UART logs are written into the data dir (`uart_logs/`, epoch-ns timestamps),
  so they are always packed alongside the scan data.
- The controller needs read/write access to `/dev/ttyAMA0` and the data dir;
  the installer's udev rule / `dialout` setup handles this so you don't need root.
- The page is plain HTML/JS (no build step, works offline). To use React
  instead, drop React + Babel `<script>` tags in `index.html` — the API is
  unchanged.
