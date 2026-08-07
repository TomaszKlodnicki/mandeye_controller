# Mandeye Measure — simple control panel

A tiny Flask web app to run a measurement session end-to-end from a phone or
laptop browser over wifi. It starts/stops the mandeye `control_program` and the
UART logger together, shows live scan progress, and lets you download recorded
sessions as zip files.

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

## What the buttons do

| Button | Action |
|--------|--------|
| **Start system** | Launches `control_program` (Unitree env) + `uart_logger.py` |
| **Stop system**  | Stops any active scan, then stops both processes |
| **▶ Start**      | `POST /trig/start_bag` — begins recording |
| **⏹ Stop**       | `POST /trig/stop_bag` — ends recording |
| **Download**     | Streams a session directory as a `.zip` |

The status panel polls the controller's `/json/status` once a second and shows
the state, sync status, point/IMU counters, elapsed time and free disk space.

## Configuration (env vars, all optional)

Defaults match the Unitree L2 + RPi 5 setup.

| Var | Default |
|-----|---------|
| `MANDEYE_REPO` | `~/mandeye_data` |
| `MANDEYE_BUILD_DIR` | `~/mandeye_controller/build` |
| `MANDEYE_CONTROLLER_BIN` | `<build>/control_program` |
| `MANDEYE_CONTROLLER_URL` | `http://127.0.0.1:8003` |
| `UART_LOGGER` | `~/mandeye_controller/tools/uart_logger.py` |
| `UART_PORT` / `UART_BAUD` | `/dev/ttyAMA0` / `115200` |
| `MEASURE_APP_PORT` | `8080` |
| `MANDEYE_LIDAR_SDK`, `MANDEYE_LIVOX_LISTEN_IP`, `MANDEYE_GPIO_SIM` | `UNITREE`, `192.168.123.120`, `1` |

## Notes

- If the mandeye controller is already running (e.g. via systemd on port 8003),
  the app detects it and just drives it — it won't spawn a second copy. "Stop
  system" can only kill a controller this app started.
- UART logs are written into `MANDEYE_REPO/uart_logs` (epoch-ns timestamps) so
  they are included when you download.
- The controller needs read/write access to `/dev/ttyAMA0` and the repo; make
  sure the udev rule / `dialout` group is set up (see project docs) so you don't
  need root.
- The page is plain HTML/JS (no build step, works offline). To use React
  instead, drop React + Babel `<script>` tags in `index.html` — the API is
  unchanged.
