#!/usr/bin/env bash
#
# Installer for the Mandeye Measure control panel.
#
#   * installs dependencies (python venv + Flask + pyserial),
#   * (optional) applies the UART/permission setup for /dev/ttyAMA0,
#   * installs a systemd service so the panel auto-starts on boot.
#
# Usage:
#   sudo bash install.sh              # full install (recommended)
#   sudo bash install.sh --no-uart    # skip the /dev/ttyAMA0 + boot-config changes
#
set -euo pipefail

SERVICE_NAME="mandeye_measure"
DO_UART=1
[[ "${1:-}" == "--no-uart" ]] && DO_UART=0

# --------------------------------------------------------------------------- #
# Resolve paths and the target (non-root) user
# --------------------------------------------------------------------------- #
if [[ "$(id -u)" -ne 0 ]]; then
  echo "Please run as root:  sudo bash install.sh" >&2
  exit 1
fi

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # .../tools/measure_app
REPO_DIR="$(cd "$APP_DIR/../.." && pwd)"                   # repo root
TARGET_USER="${SUDO_USER:-root}"
USER_HOME="$(getent passwd "$TARGET_USER" | cut -d: -f6)"
MANDEYE_REPO="${MANDEYE_REPO:-$USER_HOME/mandeye_data}"
BUILD_DIR="$REPO_DIR/build"

echo "==> App dir      : $APP_DIR"
echo "==> Repo dir     : $REPO_DIR"
echo "==> Run as user  : $TARGET_USER ($USER_HOME)"
echo "==> Data repo    : $MANDEYE_REPO"
echo "==> Build dir    : $BUILD_DIR"
[[ -x "$BUILD_DIR/control_program" ]] || \
  echo "    WARNING: $BUILD_DIR/control_program not found — build the controller first."

# --------------------------------------------------------------------------- #
# 1. System dependencies
# --------------------------------------------------------------------------- #
echo "==> Installing system packages..."
apt-get update -y
apt-get install -y python3 python3-venv python3-pip

# --------------------------------------------------------------------------- #
# 2. Python virtualenv + requirements (owned by the target user)
# --------------------------------------------------------------------------- #
echo "==> Creating virtualenv at $APP_DIR/venv ..."
sudo -u "$TARGET_USER" python3 -m venv "$APP_DIR/venv"
sudo -u "$TARGET_USER" "$APP_DIR/venv/bin/pip" install --upgrade pip >/dev/null
sudo -u "$TARGET_USER" "$APP_DIR/venv/bin/pip" install -r "$APP_DIR/requirements.txt"

# Make sure the data repo exists and is owned by the user
mkdir -p "$MANDEYE_REPO"
chown "$TARGET_USER":"$TARGET_USER" "$MANDEYE_REPO" || true

# --------------------------------------------------------------------------- #
# 3. UART + permissions (optional)
# --------------------------------------------------------------------------- #
if [[ "$DO_UART" -eq 1 ]]; then
  echo "==> Configuring UART on GPIO14/15 (/dev/ttyAMA0)..."

  # 3a. Enable the GPIO UART in the boot config (Pi 5 needs uart0 explicitly).
  CONFIG_TXT=/boot/firmware/config.txt
  [[ -f "$CONFIG_TXT" ]] || CONFIG_TXT=/boot/config.txt
  if [[ -f "$CONFIG_TXT" ]]; then
    for line in "enable_uart=1" "dtparam=uart0=on"; do
      if ! grep -qxF "$line" "$CONFIG_TXT"; then
        echo "$line" >> "$CONFIG_TXT"
        echo "    added '$line' to $CONFIG_TXT (reboot required)"
      fi
    done
  else
    echo "    WARNING: no config.txt found; enable the UART manually."
  fi

  # 3b. Stop the serial login console from grabbing the port.
  systemctl stop    serial-getty@ttyAMA0.service 2>/dev/null || true
  systemctl disable serial-getty@ttyAMA0.service 2>/dev/null || true
  systemctl mask    serial-getty@ttyAMA0.service 2>/dev/null || true

  # 3c. udev rule so the port is group-dialout / 0660 (no root needed).
  echo 'SUBSYSTEM=="tty", KERNEL=="ttyAMA0", GROUP="dialout", MODE="0660"' \
    > /etc/udev/rules.d/99-ttyAMA0.rules
  udevadm control --reload-rules || true
  udevadm trigger --action=add --sysname-match=ttyAMA0 || true

  # 3d. Make sure the user can open serial ports.
  usermod -aG dialout "$TARGET_USER" || true
else
  echo "==> Skipping UART setup (--no-uart)"
fi

# --------------------------------------------------------------------------- #
# 4. systemd service — auto-start the panel on boot
# --------------------------------------------------------------------------- #
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}.service"
echo "==> Writing $SERVICE_PATH ..."
cat > "$SERVICE_PATH" <<EOF
[Unit]
Description=Mandeye Measure control panel
After=network-online.target
Wants=network-online.target

[Service]
User=$TARGET_USER
WorkingDirectory=$APP_DIR
Environment=MANDEYE_REPO=$MANDEYE_REPO
Environment=MANDEYE_BUILD_DIR=$BUILD_DIR
ExecStart=$APP_DIR/venv/bin/python $APP_DIR/app.py
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable "$SERVICE_NAME"
systemctl restart "$SERVICE_NAME"

# --------------------------------------------------------------------------- #
# Done
# --------------------------------------------------------------------------- #
IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
echo
echo "=================================================================="
echo " Mandeye Measure installed and enabled on boot."
echo "   Service : systemctl status $SERVICE_NAME"
echo "   Logs    : journalctl -u $SERVICE_NAME -f"
echo "   Open    : http://${IP:-<rpi-ip>}:8080"
if [[ "$DO_UART" -eq 1 ]]; then
  echo
  echo " NOTE: if boot-config lines were added, REBOOT once so /dev/ttyAMA0"
  echo "       and the dialout group take effect:   sudo reboot"
fi
echo "=================================================================="
