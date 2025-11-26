#!/usr/bin/env bash
set -euo pipefail

# === CONFIG ===
VENV_DIR="./.venv"

SCENARIO="${1:-}"
DRONE_IP="${2:-}"
IFACE="${3:-wlan0}"

if [ -z "$SCENARIO" ] || [ -z "$DRONE_IP" ]; then
  echo "Usage: $0 <scenario_name> <drone_ip> [wifi_iface]"
  exit 1
fi

# === ACTIVATE VENV ===
if [ ! -d "$VENV_DIR" ]; then
  echo "[ERROR] Virtual environment not found: $VENV_DIR"
  exit 1
fi
source "$VENV_DIR/bin/activate"

# === LOG DIRECTORY ===
export ML2MQTT_LOG_DIR="./logs/$SCENARIO"
mkdir -p "$ML2MQTT_LOG_DIR"

echo "[GCS] Scenario: $SCENARIO"
echo "[GCS] ML2MQTT_LOG_DIR=$ML2MQTT_LOG_DIR"
echo "[GCS] Drone IP: $DRONE_IP"
echo "[GCS] Wi-Fi iface: $IFACE"
echo "[GCS] Using Python from: $(which python3)"

# Quando esci, prova a killare i job in background (ping + RSSI)
trap 'echo "[GCS] cleanup bg jobs"; kill 0 || true' EXIT

# --- Ping ICMP ---
ping "$DRONE_IP" -i 0.2 -D > "$ML2MQTT_LOG_DIR/ping_icmp.log" 2>&1 &

# --- Logger RSSI/Wi-Fi ogni 1s ---
(
  if [ ! -d "/sys/class/net/$IFACE" ]; then
    echo "[GCS] Wi-Fi iface '$IFACE' not found, skipping RSSI logging"
    exit 0
  fi

  while true; do
    TS=$(date +%s.%N)
    OUT=$(iw dev "$IFACE" link 2>/dev/null | grep "signal:" || true)
    echo "$TS,$OUT" >> "$ML2MQTT_LOG_DIR/wifi_rssi.log"
    sleep 1
  done
) &

# --- Avvia la GCS (CLI) in foreground ---
python3 gcs.py
