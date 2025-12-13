#!/usr/bin/env bash
set -euo pipefail

# === CONFIG ===
VENV_DIR="./.venv"

# ./run_gcs_scenario.sh <name scenario> <IP Drone> <iface> <avg_noise>

SCENARIO="${1:-}"
DRONE_IP="${2:-}"
IFACE="${3:-}"
NOISE="${4:-}"

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

# Quando esci, prova a killare i job in background (ping)
trap 'echo "[GCS] cleanup bg jobs"; kill 0 || true' EXIT

# --- Ping ICMP ---
ping "$DRONE_IP" -i 0.2 -D > "$ML2MQTT_LOG_DIR/ping_icmp.log" 2>&1 &

(
  OUT="$ML2MQTT_LOG_DIR/wifi_rssi.log"
  IFACE="wlo1"

  while true; do
      TS=$(date +%s.%N)

      LINE_SIG=$(iw dev "$IFACE" link 2>/dev/null | grep "signal:" || true)
      SIG=""
      if [ -n "$LINE_SIG" ]; then
          SIG=$(echo "$LINE_SIG" | sed -E 's/.*signal level=([-0-9]+)\s*dBm.*/\1/')
      fi

      if [ -n "$SIG" ]; then
          SNR=$((SIG - NOISE))
          echo "$TS,$SIG,$NOISE,$SNR" >> "$OUT"
      fi

      sleep 1
  done
) &


# --- Avvia la GCS (CLI) in foreground ---
python3 gcs.py
