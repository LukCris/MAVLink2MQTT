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

  OUT="$ML2MQTT_LOG_DIR/wifi_snr.log"

  # header
  echo "ts,signal_dbm,noise_db,est_snr_db" >> "$OUT"

  while true; do
    TS=$(date +%s.%N)

    # Prendi RSSI dal lato AP (station dump)
    SIG=$(iw dev "$IFACE" station dump 2>/dev/null | awk '
      /signal avg:/ {print $3; exit}
      /^signal:/ {print $2; exit}
    ')

    if [[ -z "$SIG" ]]; then
      echo "$TS,,${NOISE:-},"
      sleep 1
      continue
    fi

    NOISE_VAL="${NOISE:--69}"

    SNR=$(( SIG - NOISE_VAL ))
    echo "$TS,$SIG,$NOISE_VAL,$SNR" >> "$OUT"

    sleep 1
  done
) &


# --- Avvia la GCS (CLI) in foreground ---
python3 ground_station.py