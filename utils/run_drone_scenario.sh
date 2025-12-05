#!/usr/bin/env bash
set -euo pipefail

# per eseguire: ./run_drone_scenario.sh dist-5m_tls-on_qos1

VENV_DIR="/home/tesista/venv-ardupilot"

# es. scenario_name = dist-5m_tls-on_qos1
SCENARIO="${1:-}"
if [ -z "$SCENARIO" ]; then
  echo "Usage: $0 <scenario_name>"
  exit 1
fi

# === ACTIVATE VENV ===
if [ ! -d "$VENV_DIR" ]; then
  echo "[ERROR] Virtual environment not found: $VENV_DIR"
  exit 1
fi
source "$VENV_DIR/bin/activate"

# cartella log per questo scenario
export ML2MQTT_LOG_DIR="./logs/$SCENARIO"
mkdir -p "$ML2MQTT_LOG_DIR"

echo "[UAV] Scenario: $SCENARIO"
echo "[UAV] ML2MQTT_LOG_DIR=$ML2MQTT_LOG_DIR"

# Quando esci, prova a killare i job in background (ping)
trap 'echo "[UAV] cleanup bg jobs"; kill 0 || true' EXIT

# avvia iperf3 server (per i test di banda)
iperf3 -s > "$ML2MQTT_LOG_DIR/iperf_server.log" 2>&1 &

# avvia il bridge drone↔MQTT (foreground)
python3 drone_mqtt.py
