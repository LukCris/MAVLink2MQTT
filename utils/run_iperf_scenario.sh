#!/usr/bin/env bash
set -euo pipefail

# ./run_iperf_scenario.sh dist-5m_tls-on_qos1 10.42.0.20


# === CONFIG ===
VENV_DIR="./.venv"

SCENARIO="${1:-}"
DRONE_IP="${2:-}"

if [ -z "$SCENARIO" ] || [ -z "$DRONE_IP" ]; then
    echo "Usage: $0 <scenario_name> <drone_ip>"
    exit 1
fi

# === ACTIVATE VENV ===
if [ ! -d "$VENV_DIR" ]; then
  echo "[ERROR] Virtual environment not found: $VENV_DIR"
  exit 1
fi
source "$VENV_DIR/bin/activate"

export ML2MQTT_LOG_DIR="./logs/$SCENARIO"
mkdir -p "$ML2MQTT_LOG_DIR"

echo "[IPERF] Scenario: $SCENARIO"
echo "[IPERF] ML2MQTT_LOG_DIR=$ML2MQTT_LOG_DIR"
echo "[IPERF] Drone IP: $DRONE_IP"

# === IPERF3 TESTS ===

# TCP (30 secondi)
iperf3 -c "$DRONE_IP" -J -t 30 > "$ML2MQTT_LOG_DIR/iperf_tcp.json"

# UDP 10 Mbps (30 secondi)
iperf3 -c "$DRONE_IP" -u -b 10M -J -t 30 > "$ML2MQTT_LOG_DIR/iperf_udp_10M.json"
