#!/usr/bin/env bash
# run_gcs_scenario.sh

# Set up and launch the GCS-side components for one measurement scenario.
# Starts an ICMP ping logger in the background (for round-trip latency
# baseline measurements) and then runs the ground_station.py CLI in the
# foreground.
#
# Usage:
#   ./run_gcs_scenario.sh <scenario_name> <drone_ip> [wifi_iface]
# Example:
#   ./run_gcs_scenario.sh dist-5m_tls-on_qos1 10.42.0.20 wlo1

# -e  exit immediately if any command returns a non-zero status.
# -u  treat unset variables as an error (prevents silent bugs from typos).
# -o pipefail  a pipeline fails if any command in it fails, not just the last.
set -euo pipefail

VENV_DIR="./.venv"

SCENARIO="${1:-}"   # e.g. dist-5m_tls-on_qos1
DRONE_IP="${2:-}"   # IP address of the companion computer running drone_mqtt.py
IFACE="${3:-}"      # Wi-Fi interface name (e.g. wlo1); optional, informational only

if [ -z "$SCENARIO" ] || [ -z "$DRONE_IP" ]; then
  echo "Usage: $0 <scenario_name> <drone_ip> [wifi_iface]"
  exit 1
fi

# === Activate Python virtual environment ===
if [ ! -d "$VENV_DIR" ]; then
  echo "[ERROR] Virtual environment not found: $VENV_DIR"
  exit 1
fi
source "$VENV_DIR/bin/activate"

# === Create log directory and export path ===
# ML2MQTT_LOG_DIR is read by ground_station.py (via os.environ) to determine
# where to write the latency and completion CSV files, keeping all outputs
# for a given scenario in the same directory.
export ML2MQTT_LOG_DIR="./logs/$SCENARIO"
mkdir -p "$ML2MQTT_LOG_DIR"

echo "[GCS] Scenario: $SCENARIO"
echo "[GCS] ML2MQTT_LOG_DIR=$ML2MQTT_LOG_DIR"
echo "[GCS] Drone IP: $DRONE_IP"
echo "[GCS] Wi-Fi iface: $IFACE"
echo "[GCS] Using Python from: $(which python3)"

# === Cleanup trap ===
# Fires on any exit (normal, Ctrl+C, or error).
# 'kill 0' sends SIGTERM to all processes in the current process group,
# terminating the background ping process alongside the script itself.
# '|| true' suppresses errors if no background jobs remain to be killed.
trap 'echo "[GCS] cleanup bg jobs"; kill 0 || true' EXIT

# === Start ICMP ping logger (background) ===
# Provides a continuous baseline RTT measurement independent of MQTT,
# used to separate network-layer latency from MQTT protocol overhead.
#   -i 0.2  send one ping every 0.2 s (5 Hz) for finer time resolution
#            than the default 1 Hz, matching the MQTT command rate.
#   -D      prefix each output line with a Unix timestamp in brackets
#            (e.g. [1700000000.123456]), which parse_ping() in
#            plot_gcs_metrics.py and plot_scenario.py relies on.
ping "$DRONE_IP" -i 0.2 -D > "$ML2MQTT_LOG_DIR/ping_icmp.log" 2>&1 &

# === Start the GCS CLI (foreground) ===
# Running in the foreground means the script blocks here until the operator
# exits ground_station.py (Ctrl+C or EOF), at which point the EXIT trap
# fires and terminates the background ping process.
python3 ground_station.py