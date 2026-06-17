#!/usr/bin/env bash
# run_drone_scenario.sh

# Set up and launch the drone-side components for one measurement scenario.
# Starts an iperf3 server in the background (for bandwidth tests initiated
# from the GCS) and then runs the drone_mqtt.py bridge in the foreground.
#
# Usage:
#   ./run_drone_scenario.sh <scenario_name>
# Example:
#   ./run_drone_scenario.sh dist-5m_tls-on_qos1

# -e  exit immediately if any command returns a non-zero status.
# -u  treat unset variables as an error (prevents silent bugs from typos).
# -o pipefail  a pipeline fails if any command in it fails, not just the last.
set -euo pipefail

VENV_DIR="/home/tesista/venv-ardupilot"

# The scenario name encodes all experimental conditions and is used as the
# log subdirectory name, keeping results from different runs fully separated.
# Example: dist-5m_tls-on_qos1
SCENARIO="${1:-}"
if [ -z "$SCENARIO" ]; then
  echo "Usage: $0 <scenario_name>"
  exit 1
fi

# === Activate Python virtual environment ===
if [ ! -d "$VENV_DIR" ]; then
  echo "[ERROR] Virtual environment not found: $VENV_DIR"
  exit 1
fi
source "$VENV_DIR/bin/activate"

# === Create log directory and export path ===
# Exporting ML2MQTT_LOG_DIR as an environment variable makes it available
# to drone_mqtt.py and any other child processes without passing it as an
# argument, keeping the launch command simple.
export ML2MQTT_LOG_DIR="./logs/$SCENARIO"
mkdir -p "$ML2MQTT_LOG_DIR"

echo "[UAV] Scenario: $SCENARIO"
echo "[UAV] ML2MQTT_LOG_DIR=$ML2MQTT_LOG_DIR"

# === Cleanup trap ===
# 'kill 0' sends SIGTERM to every process in the current process group,
# which includes iperf3 and any other background jobs started by this script.
# '|| true' prevents the trap itself from triggering the -e exit-on-error
# behaviour if kill finds no processes to terminate (e.g. they already exited).
trap 'echo "[UAV] cleanup bg jobs"; kill 0 || true' EXIT

# === Start iperf3 server (background) ===
# The server listens for incoming TCP/UDP connections from the GCS-side
# iperf3 client (run_iperf_scenario.sh) and logs all output to a file.
# Running in the background (&) allows drone_mqtt.py to start immediately
# after; both processes run concurrently for the duration of the scenario.
iperf3 -s > "$ML2MQTT_LOG_DIR/iperf_server.log" 2>&1 &

# === Start the drone MQTT bridge (foreground) ===
# Running in the foreground means the script blocks here until drone_mqtt.py
# exits (e.g. via Ctrl+C), at which point the EXIT trap fires and terminates
# the background iperf3 server.
python3 drone_mqtt.py
