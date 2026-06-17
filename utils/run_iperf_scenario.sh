#!/usr/bin/env bash
# run_iperf_scenario.sh

# Run TCP and UDP throughput tests against the iperf3 server running on the
# drone companion computer (started by run_drone_scenario.sh).
#
# Throughput is independent of MQTT QoS level because iperf3 uses its own
# TCP/UDP sockets and does not interact with the MQTT broker; a single run
# per distance/TLS configuration is therefore sufficient.
#
# Usage:
#   ./run_iperf_scenario.sh <scenario_name> <drone_ip>
# Example:
#   ./run_iperf_scenario.sh dist-5m_tls-on 10.42.0.20

# -e  exit immediately if any command returns a non-zero status.
# -u  treat unset variables as an error (prevents silent bugs from typos).
# -o pipefail  a pipeline fails if any command in it fails, not just the last.
set -euo pipefail

VENV_DIR="./.venv"

SCENARIO="${1:-}"   # e.g. dist-5m_tls-on  (no QoS suffix: test is QoS-agnostic)
DRONE_IP="${2:-}"   # IP address of the companion computer running the iperf3 server

if [ -z "$SCENARIO" ] || [ -z "$DRONE_IP" ]; then
    echo "Usage: $0 <scenario_name> <drone_ip>"
    exit 1
fi

# === Activate Python virtual environment ===
# The venv is activated for consistency with the other scenario scripts even
# though iperf3 is a system binary and does not require Python packages.
if [ ! -d "$VENV_DIR" ]; then
  echo "[ERROR] Virtual environment not found: $VENV_DIR"
  exit 1
fi
source "$VENV_DIR/bin/activate"

# === Create log directory ===
export ML2MQTT_LOG_DIR="./logs/$SCENARIO"
mkdir -p "$ML2MQTT_LOG_DIR"

echo "[IPERF] Scenario: $SCENARIO"
echo "[IPERF] ML2MQTT_LOG_DIR=$ML2MQTT_LOG_DIR"
echo "[IPERF] Drone IP: $DRONE_IP"

# === iperf3 tests ===
# Both tests run sequentially (not in parallel) to avoid contending for the
# same Wi-Fi channel bandwidth and corrupting each other's measurements.
# The JSON output (-J) is written to separate files so that
# parse_iperf_tcp() and parse_iperf_udp() in plot_gcs_metrics.py and
# plot_scenario.py can read them independently.

# TCP test (30 s)
# -c  client mode: connect to the iperf3 server on DRONE_IP (default port 5201).
# -J  output results as a single JSON object, including per-interval throughput
#     and end-to-end summary (average Mbps, retransmits, RTT min/mean/max).
# -t  test duration in seconds.
iperf3 -c "$DRONE_IP" -J -t 30 > "$ML2MQTT_LOG_DIR/iperf_tcp.json"

# UDP test at 100 Mbps target rate (30 s)
# -u  UDP mode (default is TCP).
# -b  target send bitrate; 100M sets the sender to attempt 100 Mbit/s.
#     The actual throughput will be limited by the Wi-Fi link capacity,
#     making this effectively a saturation test to measure peak UDP delivery.
# Results include jitter and packet-loss statistics not available in TCP mode.
iperf3 -c "$DRONE_IP" -u -b 100M -J -t 30 > "$ML2MQTT_LOG_DIR/iperf_udp_10M.json"
