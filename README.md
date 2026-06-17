# MAVLink2MQTT
A Python-based communication bridge that wraps MAVLink commands in structured MQTT messages, enabling secure, 
bidirectional GCS–to–UAV communication over TLS-authenticated MQTT.

---

## Overview

The system translates MAVLink commands (issued by a GCS operator via an interactive CLI) into JSON-encoded MQTT 
messages, routes them through an Eclipse Mosquitto broker secured with mutual TLS, and dispatches them to an 
ArduPilot-based flight controller via DroneKit on the companion computer. Telemetry data (altitude, battery, 
autopilot status) is streamed back from drone to GCS over dedicated MQTT topics.

A **two-ACK architecture** separates communication latency from execution latency: 

- **Stage 1 — Immediate ACK** (`uav/{id}/ack`): published as soon as the drone bridge receives and accepts a command. 
RTT measures pure MQTT network latency.
- **Stage 2 — Completion notification** (`uav/{id}/completed`): published after the command has finished executing on 
the autopilot. RTT measures MQTT latency + drone-side execution time.

This gives two independent timing series from a single command exchange, logged to separate CSV files for analysis.

---

## Repository structure

```
MAVLink2MQTT/
│
├── drone_mqtt.py              # Drone-side MQTT bridge (DroneKit ↔ MQTT)
├── ground_station.py          # GCS interactive CLI (MQTT client)
├── metrics_logger.py          # CSV loggers for latency, completion, battery
│
├── utils/
│   ├── collect_rssi.py        # RSSI survey tool (CoreWLAN, macOS only)
│   ├── measure_avg_noise_hackrf.py  # Ambient noise floor via HackRF One
│   ├── measure_mac_snr.py     # Continuous RSSI/noise/SNR logger (CoreWLAN)
│   ├── plot_heatmap.py        # Wi-Fi RSSI heatmap over floor plan
│   ├── plot_scenario.py       # CDF and throughput plots per QoS/distance
│   ├── plot_wifi_summary.py   # RSSI/noise/SNR CDF plots per band
│   ├── run_drone_scenario.sh  # Launch drone-side processes
│   ├── run_gcs_scenario.sh    # Launch GCS-side processes
│   └── run_iperf_scenario.sh  # Run iperf3 TCP/UDP bandwidth tests
│
├── certs/                     # TLS certificates (not committed)
│   ├── ca.crt
│   ├── client.crt / client.key   # GCS credentials
│   └── drone.crt  / drone.key    # Drone credentials
│
└── logs/                      # Measurement output (auto-created, not committed)
    └── dist-<d>_tls-on_qos<n>_<band>/
        ├── latency_metrics.csv
        ├── completion_metrics.csv
        ├── battery_metrics.csv
        ├── ping_icmp.log
        ├── iperf_tcp.json
        └── iperf_udp_10M.json
```

---

## MQTT Topic Structure

| Topic | Direction | QoS | Content |
|---|---|---|---|
| `uav/{id}/cmd` | GCS → Drone | 0/1/2 | JSON command |
| `uav/{id}/ack` | Drone → GCS | mirrored | Immediate acceptance ACK |
| `uav/{id}/completed` | Drone → GCS | mirrored | Execution completion + result |
| `uav/{id}/status` | Drone → GCS | 1 (retained) | `online` / `offline` (LWT) |
| `uav/{id}/telemetry/altitude` | Drone → GCS | 0 | Altitude during climb/descent |
| `uav/{id}/telemetry/battery` | Drone → GCS | 0 | Voltage, current, level |
| `uav/{id}/telemetry/sys` | Drone → GCS | 0 | Autopilot STATUSTEXT messages |
| `uav/{id}/heartbeat` | GCS → Drone | 0 | GCS liveness signal |

---

## Prerequisites

**GCS (Ubuntu / Linux)**

```
Python >= 3.10
paho-mqtt
Eclipse Mosquitto (broker, with mutual TLS configured)
iperf3
```

**Drone / Companion Computer**

```
Python >= 3.10
paho-mqtt
dronekit
pymavlink
ArduPilot / SITL (for simulation)
```

**Optional (RF characterization)**

```
hackrf_sweep       (HackRF One, noise floor measurement)
pyobjc / CoreWLAN  (macOS only — RSSI/noise collection and heatmap generation)
iw                 (Linux — per-AP RSSI logging)
```

**Optional (RF characterization)**

```
hackrf_sweep   (HackRF One, noise floor measurement)
pyobjc / CoreWLAN  (macOS only, RSSI and Noise collection, heatmap generation)
iw             (Linux, per-AP RSSI logging)
```

---

## Running a measurement scenario

Each scenario is identified by a name encoding all experimental conditions:

```
dist-<distance>_tls-on_qos<level>_<band>
```

Example: `dist-5m_tls-on_qos1_2_4`

### 1 — Start SITL (on the companion computer)

```bash
sim_vehicle.py -v ArduCopter --console --map
```

### 2 — Start the drone-side processes

```bash
./utils/run_drone_scenario.sh dist-5m_tls-on_qos1
```

Starts the iperf3 server in the background and launches `drone_mqtt.py` in the foreground.

### 3 — Start the GCS-side processes

```bash
./utils/run_gcs_scenario.sh dist-5m_tls-on_qos1_2_4 10.42.0.20 wlo1
```

Starts ICMP ping logging in the background and launches the interactive GCS CLI in the foreground.

### 4 — Run iperf bandwidth tests (separate terminal, GCS side)

```bash
./utils/run_iperf_scenario.sh dist-4m_tls-on_qos1 10.42.0.20
```

Runs a 30 s TCP test followed by a 30 s UDP saturation test. 
This is QoS-agnostic and only needs to be run once per distance/TLS configuration.


---

## GCS CLI Commands

```
guided [-q 0|1|2]                          # switch to GUIDED mode
arm / disarm [-q 0|1|2]
takeoff <alt_m> [-q 0|1|2]
land / rtl [-q 0|1|2]

move <north|south|east|west|up|down> <speed_mps> <distance_m> [-q 0|1|2]
vel <vx> <vy> <vz> <duration_s> [-q 0|1|2]
yaw <heading_deg> [rate=30] [relative=0|1] [cw=1|0] [-q 0|1|2]
setspeed <mps> [-q 0|1|2]
goto <lat> <lon> <alt_m> [-q 0|1|2]

batt [-q 0|1|2]
qos <0|1|2>                                
```

The `-q` flag overrides the default QoS level for a single command. 
The `qos` command changes the default for all subsequent commands in the session.

---

## RF measurement tools

### Ambient noise floor (HackRF One)

```bash
python3 utils/measure_avg_noise_hackrf.py --band 2.4 --duration 60 --outfile noise_2.4.csv
python3 utils/measure_avg_noise_hackrf.py --band 5   --duration 60 --outfile noise_5.csv
```

Launches `hackrf_sweep` as a subprocess and estimates the noise floor using the 20th-percentile power bin across the sweep, averaged over the measurement window.

### Continuous RSSI/SNR logger (macOS)

```bash
python3 utils/measure_mac_snr.py dist-5m_tls-on_qos1_2_4
```

Logs RSSI, noise floor, and SNR at 1 Hz to `logs/<scenario>/mac_rssi_noise.csv`. 

> **Note:** on the 5 GHz band, macOS returns a fixed driver-level constant for the noise floor;
> SNR values at 5 GHz are therefore not meaningful.

### RSSI heatmap survey (macOS)

```bash
python3 utils/collect_rssi.py --out rssi_survey_2_4.csv --interval 0.5
```

Press **ENTER** at each physical measurement point to mark it. After the survey, annotate 
the pixel coordinates of each mark on the floor plan image and fill in `SURVEY_POINTS` in 
`plot_heatmap.py`, then run:

```bash
python3 utils/plot_heatmap.py
```

---

## Generating plots

All plotting scripts read from the `logs/` directory and write PNGs to a configurable 
output directory. Edit the `LOGS_DIR`, `OUT_DIR`, and `QOS` constants at the top of 
each script before running.

```bash
# CDF and throughput comparison across distances (one figure per metric)
python3 utils/plot_scenario.py

# RSSI / noise / SNR CDFs from CoreWLAN data
python3 utils/plot_wifi_summary.py

# RSSI heatmap over floor plan
python3 utils/plot_heatmap.py
```

---

## Log file formats

| File | Columns |
|---|---|
| `latency_metrics.csv` | `t_s, id, rtt_ms, lost` — immediate ACK RTT |
| `completion_metrics.csv` | `t_s, id, rtt_ms, lost` — execution completion RTT |
| `battery_metrics.csv` | `t_s, voltage_V, current_A, remaining_pct, mAh_consumed` |
| `ping_icmp.log` | Linux ping output with `-D` timestamps |
| `iperf_tcp.json` | iperf3 JSON (TCP) |
| `iperf_udp_10M.json` | iperf3 JSON (UDP) |
| `mac_rssi_noise.csv` | `t_s, rssi_dbm, noise_dbm, snr_db` |

In the latency and completion CSVs, `lost = 1` marks commands that timed out without 
receiving an ACK; `rtt_ms` is empty for those rows.

---

## Key Design Decisions

**Two-ACK latency separation** — `drone_mqtt.py` sends an immediate `ack` message as soon as
a command is received and parsed (measuring pure MQTT round-trip), and a separate completion 
notification once DroneKit confirms execution. This allows the two latency components to be 
stored and analyzed independently.

**Threaded command dispatch** — Commands that block the DroneKit event loop (e.g. `velocity`)
are dispatched in separate threads via `_handle_and_ack()`, preventing MQTT callback starvation
and broker disconnection during long-running maneuvers.

**Last Will and Testament (LWT)** — The broker publishes `offline` on `uav/{id}/status` automatically
if the drone client disconnects unexpectedly, enabling the GCS to detect link loss without polling.

**QoS per command** — The `-q` flag in the CLI allows per-command QoS override without changing 
the session default, facilitating comparative latency experiments at QoS 0, 1, and 2 without
restarting the GCS.

**Heartbeat watchdog** — A `heartbeat_watchdog` thread in `drone_mqtt.py` monitors GCS 
liveness and triggers an emergency LAND mode if no heartbeat is received within 
`HEARTBEAT_TIMEOUT` seconds. Currently disabled for SITL; should be enabled when transitioning
to real-hardware flights.

---

## Known Limitations

- The 5 GHz noise floor from CoreWLAN (`noiseMeasurement()`) is a fixed driver-level constant on macOS, not a
  live measurement; SNR computed from it is not reliable at 5 GHz.
- Experimental validation is based on SITL and a controlled laboratory Wi-Fi environment; results may differ
  under outdoor or multi-path propagation conditions.

---

## Troubleshooting

### `ModuleNotFoundError: No module named 'past'`

DroneKit depends on the `future` package, which provides the `past` module. Install it explicitly:

```bash
pip install future
```

### `TypeError: metaclass conflict` or `MutableMapping` error in DroneKit

Python 3.10+ removed several aliases from `collections` that DroneKit relies on. Open DroneKit's `__init__.py`
(usually at `<venv>/lib/pythonX.Y/site-packages/dronekit/__init__.py`) and update the `Parameters` class
declaration:

```python
# Before
class Parameters(collections.MutableMapping, HasObservers):

# After
class Parameters(collections.abc.MutableMapping, HasObservers):
```

### `SerialException` on vehicle connect

DroneKit requires `pyserial` even when connecting over UDP/TCP. Install it explicitly:

```bash
pip install pyserial
```

### TLS handshake fails or broker refuses connection

Do **not** set a PEM passphrase when generating certificates. Mosquitto and Paho-MQTT expect unencrypted
private keys; a passphrase-protected key will cause the TLS handshake to fail at startup.

```bash
# Correct — no passphrase
openssl genrsa -out client.key 2048

# Wrong — passphrase-protected key will break the connection
openssl genrsa -des3 -out client.key 2048
```

---

## License

This project is released for academic purposes. See `LICENSE` for details.
