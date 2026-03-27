# MAVLink2MQTT
A lightweight communication bridge between an ArduPilot-based drone and a 
Ground Control Station (GCS) over MQTT. Designed for experimental evaluation of 
network parameters (latency, throughput, QoS, TLS overhead) in UAV command-and-control scenarios.

---

## Architecture

```
┌──────────────────────────────────┐        Wi-Fi / TLS        ┌──────────────────────────────────┐
│           DRONE SIDE             │ ◄───────────────────────  │           GCS SIDE               │
│                                  │                           │                                  │
│  ArduPilot / SITL                │        MQTT Broker        │  ground_station.py               │
│        │                         │        (Mosquitto)        │    - Interactive CLI             │
│  drone_mqtt.py                   │      port 8883 (TLS)      │    - Publishes commands          │
│    - DroneKit bridge             │                           │    - Awaits ACKs                 │
│    - Handles commands            │                           │    - Logs RTT latency            │
│    - Publishes telemetry         │                           │    - Displays telemetry          │
│    - Battery monitoring          │                           │                                  │
└──────────────────────────────────┘                           └──────────────────────────────────┘
```

### MQTT Topics

| Topic | Direction | Description |
|---|---|---|
| `uav/{id}/cmd` | GCS → Drone | JSON command messages |
| `uav/{id}/ack` | Drone → GCS | Command acknowledgements |
| `uav/{id}/status` | Drone → GCS | Online / offline LWT |
| `uav/{id}/telemetry/altitude` | Drone → GCS | Altitude during takeoff/landing |
| `uav/{id}/telemetry/battery` | Drone → GCS | Voltage, current, level |
| `uav/{id}/telemetry/sys` | Drone → GCS | ArduPilot STATUSTEXT forwarding |
| `uav/{id}/mav/tx` | GCS → Drone | Raw MAVLink passthrough (base64) |

---

## Features

- **mTLS support**: mutual TLS authentication between drone, broker and GCS
- **Configurable QoS**: switch between QoS 0, 1, 2 per-command at runtime
- **Full command set**: mode, arm/disarm, takeoff, move, velocity, yaw, goto, speed, param get/set
- **Telemetry streaming**: battery, altitude (during manoeuvres), autopilot status messages
- **Latency metrics**: round-trip time logged for every command/ACK pair
- **ArduPilot SITL compatible**: connect to a simulated vehicle for testing

---

## Repository Structure

```
.
├── drone_mqtt.py               # Drone-side bridge (DroneKit ↔ MQTT)
├── ground_station.py           # GCS interactive CLI
├── metrics_logger.py           # Latency & battery CSV loggers
│
└── utils/
    ├── run_drone_scenario.sh   # Launch drone bridge with log dir
    ├── run_gcs_scenario.sh     # Launch GCS + ping + RSSI logging
    ├── run_iperf_scenario.sh   # Run iperf3 TCP + UDP tests
    ├── plot_gcs_metrics.py     # Plot ping, MQTT latency, iperf results
    ├── plot_drone_metrics.py   # Plot battery metrics
    └── measure_avg_noise_hackrf.py  # RF noise floor via HackRF One
```

---

## Requirements

### Drone side
- Python 3.8+
- `dronekit`
- `paho-mqtt`
- `pymavlink`
- ArduPilot SITL (or real flight controller)

### GCS side
- Python 3.8+
- `paho-mqtt`

### Analysis tools
- `pandas`, `matplotlib`
- `iperf3` (CLI)
- `hackrf_sweep` (optional, for RF noise measurement)

Install Python dependencies:
```bash
pip install dronekit paho-mqtt pymavlink pandas matplotlib
```

---

## Quick Start

### 1. Drone side

```bash
# On the drone (or SITL machine)
python3 drone_mqtt.py
```

Or use the scenario script:
```bash
./utils/run_drone_scenario.sh dist-5m_tls-on_qos1
```

### 2. GCS side

```bash
# On the ground station
python3 ground_station.py
```

Or use the scenario script (includes automatic ping and RSSI logging):
```bash
./utils/run_gcs_scenario.sh dist-5m_tls-on_qos1 10.42.0.20 wlo1 -85
```

### 3. GCS CLI Commands

```
guided              # Switch to GUIDED mode
arm                 # Arm the drone
takeoff 10          # Take off to 10 m
move north 3 20     # Move north at 3 m/s for 20 m
yaw 90 30           # Rotate to heading 90° at 30°/s
goto <lat> <lon> <alt>
land
rtl
batt                # Query battery status
```

Append `-q 0|1|2` to any command to override QoS for that specific message:
```
takeoff 15 -q 0
move east 2 10 -q 2
```

---

## Network Testing

### iperf3

```bash
./utils/run_iperf_scenario.sh dist-5m_tls-on_qos1 10.42.0.20
```

Runs a 30-second TCP test and a 30-second UDP test at 10 Mbps, saving results to `logs/<scenario>/`.

### RF Noise Floor (HackRF)

```bash
python3 utils/measure_avg_noise_hackrf.py --band 5 --duration 60 --outfile noise_5ghz.csv
```

---

## Metrics & Plots

After a scenario run, generate all plots with:

```bash
python3 utils/plot_gcs_metrics.py
```

This produces PNG plots for:
- ICMP RTT vs time and sequence number
- MQTT command RTT (raw, smoothed, histogram, boxplot)
- TCP throughput and RTT per interval
- UDP throughput, jitter, and packet loss

Battery plots:
```bash
python3 utils/plot_drone_metrics.py
# (reads battery_metrics.csv from the current directory)
```

---

## TLS / Certificate Setup

The broker and clients authenticate via mutual TLS. Expected certificate paths:

| File | Drone | GCS |
|---|---|---|
| CA certificate | `/etc/mosquitto/ca_certificates/ca.crt` | `./certs/ca.crt` |
| Client certificate | `/etc/mosquitto/certs/drone.crt` | `./certs/client.crt` |
| Client key | `/etc/mosquitto/certs/drone.key` | `./certs/client.key` |

Paths can be customised in the `CERT_*` variables at the top of each script.

To disable TLS (e.g. for local SITL testing), set `USE_TLS = False` in both `drone_mqtt.py` and `ground_station.py`, and change `BROKER_PORT` to `1883`.

---

## Configuration

Key constants at the top of each file:

| Variable | File | Description |
|---|---|---|
| `BROKER_HOST` | both | MQTT broker IP |
| `BROKER_PORT` | both | 8883 (TLS) or 1883 |
| `UAV_ID` | both | Drone identifier (namespaces topics) |
| `SITL_LINK` | `drone_mqtt.py` | MAVLink connection string |
| `QOS_CMD` | `ground_station.py` | Default command QoS level |
| `LOG_DIR` | `plot_gcs_metrics.py` | Scenario log folder to plot |

---

## Troubleshooting

### `ModuleNotFoundError: No module named 'past'`

DroneKit depends on the `future` package, which provides the `past` module. If you see this error on import, install it explicitly:

```bash
pip install future
```

### `TypeError: metaclass conflict` or `MutableMapping` error in DroneKit

Python 3.10+ removed several aliases from `collections` that DroneKit relies on. Open DroneKit's `__init__.py` (usually at `<venv>/lib/pythonX.Y/site-packages/dronekit/__init__.py`) and update the `Parameters` class declaration from:

```python
class Parameters(collections.MutableMapping, HasObservers):
```

to:

```python
class Parameters(collections.abc.MutableMapping, HasObservers):
```

---

## License

MIT — see `LICENSE` for details.
